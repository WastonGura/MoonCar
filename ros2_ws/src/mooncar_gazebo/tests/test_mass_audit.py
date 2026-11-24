import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np

SCRIPT = Path(__file__).resolve().parents[4]/'scripts/audit_r5_mass.py'
spec = importlib.util.spec_from_file_location('mass_audit', SCRIPT)
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


def urdf():
    return ET.fromstring('''<robot name="test"><link name="base_footprint"/>
      <link name="a"><inertial><mass value="2"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial></link>
      <link name="b"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial></link>
      <joint name="root" type="fixed"><parent link="base_footprint"/><child link="a"/></joint>
      <joint name="child" type="fixed"><parent link="a"/><child link="b"/><origin xyz="3 0 0"/></joint>
      </robot>''')


def sdf():
    return ET.fromstring('''<sdf version="1.10"><model name="test"><link name="base_footprint">
      <inertial><mass>3</mass><pose>1 0 0 0 0 0</pose><inertia><ixx>2</ixx><iyy>8</iyy><izz>8</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      </link><frame name="a" attached_to="base_footprint"/><frame name="b" attached_to="a"><pose>3 0 0 0 0 0</pose></frame></model></sdf>''')


def catalog(parts):
    fields = {k: {'classification':'工程假设', 'sources':['source']} for k in ('mass','local_com','mount','inertia')}
    return dict(links={p['link']:copy.deepcopy(fields) for p in parts}, sources={'source':{}},
                procurement=[dict(id=f'{i:02d}',model_status='未建模',detail='unknown mass',nominal_mass_kg=None) for i in range(1,21)])


class MassAuditTests(unittest.TestCase):
    def test_weighted_mass_center(self):
        p, frames = audit.read_urdf(urdf())
        a = audit.aggregate(p)
        self.assertEqual(a['total_mass_kg'],3)
        np.testing.assert_allclose(a['com_m'],[1,0,0])
        self.assertEqual(frames,['base_footprint'])

    def test_parallel_axis_and_fixed_merge(self):
        a=audit.aggregate(audit.read_urdf(urdf())[0])
        b=audit.aggregate(audit.read_sdf(sdf()))
        self.assertTrue(audit.compare(a,b)['passed'])
        np.testing.assert_allclose(a['inertia_kg_m2'],np.diag([2,8,8]))

    def test_rotation_translation_chain(self):
        root=urdf(); root.find("./joint[@name='root']").append(ET.fromstring('<origin xyz="2 0 1" rpy="0 0 1.5707963267948966"/>'))
        a=audit.aggregate(audit.read_urdf(root)[0])
        np.testing.assert_allclose(a['com_m'],[2,1,1],atol=1e-12)
        np.testing.assert_allclose(a['inertia_kg_m2'],np.diag([8,2,8]),atol=1e-12)

    def test_inertial_local_origin(self):
        root=urdf();root.find("./link[@name='b']/inertial/origin").set('xyz','0 3 0')
        np.testing.assert_allclose(audit.aggregate(audit.read_urdf(root)[0])['com_m'],[1,1,0])

    def test_non_root_reference_frame(self):
        np.testing.assert_allclose(audit.aggregate(audit.read_urdf(urdf(),'b')[0])['com_m'],[-2,0,0])

    def test_sdf_joint_child_default(self):
        root=sdf();m=root.find('model');m.append(ET.fromstring('<joint name="j" type="fixed"><parent>base_footprint</parent><child>b</child><pose>1 0 0 0 0 0</pose></joint>'))
        m.find("./link/inertial/pose").set('relative_to','j')
        np.testing.assert_allclose(audit.aggregate(audit.read_sdf(root))['com_m'],[5,0,0])

    def test_sdf_reference_rotation(self):
        root=sdf();root.find("./model/frame[@name='b']/pose").text='3 0 0 0 0 1.5707963267948966'
        root.find('./model/link/inertial/pose').set('relative_to','b')
        np.testing.assert_allclose(audit.aggregate(audit.read_sdf(root))['com_m'],[3,1,0],atol=1e-12)

    def test_duplicate_link_rejected(self):
        root=urdf();root.append(copy.deepcopy(root.find('./link')))
        with self.assertRaises(ValueError):audit.read_urdf(root)

    def test_duplicate_mass_record_rejected(self):
        p=audit.read_urdf(urdf())[0]
        with self.assertRaises(ValueError):audit.aggregate(p+p)

    def test_multiple_parent_rejected(self):
        root=urdf();j=copy.deepcopy(root.find('./joint'));j.set('name','extra');root.append(j)
        with self.assertRaises(ValueError):audit.read_urdf(root)

    def test_disconnected_and_cycle_rejected(self):
        for change in ('disconnected','cycle'):
            root=urdf()
            if change=='disconnected':root.append(ET.fromstring('<link name="orphan"/>'))
            else:root.find("./joint[@name='root']/parent").set('link','b')
            with self.assertRaises(ValueError):audit.read_urdf(root)

    def test_geometry_without_mass_rejected(self):
        root=urdf();root.find('./link').append(ET.fromstring('<visual/>'))
        with self.assertRaises(ValueError):audit.read_urdf(root)

    def test_invalid_mass_rejected(self):
        for value in ('0','-1','nan'):
            root=urdf();root.find('./link/inertial/mass').set('value',value)
            with self.assertRaises(ValueError):audit.read_urdf(root)

    def test_nonphysical_inertia_rejected(self):
        for value in ('-1','5','nan'):
            root=urdf();root.find('./link/inertial/inertia').set('ixx',value)
            with self.assertRaises(ValueError):audit.read_urdf(root)

    def test_missing_inertia_rejected(self):
        root=urdf();i=root.find('./link/inertial');i.remove(i.find('inertia'))
        with self.assertRaises(ValueError):audit.read_urdf(root)

    def test_sdf_cycle_and_unknown_rejected(self):
        for attached in ('b','unknown'):
            root=sdf();root.find("./model/frame[@name='a']").set('attached_to',attached)
            with self.assertRaises(ValueError):audit.read_sdf(root)

    def test_differential_sensitivity(self):
        p=audit.read_urdf(urdf())[0];a=audit.aggregate(p);audit.contributions(p,a)
        np.testing.assert_allclose(sum(np.array(x['com_contribution_m']) for x in p),a['com_m'])
        self.assertAlmostEqual(p[1]['com_derivative_per_added_g_m'][0],2/3*.001)
        self.assertAlmostEqual(p[1]['com_derivative_per_mount_mm_matrix_m'][0][0],1/3*.001)
        delta=1e-6;changed=copy.deepcopy(p);changed[1]['mass_kg']+=delta
        numerical=(np.array(audit.aggregate(changed)['com_m'])-a['com_m'])/delta*.001
        np.testing.assert_allclose(numerical,p[1]['com_derivative_per_added_g_m'],atol=1e-9)

    def test_assumptions_block_real_hardware_confirmation(self):
        p=audit.read_urdf(urdf())[0];c=catalog(p)
        self.assertFalse(audit.provenance(p,c))
        self.assertIsNone(c['procurement'][0]['nominal_mass_kg'])
        self.assertTrue(c['procurement'][0]['mass_missing'])

    def test_missing_provenance_or_source_rejected(self):
        p=audit.read_urdf(urdf())[0]
        for mode in ('missing','unknown_source','unknown_class'):
            c=catalog(p)
            if mode=='missing':c['links'].pop('a')
            elif mode=='unknown_source':c['sources']={}
            else:c['links']['a']['mass']['classification']='verified'
            with self.assertRaises(ValueError):audit.provenance(p,c)

    def test_procurement_coverage_and_status(self):
        p=audit.read_urdf(urdf())[0]
        for mode in ('missing','duplicate','status'):
            c=catalog(p)
            if mode=='missing':c['procurement'].pop()
            elif mode=='duplicate':c['procurement'][0]['id']='02'
            else:c['procurement'][0]['model_status']='zero mass'
            with self.assertRaises(ValueError):audit.provenance(p,c)

    def test_compare_each_tolerance(self):
        a=audit.aggregate(audit.read_urdf(urdf())[0])
        for key,change in [('total_mass_kg',2e-6),('com_m',2e-6),('inertia_kg_m2',2e-8)]:
            b=copy.deepcopy(a)
            if key=='total_mass_kg':b[key]+=change
            elif key=='com_m':b[key][2]+=change
            else:b[key][2][2]+=change
            self.assertFalse(audit.compare(a,b)['passed'])

    def test_output_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaises(FileExistsError):audit.run('a','b','c',d)

    def test_procurement_parser_ignores_price_annotations(self):
        s=importlib.util.spec_from_file_location('catalog',SCRIPT.with_name('build_r5_mass_catalog.py'))
        module=importlib.util.module_from_spec(s);s.loader.exec_module(module)
        text='\n'.join(f'| {i:02d} | part | 1 | 10 | 10 |\n| {i:02d} | price annotation |' for i in range(1,17))
        self.assertEqual(len(module.procurement_rows(text)),16)
        with self.assertRaises(ValueError):module.procurement_rows(text+'\n| 01 | duplicate | 1 | 10 | 10 |')

    def test_source_hash_failure_preserves_failure_receipt(self):
        with tempfile.TemporaryDirectory() as d:
            d=Path(d);u=d/'robot.urdf';s=d/'robot.sdf';c=d/'catalog.json'
            ET.ElementTree(urdf()).write(u);ET.ElementTree(sdf()).write(s)
            source=d/'snapshot.txt';source.write_text('new content')
            data=catalog(audit.read_urdf(urdf())[0])
            data['sources']={'source':dict(snapshot='snapshot.txt',sha256=hashlib.sha256(b'old content').hexdigest())}
            c.write_text(json.dumps(data))
            with self.assertRaisesRegex(ValueError,'source path/hash'):audit.run(u,s,c,d/'output')
            self.assertFalse(json.loads((d/'output/failure.json').read_text())['passed'])
            self.assertEqual(source.read_text(),'new content')

    def test_output_cannot_enclose_input(self):
        with tempfile.TemporaryDirectory() as d:
            d=Path(d);output=d/'result'
            with self.assertRaisesRegex(ValueError,'contain input'):audit.run(output/'robot',d/'sdf',d/'catalog',output)

    def test_duplicate_sdf_frame_rejected(self):
        root=sdf();root.find('model').append(copy.deepcopy(root.find('./model/frame')))
        with self.assertRaises(ValueError):audit.read_sdf(root)

    def test_sdf_auto_inertial_and_format_reject(self):
        root=sdf();root.find('./model/link/inertial').set('auto','true')
        with self.assertRaises(ValueError):audit.read_sdf(root)
        root=sdf();root.find('./model/link/inertial/pose').set('degrees','true')
        with self.assertRaises(ValueError):audit.read_sdf(root)


if __name__=='__main__':unittest.main()
