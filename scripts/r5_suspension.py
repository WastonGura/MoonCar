"""Research suspension constants, mass-neutral SDF construction and audit."""
import copy
import xml.etree.ElementTree as ET
import numpy as np

WHEELS=('front_left','front_right','rear_left','rear_right')
PROFILE='independent_linear_v1'


def parameters(model):
    from audit_r5_mass import read_sdf
    standalone=ET.Element('sdf',version='1.10');standalone.append(copy.deepcopy(model))
    bodies=[(p['link'],p['mass_kg'],np.array(p['com_m'])) for p in read_sdf(standalone)]
    sprung=[b for b in bodies if b[0] not in [w+'_wheel_link' for w in WHEELS]]
    mass=sum(m for _,m,_ in sprung);moment=sum(m*p[0] for _,m,p in sprung)
    x=abs(next(p[0] for name,_,p in bodies if name=='front_left_wheel_link'))
    result={}
    for w in WHEELS:
        force=1.62*(mass/4+(moment/(4*x) if w.startswith('front') else -moment/(4*x)))
        k=force/.01;c=2*.7*np.sqrt(k*force/1.62)
        result[w]=dict(force_N=force,k_N_m=k,c_N_s_m=float(c),reference_m=-.01,
                       lower_m=-.02,upper_m=.02,unsprung_mass_kg=.4,carrier_mass_kg=.05)
    return dict(profile=PROFILE,hardware_calibrated=False,sprung_mass_kg=mass,
                sprung_first_moment_kg_m=moment,parameters=result)


def suspended(model, observer, output):
    model=copy.deepcopy(model);proof=parameters(model)
    for w in WHEELS:
        name=w+'_wheel_link';link=model.find(f"link[@name='{name}']")
        joint=model.find(f"joint[@name='{w}_wheel_joint']")
        carrier=ET.Element('link',name=w+'_wheel_carrier_link')
        carrier.append(copy.deepcopy(joint.find('pose')))
        inertia=copy.deepcopy(link.find('inertial'))
        inertia.find('mass').text=str(float(inertia.findtext('mass'))/8)
        for e in inertia.find('inertia'):e.text=str(float(e.text)/8)
        carrier.append(inertia);model.append(carrier)
        # Preserve neutral world pose and inertia: split at the same COM.
        original=link.find('inertial');original.find('mass').text=str(float(original.findtext('mass'))*7/8)
        for e in original.find('inertia'):e.text=str(float(e.text)*7/8)
        joint.find('parent').text=carrier.get('name')
        joint.find('pose').set('relative_to',carrier.get('name'))
        joint.find('pose').text='0 0 0 0 0 0'
        sj=ET.SubElement(model,'joint',name=w+'_wheel_suspension_joint',type='prismatic')
        ET.SubElement(sj,'parent').text='base_footprint';ET.SubElement(sj,'child').text=carrier.get('name')
        ET.SubElement(sj,'pose',relative_to=carrier.get('name')).text='0 0 0 0 0 0'
        a=ET.SubElement(sj,'axis');ET.SubElement(a,'xyz').text='0 0 1'
        d=ET.SubElement(a,'dynamics');p=proof['parameters'][w]
        for key,value in [('damping',p['c_N_s_m']),('friction',0),('spring_reference',-.01),('spring_stiffness',p['k_N_m'])]:ET.SubElement(d,key).text=str(value)
        limit=ET.SubElement(a,'limit')
        for key,value in [('lower',-.02),('upper',.02),('effort',50),('velocity',.5)]:ET.SubElement(limit,key).text=str(value)
    plugin=ET.SubElement(model,'plugin',filename=observer,name='mooncar::SuspensionObserver')
    ET.SubElement(plugin,'output').text=output
    for w in WHEELS:ET.SubElement(plugin,'joint').text=w+'_wheel_suspension_joint'
    publisher=model.find("plugin[@name='gz::sim::systems::JointStatePublisher']")
    for w in WHEELS:ET.SubElement(publisher,'joint_name').text=w+'_wheel_suspension_joint'
    return model,proof
