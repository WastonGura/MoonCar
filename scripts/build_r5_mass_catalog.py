"""Freeze local hardware evidence and classify the current R5 mass assumptions."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import xml.etree.ElementTree as ET


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')


def procurement_rows(text):
    rows = []
    for line in text.splitlines():
        if re.match(r'^\| \d{2} \|', line):
            cells = [c.strip() for c in line.strip('|').split('|')]
            if len(cells) == 5:
                rows.append(cells[:3])
    if len(rows) != 16 or {r[0] for r in rows} != {f'{i:02d}' for i in range(1, 17)}:
        raise ValueError('Expected one 16-row current procurement table')
    return rows


def build(repo, hardware, output):
    repo, hardware, output = Path(repo), Path(hardware), Path(output)
    output.mkdir(parents=True, exist_ok=False)
    sources = {}

    def source(key, path, date, version, locator, url=None):
        target = output / (key + path.suffix)
        shutil.copyfile(path, target)
        sources[key] = dict(original_path=str(path), snapshot=target.name,
                            sha256=hashlib.sha256(target.read_bytes()).hexdigest(),
                            document_date=date, version=version, locator=locator, url=url,
                            reviewed_date_jst='2026-10-10')

    desc = repo/'ros2_ws/src/mooncar_description'
    source('model', desc/'urdf/mooncar_r5.urdf.xacro', None, 'current working tree', 'R5-MASS / MOUNTS / GIMBAL / UNMODELED-PAYLOAD')
    source('inertial-macros', desc/'urdf/common/inertial_macros.xacro', None, 'current working tree', 'box_inertial / cylinder_inertial')
    source('description', desc/'README.md', '2026-10-02', 'R5 proposal', 'R5 proposal model')
    source('gaps', repo/'docs/hardware_parameter_gaps.md', '2026-10-02', 'hardware gap audit', 'R5-MASS / MOUNTS / GIMBAL')
    procurement = hardware/'reports/procurement_2026-09-07/MoonCar_国内器件推荐清单_2026-09-08.md'
    source('procurement', procurement, '2026-09-08', 'v2, proposed, not frozen or purchased', '20 rows; size/weight clause 72')
    source('selection-history', hardware/'reports/procurement_2026-09-07/MoonCar_整车约束与动力轮系选型补充_2026-09-08.md', '2026-09-08', 'historical drivetrain route, withdrawn as procurement mainline', '8 kg goal / alternative motors, not current R5')
    source('battery-manual', hardware/'tmp/pdfs/wheeltec_24v_battery_manual.pdf', None, 'archived WHEELTEC 24 V manual, revision not printed in reviewed table', 'PDF page 12, P7120S column, dimensions and weight; visually checked', 'https://wheeltec.net/24-DC.pdf')
    e = repo/'reports/evidence/terrain-v2'
    source('s3-snapshot', e/'20261001T031600+0900/slamtec-spec-lines.txt', '2026-10-01', 'official page snapshot; current live parser omits dimensions', 'lines 107-109: 115 g, 55.6 x 55.6 x 41.3 mm', 'https://www.slamtec.com/en/s3/spec')
    source('oak-record', e/'20261001T212454+0900/sources/luxonis-store-verified-values.json', '2026-10-01', 'reviewer transcription, raw store fetch 429 retained previously', 'manufacturer_values.mass_g=61; dimensions_mm; weaker than original raw snapshot', 'https://checkout.luxonis.com/products/oak-d-lite-1')
    source('invalid-r5-fetch', e/'20261002T021949+0900/sources/r5-jd-lines.txt', '2026-10-02', 'homepage response, rejected as R5 specification evidence', 'no valid corresponding R5 mass breakdown', 'https://item.jd.com/10045479429161.html')
    source('excluded-r550', e/'20261002T021949+0900/sources/wheeltec-r550-text.txt', '2026-10-02', 'R550 educational robot; not selected R5', 'excluded vehicle identity')
    source('pi-mechanical', e/'20261002T021949+0900/sources/pi5-mechanical.pdf', None, 'archived official mechanical drawing', 'page 1; dimensions do not establish populated mass/installation', 'https://datasheets.raspberrypi.com/rpi5/raspberry-pi-5-mechanical-drawing.pdf')
    live = dict(reviewed_date_jst='2026-10-10', method='web open, concise metadata transcription, not full raw response',
                records=[dict(url='https://www.dfrobot.com/product-1915.html', locator='Specification / Others / Weight', mass_g=12, sku='SEN0309'),
                         dict(url='https://www.slamtec.com/en/s3/spec', locator='current extracted page', mass_available=False, archived_mass_source='s3-snapshot'),
                         dict(url='https://shop.luxonis.com/products/oak-d-lite-1', locator='current extracted page, Technical Specifications collapsed', mass_available=False, archived_mass_source='oak-record')])
    save(output/'live-check.json', live)
    sources['live-check'] = dict(snapshot='live-check.json', sha256=hashlib.sha256((output/'live-check.json').read_bytes()).hexdigest(), reviewed_date_jst='2026-10-10', document_date=None, version='current page extraction; no hidden content guessed', locator='records')

    def value(kind, refs, detail):
        return dict(classification=kind, sources=refs, detail=detail)

    def record(mass, refs, detail, shape):
        return dict(mass=value(mass, refs, detail),
                    local_com=value('工程假设', ['model', 'inertial-macros'], 'Uniform '+shape+' approximation, not measured internal density'),
                    mount=value('工程假设', ['model', 'gaps'], 'Nominal URDF joint arrangement; no confirmed assembly drawing'),
                    inertia=value('工程假设', ['model', 'inertial-macros'], 'Analytic homogeneous '+shape+' tensor; not CAD or measured tensor'))

    links = {'base_link': record('工程假设', ['model', 'procurement', 'invalid-r5-fetch'], '4.1 kg body + four 0.4 kg wheels = assumed 5.7 kg bare estimate. Motor/gearbox/encoder allocation not independently confirmed.', 'box'),
             'battery_link': record('厂家标称', ['battery-manual'], 'P7120S 12 Ah nominal 3.05 kg; installed hardware/version not measured; charger excluded.', 'box'),
             'lidar_link': record('厂家标称', ['s3-snapshot'], 'S3 nominal 115 g; kit adapter/cable/bracket not included in this body mass.', 'cylinder'),
             'depth_camera_link': record('厂家标称', ['oak-record'], '61 g historical official-store verification record; raw original unavailable, current page extraction does not re-confirm; AF/FF SKU awaits hardware.', 'box'),
             'imu_link': record('工程假设', ['model', 'gaps'], 'SEN0374 model 80 g and 35 mm envelope are provisional; PCB thickness is not mass.', 'box')}
    for axle in ('front', 'rear'):
        for side in ('left', 'right'):
            links[f'{axle}_{side}_wheel_link'] = record('工程假设', ['model', 'description'], '0.4 kg each, provisional split, not four measured wheels.', 'cylinder')
    for link, mass, shape in [('camera_mount_link', .08, 'cylinder'), ('camera_pan_link', .06, 'box'), ('camera_tilt_link', .04, 'box')]:
        links[link] = record('工程假设', ['model', 'gaps'], f'{mass} kg research gimbal bracket; no selected actuator in procurement.', shape)
    status = {
        '01': ('归属待确认', 'Chassis and four wheels modeled using assumed split. Motors/gearboxes/encoders are abstracted into bare chassis; vendor 5.7 kg inclusion and boards not confirmed.'),
        '02': ('独立建模', 'Battery 3.05 kg body modeled; charger is external and excluded from vehicle mass.'),
        '03': ('归属待确认', 'C50C and bundled OLED have no independent mass links; bare-chassis estimate may/may not include them. Do not add before inclusion is checked.'),
        '04': ('归属待确认', 'Two D50A boards have no independent mass links; bare-chassis inclusion unknown.'),
        '05': ('未建模', 'Pi 5 populated board, not modeled; mass and mounting unknown.'),
        '06': ('未建模', 'Package contains case/cooler/card potentially onboard plus reader/desktop supply/cables offboard; split assembly inventory before counting.'),
        '07': ('独立建模', 'S3 body only; kit USB adapter, wiring and mounts not fully accounted.'),
        '08': ('独立建模', 'Camera body only; actual variant, cable and mounting hardware need confirmation.'),
        '09': ('独立建模', 'SEN0374 separate IMU, mass 80 g is assumption, not OAK internal IMU.'),
        '10': ('未建模', 'Four TFmini Plus: 12 g each nominal, total 48 g; positions and bracket/wiring mass absent.'),
        '11': ('未建模', 'DDR-60G-5 DC/DC power supply and mounting not modeled.'),
        '12': ('未建模', 'HUB HAT and mounting not modeled.'),
        '13': ('未建模', 'Current sensor and signal conditioning not modeled.'),
        '14': ('未建模', 'Emergency-stop button and panel not modeled.'),
        '15': ('未建模', 'EV200 contactor, bracket and wiring not modeled.'),
        '16': ('未建模', 'STLINK is a bench tool, not counted onboard unless future assembly inventory specifies permanent installation.'),
        '17': ('未建模', 'Distribution/fuse/enable/conditioning bundle: quantities and installed mass not frozen; avoid recounting items 14/15.'),
        '18': ('未建模', 'Power and signal harnesses absent; no zero-mass assumption.'),
        '19': ('未建模', 'USB and power cable bundle absent; bundled kit cables must not be counted twice.'),
        '20': ('归属待确认', 'Real mounting structures not frozen. Model has hypothetical gimbal 180 g, not proof that physical assembly hardware is included.')}
    text = procurement.read_text(encoding='utf-8')
    items = []
    for ident, name, quantity in procurement_rows(text):
        state, detail = status[ident]
        nominal = {'02': 3.05, '07': .115, '08': .061, '10': .012}.get(ident)
        items.append(dict(id=ident, name=name, quantity=int(quantity), model_status=state,
                          nominal_mass_kg=nominal, detail=detail, source='procurement',
                          nominal_mass_scope='per module only; not entire kit' if nominal is not None else None))
    for i in range(17, 21):
        ident = str(i)
        state, detail = status[ident]
        items.append(dict(id=ident, name={'17':'配电安全包', '18':'动力与信号线束', '19':'USB与供电线', '20':'安装与整套装配'}[ident], quantity=1, model_status=state, nominal_mass_kg=None, detail=detail, source='procurement'))
    conflicts = [
        dict(topic='bare mass / scope', alternatives=['conditional 5.7 kg procurement estimate', 'model split 4.1 + 4*.4 kg'], resolution='not independently verified, not an actual numeric conflict; motor/gearbox/board inclusion unknown; no averaging'),
        dict(topic='historical 8 kg target vs current model', alternatives=['historical maximum ready mass 8 kg', 'current incomplete model 9.186 kg'], resolution='different requirement/status, not supplier mass; no unauthorized target change'),
        dict(topic='old alternative motors vs R5', alternatives=['historical separately selected GMP motors', 'current complete R5 drivetrain'], resolution='different procurement routes; never add alternative motor masses to R5'),
        dict(topic='research gimbal vs purchase list', alternatives=['model includes 0.18 kg gimbal structures', 'current 20-item list has no gimbal actuator'], resolution='research assumption; physical configuration undecided'),
        dict(topic='OAK sales variant', alternatives=['procurement 2026-09-08 SKU 21545: sales version without IMU', 'current official OAK-D Lite page: BMI270', 'historical 61 g nominal record'], resolution='variant mismatch unclosed; keep 61 g as historical nominal, not measured selected SKU; separate SEN0374 must not be substituted or removed; no extra internal IMU mass added')]
    priorities = [
        dict(priority=1, item='裸底盘总质量及质量范围', method='Confirm exact R5 24V/27 four-wheel version, packing list; weigh chassis with documented wheels/motors/gearboxes/boards inclusion; weigh each wheel and remaining body.', reason='5.7 kg dominates modeled mass, split and inclusion lack evidence.'),
        dict(priority=2, item='电池安装与局部质心', method='Verify P7120S variant and weigh battery without charger; measure pose relative to base_footprint/axle midpoint; obtain CAD or balance-based local COM. Preserve uncertainty.', reason='3.05 kg, about one third of model mass; mount is assumed.'),
        dict(priority=3, item='车载 BOM 与遗漏部件', method='Separate onboard/offboard/bundled items; weigh electronics, power/safety components, cables, brackets, four TFmini; record mounting XYZ and ownership to avoid double counts.', reason='Unknown omitted masses cannot be bounded or set to zero; ordering is provisional.'),
        dict(priority=4, item='云台与 IMU', method='Decide actual gimbal architecture; weigh complete actuator/brackets/camera assembly and SEN0374; record mounts and density/centroid data.', reason='180 g gimbal and 80 g IMU are engineering values.'),
        dict(priority=5, item='独立整车载荷核对', method='After fixed assembly, record total weight and per-wheel/front-rear static loads on level surface, axle geometry and device condition; derive horizontal COM with uncertainty. Do not infer vertical COM from level loads alone.', reason='Independent closure check against component-derived COM; real measured loads are Earth-gravity observations, not lunar simulation validation.')]
    catalog = dict(version=1, reviewed_date_jst='2026-10-10', sources=sources, links=links,
                   procurement=items, conflicts=conflicts, measurement_priorities=priorities,
                   interpretation='Source audit only. No full hardware measurements, no mass redistribution, no new simulation.')
    save(output/'catalog.json', catalog)
    return catalog


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('repo', 'hardware', 'output'):
        p.add_argument('--'+name, required=True)
    args = p.parse_args()
    build(args.repo, args.hardware, args.output)
