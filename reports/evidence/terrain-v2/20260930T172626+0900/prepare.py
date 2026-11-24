"""Make the two frozen diagnostic worlds without changing ROS package files."""

from copy import deepcopy
from pathlib import Path
import hashlib
import xml.etree.ElementTree as ET


HERE = Path(__file__).resolve().parent
SOURCE = HERE.parents[3] / "ros2_ws/src/mooncar_gazebo/worlds/mooncar_ramp_v2.sdf"
EXPECTED = "f919b081532356d448e21492c5dac0a74d05e67f9f8fc29bd821b0a1d9c830ad"


def make_world(low: bool) -> ET.ElementTree:
    world = ET.parse(SOURCE)
    if low:
        for model in world.findall("./world/model"):
            if model.get("name", "").endswith("_ground"):
                for tag in ("ode/mu", "ode/mu2", "bullet/friction", "bullet/friction2"):
                    item = model.find(f"./link/collision/surface/friction/{tag}")
                    assert item is not None, (model.get("name"), tag)
                    item.text = "0.02"
    rover = deepcopy(ET.parse(HERE / "r5.sdf").find("./model"))
    assert rover is not None
    rover.insert(0, ET.Element("pose"))
    rover[0].text = "0 0 0.02 0 0 0"
    world.find("./world").append(rover)
    return world


def main() -> None:
    assert hashlib.sha256(SOURCE.read_bytes()).hexdigest() == EXPECTED
    for name, low in (("baseline", False), ("low_friction", True)):
        world = make_world(low)
        ground = [m for m in world.findall("./world/model") if m.get("name", "").endswith("_ground")]
        assert ground and len(world.findall("./world/model[@name='mooncar_r5']")) == 1
        if low:
            assert all(m.findtext("./link/collision/surface/friction/bullet/friction") == "0.02" for m in ground)
        path = HERE / f"{name}.sdf"
        world.write(path, encoding="utf-8", xml_declaration=True)
        print(f"{name}: {len(ground)} ground models, {path.name}")


if __name__ == "__main__":
    main()
