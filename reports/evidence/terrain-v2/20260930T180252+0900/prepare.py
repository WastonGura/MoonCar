"""Scale the frozen v2 ground friction pairs for a response scan."""

from pathlib import Path
import hashlib
import xml.etree.ElementTree as ET


HERE = Path(__file__).resolve().parent
SOURCE = HERE.parent / "20260930T172626+0900/baseline.sdf"
EXPECTED = "b21a4d9f135885d81b84cb53c594bda2de8f8748881e99643585bf3d65de4448"
SCALES = {"scale_025": 0.25, "scale_050": 0.50, "scale_075": 0.75, "scale_100": 1.00}


def main():
    assert hashlib.sha256(SOURCE.read_bytes()).hexdigest() == EXPECTED
    for name, scale in SCALES.items():
        world = ET.parse(SOURCE)
        ground = [m for m in world.findall("./world/model") if m.get("name", "").endswith("_ground")]
        assert len(ground) == 4
        for model in ground:
            for tag in ("ode/mu", "ode/mu2", "bullet/friction", "bullet/friction2"):
                item = model.find(f"./link/collision/surface/friction/{tag}")
                assert item is not None
                item.text = f"{float(item.text) * scale:.6g}"
        world.write(HERE / f"{name}.sdf", encoding="utf-8", xml_declaration=True)
        print(f"{name}: scale {scale}, {len(ground)} ground models")


if __name__ == "__main__":
    main()
