"""Use qualified loaded geometry for research flat-world height checks."""
import hashlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET
from r5_flat_scene import validate_flat

FLAT_PROFILE='qualified_flat_mesh_v1'

def prepare_reference(args,world):
    profile=getattr(args,'ground_reference_profile','legacy_scene_grid')
    if profile=='legacy_scene_grid':return None,None
    if profile!=FLAT_PROFILE:raise ValueError('Unknown ground reference profile')
    distance=float(args.linear)*20
    proof=validate_flat(ET.parse(world),distance)
    proof.update(profile=profile,world_sha256=hashlib.sha256(Path(world).read_bytes()).hexdigest(),ground_height_m=0.)
    lo,hi=min(0,distance)-.5,max(0,distance)+.5
    def height(x,y):
        if not math.isfinite(x) or not math.isfinite(y) or not lo<=x<=hi or not -.5<=y<=.5:
            raise ValueError('Ground query outside the qualified flat corridor')
        return 0.
    return height,proof
