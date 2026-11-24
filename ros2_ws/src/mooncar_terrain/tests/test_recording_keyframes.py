import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from map_recorder import keyframe_needed

class KeyframeTests(unittest.TestCase):
    def test_initial_support_then_translation_or_rotation(self):
        frame = (1., [], (0.,0.,0.), (0.,0.,0.,1.))
        self.assertTrue(keyframe_needed([frame]*2, (0,0,0), (0,0,0,1), .05, 5.))
        self.assertFalse(keyframe_needed([frame]*3, (.01,0,0), (0,0,0,-1), .05, 5.))
        self.assertTrue(keyframe_needed([frame]*3, (.06,0,0), (0,0,0,1), .05, 5.))
        yaw=math.radians(6.)
        self.assertTrue(keyframe_needed([frame]*3, (0,0,0), (0,0,math.sin(yaw/2),math.cos(yaw/2)), .05, 5.))
