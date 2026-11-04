"""Attach the GUI after the spawned rover exists in the server's scene snapshot."""
import os
from pathlib import Path
import subprocess
import sys
import time


def main():
    folder=Path(sys.argv[1])
    world_name=sys.argv[2] if len(sys.argv)>2 else 'lunar_yard'
    render_engine=sys.argv[3] if len(sys.argv)>3 else 'ogre'
    deadline=time.monotonic()+120
    while time.monotonic()<deadline:
        try:
            result=subprocess.run(['gz','service','-s',f'/world/{world_name}/scene/info',
                                   '--reqtype','gz.msgs.Empty','--reptype','gz.msgs.Scene',
                                   '--timeout','1500','--req',''],capture_output=True,text=True,timeout=3)
            if result.returncode==0 and 'name: "mooncar"' in result.stdout:
                (folder/'gui-ready.txt').write_text('Server scene contains mooncar before GUI startup\n')
                os.execvp('gz',['gz','sim','-g','-v','3','--render-engine',render_engine,'--gui-config',str(folder/'gui.config')])
        except subprocess.TimeoutExpired: pass
        time.sleep(.5)
    raise RuntimeError('Rover did not appear in the server scene; refusing an empty scene preview')


if __name__=='__main__':main()
