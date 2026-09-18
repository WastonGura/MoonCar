"""Publish the actual Gazebo GUI display as JPEG; never feed it into mapping."""
import os
from pathlib import Path
import sys
import time
from PIL import ImageGrab

directory=Path(sys.argv[1])
while True:
    try:
        picture=ImageGrab.grab(xdisplay=os.environ['DISPLAY'])
        picture.thumbnail((1000,700))
        temporary=directory/'scene.next.jpg'
        picture.save(temporary,quality=80)
        temporary.replace(directory/'scene.jpg')
    except Exception as error:
        print(f'Scene capture: {error}',flush=True)
    time.sleep(.5)
