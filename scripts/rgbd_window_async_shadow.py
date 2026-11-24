#!/usr/bin/env python3
"""Explicit asynchronous sensor-only sidecar; requires --cached-query.

Load the existing ROS transport in an isolated namespace. No publishers,
default changes, workbench registration or navigation/control integration.
"""
import importlib.util
from pathlib import Path
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent))


def main():
    path = Path(__file__).with_name('rgbd_window_shadow.py')
    spec = importlib.util.spec_from_file_location('_async_rgbd_sensor_transport', path)
    node = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = node
    spec.loader.exec_module(node)
    from offline_rgbd_window.transport_async import install_async_transport
    install_async_transport(node)
    node.main()


if __name__ == '__main__':
    main()
