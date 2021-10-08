#!/usr/bin/env python

import os
import subprocess
import sys
from time import sleep

def is_venv():
    return (hasattr(sys, 'real_prefix') or
            (hasattr(sys, 'base_prefix') and sys.base_prefix != sys.prefix))

arg='--venv'
args = sys.argv[1:]

path_to_venv = args[args.index(arg) + 1].rstrip('/')

if not is_venv():
    python_bin = path_to_venv + "/bin/python"
else:
    python_bin = sys.executable

dirname, filename = os.path.split(os.path.abspath(__file__))
script_file = dirname + "/default_server_endpoint.py"

try:
    s = subprocess.Popen([python_bin, script_file] + args)
    return_node = s.wait()
    exit(return_node)

except KeyboardInterrupt:

    while s.poll() is None:
        sleep(5)

    return_node = s.wait()
    exit(return_node)