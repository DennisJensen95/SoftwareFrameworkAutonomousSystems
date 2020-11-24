import pdoc
import os
import sys

print(os.getcwd())
libpath = "../"
sys.path.append(libpath)
pdoc.import_path.append("frame_utilities.py")
mod = pdoc.import_module('kabsch')
doc = pdoc.Module(mod)
string = pdoc.html()
