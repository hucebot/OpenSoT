import os
import types
import sys

_pkg_dir = os.path.dirname(os.path.abspath(__file__))
_platlib_root = os.path.dirname(_pkg_dir)

_resources = os.path.join(_platlib_root, "share", "OpenSoT", "resources")
if os.path.isdir(_resources):
    os.environ.setdefault("OPENSOT_RESOURCES_PATH", _resources)


# this is needed to access submodules defined in c++ in a more intuitive way
# (eg. pyopensot.tasks instead of _pyopensot.tasks)
from . import _pyopensot
def _register_submodules(mod, fullname):
    for name in dir(mod):
        sub = getattr(mod, name)
        if isinstance(sub, types.ModuleType) and sub.__name__.startswith(mod.__name__ + "."):
            subname = fullname + "." + name
            sys.modules[subname] = sub
            _register_submodules(sub, subname)

_register_submodules(_pyopensot, __name__)


# import 1st level objects
from ._pyopensot import *

# temp solution: make replay easier to import TODO refactor viser_opensot_tools
# optional, as user may not have required visualization tools
try:
    from .viser_opensot_tools import replay
except ImportError:
    pass

# XXX TODO useless ? remove
# try:
#     from ._pyopensot_collision import *
# except ImportError:
#     pass
