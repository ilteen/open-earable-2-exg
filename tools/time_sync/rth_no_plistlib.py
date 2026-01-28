# Runtime hook to prevent plistlib import issues on Windows
# This hooks into import system to avoid pyexpat DLL issues

import sys

class PlistlibBlocker:
    """Import hook to block plistlib on Windows (not needed)"""
    
    def find_module(self, name, path=None):
        if name == 'plistlib':
            return self
        return None
    
    def load_module(self, name):
        # Create a dummy module
        import types
        module = types.ModuleType(name)
        module.__file__ = '<blocked>'
        module.__loader__ = self
        module.__spec__ = None
        sys.modules[name] = module
        return module

# Only apply on Windows
if sys.platform == 'win32':
    sys.meta_path.insert(0, PlistlibBlocker())
