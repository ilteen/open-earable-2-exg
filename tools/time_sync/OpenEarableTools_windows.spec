# -*- mode: python ; coding: utf-8 -*-
# Windows-specific spec file for OpenEarable Tools

from PyInstaller.utils.hooks import collect_data_files, collect_submodules, collect_all, collect_dynamic_libs

# Force-include all pandas submodules
import sys
import os

datas = []
binaries = []
hiddenimports = []

# Collect bleak
datas += collect_data_files('bleak')

# Collect PyQt6 completely (critical for Windows DLL loading)
pyqt6_datas, pyqt6_binaries, pyqt6_hiddenimports = collect_all('PyQt6')
datas += pyqt6_datas
binaries += pyqt6_binaries
hiddenimports += pyqt6_hiddenimports

# Collect matplotlib
mpl_datas, mpl_binaries, mpl_hiddenimports = collect_all('matplotlib')
datas += mpl_datas
binaries += mpl_binaries
hiddenimports += mpl_hiddenimports

# Collect pandas
pandas_datas, pandas_binaries, pandas_hiddenimports = collect_all('pandas')
datas += pandas_datas
binaries += pandas_binaries
hiddenimports += pandas_hiddenimports

# Collect scipy
scipy_datas, scipy_binaries, scipy_hiddenimports = collect_all('scipy')
datas += scipy_datas
binaries += scipy_binaries
hiddenimports += scipy_hiddenimports

# Find and include pyexpat DLL explicitly from Python's DLLs folder
python_dir = os.path.dirname(sys.executable)
dlls_dir = os.path.join(python_dir, 'DLLs')

# Add all potentially needed DLLs
for dll_name in ['pyexpat.pyd', 'libexpat.dll', '_elementtree.pyd']:
    dll_path = os.path.join(dlls_dir, dll_name)
    if os.path.exists(dll_path):
        print(f"Including: {dll_path}")
        binaries += [(dll_path, '.')]


a = Analysis(
    ['time_sync.py'],
    pathex=[],
    binaries=binaries,
    datas=datas,
    hiddenimports=[
        'bleak.backends.corebluetooth',
        'bleak.backends.bluezdbus',
        'bleak.backends.winrt',
        'bleak.backends.p4android',
        'PyQt6.sip',
        'PyQt6.QtCore',
        'PyQt6.QtGui',
        'PyQt6.QtWidgets',
        'pyexpat',
        'xml.parsers.expat',
        'xml.etree.ElementTree',
        'xml.etree.cElementTree',
        'pandas',
        'pandas._libs',
        'pandas._libs.tslibs.base',
        'pkg_resources.py2_warn',
        'scipy',
        'scipy.signal',
    ] + hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[
        'torch', 'torchvision', 'tensorflow', 
        'IPython', 'jupyter', 'notebook', 'sympy',
        'PyQt5', 'PySide2', 'PySide6',
    ],
    noarchive=False,
    optimize=0,
)

pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name='OpenEarableTools',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    console=True,  # Show console window for debugging
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)

coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=['pyexpat.pyd', 'libexpat.dll', '_elementtree.pyd', 'vcruntime*.dll', 'python*.dll'],
    name='OpenEarableTools',
)