# -*- mode: python ; coding: utf-8 -*-
import os
from PyInstaller.utils.hooks import collect_data_files, collect_submodules

datas = []
binaries = []
hiddenimports = []

# Collect bleak
datas += collect_data_files('bleak')

# Only collect the PyQt6 submodules we actually need (avoid framework symlink conflicts on macOS)
hiddenimports += collect_submodules('PyQt6.QtCore')
hiddenimports += collect_submodules('PyQt6.QtGui')
hiddenimports += collect_submodules('PyQt6.QtWidgets')
hiddenimports += collect_submodules('scipy.signal')
hiddenimports += ['PyQt6.sip']


a = Analysis(
    [os.path.join(SPECPATH, 'time_sync.py')],
    pathex=[],
    binaries=binaries,
    datas=datas,
    hiddenimports=['bleak.backends.corebluetooth', 'bleak.backends.bluezdbus', 'bleak.backends.winrt', 'bleak.backends.p4android', 'PyQt6.sip', 'PyQt6.QtCore', 'PyQt6.QtGui', 'PyQt6.QtWidgets'] + hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=['torch', 'torchvision', 'tensorflow', 'IPython', 'jupyter', 'notebook', 'sympy', 'PyQt5', 'PySide2', 'PySide6'],
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
    console=False,
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
    upx_exclude=[],
    name='OpenEarableTools',
)
app = BUNDLE(
    coll,
    name='OpenEarableTools.app',
    icon=None,
    bundle_identifier='com.openearable.tools',
    version='1.0.0',
    info_plist={
        'CFBundleName': 'OpenEarable Tools',
        'CFBundleDisplayName': 'OpenEarable Tools',
        'CFBundleVersion': '1.0.0',
        'CFBundleShortVersionString': '1.0.0',
        'NSBluetoothAlwaysUsageDescription': 'OpenEarable Tools needs Bluetooth access to connect to your OpenEarable device for time synchronization.',
        'NSBluetoothPeripheralUsageDescription': 'OpenEarable Tools needs Bluetooth access to connect to your OpenEarable device for time synchronization.',
        'NSHighResolutionCapable': True,
        'LSMinimumSystemVersion': '10.15',
        'NSPrincipalClass': 'NSApplication',
    },
)
