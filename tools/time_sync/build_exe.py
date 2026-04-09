#!/usr/bin/env python3
"""
Build script to create standalone executables for OpenEarable Tools.

Creates:
- macOS: OpenEarableTools.app
- Windows: OpenEarableTools.exe

Usage:
    python build_exe.py

Requirements:
    pip install pyinstaller

Cross-platform building:
    - macOS .app can only be built on macOS
    - Windows .exe can only be built on Windows
    - To build for both platforms, run this script on each OS
"""

import os
import platform
import subprocess
import sys
import time

APP_NAME = "OpenEarableTools"
SCRIPT_NAME = "time_sync.py"
SPEC_FILE = "OpenEarableTools.spec"
ICON_NAME_MAC = None  # Add "icon.icns" if you have one
ICON_NAME_WIN = None  # Add "icon.ico" if you have one


def check_pyinstaller():
    """Check if PyInstaller is installed."""
    try:
        import PyInstaller
        return True
    except ImportError:
        return False


import shutil


def install_pyinstaller():
    """Install PyInstaller."""
    print("Installing PyInstaller...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "pyinstaller"])


def clean_build_dirs(script_dir):
    """Clean build and dist directories to avoid symlink conflicts."""
    for dirname in ["build", "dist"]:
        dirpath = os.path.join(script_dir, dirname)
        if os.path.exists(dirpath):
            print(f"Removing {dirname}/ directory...")
            last_err = None
            for _ in range(5):
                try:
                    shutil.rmtree(dirpath)
                    last_err = None
                    break
                except OSError as e:
                    last_err = e
                    # macOS can transiently report ENOTEMPTY while deleting app bundles.
                    time.sleep(0.2)
            if last_err is not None and os.path.exists(dirpath):
                raise last_err


def build_app():
    """Build the application for the current platform."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    script_path = os.path.join(script_dir, SCRIPT_NAME)
    spec_path = os.path.join(script_dir, SPEC_FILE)
    spec_path_windows = os.path.join(script_dir, "OpenEarableTools_windows.spec")
    
    if not os.path.exists(script_path):
        print(f"Error: {SCRIPT_NAME} not found!")
        return False
    
    # Clean build directories to avoid symlink conflicts on macOS
    clean_build_dirs(script_dir)
    
    system = platform.system()
    
    # Use platform-specific spec file if available
    if system == "Darwin" and os.path.exists(spec_path):
        print("Building for macOS (.app) using spec file...")
        args = [
            sys.executable, "-m", "PyInstaller",
            "--noconfirm",
            spec_path
        ]
    elif system == "Windows" and os.path.exists(spec_path_windows):
        print("Building for Windows (.exe) using spec file...")
        args = [
            sys.executable, "-m", "PyInstaller",
            "--noconfirm",
            spec_path_windows
        ]
    else:
        # Base PyInstaller arguments for Windows/Linux or fallback
        args = [
            sys.executable, "-m", "PyInstaller",
            "--name", APP_NAME,
            "--windowed",         # No console window (GUI app)
            "--clean",            # Clean build
            "--noconfirm",        # Replace output without asking
            # Hidden imports for bleak (BLE library)
            "--hidden-import", "bleak.backends.corebluetooth",
            "--hidden-import", "bleak.backends.bluezdbus", 
            "--hidden-import", "bleak.backends.winrt",
            "--hidden-import", "bleak.backends.p4android",
            # Hidden imports for PyQt6
            "--hidden-import", "PyQt6.sip",
            "--hidden-import", "PyQt6.QtCore",
            "--hidden-import", "PyQt6.QtGui",
            "--hidden-import", "PyQt6.QtWidgets",
            # Hidden imports for XML parsing (needed on Windows)
            "--hidden-import", "xml.parsers.expat",
            "--collect-submodules", "xml",
            # Collect all bleak data files
            "--collect-data", "bleak",
            # Collect ALL PyQt6 files (critical for Windows DLL loading)
            "--collect-all", "PyQt6",
            # Exclude unnecessary large packages to reduce size
            "--exclude-module", "torch",
            "--exclude-module", "torchvision", 
            "--exclude-module", "tensorflow",
            "--exclude-module", "IPython",
            "--exclude-module", "jupyter", 
            "--exclude-module", "notebook",
            "--exclude-module", "sympy",
            "--exclude-module", "PyQt5",
            "--exclude-module", "PySide2",
            "--exclude-module", "PySide6",
        ]
        
        # Platform-specific options
        if system == "Darwin":
            print("Building for macOS (.app)...")
            args.extend(["--osx-bundle-identifier", "com.openearable.tools"])
            # Use onedir mode for macOS (recommended for .app bundles)
            args.append("--onedir")
            # Add custom Info.plist with Bluetooth permissions
            info_plist = os.path.join(script_dir, "Info.plist")
            if os.path.exists(info_plist):
                args.extend(["--osx-entitlements-file", info_plist])
            if ICON_NAME_MAC and os.path.exists(os.path.join(script_dir, ICON_NAME_MAC)):
                args.extend(["--icon", os.path.join(script_dir, ICON_NAME_MAC)])
        elif system == "Windows":
            print("Building for Windows (.exe)...")
            # Use onefile mode for Windows (single .exe)
            args.append("--onefile")
            if ICON_NAME_WIN and os.path.exists(os.path.join(script_dir, ICON_NAME_WIN)):
                args.extend(["--icon", os.path.join(script_dir, ICON_NAME_WIN)])
        elif system == "Linux":
            print("Building for Linux...")
            args.append("--onefile")
        
        # Add the script path
        args.append(script_path)
    
    # Run PyInstaller
    print(f"Running: {' '.join(args)}")
    print()
    
    try:
        result = subprocess.run(args, cwd=script_dir)
        if result.returncode == 0:
            dist_dir = os.path.join(script_dir, "dist")
            if system == "Darwin":
                output_path = os.path.join(dist_dir, f"{APP_NAME}.app")
            elif system == "Windows":
                output_path = os.path.join(dist_dir, f"{APP_NAME}.exe")
            else:
                output_path = os.path.join(dist_dir, APP_NAME)
            
            print()
            print("=" * 50)
            print("BUILD SUCCESSFUL!")
            print("=" * 50)
            print(f"Output: {output_path}")
            print()
            
            if system == "Darwin":
                print("To run:")
                print(f"  open {output_path}")
                print()
                print("Or double-click the .app in Finder.")
                print()
                print("To distribute: zip the .app bundle:")
                print(f"  cd dist && zip -r {APP_NAME}-macOS.zip {APP_NAME}.app")
            elif system == "Windows":
                print("Double-click the .exe to run.")
                print()
                print("To distribute: copy the .exe file.")
            
            return True
        else:
            print("Build failed!")
            return False
    except Exception as e:
        print(f"Build error: {e}")
        return False


def main():
    print("=" * 50)
    print("OpenEarable Tools - Build Script")
    print("=" * 50)
    print()
    
    # Check and install PyInstaller if needed
    if not check_pyinstaller():
        install_pyinstaller()
    
    # Build
    success = build_app()
    
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
