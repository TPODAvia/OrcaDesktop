#!/usr/bin/env bash
#
# Installer for Orca Cloud AppImage on Ubuntu 22.04 x86_64
# - installs docker + docker compose plugin (only if missing)
# - installs python3, pip, PyQt6, PyInstaller
# - downloads appimagetool
# - builds OrcaCloud.AppImage that:
#     * on start:  make up && make cloud   (inside $PROJECT_DIR)
#     * opens browser to $CLOUD_URL
#     * on exit:  make down
#

set -e

# ==== CONFIG ====
PROJECT_DIR="$HOME/OrcaDesktops/docker"
CLOUD_URL="http://localhost:9000"   # change to your real RMF web UI URL
APP_NAME="OrcaCloud"
BUILD_ROOT="$PROJECT_DIR/cloud_appimage_build"
APPDIR="$BUILD_ROOT/${APP_NAME}.AppDir"
APPIMAGE_OUTPUT_DIR="$HOME/Applications"
APPIMAGE_NAME="${APP_NAME}.AppImage"
APPIMAGETOOL_DIR="$HOME/AppImageTools"
APPIMAGETOOL_BIN="$APPIMAGETOOL_DIR/appimagetool-x86_64.AppImage"

echo ">>> Orca Cloud AppImage installer"
echo "    Project dir:   $PROJECT_DIR"
echo "    Web UI URL:    $CLOUD_URL"
echo

# ---- Sanity checks ----
if [ ! -d "$PROJECT_DIR" ]; then
  echo "ERROR: PROJECT_DIR '$PROJECT_DIR' does not exist."
  echo "Create it and put docker-compose.yml & Makefile there."
  exit 1
fi

if [ ! -f "$PROJECT_DIR/Makefile" ]; then
  echo "ERROR: Makefile not found in $PROJECT_DIR."
  echo "You must have 'up', 'down' and 'cloud' targets in that Makefile."
  exit 1
fi

# ---- System packages (WITHOUT forcing docker.io if docker already exists) ----
echo ">>> Installing system packages (requires sudo)..."
sudo apt-get update

# Docker only if missing
if ! command -v docker >/dev/null 2>&1; then
  echo ">>> Docker not found, installing docker.io + docker-compose-plugin..."
  sudo apt-get install -y docker.io docker-compose-plugin
  sudo systemctl enable --now docker
else
  echo ">>> Docker already installed, skipping docker.io."
  # only ensure compose plugin is present
  sudo apt-get install -y docker-compose-plugin || true
  sudo systemctl enable --now docker || true
fi

echo ">>> Installing Python & tools..."
sudo apt-get install -y \
  python3 python3-pip python3-venv \
  libfuse2 fuse wget

# ---- Python packages ----
echo ">>> Installing Python packages (PyQt6, PyInstaller)..."
python3 -m pip install --user --upgrade pip
python3 -m pip install --user PyQt6 pyinstaller

PYINSTALLER_BIN="$HOME/.local/bin/pyinstaller"
if [ ! -x "$PYINSTALLER_BIN" ]; then
  echo "ERROR: pyinstaller not found at $PYINSTALLER_BIN"
  exit 1
fi

# ---- appimagetool ----
mkdir -p "$APPIMAGETOOL_DIR"
if [ ! -x "$APPIMAGETOOL_BIN" ]; then
  echo ">>> Downloading appimagetool..."
  wget -O "$APPIMAGETOOL_BIN" \
    "https://github.com/AppImage/AppImageKit/releases/download/continuous/appimagetool-x86_64.AppImage"
  chmod +x "$APPIMAGETOOL_BIN"
else
  echo ">>> appimagetool already present."
fi

# ---- Prepare build dir ----
echo ">>> Preparing build directory..."
rm -rf "$BUILD_ROOT"
mkdir -p "$BUILD_ROOT"
cd "$BUILD_ROOT"

# ---- Python GUI script ----
echo ">>> Creating Python GUI script..."
cat > orca_cloud.py << EOF
#!/usr/bin/env python3
import subprocess
import sys
from pathlib import Path

from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QLabel, QPushButton, QMessageBox
)
from PyQt6.QtCore import Qt

PROJECT_DIR = Path("${PROJECT_DIR}").resolve()
CLOUD_URL = "${CLOUD_URL}"

def run_make(target: str, quiet: bool = False) -> bool:
    try:
        proc = subprocess.run(
            ["make", "-C", str(PROJECT_DIR), target],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
    except FileNotFoundError:
        QMessageBox.critical(
            None,
            "Error",
            "Could not run 'make'. Is it installed?"
        )
        return False

    if proc.returncode != 0:
        if not quiet:
            QMessageBox.critical(
                None,
                f"make {target} failed",
                proc.stdout[:4000]
            )
        return False

    if not quiet:
        print(proc.stdout)
    return True


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Orca Cloud Controller")
        self.setMinimumWidth(380)

        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignmentFlag.AlignTop)

        info = QLabel(
            f"Project: {PROJECT_DIR}\\n"
            f"Web UI:  {CLOUD_URL}\\n\\n"
            "When this window opens, the Docker stack and RMF cloud server\\n"
            "will start automatically. When you close this window,\\n"
            "the Docker stack will be stopped (make down)."
        )
        info.setWordWrap(True)
        layout.addWidget(info)

        btn_open = QPushButton("Open Web UI")
        btn_open.clicked.connect(self.open_browser)
        layout.addWidget(btn_open)

        btn_stop = QPushButton("Stop Now and Quit")
        btn_stop.clicked.connect(self.stop_and_quit)
        layout.addWidget(btn_stop)

        self.setLayout(layout)

        # start stack + cloud immediately
        self.start_stack_and_cloud()
        self.open_browser()

    def start_stack_and_cloud(self):
        if not run_make("up", quiet=False):
            return
        run_make("cloud", quiet=False)

    def open_browser(self):
        try:
            subprocess.Popen(["xdg-open", CLOUD_URL])
        except Exception as e:
            QMessageBox.warning(
                self,
                "Browser error",
                f"Stack started, but browser could not be opened automatically:\\n{e}"
            )

    def stop_and_quit(self):
        run_make("down", quiet=False)
        QApplication.instance().quit()

    def closeEvent(self, event):
        # When user closes window with X
        run_make("down", quiet=True)
        event.accept()


def main():
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
EOF

# ---- Build single binary ----
echo ">>> Building standalone binary with PyInstaller..."
"$PYINSTALLER_BIN" --onefile orca_cloud.py

if [ ! -f "dist/orca_cloud" ]; then
  echo "ERROR: dist/orca_cloud not found"
  exit 1
fi

# ---- Build AppDir ----
echo ">>> Creating AppDir..."
rm -rf "$APPDIR"
mkdir -p "$APPDIR/usr/bin"
mkdir -p "$APPDIR/usr/share/applications"
mkdir -p "$APPDIR/usr/share/icons/hicolor/256x256/apps"

cp dist/orca_cloud "$APPDIR/usr/bin/"

cat > "$APPDIR/${APP_NAME}.desktop" << EOF
[Desktop Entry]
Type=Application
Name=Orca Cloud
Comment=Start RMF Cloud (Docker stack) and open web UI
Exec=orca_cloud
Icon=orcacloud
Terminal=false
Categories=Utility;Development;
EOF

# use terminal icon as placeholder
cp /usr/share/icons/hicolor/256x256/apps/utilities-terminal.png \
   "$APPDIR/usr/share/icons/hicolor/256x256/apps/orcacloud.png" 2>/dev/null || true

cat > "$APPDIR/AppRun" << 'EOF'
#!/bin/bash
HERE="$(dirname "$(readlink -f "$0")")"
export PATH="$HERE/usr/bin:$PATH"
exec orca_cloud "$@"
EOF

chmod +x "$APPDIR/AppRun"

# ---- Build AppImage ----
echo ">>> Building AppImage..."
mkdir -p "$APPIMAGE_OUTPUT_DIR"
"$APPIMAGETOOL_BIN" "$APPDIR" "$APPIMAGE_OUTPUT_DIR/$APPIMAGE_NAME"

chmod +x "$APPIMAGE_OUTPUT_DIR/$APPIMAGE_NAME"

echo
echo "============================================"
echo "✅ AppImage built:"
echo "   $APPIMAGE_OUTPUT_DIR/$APPIMAGE_NAME"
echo
echo "Usage:"
echo "  - Double-click the AppImage."
echo "  - It will run 'make up' and 'make cloud' in:"
echo "      $PROJECT_DIR"
echo "    and open the browser at:"
echo "      $CLOUD_URL"
echo "  - When you close the small window, it will run 'make down'"
echo "    to stop the docker stack."
echo "============================================"
