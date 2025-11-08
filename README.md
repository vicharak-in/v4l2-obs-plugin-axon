# v4l2 obs plugin axon

For now it only tested with Raspberry Pi Camera and HDMI-RX.

## Requirements
- `Mesa3d` for GPU acceleration and `OpenCL`.

Override some env variables for passing `OpenGL` version check.
```bash
export DRI_DRIVER=panfrost
export MESA_GL_VERSION_OVERRIDE=3.3
export MESA_GLSL_VERSION_OVERRIDE=330
export RUSTICL_ENABLE=panfrost
```

## Building
```bash
git clone https://github.com/vicharak-in/v4l2-obs-plugin-axon.git
cd v4l2-obs-plugin-axon
mkdir build && cd build
cmake ..
make
sudo make install
```

Or download the prebuilt plugin `obs-plugin-axon.so`.
```
wget https://github.com/vicharak-in/v4l2-obs-plugin-axon/raw/main/prebuilt/obs-plugin-axon.so
mkdir -p ~/.config/obs-studio/plugins/obs-plugin-axon/bin/64bit/
cp obs-plugin-axon.so ~/.config/obs-studio/plugins/obs-plugin-axon/bin/64bit/
```

Select the plugin named `V4L2 axon camera`.
For Raspberry Pi Camera select `video11` and for HDMI-RX select `vidoe20`.
