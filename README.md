# v4l2 obs plugin axon

For now it only tested with Raspberry Pi Camera.

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

Select the plugin named `V4L2 axon camera` for camera capturing.
