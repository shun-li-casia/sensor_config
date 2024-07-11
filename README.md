# sensor_config

> this package is the open camera and the imu, then publish the messages;

## depends

[Sophus](https://github.com/strasdat/Sophus.git)

```bash
sudo apt install libfmt-dev
wget https://github.com/strasdat/Sophus/archive/refs/tags/1.22.4.tar.gz
tar -xzvf 1.22.4.tar.gz
cd Sophus-1.22.4
mkdir build && cd build
cmake ..
sudo make -j7 install
```

## ros nodes

convert_kalibr_to_vins_param
