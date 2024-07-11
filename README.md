# sensor_config

## Overview

1. this package is the open camera and the imu, then publish the messages;

## Depends

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

## Nodes and Usage

1. convert the kalibr output files to the vins params

   ```bash
   rosrun sensor_config convert_kalibr_to_vins_param --k_imu <kalibr imu files> --k_cam_imu <kalibr camera imu file>
   ```

2. open the camera and imu and publish them

   ```bash
   rosrun sensor_config open_camera_imu_to_rosmsg --imu_uart <imu uart number> --camera_id <camera id>
   ```

   **Published Topics:**

   - `/hconcate_img_cam_<camera id>`
   - `/imu_raw_0`

3. preview thw ros image msg with opencv

   ```bash
   rosrun sensor_config preview_rosimage -i <ros image topic>
   ```

   **Subscribed Topics:**

   - `/<ros image topic>`

4. save the rosimage to the file

   ```bash
   rosrun sensor_config save_rosimg_to_file -i <ros image topic>
   ```

   **Subscribed Topics:**

   - `/<ros image topic>`

5. split the hconcated image into left and right image at the given rate.

   ```bash
   rosrun sensor_config split_hconcate_image -c <camera id> -r <publish rate>
   ```

   **Published Topics:**

   - `/cam_<camera id>_img_0`
   - `/cam_<camera id>_img_1`

   **Subscribed Topics:**

   - `/hconcate_image_cam_<camera id>`
