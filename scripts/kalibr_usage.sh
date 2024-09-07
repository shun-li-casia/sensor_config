gnome-terminal --window  -e 'bash -c "taskset -c 2 roslaunch sensor_config open_camera_split_imu.launch uav_id:=$UAV_ID; exec bash"' \
    --tab -e 'bash -c "source ~/.bashrc; sleep 5;rosrun sensor_config preview_rosimage -i /uav_${UAV_ID}/cam_0; exec bash"' \
    --tab -e 'bash -c "source ~/.bashrc; sleep 5;rosrun sensor_config preview_rosimage -i /uav_${UAV_ID}/cam_1; exec bash"' \
