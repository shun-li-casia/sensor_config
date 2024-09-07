gnome-terminal --window  -e 'bash -c "taskset -c 2 roslaunch sensor_config open_camera_split_imu.launch uav_id:=$UAV_ID; exec bash"' \
