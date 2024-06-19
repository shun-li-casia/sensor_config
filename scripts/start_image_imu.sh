gnome-terminal --window  -e 'bash -c "source ~/.bashrc; roscore; exec bash"' \
--tab -e 'bash -c "source ~/.bashrc; sleep 1;rosrun sensor_config open_camera_imu_to_rosmsg -i 0 -c 0;  exec bash"' \
--tab -e 'bash -c "source ~/.bashrc; sleep 10;rosrun sensor_config split_hconcate_image -c 0 -r 5; exec bash"' \
--tab -e 'bash -c "source ~/.bashrc; sleep 15;rosrun sensor_config preview_rosimage -i /hconcate_image_cam_0; exec bash"' \
