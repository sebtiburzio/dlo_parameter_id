# Some instructions for lab experiments 
(from memory, maybe double check the syntax...)

## Launching the FR3 and MoveIt

First clone the modified [`franka_ros`](https://github.com/sebtiburzio/franka_ros) and [`panda_moveit_config`](https://github.com/sebtiburzio/panda_moveit_config) repos. These add config for the FR3 to MoveIt.
Then on the real time kernel PC:

```roslaunch panda_moveit_config franka_control.launch robot:=fr3 robot_ip:=<ROBOT_IP> load_gripper:=false base_offset_z:=<OFFSET>```

[`base_offset_z`](https://github.com/sebtiburzio/franka_ros/blob/f6f67fd1ddffe9b0d6bf0bc4ddb692421523e43d/franka_description/robots/common/franka_robot.xacro#L19) offsets the [TCP used for motion control](https://github.com/sebtiburzio/panda_moveit_config/blob/a41e879681428ca9c8a0da606aec49c93ff4cc88/config/fr3.srdf.xacro#L14) in MoveIt from the `fr3_link_8` frame. Note that MoveIt motion planning does not use the TCP set in Franka Desk.

### Using the Bota FT sensor

When using the FT sensor, include `use_FT_sensor:=true`. This will add the sensor to the loaded URDF and SRDF. The TCP will then be offset from the FT wrench frame instead. 

### Using the Franka Hand
Note that you can also set `load_gripper:=true` both with & without the FT sensor. Check which TCP MoveIt uses to plan if you do this.

## Launching the FT sensor

There is also a modified [`bota_demo`](https://github.com/sebtiburzio/bota_demo) repo, further information about using the FT sensor can be found there.

```roslaunch bota_demo BFT_SENS_SER_M8_fr3.launch```

## Launching the camera

```roslaunch realsesnse2_camera rs_camera.launch color_height:=1080 color_width:=1920 color_fps:=30```

Check the output when launching since if any of the parameters don't match a supported profile (or USB port isn't fast enoguh) it will set everything to default.

# Camera extrinsic calibration

The extrinsic robot base to camera transform can be calibrated using the MoveIt HandEye Calibration module. In Rviz, add the module from under `moveit_calibration_gui`. Fill in the Target Params as:
```      
target_type: HandEyeTarget/Charuco
squares, X: 4
squares, Y: 4
marker size (px): 50
square size (px): 80
ArUco dictionary: DICT_5X5_250
margin size (px): 2
marker border (bits): 1
longest board side (m): 0.110000
measured marker size (m): 0.012000
camera_info_topic: /camera/color/camera_info
image_topic: /camera/color/image_raw
```
This is for the 4x4 Charuco marker. You can also try opening the Rviz config located in `launch/camera_calibration.rviz` instead, which has all the information filled in (note: loading the config hasn't worked properly recently.)

Once the calibration is done, you can save the launch file it generates. As a subsequent step to check the calibration, you can use `calibrations/camera_calibration_adjustment.py`. First save a bagfile with at least one image from the camera into the same folder as the calibration launch file, then step through the script in interactive mode. A camera image will be displayed with some known points (in the robot base frame) plotted over it, which can be used to adjust the transform from the calibration until they line up. The original and adjusted calibration matrices will then be saved to .npz files.

# Topics to record

Camera and robot state:

```rosbag record -O bagname /camera/color/image_raw/compressed /franka_state_controller/franka_states /camera/color/camera_info```

And FT readings:

```rosbag record -O bagname /bus0/ft_sensor0/ft_sensor_readings/wrench /ft_compensated /camera/color/image_raw/compressed /franka_state_controller/franka_states /camera/color/camera_info```

Use `scripts/extract_bagdata.py` to process the bagfile.
