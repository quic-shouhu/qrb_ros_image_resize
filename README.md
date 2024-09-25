# QRB Image Resize ROS Node
qrb_ros_image_resize provide a ROS Node that enables input image downscaling using EVA acceleration.

## Overview

Qualcomm's smart devices, use NV12 as the default image color space format. To embrace open source and facilitate developers in NV12 image to downscaling, we have developed the image resize ROS node that support EVA acceleration. The feature as follows:

- Provide ROS node include
  - API to downscale nv12 image.

- limitation
  - Supprots input nv12 image and outputs downsized nv12 image.
  - Supports a maximum input image size of 3840 x 2160.
    - 0 < input_width ≤ 3840.
    - 0 < input_height ≤ 2160.
  - Supports a maximum downscale ratio of 1/8.
    -  1/8 input_width ≤ output_width ≤ input_width.
    -  1/8 input_ height ≤ output_ height ≤ input_ height.
  - Supports a minimum output image size of 64 x 64.
    - 64 ≤ output_width ≤ input_width.
    - 64 ≤ output_height ≤ input_height.
  - Supports interpolation methods are EVA_SCALEDOWN_BILINEAR and EVA_SCALEDOWN_BICUBIC.
    - 0: EVA_SCALEDOWN_BILINEAR
    - 1: EVA_SCALEDOWN_BICUBIC
  - Since OpenCV does not support nv12 images to resize, and must convert NV12 to RGB format, which will result in color loss.
    - example: RB3 Gen2 was not support EVA hard ware, it just use opemcv to resize, which will result in color loss.
    - ps: RB3 Gen2 was not support EVA hardware.

- Support dmabuf fd as input / output.

- Input / output image receive/send with QRB ROS transport.
- Hardware accelerates with EVA.

## Build

Currently, we only support NV12 color space format downscale that based on Qualcomm platform that support EVA acceleration.

1. Setup environments follow this document 's [Set up the cross-compile environment.](https://docs.qualcomm.com/bundle/publicresource/topics/80-65220-2/develop-your-first-application_6.html?product=1601111740013072&facet=Qualcomm%20Intelligent%20Robotics%20(QIRP)%20Product%20SDK&state=releasecandidate) part

2. Create `ros_ws` directory in `<qirp_decompressed_workspace>/qirp-sdk/`

3. Clone this repository under `<qirp_decompressed_workspace>/qirp-sdk/ros_ws`
     ```bash
     git clone https://github.com/quic-qrb-ros/lib_mem_dmabuf.git
     git clone https://github.com/quic-qrb-ros/qrb_ros_transport.git
     git clone https://github.com/quic-qrb-ros/qrb_ros_image_resize.git
     ```
4. Build this project
     ```bash
     export AMENT_PREFIX_PATH="${OECORE_TARGET_SYSROOT}/usr;${OECORE_NATIVE_SYSROOT}/usr"
     export PYTHONPATH=${PYTHONPATH}:${OECORE_TARGET_SYSROOT}/usr/lib/python3.10/site-packages

     colcon build --merge-install --cmake-args \
       -DPython3_ROOT_DIR=${OECORE_TARGET_SYSROOT}/usr \
       -DPython3_NumPy_INCLUDE_DIR=${OECORE_TARGET_SYSROOT}/usr/lib/python3.10/site-packages/numpy/core/include \
       -DPYTHON_SOABI=cpython-310-aarch64-linux-gnu -DCMAKE_STAGING_PREFIX=$(pwd)/install \
       -DCMAKE_PREFIX_PATH=$(pwd)/install/share \
       -DBUILD_TESTING=OFF --continue-on-error
     ```
5. Push to the device & Install
     ```bash
     cd `<qirp_decompressed_workspace>/qirp-sdk/ros_ws/install`
     tar czvf qrb_ros_image_resize.tar.gz lib share
     scp qrb_ros_image_resize.tar.gz root@[ip-addr]:/opt/
     ssh root@[ip-addr]
     (ssh) tar -zxf /opt/qrb_ros_image_resize.tar.gz -C /opt/qcom/qirp-sdk/usr/
     ```
## Run  //goto

- Source this file to set up the environment on your device:

```bash
ssh root@[ip-addr]
(ssh) export XDG_RUNTIME_DIR=/dev/socket/weston/
(ssh) export WAYLAND_DISPLAY=wayland-1
(ssh) export HOME=/opt
(ssh) source /opt/qcom/qirp-sdk/qirp-setup.sh
(ssh) export ROS_DOMAIN_ID=99
(ssh) source /usr/bin/ros_setup.bash

```

Run the ROS2 package.

```
(ssh) ros2 launch qrb_ros_image_resize qti_image_resize.launch.py
```

- You can modify the qti_image_resize.launch.py to set the resize.

```python
def generate_launch_description():
    return LaunchDescription([ComposableNodeContainer(
        name='resize_container',
        namespace='container',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='qrb_ros_image_resize',
                plugin='qrb_ros::resize::ResizeNode',
                name='resize_1',
                parameters=[{
                    'use_scale': False,
                    'height': 400,
                    'width': 400,
                }],
            ),
        ]
    )])
```

## Acceleration 

The image resize downscale process accelerate by EVA.

## Packages

Will update in the future.

## Resources

- [ROS2 Type Adaption](https://ros.org/reps/rep-2007.html)

## Contributions

Thanks for your interest in contributing to qrb_image_resize_lib! Please read our [Contributions Page](CONTRIBUTING.md) for more information on contributing features or bug fixes. We look forward to your participation!

## License

qrb_image_resize_lib is licensed under the BSD-3-clause "New" or "Revised" License. 

Check out the [LICENSE](LICENSE) for more details.
