<div align="center">
  <h1>QRB ROS Image Resize</h1>
  <p align="center">
    <img src="https://s7d1.scene7.com/is/image/dmqualcommprod/rb3gen2-dev-kits-hero-7" alt="Qualcomm QRB ROS" title="Qualcomm QRB ROS" />
  </p>
  <p>ROS Packages for Image Resize on Qualcomm Robotics Platforms</p>
  <a href="https://ubuntu.com/download/qualcomm-iot" target="_blank"><img src="https://img.shields.io/badge/Qualcomm%20Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white" alt="Qualcomm Ubuntu"></a>
  <a href="https://docs.ros.org/en/jazzy/" target="_blank"><img src="https://img.shields.io/badge/ROS%20Jazzy-1c428a?style=for-the-badge&logo=ros&logoColor=white" alt="Jazzy"></a>
</div>

---

## 👋 Overview

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

## ⚓ APIs

### 🔹 `qrb_ros_image_resize` APIs

#### ROS interfaces

<table>
  <tr>
    <th>Interface</th>
    <th>Name</th>
    <th>Type</th>
    <td>Description</td>
  </tr>
  <tr>
    <td>Subscription</td>
    <td>/image_raw</td>
    <td>qrb_ros/transport/type/Image</td>
    <td>output image</td>
  </tr>
  <tr>
    <td>Publisher</td>
    <td>/image_resize</td>
    <td>qrb_ros/transport/type/Image</td>
    <td>output image</td>
  </tr>
</table>

#### ROS parameters

<table>
  <tr>
    <th>Name</th>
    <th>Type</th>
    <th>Description</td>
    <th>Default Value</td>
  </tr>
  <tr>
    <td>interpolation</td>
    <td>int32</td>
    <td>Interpolation methods</td>
    <td>0</td>
  </tr>
  <tr>
    <td>use_scale</td>
    <td>Uint32</td>
    <td>whether enable downscale ratio</td>
    <td>false</td>
  </tr>
  <tr>
    <td>height_scale</td>
    <td>Uint32</td>
    <td>Height downscale ratio of the output</td>
    <td>1</td>
  </tr>
  <tr>
    <td>width_scale</td>
    <td>Uint32</td>
    <td>Width downscale ratio of the output</td>
    <td>1</td>
  </tr>
  <tr>
    <td>height</td>
    <td>Uint32</td>
    <td>Height of the output</td>
    <td>-1</td>
  </tr>
  <tr>
    <td>width</td>
    <td>Uint32</td>
    <td>Width of the output</td>
    <td>-1</td>
  </tr>
</table>

## 🎯 Supported Targets

<table >
  <tr>
    <th>Development Hardware</th>
  </tr>
  <tr>
    <th>Hardware Overview</th>
  </tr>
  <tr>
    <th>MIPI-CSI Camera Support</th>
  </tr>
  <tr>
    <th>GMSL Camera Support</th>
  </tr>
</table>

---

## 👨‍💻 Build from Source

Currently, we only support NV12 color space format downscale that based on Qualcomm platform that support EVA acceleration.

1. Setup environments follow this document 's [Set up the cross-compile environment.](https://docs.qualcomm.com/bundle/publicresource/topics/80-65220-2/develop-your-first-application_6.html?product=1601111740013072&facet=Qualcomm%20Intelligent%20Robotics%20(QIRP)%20Product%20SDK&state=releasecandidate) part

2. Create `ros_ws` directory in `<qirp_decompressed_workspace>/qirp-sdk/`

3. Clone this repository under `<qirp_decompressed_workspace>/qirp-sdk/ros_ws`
     ```bash
     git clone https://github.com/qualcomm-qrb-ros/lib_mem_dmabuf.git
     git clone https://github.com/qualcomm-qrb-ros/qrb_ros_transport.git
     git clone https://github.com/qualcomm-qrb-ros/qrb_ros_image_resize.git
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
## Run

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

## 🤝 Contributing

We love community contributions! Get started by reading our [CONTRIBUTING.md](CONTRIBUTING.md).
Feel free to create an issue for bug reports, feature requests, or any discussion 💡.

## ❤️ Contributors (Optional)

Thanks to all our contributors who have helped make this project better!

<table>
  <tr>
    <td align="center"><a href="https://github.com/quic-zhanlin"><img src="https://avatars.githubusercontent.com/u/174774501?v=4" width="100" height="100" alt="quic-zhanlin"/><br /><sub><b>quic-shouhu</b></sub></a></td>
  </tr>
</table>

## ❔ FAQs (Optional)

> 📌 Include common and popular questions and answers

## 📜 License

> 📌 Add license declaration and a link to the license file
