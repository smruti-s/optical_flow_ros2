# optical_flow_ros2
Ros2 package for publishing optical flow values to the OPTICAL_FLOW_RAD message. Adopted from the [PX4-OpticalFlow repository](https://github.com/PX4/PX4-OpticalFlow/tree/master). Refer to the [repo](https://github.com/PX4/PX4-OpticalFlow/tree/master) for documentation on the functions. Refer to [MAVLINK message docs](https://mavlink.io/en/messages/common.html#OPTICAL_FLOW_RAD) for documentation on the OPTICAL_FLOW_RAD message. 

## How to use this package -

### Clone inside the src directory of your workspace. (SSH method)
```
cd your_ws/src/
git clone git@github.com:smruti-s/optical_flow_ros2.git
```
### If your workspace is a repository then add this package as a git submodule.

```
git  add submodule https://github.com/smruti-s/optical_flow_ros2
```
### Change the camera calibration matrix and Camera distortion matrix in optical_flow_ros/optical_flow_ros/flow_opencv.py

```
def set_camera_distortion(self):
        self.camera_distortion = np.array(
            [
                [
                    2.56660190e-01,
                    -2.23374068e00,
                    -2.54856563e-03,
                    4.09905103e-03,
                    9.68094572e00,
                ]
            ]
        )
        self.set_camera_distortion_bool = True

def set_camera_matrix(self):

      self.camera_matrix = np.array(
              [
                  [1.29650191e03, 0.00000000e00, 4.14079631e02],
                  [0.00000000e00, 1.29687850e03, 2.26798449e02],
                  [0.00000000e00, 0.00000000e00, 1.00000000e00],
              ]
          )
          self.focal_length_x = self.camera_matrix[0,0]
          self.focal_length_y = self.camera_matrix[1,1]
          self.set_camera_distortion_bool = True
```
### Create a subscription for your range sensor in the flow_publisher node (scripts/flow_publisher.py)

```
def __init__(self):
        super().__init__("optical_flow_publisher")
        self.br_ = CvBridge()
        self.publisher = self.create_publisher(OpticalFlowRad, "optical_flow", 10)
        self.create_subscription(Image, "/image_raw", self.receive_image_data, 10)
        # TODO Create subscription for getting Altitude from Range Sensor
        self.get_logger().info("OpticalFlowPublisher Node has been started.")
```
### Change the optical_flow_msg.distance to take the altitude received from your range sensor
```
            optical_flow_msg.time_delta_distance_us = 10000
            optical_flow_msg.distance = 2.0 # TODO Change to get from Range sensor 
            self.publisher.publish(optical_flow_msg)
```