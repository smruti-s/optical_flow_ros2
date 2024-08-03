#!/usr/bin/python3
import cv2
import numpy as np
from pymavlink import mavutil
import math

# import queue
from optical_flow import OpticalFlowParameters

DEFAULT_OUTPUT_RATE = 15
DEFAULT_IMAGE_WIDTH = 64
DEFAULT_IMAGE_HEIGHT = 64
DEFAULT_CONFIDENCE_MULTIPLIER = 1.645
DEFAULT_NUMBER_OF_FEATURES = 20
DEFAULT_FLOW_FEATURE_THRESHOLD = 30
DEFAULT_FLOW_VALUE_THRESHOLD = 3000
DEFAULT_SEARCH_SIZE = 6

lk_params = dict(
    winSize=(15, 15),
    maxLevel=2,
    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
)

feature_params = dict(maxCorners=20, qualityLevel=0.3, minDistance=10, blockSize=7)

op_params = OpticalFlowParameters()

op_params.set_output_rate(out_rate=DEFAULT_OUTPUT_RATE)


class OpticalFlowOpenCV:

    def __init__(
        self,
        search_size=DEFAULT_SEARCH_SIZE,
        flow_feature_threshold=DEFAULT_FLOW_FEATURE_THRESHOLD,
        flow_value_threshold=DEFAULT_FLOW_VALUE_THRESHOLD,
        output_rate=DEFAULT_OUTPUT_RATE,
        img_width=DEFAULT_IMAGE_WIDTH,
        img_height=DEFAULT_IMAGE_HEIGHT,
        conf_multi=DEFAULT_CONFIDENCE_MULTIPLIER,
        n_features=DEFAULT_NUMBER_OF_FEATURES,
    ):  
        self.focal_length_x = None
        self.focal_length_y = None
        self.output_rate = output_rate
        self.img_width = img_width
        self.img_height = img_height
        self.search_size = search_size
        self.flow_feature_threshold = flow_feature_threshold
        self.flow_value_threshold = flow_value_threshold
        self.num_features = n_features
        self.conf_multiplier = conf_multi
        self.camera_matrix = np.zeros([3, 3])
        self.camera_distortion = np.zeros([1, 5])
        self.set_camera_matrix_bool = False
        self.set_camera_distortion_bool = False
        self.update_list = np.empty((0, 1), dtype=np.float32)
        self.err = []
        self.features_current = np.empty((0, 2), dtype=np.float32)
        self.features_previous = np.empty((0, 2), dtype=np.float32)
        self.features_tmp = np.empty((0, 2), dtype=np.float32)
        self.prev_image = None
        # self.useless = np.empty((0, 2), dtype=np.float32)
        self.integrated_xgyro = 0.0
        self.integrated_ygyro = 0.0
        self.integrated_zgyro = 0.0

    def set_camera_distortion(self):
        #Change the matrix to your camera distortion matrix
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
        #Change the matrix to your camera matrix    
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

    def update_gyro(self, gyro_data, dt):
        gx, gy, gz = gyro_data
        self.integrated_xgyro += gx * dt
        self.integrated_ygyro += gy * dt
        self.integrated_zgyro += gz * dt

    def calc_flow(self, img_current, img_time_us: int):

        meancount = 0
        pixel_flow_x_mean = 0.0
        pixel_flow_y_mean = 0.0
        pixel_flow_x_stddev = 0.0
        pixel_flow_y_stddev = 0.0

        self.set_camera_matrix()
        self.set_camera_distortion()

        # frame_gray = np.zeros((self.image_height, self.image_width), dtype=np.uint8)
        # frame_gray.data = img_current
        frame_gray = cv2.cvtColor(img_current, cv2.COLOR_BGR2GRAY)

        mask = np.zeros_like(frame_gray)
        mask[:] = 255

        p = cv2.goodFeaturesToTrack(frame_gray, mask=mask, **feature_params)

        if p is not None:
            self.features_current = p

        if self.prev_image is None:
            self.prev_image = img_current
            self.features_previous = self.features_current

        _, self.update_list, _ = cv2.calcOpticalFlowPyrLK(
            self.prev_image, img_current, self.features_current, None, **lk_params
        )

        self.prev_image = img_current
        self.features_previous = self.features_previous.reshape(-1, 2)
        self.features_current = self.features_current.reshape(-1, 2)
        self.update_list = np.array(self.update_list)

        if self.set_camera_matrix_bool and self.set_camera_distortion_bool:
            self.features_tmp = self.features_current
            self.features_current = cv2.undistortPoints(
                self.features_tmp,
                self.camera_matrix,
                self.camera_distortion,
            )

        for xc, yc in self.features_current:
            xc = xc * self.camera_matrix[0, 0] + self.camera_matrix[0, 2]
            yc = yc * self.camera_matrix[1, 1] + self.camera_matrix[1, 2]

        if self.features_current.size == 0 or self.features_previous.size == 0:
            return 0.0, 0.0, np.array([]), 0, 0  # Returning default values

        if (
            not np.isnan(self.features_current).any()
            and not np.isnan(self.features_previous).any()
        ):
            for i, (xc, yc), (xp, yp) in zip(
                self.update_list, self.features_current, self.features_previous
            ):

                # for i in range(len(self.features_current)):
                if i == 1:
                    pixel_flow_x_mean += xc - xp
                    pixel_flow_y_mean += yc - yp
                    meancount += 1

            # Check if there are any active features
            if meancount:
                pixel_flow_x_mean /= meancount
                pixel_flow_y_mean /= meancount

                # Calculate variance

                for i, (xc, yc), (xp, yp) in zip(
                    self.update_list, self.features_current, self.features_previous
                ):
                    if i == 1:
                        pixel_flow_x_stddev += pow(xc - xp - pixel_flow_x_mean, 2)
                        pixel_flow_y_stddev += pow(
                            yc - yp - pixel_flow_y_mean,
                            2,
                        )

                # convert to standard deviation
                pixel_flow_x_stddev = math.sqrt(pixel_flow_x_stddev / meancount)
                pixel_flow_y_stddev = math.sqrt(pixel_flow_y_stddev / meancount)

                # recalculate pixel flow with 90% confidence interval
                temp_flow_x_mean = 0.0
                temp_flow_y_mean = 0.0
                meancount = 0

                for i, (xc, yc), (xp, yp) in zip(
                    self.update_list, self.features_current, self.features_previous
                ):
                    if i == 1:
                        #  flow of feature i
                        temp_flow_x = xc - xp
                        temp_flow_y = yc - yp

                        # check if inside confidence interval
                        if (
                            math.fabs((temp_flow_x - pixel_flow_x_mean))
                            < pixel_flow_x_stddev * self.conf_multiplier
                        ) and (
                            math.fabs((temp_flow_x - pixel_flow_x_mean))
                            < pixel_flow_x_stddev * self.conf_multiplier
                        ):
                            temp_flow_x_mean += temp_flow_x
                            temp_flow_y_mean += temp_flow_y
                            meancount += 1
                        else:
                            i = 0

                if meancount:
                    # new mean
                    pixel_flow_x_mean = temp_flow_x_mean / meancount
                    pixel_flow_y_mean = temp_flow_y_mean / meancount

        # remember features
        self.features_previous = self.features_current

        # output
        flow_x = pixel_flow_x_mean
        flow_y = pixel_flow_y_mean
        flow_quality = round(255.0 * meancount / len(self.update_list))
        flow_quality, dt_us = op_params.limit_rate(
            flow_quality=flow_quality,
            flow_x=flow_x,
            flow_y=flow_y,
            frame_time_us=img_time_us,
        )
        
        if flow_quality == -1:
            self.get_logger().warning("Flow quality indicates not ready for publishing")

        # convert pixel flow to angular flow
        flow_x = math.atan2(flow_x, self.focal_length_x)
        flow_y = math.atan2(flow_y, self.focal_length_y)

        return flow_x, flow_y, flow_quality, dt_us
