#!/usr/bin/python3


import cv2
import numpy as np
import math


class OpticalFlowParameters:
    def __init__(self):
        self.image_width = None
        self.image_height = None
        self.focal_length_x = None
        self.focal_length_y = None
        self.output_rate = None
        self.num_features = None
        self.conf_multiplier = None
        self.sum_flow_x = 0.0
        self.sum_flow_y = 0.0
        self.valid_frame_count = 0
        self.sum_flow_quality = 0
        self.time_last_pub = 0

    def set_image_width(self, img_width):
        self.image_width = img_width

    def set_image_height(self, img_height):
        self.image_height = img_height

    def set_focal_length_x(self, f_length):
        self.focal_length_x = f_length

    def set_focal_length_y(self, f_length):
        self.focal_length_y = f_length

    def set_output_rate(self, out_rate):
        # TODO: Check valid range 10-20?
        self.output_rate = out_rate

    def set_num_features(self, n_features):
        self.num_features = n_features

    def set_conf_multiplier(self, cf_multiplier):
        self.num_features = cf_multiplier

    def get_image_width(self):
        return self.image_width

    def get_image_height(self):
        return self.image_height

    def get_focal_length_x(self):
        return self.focal_length_x

    def get_focal_length_y(self):
        return self.focal_length_y

    def get_output_rate(self):
        return self.output_rate

    def get_num_features(self):
        return self.n_features

    def get_conf_multiplier(self):
        return self.cf_multiplier

    def init_limit_rate(self):
        self.sum_flow_x = 0.0
        self.sum_flow_y = 0.0
        self.valid_frame_count = 0
        self.sum_flow_quality = 0

    def limit_rate(
        self,
        flow_quality: int,
        frame_time_us: int,
        flow_x: float,
        flow_y: float,
    ):

        # self.time_last_pub = 0

        if self.output_rate <= 0:
            dt_us = frame_time_us - self.time_last_pub
            self.time_last_pub = frame_time_us

            if frame_time_us == dt_us:
                dt_us = 0
            
            return flow_quality

        if flow_quality > 0:
            self.sum_flow_x += flow_x
            self.sum_flow_y += flow_y
            self.sum_flow_quality += flow_quality
            self.valid_frame_count += 1

        # Limit rate according to the parameter output rate

        if (frame_time_us - self.time_last_pub) > (1.0e6 / self.output_rate):

            average_flow_quality = 0

            if self.valid_frame_count > 0:
                average_flow_quality = math.floor(
                    self.sum_flow_quality / self.valid_frame_count
                )

            flow_x = self.sum_flow_x
            flow_y = self.sum_flow_y

            # reset variables
            self.init_limit_rate()
            dt_us = frame_time_us - self.time_last_pub
            self.time_last_pub = frame_time_us

            if self.time_last_pub == dt_us:
                dt_us = 0

            return average_flow_quality, dt_us

        else:

            return -1, 0 # signaling that it should not yet publish the values
