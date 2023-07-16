#!/usr/bin/env python
"""Refgen Node"""

import cv2
import numpy as np
from typing import Tuple

# ros2
import rclpy
from rclpy.node import Node

# messages
from trylo_control.msg import Reference
from trylo_vision.msg import MarkersDetected
from trylo_gpio.msg import CmdLed

# Packages
from src.definitions import MARKER, STATE, Led
from src.trylo_control.trylo_control.VisionKalmanFilter import VisionKalmanFilter
from src.parameters import (
    D_MIN,
    KF_ENABLE,
    VISION_KF_SETTINGS,
    MARKER_POINTS,
    DIST_COEFFS,
    INTRINSIC_MATRIX,
    MAX_LOST_FRAME,
    MIN_KF_SAMPLES
)
from src.trylo_control.trylo_control.utils import (
    extract_desired_corners,
    find_targets,
    compute_state,
    compute_refs,
    save_data,
    chose_target
)

filename_raw = "data_trylo_cv.csv"
filename_filtered = "data_trylo_kf.csv"
fields = ['Frame', 'Tx', 'Ty', 'Tz']

OFF_STATE = (224, 224, 224)
ENABLED_STATE = (0, 255, 0)
FOLLOW_STATE = (51, 51, 255)
LOST_TARGET = (153, 0, 76)

class Refgen(Node):
    def __init__(self, d_signed: float, kf_enable: bool, max_lost_frame: int):
        super().__init__("n_refgen")
        self.get_logger().info(f'[ REFGEN NODE ]: init {self.get_name()}')
        
        # init params
        self.d_signed: float = d_signed 
        self.max_lost_frame: int = max_lost_frame

        # connection
        self.subscription = self.create_subscription(MarkersDetected, "/vision/aruco", self.cbk_get_markers, 10)
        self.ref_publisher = self.create_publisher(Reference, "/control/ref", 5)
        self.led_publisher = self.create_publisher(CmdLed, "/gpio/led", 5)
        self.create_timer(0.02, self.cbk_pub_ref)
        self.create_timer(0.02, self.cbk_pub_led)

        # attributes
        self.kf = VisionKalmanFilter(VISION_KF_SETTINGS['NX'], VISION_KF_SETTINGS['NZ'], min_samples=MIN_KF_SAMPLES)
        self.target: MARKER = None
        self.state: STATE = STATE.off
        
        # references
        self.d_ref = 0.0  
        self.theta_ref = 0.0  
        self.num_lost_frame = 0
        self.num_frame = 0
        
        # data
        self.data_raw = []
        self.data_filtered = []
        
        # led
        self.led_function = Led.FILL
        self.led_color = OFF_STATE
        
        # setting
        if kf_enable is True: 
            self.kf.start()
            self.kf.enable_vision_mode(VISION_KF_SETTINGS['K1'], VISION_KF_SETTINGS['K2'], VISION_KF_SETTINGS['DT'])
        
    def get_collect_data(self) -> Tuple[list, list]:
        return self.data_raw, self.data_filtered

    #################################################################
    def cbk_get_markers(self, msg_marker):
        ids, corners = msg_marker.ids, msg_marker.corners
        _ids = []
        _corners = []
        for i in range(len(corners)):
            _ids.append(ids[i])
            _corners.append(np.array([[[corners[i].top_left.x, corners[i].top_left.y],
                                        [corners[i].top_right.x, corners[i].top_right.y],
                                        [corners[i].bottom_right.x, corners[i].bottom_right.y],
                                        [corners[i].bottom_left.x, corners[i].bottom_left.y]]], np.float32))
        self.on_iteration(np.array([_ids]), tuple(_corners))
        
    def cbk_pub_ref(self):
        msg = Reference()
        msg.theta = float(self.theta_ref)
        msg.distance = float(self.d_ref)
        self.ref_publisher.publish(msg)
        
    def cbk_pub_led(self):
        msg = CmdLed()
        msg.func = self.led_function.value
        msg.r = self.led_color[0]
        msg.g = self.led_color[1]
        msg.b = self.led_color[2]
        self.led_publisher.publish(msg)
    
    #################################################################
    def clean_signal(self, tvec, rvec):
        measurement = compute_state(t_vec=tvec, r_vec=rvec)
        self.kf.update(measurement)
        self.data_raw.append([self.num_frame, tvec[0][0], tvec[1][0], tvec[2][0]])
        if not self.kf.is_ready(): return tvec
        self.get_logger().info(f'[ REFGEN ]: CLEAN SIGNAL for {str(self.target)}')
        state_hat = self.kf.predict()
        tvec = np.array([state_hat[0], state_hat[1], state_hat[2]], np.float32)
        self.data_filtered.append([self.num_frame, tvec[0][0], tvec[1][0], tvec[2][0]])
        return tvec

    def predict_signal(self):
        if not self.kf.is_ready(): return None
        self.get_logger().info(f"[ REFGEN ]: PREDICTION for {str(self.target)}")
        state_hat = self.kf.predict()
        tvec = np.array([state_hat[0], state_hat[1], state_hat[2]], np.float32)
        self.data_filtered.append([self.num_frame, tvec[0][0], tvec[1][0], tvec[2][0]])
        return tvec
    
    #################################################################
    def check_for_disable(self, ids, corners) -> bool:
        is_detected, _ = extract_desired_corners(id_desired=MARKER.disable_tracker.value, ids=ids, corners=corners)
        if not is_detected: return False
        self.get_logger().info(f'[ REFGEN ]: disable marker detected')
        self.num_lost_frame = 0
        self.state = STATE.off
        return True
    
    #################################################################
    def off_state(self, ids, corners):
        # self.get_logger().info(f'[ REFGEN ]: OFF state, parsing ids = {ids}')
        flag, _ = extract_desired_corners(MARKER.enable_tracker.value, ids, corners)
        self.d_ref = 0.0
        self.theta_ref = 0.0
        
        if len(self.data_filtered) > 0:
            self.get_logger().info('[ REFGEN ]: saving raw data')
            save_data(filename_filtered, self.data_filtered, fields)
        if len(self.data_raw) > 0:
            self.get_logger().info('[ REFGEN ]: saving filtered data')
            save_data(filename_raw, self.data_raw, fields)
        
        self.target = None
        if flag is True: 
            self.get_logger().info(f'[ REFGEN ]: enable marker detected')
            self.state = STATE.enable

    def enable_state(self, ids, corners):
        # extract markers detected and compute reference marker
        detected_targets, detected_corners = find_targets(ids, corners)
        ref_marker = chose_target(detected_targets, detected_corners)
        self.get_logger().info(f'[ REFGEN ]: chosen marker = {ref_marker}')

        # change state iff reference marker is detected and computed
        if ref_marker is None: return
        self.target = MARKER(ref_marker)
        self.kf.reset()
        self.data_raw.clear()
        self.data_filtered.clear()
        self.state = STATE.follow
        
    def follow_state(self, ids, corners):
        # detected targets
        flag, corners = extract_desired_corners(id_desired=self.target.value, ids=ids, corners=corners)
        tvec_hat = None
        if not flag:
            self.get_logger().info(f"[ REFGEN ]: {str(self.target)} LOST")
            self.led_color = LOST_TARGET
            if self.num_lost_frame < self.max_lost_frame:
                self.num_lost_frame += 1
                tvec_hat = self.predict_signal()
        else:
            self.get_logger().info(f"[ REFGEN ]: FOLLOW {str(self.target)}")
            self.num_lost_frame = 0
            _, rvec, tvec = cv2.solvePnP(objectPoints=MARKER_POINTS, imagePoints=corners, cameraMatrix=INTRINSIC_MATRIX, distCoeffs=DIST_COEFFS, flags=cv2.SOLVEPNP_ITERATIVE)
            tvec_hat = self.clean_signal(tvec=tvec, rvec=rvec)
        
        if tvec_hat is None: 
            self.d_ref = 0.0
            self.theta_ref = 0.0
            return
        
        self.get_logger().info(f'[ REFGEN ]: tvec = {tvec_hat}')
        self.d_ref, self.theta_ref = compute_refs(tvec_hat[0][0], tvec_hat[1][0], tvec_hat[2][0])        
        self.get_logger().info(f'[ REFGEN ]: d = {self.d_ref}, theta = {self.theta_ref}')
        if self.d_ref < self.d_signed:
            self.get_logger().info(f'[ REFGEN ]: target reached')
            self.d_ref = 0.0
            self.theta_ref = 0.0
            self.state = STATE.enable
        else:
            self.get_logger().info(f'[ REFGEN ]: target following')
            
    #################################################################
    def on_iteration(self, ids, corners) -> None:
        self.get_logger().info(f'[ REFGEN ]: parsing ids = {ids}, corners = {corners}')
        self.num_frame += 1

        # OFF STATE
        if self.state.value == STATE.off.value:
            self.get_logger().info('[ REFGEN ]: OFF STATE')
            self.led_color = OFF_STATE
            self.off_state(ids, corners)
            return
        
        # ENABLE STATE
        elif self.state.value == STATE.enable.value:
            self.get_logger().info('[ REFGEN ]: ENABLE STATE')
            self.led_color = ENABLED_STATE
            if self.check_for_disable(ids, corners) is False: 
                self.enable_state(ids, corners)
            return

        # FOLLOW STATE
        elif self.state.value == STATE.follow.value:
            self.get_logger().info('[ REFGEN ]: FOLLOW STATE')
            self.led_color = FOLLOW_STATE
            if self.check_for_disable(ids, corners) is False: 
                self.follow_state(ids, corners)
            return


def main(args=None):
    rclpy.init(args=args)
    node = Refgen(d_signed=D_MIN, 
                  kf_enable=KF_ENABLE, 
                  max_lost_frame=MAX_LOST_FRAME)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
