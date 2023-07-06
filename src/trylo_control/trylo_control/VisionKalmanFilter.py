""" Define VisionKalmanFilter Class to handle aruco marker filtering

    Predict:
        x_prior = F * x + B * u
        P_prior = F * P * F_T + Q
        
    Update:
        x_posterior = x_prior + K * innovation
        innovation = z - H * x_prior
        P_posterior = (I - K * H) * P_prior
        K = P_prior * H_T * (H * P_prior * H_T + R)

    The main goal is generate a robust reference and its obtained in two ways:
        - reducing acquiring noise
        - reference prediction
"""

__author__ = "Angelone"
__version__ = "1.1.0"
__main__ = "angelo.damante16@gmail.com"

import cv2
import numpy as np


class VisionKalmanFilter:
    def __init__(self, nX: int, nZ: int, min_samples: int = 10, nU: int = 0) -> None:
        """Filter creation

        Args:
            nX (int): number of dynamic params
            nZ (int): number of measure params
            nU (int): number of control params (optional)
            min_samples (int): minimum number of samples to collect initial measurements
        """
        self.nX = nX
        self.nZ = nZ
        self.nU = nU
        self.min_samples = min_samples

        self.num_samples = 0
        self.is_enabled = False
        self.kf = cv2.KalmanFilter(dynamParams=nX, measureParams=nZ, controlParams=nU)

    def set_matrixes(self, F: np.ndarray = None, H: np.ndarray = None, Q: np.ndarray = None, R: np.ndarray = None, B: np.ndarray = None):
        """Set Main Matrixes of KF

        Args:
            F (np.ndarray, optional): Transition Matrix. Defaults to None.
            H (np.ndarray, optional): Measurement Matrix. Defaults to None.
            Q (np.ndarray, optional): Process Noise Covariance Matrix. Defaults to None.
            R (np.ndarray, optional): Measurement Noise Covariance Matrix. Defaults to None.
            B (np.ndarray, optional): Control Matrix. Defaults to None.
        """
        if F is not None:
            self.kf.transitionMatrix = F.astype(np.float32)
        if H is not None:
            self.kf.measurementMatrix = H.astype(np.float32)
        if Q is not None:
            self.kf.processNoiseCov = Q.astype(np.float32)
        if R is not None:
            self.kf.measurementNoiseCov = R.astype(np.float32)
        if B is not None and self.nU > 0:
            self.kf.ControlMatrix = B.astype(np.float32)

    def enable_vision_mode(self, k1: float, k2: float, dT: float) -> None:
        """Set Matrixes for vision mode

        F = [[1. 0. 0. 0. 0. 0. 0. dT. 0. 0. 0. 0. 0. 0.]
             [0. 1. 0. 0. 0. 0. 0. 0. dT. 0. 0. 0. 0. 0.]
             [0. 0. 1. 0. 0. 0. 0. 0. 0. dT. 0. 0. 0. 0.]
             [0. 0. 0. 1. 0. 0. 0. 0. 0. 0. dT. 0. 0. 0.]
             [0. 0. 0. 0. 1. 0. 0. 0. 0. 0. 0. dT. 0. 0.]
             [0. 0. 0. 0. 0. 1. 0. 0. 0. 0. 0. 0. dT. 0.]
             [0. 0. 0. 0. 0. 0. 1. 0. 0. 0. 0. 0. 0. dT.]
             [0. 0. 0. 0. 0. 0. 0. 1. 0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0. 0. 0. 1. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0. 0. 0. 0. 1. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 1. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 1. 0. 0.]
             [0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 1. 0.]
             [0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 1.]]

        H = [[1. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0.]
             [0. 1. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0.]
             [0. 0. 1. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 1. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 1. 0. 0. 0. 0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 1. 0. 0. 0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0. 1. 0. 0. 0. 0. 0. 0. 0.]]        

        Q = k1 * Id(nX)
        R = k2 * Id(nZ)

        Args:
            k1 (float):
            k2 (float):
            dT (float):
        """
        if not self.is_enabled: return

        # Transition Matrix
        F = np.identity(self.nX)
        F[np.arange(0, self.nZ), np.arange(self.nZ, self.nX)] = dT

        # Measurement Matrix
        H = np.hstack((np.eye(self.nZ), np.zeros((self.nZ, self.nZ))))

        # Process Noise Covariance Matrix
        Q = k1 * np.eye(self.nX)

        # Measurement Noise Covariance Matrix
        R = k2 * np.eye(self.nZ)
        self.set_matrixes(F=F, H=H, Q=Q, R=R)

    def update(self, z: np.ndarray) -> None:
        """Update KF

        Args:
            z (np.ndarray): measurements
        """
        if not self.is_enabled: return
        self.num_samples += 1
        self.kf.correct(z)
        if not self.is_ready(): _ = self.kf.predict()

    def predict(self) -> np.ndarray:
        """Take predict state

        Returns:
            np.ndarry: x_prior
        """
        if not self.is_enabled: return
        return self.kf.predict()

    def start(self) -> None:
        self.is_enabled = True

    def stop(self) -> None:
        if not self.is_enabled: return
        self.num_samples = 0
        self.is_enabled = False
        
    def reset(self) -> None:
        if not self.is_enabled: return
        self.num_samples = 0

    def is_ready(self) -> bool:
        if not self.is_enabled: return False
        return self.num_samples > self.min_samples
