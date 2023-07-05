"""Parameters for Vision settings"""

import numpy as np

FPS = 60
DELTA = 0.2
DEG_RANGE = (-4.0, 4.0)
MARKER_SIZE = 0.084
VISION_KF_SETTINGS = {'NX': 14,
                      'NZ': 7,
                      'DT': 1.0,
                      'K1': 0.003,
                      'K2': 1.0}

# camera settings
intrinsic_parameters = np.load("calib_data.npz", allow_pickle=True)
INTRINSIC_MATRIX = intrinsic_parameters['camera_matrix']
DIST_COEFFS = intrinsic_parameters['dist_coeffs']

# obj point
MARKER_POINTS = np.array([[-MARKER_SIZE / 2, MARKER_SIZE / 2,  0],
                          [MARKER_SIZE / 2, MARKER_SIZE / 2,  0],
                          [MARKER_SIZE / 2, -MARKER_SIZE / 2, 0],
                          [-MARKER_SIZE / 2, -MARKER_SIZE / 2, 0]], dtype=np.float32)
