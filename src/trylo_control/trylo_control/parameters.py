import numpy as np

# control
DEG_RANGE = (-4.0, 4.0)
D_MIN = 0.2
D_MAX = 2.0
SPEED_MIN = 0.6
SPEED_MAX = 1.0
KF_ENABLE = True
MAX_LOST_FRAME = 20
MIN_KF_SAMPLES = 10

# vision
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
