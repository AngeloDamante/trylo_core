"""Utility functions"""

import numpy as np
import cv2
import csv
from typing import Tuple, List
from scipy.spatial.transform import Rotation as R
from src.trylo_control.trylo_control.definitions import MARKER, TARGETS
from src.trylo_control.trylo_control.parameters import (
    D_MIN, 
    DIST_COEFFS, 
    INTRINSIC_MATRIX, 
    MARKER_POINTS    
)

def remap(value: float, x_a: float, x_b: float, y_a: float, y_b: float) -> float:
    """Remap

    Args:
        value (float):
        x_a (float):
        x_b (float):
        y_a (float):
        y_b (float):

    Returns:
        float:
    """
    value = np.clip(value, x_a, x_b)
    mapped_value = ((value - x_a) / (x_b - x_a)) * (y_b - y_a) + y_a
    return mapped_value


def extract_desired_corners(id_desired, ids, corners) -> Tuple[bool, np.ndarray]:
    """Found and extract desired marker and its corners from list of markers
      Args:
        id_desired (int)
        ids (np.ndarray): Nx1
        corners (tuple(np.ndarray)): Nx4x2
      Returns:
        flag (bool): true if found else otherwise
    """
    if len(ids) != len(corners):
        return False, []
    if not id_desired in ids:
        return False, []

    desired_index = np.where(ids == id_desired)[0][0]
    return True, corners[desired_index][0]


def find_targets(ids, corners, targets=TARGETS) -> Tuple[list, list]:
    """Find Targets for input markers

    Args:
        ids (np.ndarray): Nx1
        corners (tuple(np.ndarray)): Nx4x2
        targets (list[int], optional):

    Returns:
        detected targets Tuple[list, list]:
    """
    ids_targets = []
    corners_targets = []
    for target in TARGETS:
        flag, _corners = extract_desired_corners(target.value, ids, corners)
        if flag:
            ids_targets.append(target.value)
            corners_targets.append(_corners)
    return ids_targets, corners_targets


def compute_distances(corners) -> List[float]:
    """ Compute distance for each input corners

    Args:
        corners (Nx4x2):

    Returns:
        distance for each corners
    """
    distances = []
    for target in corners:
        _, _, tvec = cv2.solvePnP(objectPoints=MARKER_POINTS, imagePoints=target, cameraMatrix=INTRINSIC_MATRIX, distCoeffs=DIST_COEFFS, flags=cv2.SOLVEPNP_ITERATIVE)
        distances.append(np.sqrt(tvec[0][0]**2 + tvec[1][0]**2 + tvec[2][0]**2))
    return distances


def chose_target(markers, corners, delta_criteria=D_MIN) -> MARKER:
    """Chose target with minimum distance and that satisfies a certain criteria

    Args:
        markers (np.ndarray):
        corners (list[np.ndarray]):
        delta_criteria (float, optional): Defaults to DELTA param

    Returns:
        MARKER:
    """
    distances = compute_distances(corners)
    _md = np.array([(markers[i], distances[i]) for i in range(len(distances)) if distances[i] > delta_criteria], dtype=object)
    if _md.size == 0:
        return None
    return _md[np.argmin(_md[:, 1], axis=0), 0]


def compute_state(t_vec: np.ndarray, r_vec: np.ndarray) -> np.ndarray:
    """_summary_

    Args:
        t_vec (np.ndarray): _description_
        r_vec (np.ndarray): _description_

    Returns:
        np.ndarray: _description_
    """
    rotMat, _ = cv2.Rodrigues(np.array(r_vec))
    Tx, Ty, Tz = t_vec[0][0], t_vec[1][0], t_vec[2][0]
    qx, qy, qz, qw = R.from_matrix(rotMat).as_quat()
    return np.array([Tx, Ty, Tz, qx, qy, qz, qw], dtype=np.float32)


def compute_refs(x: float, y: float, z: float) -> Tuple[float, float]:
    """Compute references

    Args:
        x (float):
        y (float):
        z (float):

    Returns:
        Tuple[float, float]: d, theta
    """
    ref_d = np.sqrt(x**2 + y**2 + z**2)
    ref_theta = np.rad2deg(np.arctan(x / z))
    return ref_d, ref_theta


def save_data(filename:str, data:list, fields:list = None) -> None:
    """Save data

    Args:
        filename (str): 
        data (list): 
        fields (list, optional): Defaults to None.
    """
    with open(filename, 'w') as csvfile:
        csvwriter = csv.writer(csvfile)
        if fields: csvwriter.writerow(fields)
        csvwriter.writerows(data)