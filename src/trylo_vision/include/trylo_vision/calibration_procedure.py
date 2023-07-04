"""To Expose Stream Video from PiCamera to Web Page"""

import numpy as np
from http.server import HTTPServer, BaseHTTPRequestHandler
import cv2
from cv2 import aruco
import logging

print(cv2.__version__)

log_level = logging.DEBUG
logging.basicConfig(level=log_level, format='%(name)s - %(levelname)s - %(message)s')

# Buttons
SPACE_BAR = 32
ESCAPE = 27

# create a webcam
webcam = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L)

# Connection
HOST_ETH = "192.168.0.248"
HOST_WIFI = "192.168.0.250"
PORT = 8000

# page for finish
PAGE = """<html><body><h1>FINISH</h1></body></html>"""

# Charuco Params
BOARD_ROWS = 7
BOARD_COLS = 5
SQUARE_LENGTH = 0.04
MARKER_LENGTH = 0.02
ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
CHARUCO_BOARD = aruco.CharucoBoard((BOARD_COLS, BOARD_ROWS), squareLength=SQUARE_LENGTH, markerLength=MARKER_LENGTH, dictionary=ARUCO_DICT)

# global variables for calibration phase
charuco_detector = aruco.CharucoDetector(CHARUCO_BOARD)
detected_corners = []  # Corners discovered in all images processed
detected_ids = []  # Aruco ids corresponding to corners discovered


def make_intrinsic_matrix(_corners, _ids, _charuco_board, _img_size, _filename) -> None:
    if len(_corners) <= 1:
        logging.debug("---> FAIL")
        return
    is_calibrated, camera_matrix, dist_coeffs, rvecs, tvecs = aruco.calibrateCameraCharuco(
        charucoCorners=_corners,
        charucoIds=_ids,
        board=_charuco_board,
        imageSize=(_img_size[1], _img_size[0]),
        cameraMatrix=None,
        distCoeffs=None
    )
    np.savez(_filename, camera_matrix=camera_matrix, dist_coeffs=dist_coeffs, rvecs=rvecs, tvecs=tvecs)


class CameraHTTP(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=jpgboundary')
        self.end_headers()

        # capture images to use them in calibration phase
        while webcam.isOpened():
            try:
                # take a frame
                flag, img = webcam.read()
                if not flag:
                    continue
                frame = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

                # detection markers and charuco Board
                corners, charuco_ids, _, _ = charuco_detector.detectBoard(frame)
                
                logging.debug(corners)
                if corners is not None:
                    if len(corners) > 20:
                        img = aruco.drawDetectedCornersCharuco(
                            image=img,
                            charucoCorners=corners,
                            charucoIds=charuco_ids
                        )

                        cv2.putText(
                            img=img,
                            org=(10, 50),
                            text="captured!",
                            fontFace=cv2.FONT_HERSHEY_DUPLEX,
                            fontScale=1.0,
                            color=(118, 185, 0),
                            thickness=2
                        )

                        detected_corners.append(corners)
                        detected_ids.append(charuco_ids)

                # show number of collected images
                cv2.putText(img=img, org=(500, 450), text=str(len(detected_corners)), fontFace=cv2.FONT_HERSHEY_DUPLEX, fontScale=1.0, color=(118, 185, 0), thickness=2)

                # take image buffer
                img_bfr = cv2.imencode('.jpg', img)[1]

                # convert image buffer into string
                img_str = img_bfr.tobytes()

                # set header
                self.send_header('Content-type', 'image/jpeg')
                self.end_headers()

                # buffer writer
                self.wfile.write(img_str)
                self.wfile.write(b"\r\n--jpgboundary\r\n")

            except KeyboardInterrupt:
                self.send_header('Content-type', 'text/html')
                self.end_headers()
                self.wfile.write(PAGE.encode())
                break


def main():
    flag, img = webcam.read()
    detected_ids.clear()
    detected_corners.clear()

    # take images
    if flag:
        try:
            HOST = HOST_WIFI
            server = HTTPServer((HOST, PORT), CameraHTTP)
            print("Server now running...")
            server.serve_forever()
        except KeyboardInterrupt:
            webcam.release()
            server.socket.close()
            # calib procedure
            logging.debug("Calibration Phase")
            make_intrinsic_matrix(detected_corners, detected_ids, CHARUCO_BOARD, img.shape, 'calib_data.npz')
            print("Server stop...")

if __name__ == '__main__':
    main()
