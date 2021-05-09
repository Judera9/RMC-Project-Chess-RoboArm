import cv2

from raspberryturk.core.vision.chessboard_frame import ChessboardFrame
from raspberryturk.core.vision.constants import BOARD_SIZE
from raspberryturk.embedded.vision.chessboard_perspective_transform import get_chessboard_perspective_transform
from raspberryturk.embedded.vision.square_color_detector import SquareColorDetector


class ChessCamera(object):
    def __init__(self):
        self._capture = cv2.VideoCapture(0)
        self._color_detector = SquareColorDetector()

    def current_chessboard_frame(self):
        _, frame = self._capture.read()
        h, w = frame.shape[:2]
        M = get_chessboard_perspective_transform()
        bgr_img = cv2.warpPerspective(frame, M, (BOARD_SIZE, BOARD_SIZE))
        img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)
        return ChessboardFrame(img)

    def current_colored_board_mask(self):
        cbf = self.current_chessboard_frame()
        cbm = [None] * 64
        for i in range(64):
            sq = cbf.square_at(i)
            cbm[i] = self._color_detector.detect(sq)
        return cbm
