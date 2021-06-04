import numpy as np
import chess
import logging
import time
from raspberryturk.embedded.motion.gripper import Gripper
from raspberryturk.embedded.motion.arm import Arm
from raspberryturk.embedded.motion.transform import *


def _castling(move, board):  # 王车易位
    return move.from_square in [chess.E1, chess.E8] \
           and move.to_square in [chess.C1, chess.G1, chess.C8, chess.G8] \
           and board.piece_at(move.from_square).piece_type == chess.KING      # 三个并列条件


def _sq_to_pt(sq):  # converting square to point for arm 从棋盘位置转换成（x,y）坐标
    i = 63 - sq
    return np.array([i % 8, i / 8]) * 3.55 + 1.775  # changed according to board size.  (plane parallel to z=0 )


def _sq_to_sq(sq):
    i = 63 - sq
    return np.array([i % 8, i / 8])


class Coordinator(object):
    def __init__(self):
        self.gripper = Gripper()
        self.arm = Arm()
        self._logger = logging.getLogger(__name__)

    def move_piece(self, move, board):  # 应该是最上层的move方法
        if _castling(move, board):  # 满足王车易位的条件
            a_side = chess.square_file(move.to_square) < chess.square_file(move.from_square) # 确认易位方向
            from_file_index = 0 if a_side else 7
            to_file_index = 3 if a_side else 5
            rank_index = 0 if board.turn is chess.WHITE else 7  # chess color
            rook_from_sq = chess.square(from_file_index, rank_index)
            rook_to_sq = chess.square(to_file_index, rank_index)   # 车的移动
            self._execute_move(_sq_to_sq(rook_from_sq),
                               _sq_to_sq(rook_to_sq),
                               chess.ROOK)
        else:
            captured_piece = board.piece_at(move.to_square)
            if captured_piece is not None:
                self._execute_move(_sq_to_sq(move.to_square), [20, 13.5],   # 这个数据应该需要修改
                                   captured_piece.piece_type)  # 看起来是吃棋子的过程，先把前面的棋子拿走再移动
        piece = board.piece_at(move.from_square)
        self._execute_move(_sq_to_sq(move.from_square),    # 棋子移动
                           _sq_to_sq(move.to_square),
                           piece.piece_type)

    '''该函数传入位置坐标均为由_sq_to_sq()得来的棋盘格数坐标'''
    def _execute_move(self, origin, destination, piece_type):
        self._logger.info("Moving piece {} at {} to {}...".format(piece_type, origin, destination))
        # writing adjustment message
        t0 = time.time()
        origin_array = transform_from_piece_inboard(origin, piece_type)
        des_array = transform_from_piece_inboard(destination, piece_type)
        self.arm.move_from_to(origin_array, des_array)
        elapsed_time = time.time() - t0  # time count
        self._logger.info("Done moving piece (elapsed time: {}s).".format(elapsed_time))
        # writing move done message

    """def move_piece(self, move, board):  # 应该是最上层的move方法
        if _castling(move, board):  # 满足王车易位的条件
            a_side = chess.square_file(move.to_square) < chess.square_file(move.from_square) # 确认易位方向
            from_file_index = 0 if a_side else 7
            to_file_index = 3 if a_side else 5
            rank_index = 0 if board.turn is chess.WHITE else 7  # chess color
            rook_from_sq = chess.square(from_file_index, rank_index)
            rook_to_sq = chess.square(to_file_index, rank_index)   # 车的移动
            self._execute_move(_sq_to_pt(rook_from_sq),
                               _sq_to_pt(rook_to_sq),
                               chess.ROOK)
        else:
            captured_piece = board.piece_at(move.to_square)
            if captured_piece is not None:
                self._execute_move(_sq_to_pt(move.to_square), [20, 13.5],   # 这个数据应该需要修改
                                   captured_piece.piece_type)  # 看起来是吃棋子的过程，先把前面的棋子拿走再移动
        piece = board.piece_at(move.from_square)
        self._execute_move(_sq_to_pt(move.from_square),    # 棋子移动
                           _sq_to_pt(move.to_square),
                           piece.piece_type)

    '''该函数传入位置坐标均为由_sq_to_pt()得来的（x,y）坐标'''
    '''以此函数开始改，gkd！！！'''
    def _execute_move(self, origin, destination, piece_type):
        self._logger.info("Moving piece {} at {} to {}...".format(piece_type, origin, destination))
        # writing adjustment message
        t0 = time.time()
        self.arm.move_to_point_new(origin, piece_type)  # move to pickup point
        self.gripper.pickup()
        self.arm.move_to_point_new(destination, piece_type)  # move to destination
        self.gripper.dropoff()
        self.arm.return_to_rest_new()     # QAQ 已改成角度rest
        elapsed_time = time.time() - t0  # time count
        self._logger.info("Done moving piece (elapsed time: {}s).".format(elapsed_time))
        # writing move done message"""

    def reset(self):
        #  reset
        # self.gripper.calibrate()
        self.arm.return_to_rest_new()

    def close(self):
        self.gripper.cleanup()
