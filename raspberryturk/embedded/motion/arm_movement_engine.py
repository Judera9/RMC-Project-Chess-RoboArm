import numpy as np
# import os
import pickle
import logging
from raspberryturk import RaspberryTurkError, opt_path, cache_path
from sklearn.neighbors import KDTree
import chess

length_default_z = 20

PIECE_HEIGHTS = {  # changed according to the chess height (mm)
    chess.KING: 66,
    chess.QUEEN: 56,
    chess.ROOK: 37,
    chess.BISHOP: 48,
    chess.KNIGHT: 43,
    chess.PAWN: 30,
}

MAX_PIECE_HEIGHT = max(PIECE_HEIGHTS.values())
RESTING_HEIGHT = MAX_PIECE_HEIGHT + 15   # set rest size considering the length of metal rod ?


def _load_tree(logger):
    tree = None

    cached_tree_path = cache_path('arm_movement_engine.kdtree')
    try:
        with open(cached_tree_path, 'rb') as f:
            logger.info("Attemptimg to load kd-tree from cache...")
            tree = pickle.load(f)
            logger.info("Successfully loaded kd-tree from cache.")
    except (OSError, IOError):
        logger.info("Failed to load kd-tree from {}".format(cached_tree_path))

    if tree is None:
        try:
            pts_path = opt_path('arm_movement_engine_pts.npy')
            logger.info("Loading {}...".format(pts_path))
            with open(pts_path, 'rb') as f:
                pts = np.load(f)
                logger.info("Building kd-tree...")
                tree = KDTree(pts)
                logger.info("Done building kd-tree.")
                needs_to_write_cache = True
        except IOError:
            raise RaspberryTurkError("Arm movement engine can't find required file: {}".format(pts_path))
        else:
            try:
                with open(cached_tree_path, 'wb') as f:
                    logger.info("Writing kd-tree to cache...")
                    pickle.dump(tree, f)
                    logger.info("Done writing kd-tree to cache.")
            except (OSError, IOError) as e:
                logger.warn("Failed to write kdtree to {}. Reason: {}.".format(cached_tree_path, e))

    return tree


class ArmMovementEngine(object):
    def __init__(self):
        logger = logging.getLogger(__name__)
        # self._tree = _load_tree(logger)

    def convert_point(self, pt):  # 在二维坐标系中，将（x,y）转化为俩个关节的角度
        pt = np.array(pt).reshape(-1, 2)  # reshape
        index = self._tree.query(pt, return_distance=False).ravel()[0]
        calibrated_offset = 105
        return np.array([index / 1024 + calibrated_offset, index % 1024])

    def convert_point_new(self, pt, piece_type):  # # 将（x,y）+ piece_type 转化为关节的角度
        pt = np.array(pt).reshape(-1, 2)  # reshape
        piece_height = PIECE_HEIGHTS[piece_type]
        point_coordinate = [pt(0), pt(1), piece_height+length_default_z]  # 利用改三维坐标求就逆运动学

        index = self._tree.query(pt, return_distance=False).ravel()[0]
        calibrated_offset = 105
        return np.array([index / 1024 + calibrated_offset, index % 1024])
