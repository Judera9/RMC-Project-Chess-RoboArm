chess_height_types = {'chess_KING': 0.066, 'chess_QUEEN': 0.060,
                      'chess_ROOK': 0.037, 'chess_BISHOP': 0.047,
                      'chess_KNIGHT': 0.046, 'chess_PAWN': 0.037}
distance_to_board = 0.105
length_of_square = 0.036
height_of_board = 0.021
height_of_base = 0.1077

"""
a8 b8 c8 ...
a7 ...
a6 ...
a5 ...
a4 ...
a3 ...
a2[0, 1] ...
a1[0, 0] b1[1, 0] c1
"""


def transform_from_piece_inboard(piece_location, piece_type):
    inboard_x = length_of_square / 2 + length_of_square * piece_location[0]
    inboard_y = length_of_square / 2 + length_of_square * piece_location[1]
    inboard_z = 0
    for chess in chess_height_types:
        if piece_type is chess:
            print('find piece type in dict')
            inboard_z = chess_height_types.get(chess)

    arm_y = inboard_y - 4 * length_of_square
    arm_x = inboard_x + distance_to_board
    arm_z = inboard_z + height_of_board - height_of_base
    return [arm_x, arm_y, arm_z]

