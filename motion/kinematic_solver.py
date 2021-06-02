import math

import numpy
import numpy as np
from numpy import sin, cos, arctan2
import matplotlib.pyplot as plt

# ** RETURN VALUE ** 'joint_positions' is a list contains joint state in the form as following:
# [position_0, position1_0, position2_0]
# each elements position_i is a 4 orders 'numpy.ndarray', which specify a column vector
#  ie:
#    position_0 = np.array([0, 0, 0, 1]).T

l_1 = 0.2284
l_2 = 0.1897
d_1 = 0
d_2 = 0
alpha_1 = 0
alpha_2 = 0
constant_err = 0.175  # FIXME: adjust constant error
gripper_err = 0.038
raise_height = 0.1
gripper_height = 0.075


# FIXME: adjust the left parameter


def dh_matrix(delta, d, a, alpha):  # transform matrix of DH
    return np.array([[cos(delta), -sin(delta) * cos(alpha), sin(delta) * sin(alpha), a * cos(delta)],
                     [sin(delta), cos(delta) * cos(alpha), -cos(delta) * sin(alpha), a * sin(delta)],
                     [0, sin(alpha), cos(alpha), d],
                     [0, 0, 0, 1]])


# def fk(theta_1, theta_2):
#     position_0 = np.array([0, 0, 0, 1]).T  # transpose, column vector
#     position_1 = np.array([0, 0, 0, 1]).T
#     position_2 = np.array([0, 0, 0, 1]).T
#     position1_0 = dh_matrix(theta_1, d_1, l_1, alpha_1).dot(position_1)  # results of end of link1 position
#     position2_0 = dh_matrix(theta_1, d_1, l_1, alpha_1).dot(dh_matrix(theta_2, d_2, l_2, alpha_2)).dot(
#         position_2)  # link2
#     return [position_0, position1_0, position2_0]  # positions of links' end


# def fk_solver(theta_1, theta_2, show):  # theta_2 multiply a minus here because we use upper elbow
#     print('RUNNING INTO FK_SOLVER')
#     theta_1_rad = np.radians(theta_1)
#     theta_2_rad = np.radians(-theta_2)
#     if show:
#         print('[INFO] Input [theta_1 theta_2] in degree:', [theta_1, -theta_2])
#     joint_positions = fk(theta_1_rad, theta_2_rad)
#     if show:
#         position_plot(joint_positions)
#     return joint_positions


def ik(des_position_3_1, show_error):
    if show_error:
        print('[INFO] destination in ik is (minus error):', des_position_3_1)
    position_0 = np.array([0, 0, 0, 1]).T
    position_1 = np.array([0, 0, 0, 1]).T
    position_2 = np.array([0, 0, 0, 1]).T

    D = (des_position_3_1[0] ** 2 + des_position_3_1[1] ** 2 - l_1 ** 2 - l_2 ** 2) / (2 * l_1 * l_2)
    theta_2 = arctan2(-(1 - D ** 2) ** 0.5, D)  # note that here use a upper elbow state
    theta_1 = arctan2(des_position_3_1[1], des_position_3_1[0]) - arctan2(l_2 * sin(theta_2), l_1 + l_2 * cos(theta_2))
    theta_3 = theta_1 + theta_2 + 3.1415 / 2

    if show_error:
        print('[INFO] Output [theta_1 theta_2 theta_3] in degree:', numpy.degrees([theta_1, theta_2, theta_3]))

    position1_0 = dh_matrix(theta_1, d_1, l_1, alpha_1).dot(position_1)
    position2_0 = dh_matrix(theta_1, d_1, l_1, alpha_1).dot(dh_matrix(theta_2, d_2, l_2, alpha_2)).dot(position_2)
    return [[theta_1, theta_2, theta_3], [position_0, position1_0, position2_0]]  # positions of links' end


def ik_solver(des_position_3_1, show):  # FIXME: adjust gripper error, constant error
    print('RUNNING INTO IK_SOLVER')
    if show:
        print('[INFO] destination of the end:', des_position_3_1)
    des_position_3_1 = [des_position_3_1[0] - gripper_err, des_position_3_1[1]]
    angles, joint_positions = ik(des_position_3_1, True)
    angles = [angles[0] + constant_err, angles[1] - constant_err, angles[2]]  # FIXME: checkout + and -
    if show:
        position_plot(joint_positions, des=des_position_3_1)
    return [angles, joint_positions]


def base_solver(des_position_1_0, show):
    print('RUNNING INTO BASE_SOLVER')
    des_angle = arctan2(des_position_1_0[0], des_position_1_0[1])
    if show:
        print('[INFO] base angle after rotation is:', des_angle)
    return des_angle


def star_to_des_solver(star_position_3_1, des_position_3_1, show):
    print('RUNNING INTO START TO DESTINATION SOLVER')
    if star_position_3_1[2] != des_position_3_1[2]:
        print('[WARN] piece heights changed, something might be wrong')
    if star_position_3_1 == des_position_3_1:
        print('[WARN] position haven\'t changed in inputs')
    solved_angles = list()

    ik_x_length_start = math.sqrt(star_position_3_1[0] ** 2 + star_position_3_1[1] ** 2)
    ik_x_length_end = math.sqrt(des_position_3_1[0] ** 2 + des_position_3_1[1] ** 2)

    pick_solved_angles = ik_solver([ik_x_length_start, star_position_3_1[2] + raise_height], show)[0]
    solved_angles.append(pick_solved_angles)

    base_solved_angle = base_solver(des_position_3_1[0:2], show)
    solved_angles.append(base_solved_angle)

    move_2_solved_angles = ik_solver([ik_x_length_end, des_position_3_1[2] + raise_height], show)[0]
    solved_angles.append(move_2_solved_angles)

    place_solved_angles = ik_solver([ik_x_length_end, des_position_3_1[2]], show)[0]
    solved_angles.append(place_solved_angles)

    if show:
        print('[INFO] solved angles to archive the motion:\n', solved_angles[0], '\n', solved_angles[1], '\n',
              solved_angles[2])
    return solved_angles


def ik_plot_from_to(from_position, to_position):
    generated_positions = list()
    generated_positions.append(np.linspace(from_position[0], to_position[0], 10))
    print(generated_positions[0])
    generated_positions.append(np.linspace(from_position[1], to_position[1], 10))
    print(generated_positions[1])
    joint_positions_list = list()
    for i in range(len(generated_positions[0])):
        joint_positions_list.append(ik_solver([generated_positions[0][i], generated_positions[1][i]], False)[1])
    position_plot(joint_positions_list, is_from_to=True)


def position_plot(joint_positions, des=None, is_from_to=False):
    fig, ax = plt.subplots()
    if not is_from_to:
        ax.plot([joint_positions[0][0], joint_positions[1][0], joint_positions[2][0]],
                [joint_positions[0][1], joint_positions[1][1], joint_positions[2][1]], '-bo')
        # print('[INFO] position of joint0:', joint_positions[0][0:2])
        # print('[INFO] position of joint1:', joint_positions[1][0:2])
        # print('[INFO] position of joint2:', joint_positions[2][0:2])
    else:
        joint_positions_list = joint_positions
        for i in range(len(joint_positions_list)):
            ax.plot([joint_positions_list[i][0][0], joint_positions_list[i][1][0], joint_positions_list[i][2][0]],
                    [joint_positions_list[i][0][1], joint_positions_list[i][1][1],
                     joint_positions_list[i][2][1]], '-bo')
            # print('[INFO] position of joint0:', joint_positions_list[i][0][0:2])
            # print('[INFO] position of joint1:', joint_positions_list[i][1][0:2])
            # print('[INFO] position of joint2:', joint_positions_list[i][2][0:2])
    if des is not None:
        ax.plot(des[0], des[1] - gripper_height, '-rx')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    plt.xlim(0, 0.5)
    plt.ylim(0, 0.5)
    # fig.savefig('./pic/temp.pdf')
    plt.show()


# solver could be used in project
# fk_solver(20, 30, True)
# ik_solver([0.3, 0.03], True)

# simulate the progress
# ik_plot_from_to([0.3, 0.03], [0.3, 0.2])
# ik_plot_from_to([0.3, 0.2], [0.2, 0.2])
# ik_plot_from_to([0.2, 0.2], [0.2, 0.03])

# star_to_des_solver([0.2, 0.1, 0.075], [0.3, 0.1, 0.075], True)
# {'pick_solved_angles': [1.053612606132627, -0.8869417850622838, 1.7374208210703432],
#  'base_solved_angle': 0.982793723247329,
#  'move_2_solved_angles': [1.5256459016886366, -1.5159843696859385, 1.5804115320026981],
#  'place_solved_angles': [0.9790411018349913, -1.9784215832865615, 0.5713695185484299]}
