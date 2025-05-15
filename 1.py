import numpy as np
from scipy.spatial.transform import Rotation
import math

def euler_to_rotmat(euler, order='xyz'):
    rx, ry, rz = euler
    Rx = np.array([
        [1, 0, 0],
        [0, math.cos(rx), -math.sin(rx)],
        [0, math.sin(rx), math.cos(rx)]
    ])
    Ry = np.array([
        [math.cos(ry), 0, math.sin(ry)],
        [0, 1, 0],
        [-math.sin(ry), 0, math.cos(ry)]
    ])
    Rz = np.array([
        [math.cos(rz), -math.sin(rz), 0],
        [math.sin(rz), math.cos(rz), 0],
        [0, 0, 1]
    ])
    # 采用xyz顺序假设： R = Rx * Ry * Rz
    return Rx @ Ry @ Rz

def rotmat_to_euler(R, order='xyz'):
    r = Rotation.from_matrix(R)
    return r.as_euler(order, degrees=True)

def robot_pose_to_camera_pose(robot_pose, calib_matrix, euler_order='xyz'):
    # robot_pose: [x, y, z, rx, ry, rz]，平移单位毫米，旋转角为°（xyz顺序）
    # 1. 构造 robot_pose_matrix
    translation = np.array(robot_pose[:3]) / 1000.0  # 转换回米
    # 转换角度为弧度
    angles_rad = np.radians(robot_pose[3:])
    R_robot = euler_to_rotmat(angles_rad, order=euler_order)
    robot_pose_matrix = np.eye(4)
    robot_pose_matrix[:3, :3] = R_robot
    robot_pose_matrix[:3, 3] = translation

    # 2. 求 calib_matrix 的逆
    calib_inv = np.linalg.inv(np.array(calib_matrix))

    # 3. 计算 camera_pose_matrix
    camera_pose_matrix = calib_inv @ robot_pose_matrix

    # 4. 从 camera_pose_matrix 提取平移和旋转
    cam_translation = camera_pose_matrix[:3, 3] * 1000.0  # 换回毫米
    cam_R = camera_pose_matrix[:3, :3]
    cam_euler = rotmat_to_euler(cam_R, order=euler_order)  # 单位为度

    # 拼接为1*6向量
    camera_pose = np.concatenate((cam_translation, cam_euler))
    return camera_pose.tolist()

if __name__ == '__main__':
    # 示例数据
    robot_pose = [284.0, 474.0, 58.0, 113.598, -23.144, 111.01]
    calib_matrix = [
        [-0.011535504535896019, 0.5129719491692346, 0.8583278578146107, -0.14996836830630386],
        [0.040977899594497665, -0.8574213631722166, 0.512980913602759, 0.09186953369009686],
        [0.999093461033514, 0.04108996643238013, -0.01112972500380337, -0.15043968854307396],
        [-0.0, 0.0, 0.0, 1.0]
    ]
    camera_pose = robot_pose_to_camera_pose(robot_pose, calib_matrix, euler_order='xyz')
    print("Camera坐标系下的1*6位姿：", camera_pose)

