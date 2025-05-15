import threading
import time
import kingfisher
import cv2
import numpy as np
from datetime import datetime
import os
from embodychain.deploy.w1.ros2_controller.w1_controller import W1Controller, EndEffectorType
import json
import rlia

# 配置参数
CAMERA_IP = "192.168.158.79"
EXPOSURE = 150000
SAVE_INTERVAL = 1  # 100ms
DURATION = 3         # 3秒
SAVE_DIR = "captured_images"

class DataCollector:
    def __init__(self):
        self.camera = None
        self.w1_controller = None
        self.running = False
        # self.calibrator = rlia.calibration.AutoCalibratePTPR(
        #     self.is_eye_in_hand,
        #     intrinsic,
        #     distortion,
        #     resolution,
        #     manipulator_factory,
        #     manipulator_type,
        #     0.5,
        # )
    def initialize(self):
        """初始化所有硬件"""
        try:
            self.camera = kingfisher.connect(CAMERA_IP)
            kingfisher.SetExposure(EXPOSURE)
            
            self.w1_controller = W1Controller(
                end_effector_type=EndEffectorType.PGC_GRIPPER)
            
            # 初始化数据文件（清空旧内容）
            with open(f"{SAVE_DIR}/robot_data.txt", "w") as f:
                pass
            
            print("All devices initialized")
            return True
        except Exception as e:
            print(f"Initialization failed: {e}")
            return False

    def capture_and_save_data(self):
        """采集并保存数据"""
        # 采集数据
        left, right = kingfisher.capture()
        
        robot_data = {
            "right_arm": self.w1_controller.get_current_xpos("right_arm").reshape(-1).tolist(),
            "left_arm": self.w1_controller.get_current_xpos("left_arm").reshape(-1).tolist(),
            "timestamp": datetime.now()
        }
        
        # 格式化时间戳
        time_str = robot_data["timestamp"].strftime("%Y%m%d_%H%M%S_%f")[:-3]
        
        # 保存图像
        left_path = f"{SAVE_DIR}/{time_str}_left.png"
        right_path = f"{SAVE_DIR}/{time_str}_right.png"
        cv2.imwrite(left_path, left)
        cv2.imwrite(right_path, right)
        
        with open(f"{SAVE_DIR}/../robot_data.txt", "a") as f:
            json_record = {
                "timestamp": time_str,
                "right_arm_xpos": robot_data['right_arm'],
                "left_arm_xpos": robot_data['left_arm']
            }
            f.write(json.dumps(json_record) + "\n")

        
    def run(self, duration):
        self.running = True
        end_time = time.time() + duration
        # qpos = np.array(
        #     [
        #         -np.pi / 2.2,
        #         -np.pi / 4,
        #         np.pi / 2,
        #         -np.pi / 2,
        #         -np.pi / 1.8,
        #         -np.pi / 4,
        #         -np.pi / 8,
        #     ]
        # )
        # 机械臂获取初始位置

        # 设置tcp
        tcp_xpos = np.eye(4)
        tcp_xpos[:3, 3] = [0.0, 0.0, 0.143]
        self.w1_controller.set_tcp_xpos(tcp_xpos,name="left_arm")
        self.w1_controller.set_tcp_xpos(tcp_xpos,name="right_arm")

        from IPython import embed
        embed()

        # init_pose_left = self.w1_controller.get_current_pose(name="left_arm")
        # init_pose_right = self.w1_controller.get_current_pose(name="right_arm")        

        # self.w1_controller.move_linear_dual(init_pose_left, init_pose_right)
        # # 生成走点姿态
        # init_pos_leftarm = self.w1_controller.get_current_xpos(name="left_arm")
        # init_pos_rightarm = self.w1_controller.get_current_xpos(name="right_arm")
        # res, trans_pos, rot_pos = self.calibrator.generate_pose(
        #     init_pos_leftarm,
        #     col_num=2,
        #     row_num=2,
        #     layer_num=2,
        #     col_size=0.05,
        #     row_size=0.05,
        #     layer_size=0.05,
        #     rot_rad=np.deg2rad(10),
        #     is_use_kinematics=False,
        # )
        # trans_pos = list(np.concatenate((trans_pos, rot_pos), axis=0))




        # self.robot.set_current_qpos(self.control_arm, qpos)

        try:
            for _ in range(3):
                # 采集和保存数据
                self.capture_and_save_data()

                # 机械臂移动位置
                # self.w1_controller.movej()

                # time.sleep(SAVE_INTERVAL)
                
        except KeyboardInterrupt:
            print("Collection stopped by user")
        finally:
            self.cleanup()

    def cleanup(self):
        """资源清理"""
        self.running = False
        cv2.destroyAllWindows()
        print(f"All data saved to {SAVE_DIR}/robot_data.txt")

if __name__ == "__main__":
    np.set_printoptions(4, suppress=True)
    collector = DataCollector()
    if collector.initialize():
        collector.run(DURATION)