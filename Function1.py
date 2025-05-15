import numpy as np  
from scipy.spatial.transform import Rotation  
import logging  
import time
from math import cos, sin, radians
from Initialize import Initialize  
import asyncio  
from rich.logging import RichHandler
from unittest.mock import MagicMock
import math
from threading import Thread, Event  
import threading
import json
from copy import deepcopy
global running
class W1_Instruction:

    def __init__(self,log_msg,app):  
        from embodychain.deploy.w1.ros2_controller.w1_controller import W1Controller, EndEffectorType
        self.W1Controller = W1Controller
        self.EndEffectorType = EndEffectorType
        self.w1_controller = self.W1Controller(end_effector_type=self.EndEffectorType.PGC_GRIPPER)

        self.log_msg = log_msg
        self.loop = None 
        self.app = app
        self.T_matrix = np.eye(4)

    def initialize(self):
        # 设置初始位置, 后面计算出来的关节, 会插入这个初始位置里面
        # self.w1_controller.real_w1_init_qpos = np.array([0.85018, -1.70029, 1.19915, -0.00019, -0.00016, -0.80001, -0.56288, -0.70259,  0.00001, -1.38697,  0.77098, -0.148, -1.70108,  0.56291,  0.66846,  0.00001,  1.3869 , -0.66848, 0.147,  1.69997])
        # self.w1_controller.real_current_qpos = deepcopy(self.w1_controller.real_w1_init_qpos)
        pass

    async def async_MoveJ1(self, pose_tuple):
        """
        说明:
            pose_tuple: {"left_arm":robot1pose,"right_arm":robot2pose} 机器人坐标系坐标
        """  
        # if len(pose_tuple)!=2:
        #     exit()
        left_joint_pose = (pose_tuple["left_arm"]) if "left_arm" in pose_tuple else self.get_current_pose('l')[1]
        right_joint_pose = (pose_tuple["right_arm"]) if "right_arm" in pose_tuple else self.get_current_pose('r')[1]
        # from IPython import embed; embed()
        if self.loop is None:
            self.loop = asyncio.get_running_loop()
        success = self.w1_controller.move_joint_dual(
            left_joint_positions= left_joint_pose,
            right_joint_positions= right_joint_pose,
            ratio=0.1,  # 速度
            )

        return success

    def MoveL1(self, pose_tuple,gripper_list):
        """
        说明:
            输入pose_tuple若只含一个运动坐标,自动生成另一个臂当前坐标
            pose_tuple: {"left_arm":robot1pose,"right_arm":robot2pose} 机器人坐标系坐标
        """
        from IPython import embed; embed()
        # 设置TCP坐标系
        right_tcp_xpos = np.eye(4)
        right_tcp_xpos[2, 3] = 0.143
        self.w1_controller.set_tcp_xpos(name="right_arm", tcp_xpos=right_tcp_xpos)
        left_tcp_xpos = np.eye(4)
        left_tcp_xpos[2, 3] = 0.143
        left_tcp_xpos[:3, :3] = Rotation.from_rotvec([0.0, 0.0, 180.], degrees=True).as_matrix()
        self.w1_controller.set_tcp_xpos(name="left_arm", tcp_xpos=left_tcp_xpos)

        
        success = self.w1_controller.move_linear_dual(
            left_pose_list=pose_tuple["left_arm"],
            right_pose_list=pose_tuple["right_arm"],
            threshold=0.01,  # 直线插值间距为2mm
            ratio=15,      # 5倍速度
            grasp_states=gripper_list,
        )
        self.GoHome()
        return success

    def gripper(self,name,status):
        """
        夹具开合状态
        
        参数:
            name: r / l
            status: float(0-1) 开合状态
        """
        if name == "right_arm": ee_name = 'right_gripper' 
        elif name == "left_arm": ee_name = 'left_gripper'
        print("grabber name: ",name)
        self.w1_controller.set_gripper_state(status, ee_name)

    def get_move_pose(self,pose_tuple):
        """
        说明:
            pose_tuple: {"left_arm":robot1pose,"right_arm":robot2pose} 机器人坐标系坐标
        """
        # from IPython import embed; embed()
        left_pose = (pose_tuple["left_arm"]) if "left_arm" in pose_tuple else self.get_current_pose('l')[0]
        right_pose = (pose_tuple["right_arm"]) if "right_arm" in pose_tuple else self.get_current_pose('r')[0]


        left_pose[3], left_pose[5] = left_pose[5], left_pose[3]
        right_pose[3], right_pose[5] = right_pose[5], right_pose[3]
        left_fk_pose_list, right_fk_pose_list = self.w1_controller.get_linear_trajectory(
            left_pose_list=left_pose,
            right_pose_list=right_pose,
            qpos_seed=self.w1_controller.real_current_qpos,
            rotation_sequence="ZYX",
            threshold=0.01,
            ratio=0.5,
            sample_num=10
        )
        return left_fk_pose_list, right_fk_pose_list 

    def GoHome(self):
        _ = self.w1_controller.go_home()
        self.w1_controller.set_gripper_state(1.0, ee_name="right_gripper")
        self.w1_controller.set_gripper_state(1.0, ee_name="left_gripper")
        time.sleep(1.0)

    def calibration(self,robot_arm,sign,use_movepse):
        running = True
        tcp_xpos = np.eye(4)
        tcp_xpos[2, 3] = 0
        self.w1_controller.set_tcp_xpos(name="left_arm", tcp_xpos=tcp_xpos)
        self.w1_controller.set_tcp_xpos(name="right_arm", tcp_xpos=tcp_xpos)

        self.w1_controller.real_w1_init_qpos = self.w1_controller.zero_err_node.get_qpos()
        self.w1_controller.real_current_qpos = deepcopy(self.w1_controller.real_w1_init_qpos)
        if self.log_msg["wait_calib_sign"][1]==1: logging.info(self.log_msg["wait_calib_sign"][0].format(sign))
        app_msg=self.app.orecv('d,0,0,0,0,0,0,2',calib_now=True) # 接受视觉返回信息
        if robot_arm == "right_arm":
            calib_pose=[ # 10个坐标俯视 末端旋转90度
            [0.08737698378495473, 0.5564994677051378, 0.028606344849086085, 1.8605149638818919, -0.6719674746682482, 0.7679251634854385, -0.026221484092920022],
            [0.08732904688533338, 0.5564635150304218, -0.09430386578025107, 1.8605149638818919, -0.8290926474023785, 0.7137924195879428, -0.32287898740011656],
            [-0.01747299991200979, 0.3630261408330613, -0.14666294439175376, 1.9285733771194113, -0.8290806631774732, 0.7138044038128482, -0.3228909716250219],
            [0.03489806292439823, 0.17103885784924877, -0.14666294439175376, 1.8762023142830033, -0.8290926474023785, 0.6614453252013455, -0.5149022230586451],
            [0.03489806292439823, 0.17103885784924877, -0.23393207015256046, 1.8761783458331927, -0.9861818674617924, 0.7138163880377535, -0.5672373332203371],
            [0.13961622014740405, 0.43282226668185775, -0.37356027452486984, 1.8761903300580984, -0.9861818674617924, 0.7138163880377535, -0.5672373332203371],
            [0.13959225169759337, 0.4328102824569524, -0.3735482902999645, 1.7365980783605046, -1.0908640720100817, 0.7138163880377535, -0.5672373332203371],
            [0.31414248744411166, 0.43282226668185775, -0.42589538468656185, 1.85876526704571, -1.1606841663086893, 0.7312654194999526, -0.6545424116558598],
            [0.31414248744411166, 0.43282226668185775, -0.4259193531363725, 1.8587412985958993, -1.0035829620243701, 0.7312774037248579, -0.6545543958807651],
            [0.22688534590821074, 0.5375643923546742, -0.4259193531363725, 1.8587532828208042, -1.0385169776234844, 0.5916252309027379, -0.5148662703839291]
            ]  
        elif robot_arm == "left_arm":
            calib_pose=[ # 10个坐标俯视 末端旋转90度
            [-0.7872796867075893, -1.0178202212119336, 0.740685020275563, -1.0751767216089703, -0.13421133471508817, 0.8830815806010097, -2.0591415074632717],
            [-0.5955560566716946, -0.9218984850694598, 0.740685020275563, -1.0751527531591596, -0.1342233189399935, 0.8830815806010097, -2.0591534916881766],
            [-0.823232361423659, -0.9615183326065684, 0.8365468352935101, -0.9912871472714744, -0.11024288490437506, 0.8830935648259146, -2.0591415074632717],
            [-0.8232203771987536, -0.9615183326065684, 0.8724995100095798, -1.135097846135753, 0.00961134837409583, 1.038876504370645, -2.346774889416735],
            [-1.0509206504005286, -0.9734905732870196, 0.9923417590631454, -1.18305871420699, 0.03357979818480894, 0.9914189737454331, -2.346774889416735],
            [-1.0509566030752446, -0.9734306521624929, 0.9923417590631454, -1.182986808857558, 0.0335678139599036, 0.9789553798438622, -2.346774889416735],
            [-1.038972378169888, -0.7097777042446483, 0.9923537432880511, -1.2908568172306725, 0.03357979818480894, 0.9789433956189573, -2.3467509209669237],
            [-0.9910234943235565, -0.7936792628070495, 0.884507703364747, -1.2908688014555778, -0.07430219441321073, 0.9190462395419852, -2.4426486886595873],
            [-0.8591850361397286, -0.7937152154817659, 1.0043379681934077, -1.2549281109644135, -0.2420693588632976, 1.0987976288974277, -2.2748575557596897],
            [-1.074901084436147, -0.7217379607001941, 0.9689485520478893, -1.2549161267395081, -0.24205737463839183, 1.0988096131223335, -2.274869539984595]
            ]    
        if app_msg==sign:
            if use_movepse==1: # 使用自动走点功能
                def check_exit():
                    nonlocal running
                    input("按 Enter 键停止...\n")  # 阻塞，直到用户按回车
                    running = False  # 修改标志，退出循环
                threading.Thread(target=check_exit, daemon=True).start() # 启动线程监听回车键

                for i in range(len(calib_pose)):
                    success = self.w1_controller.set_current_qpos(
                        name=robot_arm,
                        qpos=calib_pose[i]
                    )
                    time.sleep(0.5)
                    while running:
                        print("回车键跳出: ", running)
                        W1_deg_pose = self.w1_controller.get_current_pose(name=robot_arm)# 获取臂当前位姿矩阵
                        W1_joint_pose = self.w1_controller.get_current_qpos(name=robot_arm)
                        if self.log_msg["robot_cart_pose"][1]==1: logging.info(self.log_msg["robot_cart_pose"][0].format(robot_arm,W1_deg_pose.tolist())) 
                        if self.log_msg["robot_joint_pose"][1]==1: logging.info(self.log_msg["robot_joint_pose"][0].format(robot_arm,W1_joint_pose.tolist())) 
                        # W1_deg_pose[3], W1_deg_pose[5] = W1_deg_pose[5], W1_deg_pose[3]
                        send_msg = "p," + ",".join([f"{num:.3f}" for num in W1_deg_pose.tolist()])
                        if self.log_msg["send_msg"][1]==1: logging.info(self.log_msg["send_msg"][0].format(robot_arm,send_msg)) 
                        self.app.osend(send_msg)
                        # self.app.orecv(send_msg,calib=1)
                        time.sleep(1)
                    if not running:
                        print("回车键跳出: ", running)
                        running = True
                        threading.Thread(target=check_exit, daemon=True).start() # 启动线程监听回车键
                        continue
                        
            elif use_movepse==0: # 不使用自动走点功能
                while True:
                    W1_deg_pose = self.w1_controller.get_current_pose(name=robot_arm)# 获取臂当前位姿矩阵
                    W1_joint_pose = self.w1_controller.get_current_qpos(name=robot_arm)
                    if self.log_msg["robot_cart_pose"][1]==1: logging.info(self.log_msg["robot_cart_pose"][0].format(robot_arm,W1_deg_pose.tolist())) 
                    if self.log_msg["robot_joint_pose"][1]==1: logging.info(self.log_msg["robot_joint_pose"][0].format(robot_arm,W1_joint_pose.tolist())) 
                    # W1_deg_pose[3], W1_deg_pose[5] = W1_deg_pose[5], W1_deg_pose[3]
                    send_msg = "p," + ",".join([f"{num:.3f}" for num in W1_deg_pose.tolist()])
                    if self.log_msg["send_msg"][1]==1: logging.info(self.log_msg["send_msg"][0].format(robot_arm,send_msg)) 
                    # from IPython import embed; embed()
                    self.app.send(send_msg) 
            else:
                print("use_movepse要等于0或1")
        else:
            if self.log_msg["calibration_error"][1]==1: logging.info(self.log_msg["calibration_error"][0].format(app_msg,sign)) 

    def get_current_pose(self,robot):
        """
        获取机械臂当前坐标
        注意是xyz坐标系
        
        参数:
            robot: r / l
            
        返回:
            W1_deg_pose,W1_joint_pose
        """
        if robot == "r": name = 'right_arm' 
        elif robot == "l": name = 'left_arm'
        W1_deg_pose = self.w1_controller.get_current_pose(name)# 获取臂当前位姿矩阵
        W1_joint_pose = self.w1_controller.get_current_qpos(name)
        W1_deg_pose = [round(num, 3) for num in W1_deg_pose]
        W1_joint_pose = [round(num, 3) for num in W1_joint_pose]
        if self.log_msg["robot_cart_pose"][1]==1: logging.info(self.log_msg["robot_cart_pose"][0].format(name,W1_deg_pose)) 
        if self.log_msg["robot_joint_pose"][1]==1: logging.info(self.log_msg["robot_joint_pose"][0].format(name,W1_joint_pose)) 
        return W1_deg_pose,W1_joint_pose
  
class Function:

    def __init__(self, init, log_msg,robot,app):  
        self.init = init  
        self.log_msg = log_msg
        self.loop = None 
        self.w1_instruction = W1_Instruction(log_msg,app)
        self.robot=robot
        self.app = app


    def plane_fence(self,pose_tuple):
        '''
        平面围栏功能，判断机械臂是否在平面围栏内。
        对于每个手臂的每个坐标，先转换至平面坐标系，
        根据 z 值限幅后再转换回原始坐标系，
        最终返回经过 z 限幅调整后的坐标数据元组（字典）。
        '''
        from IPython import embed; embed()
        # 设置用户坐标系 --- 右臂
        ro_pos = [336.075, 418.559, 230.13]  # 原点
        rx_pos = [333.064, 418.572, 85.135]   # X 轴方向
        rxy_pos = [296.07, 513.566, 135.14]    # XY 平面内其他点
        self.right_T_matrix = self.cal_userframe_xpos(ro_pos, rx_pos, rxy_pos)
        # 设置用户坐标系 --- 左臂
        lo_pos = [-317.646, 409.471, 295.444]  # 原点
        lx_pos = [-322.595, 406.478, 173.461]    # X 轴方向
        lxy_pos = [-290.381, 488.574, 211.744]   # XY 平面内其他点
        self.left_T_matrix = self.cal_userframe_xpos(lo_pos, lx_pos, lxy_pos)

        # 定义 z 限制
        l_z_down_limit = 0.5   # 左臂，平面z值最小值
        r_z_down_limit = -0.5  # 右臂，平面z值最大值

        processed_pose = {}  # 存放处理后的结果
        # 针对每个臂处理
        for arm in pose_tuple:
            processed_pose[arm] = []
            for pose in pose_tuple[arm]:
                # 根据不同臂选择相应的用户坐标变换
                if arm == "left_arm":
                    plane_pose = self.transform_to_plane(pose, self.left_T_matrix)
                    if plane_pose[2] < l_z_down_limit:
                        if self.log_msg["exit1"][1] == 1:
                            logging.info(self.log_msg["exit1"][0].format(plane_pose, l_z_down_limit,l_z_down_limit))
                        plane_pose[2] = l_z_down_limit
                        new_pose = self.inverse_transform_from_plane(plane_pose, self.left_T_matrix)
                    else:
                        new_pose = pose
                elif arm == "right_arm":
                    plane_pose = self.transform_to_plane(pose, self.right_T_matrix)
                    if plane_pose[2] > r_z_down_limit:
                        if self.log_msg["exit1"][1] == 1:
                            logging.info(self.log_msg["exit1"][0].format(plane_pose, r_z_down_limit,r_z_down_limit))
                        plane_pose[2] = r_z_down_limit
                        new_pose = self.inverse_transform_from_plane(plane_pose, self.right_T_matrix)
                    else:
                        new_pose = pose
                else:
                    new_pose = pose
                processed_pose[arm].append(new_pose)
        return processed_pose


    def save_point(self, sequence_move_tuple):
        """
        保存 sequence_move_tuple 信息到 JSON 文件:
        - "left" 下的 "pose_list" 保存 sequence_move_tuple["left_arm"]
        - "right" 下的 "pose_list" 保存 sequence_move_tuple["right_arm"]
        """
        file_path = "/home/dexforce/Documents/AGILE/move_pose.json"
        
        # 尝试读取已有数据；如果不存在则构造默认结构
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception:
            data = {"left": {"pose_list": [], "mat": []}, "right": {"pose_list": [], "mat": []}}
        
        # 更新左右臂的 pose_list
        data["left"]["pose_list"] = sequence_move_tuple.get("left_arm", [])
        data["right"]["pose_list"] = sequence_move_tuple.get("right_arm", [])
        
        # 写入 JSON 文件
        with open(file_path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=4)
        
        print("Saved point data to", file_path)

    async def Move_posetuning_list(self, pose_tuple, work_name, index=0, arm="all"):
        """
        说明:
            movel可以每次移动一个坐标
            根据输入pose,示教法移动多组坐标。
            机械臂选择,抓取点顺序,抓取点示教及调整均在config的Pose_List中完成。
            robot1pose = [[grabpose],[placepose]],注意可能无placepose,config中设置。
        参数:
            pose_tuple: {"left_arm":robot1pose,"right_arm":robot2pose} 机器人坐标系坐标
            work_name: 任务名
            arm: 臂选择，'all'代表使用config里的arm,left_arm或right_arm则是指定"work_name"固定使用该臂
        """
        def apply_offset(pose, offset, offset_type="base", arm=None):
            """应用偏移量"""
            if not isinstance(pose, dict) or not isinstance(arm, list):
                raise ValueError("pose 应该是字典，arm 应该是列表")

            # 初始化结果字典
            result_pose = {key: [] for key in pose.keys()}

            # 初始化每个手臂的索引计数器
            arm_indices = {"left_arm": 0, "right_arm": 0}

            for current_arm in arm:
                # 获取当前手臂的索引
                arm_index = arm_indices[current_arm]

                # 检查索引是否超出范围
                if current_arm not in pose or arm_index >= len(pose[current_arm]):
                    print(f"跳过无效的 arm 或超出索引范围: {current_arm}, 索引: {arm_index}")
                    continue

                single_pose = pose[current_arm][arm_index].copy()  # 获取当前 arm 的单个 pose

                if offset != [0]:
                    if offset_type == "base":
                        single_pose[2] += offset[2]
                    elif offset_type == "plane":
                        if current_arm == 'right_arm':
                            # 设置用户坐标系
                            o_pos = [336.075, 418.559, 230.13]  # 第一个点 (将作为原点)
                            x_pos = [333.064, 418.572, 85.135]  # 第二个点 (X 轴方向)
                            xy_pos = [296.07, 513.566, 135.14]  # 第三个点 (Y 轴方向)
                            T_matrix = self.cal_userframe_xpos(o_pos, x_pos, xy_pos)  # 设置用户坐标系
                            # 将左右臂坐标转换至平面坐标系
                            plane_pose = self.transform_to_plane(single_pose, T_matrix)
                            plane_pose[2] -= offset[2]
                        else:
                            lo_pos = [-317.646, 409.471, 295.444]  # 第一个点 (将作为原点)
                            lx_pos = [-322.595, 406.478, 173.461]  # 第二个点 (X 轴方向)
                            lxy_pos = [-290.381, 488.574, 211.744]  # 第三个点 (Y 轴方向)
                            T_matrix = self.cal_userframe_xpos(lo_pos, lx_pos, lxy_pos)  # 设置用户坐标系
                            plane_pose = self.transform_to_plane(single_pose, T_matrix)
                            plane_pose[2] += offset[2]
                        single_pose = self.inverse_transform_from_plane(plane_pose, T_matrix)
                    elif offset_type == "grab":
                        single_pose = self.grabpose_xyz_offset(
                            single_pose, {"x": offset[0], "y": offset[1], "z": offset[2]}
                        )

                # 将处理后的 pose 添加到结果中
                result_pose[current_arm].append(
                    [round(num, 3 - len(str(int(num)))) if num != 0 else 0 for num in single_pose]
                )

                # 更新当前手臂的索引
                arm_indices[current_arm] += 1

            return result_pose
        
        async def execute_move(sequence_move_tuple,sequence_move_cfg_tuple,gripper_list,task_type="MoveL"):


            """执行序列运动任务"""
            # if self.log_msg[f"{task_type} success"][1] == 1:
            #     logging.debug(self.log_msg[f"{task_type} success"][0].format(robot_cfg[4] if arm == 'all' else arm, pose))

            if task_type == "MoveL" and self.robot == "W1":
                # left_fk_pose_list, right_fk_pose_list = self.w1_instruction.get_move_pose({arm: pose})
                
                self.w1_instruction.MoveL1(sequence_move_tuple,gripper_list)

            # elif task_type == "MoveJ" and self.robot == "W1":
            #     pose_t[arm] = pose
            #     return await self.w1_instruction.async_MoveJ1(pose_t)

        # from IPython import embed; embed()
        config_list = self.init["Pose_List"][work_name]
        euler = self.init["ROBOT1"]["Euler"]
        sequence_move_cfg_tuple= {"left_arm": [], "right_arm": []}
        sequence_move_tuple= {"left_arm": [], "right_arm": []}
        id1=0
        pose_tuple1 = self.plane_fence(pose_tuple)
        # from IPython import embed; embed()
        gripper_list = []

        for pose_cfg in config_list:
            # 构建运动坐标序列
            if "tpose" in pose_cfg or "jpose" in pose_cfg:
                if self.log_msg["move_status"][1] == 1:
                    logging.info(self.log_msg["move_status"][0].format(work_name, pose_cfg))
                cfg = config_list[pose_cfg]
                gripper_list.append(cfg["gripper"])
                task_type = "MoveL" if "tpose" in pose_cfg else "MoveJ"
                cfg_use_arm = cfg["use_arm"]
                
                # 计算目标姿态
                if task_type == "MoveL":
                    o_pose = {"left_arm": [], "right_arm": []}  # 初始化为字典，包含两个空列表
                    if "t" in cfg and isinstance(cfg["t"], str): # 字符串，用basic_pose的固定坐标
                        pose_name = cfg["t"]
                        if cfg_use_arm == "same" and not pose_name.startswith(("l_", "r_")):
                            pose_name = f"{'l_' if arm == 'left_arm' else 'r_'}{pose_name}"
                        elif cfg_use_arm == "dif" and not pose_name.startswith(("l_", "r_")):
                            pose_name = f"{'r_' if arm == 'left_arm' else 'l_'}{pose_name}"
                        elif cfg_use_arm == "all" and not pose_name.startswith(("l_", "r_")) :
                            # pose_name = f"{'l_' if cfg == 'left_arm' else 'r_'}{pose_name}"
                            pose_name_l = f"{'l_'}{pose_name}"
                            pose_name_r = f"{'r_'}{pose_name}"
                            o_pose['left_arm'] = [self.init["Pose_List"]["basic_pose"][pose_name_l]]
                            o_pose['right_arm'] = [self.init["Pose_List"]["basic_pose"][pose_name_r]]

                    elif "t" in cfg and len(cfg["t"])==6 and not isinstance(cfg["t"], str): # 有使用示教
                        for i in range(len(arm)):
                            o_pose[arm[i]].append(self.posetuning_calculate(
                                pose_tuple[arm[i]][i],
                                config_list["apose"][arm[0]],
                                cfg["t"],
                                euler
                            ))
                    else: # 没使用示教
                        arm_indices = {"left_arm": 0, "right_arm": 0}

                        for current_arm in arm:
                            arm_index = arm_indices[current_arm]
                            pose = pose_tuple[current_arm][arm_index]
                            o_pose[current_arm].append(pose)
                            arm_indices['left_arm'] += 1
                            arm_indices['right_arm'] += 1
                    if "b" in cfg:
                        o_pose = apply_offset(o_pose, cfg["b"], offset_type="plane",arm=arm)
                    if "g" in cfg:
                        o_pose = apply_offset(o_pose, cfg["g"], offset_type="grab", arm=arm)
                    o_pose1 = deepcopy(o_pose)
                    # for key in o_pose:
                    #     o_pose[key] = [[id1] + cfg["gripper"] + (item if isinstance(item, list) else [item]) + ['movel'] for item in o_pose[key]]

                    # for key in sequence_move_cfg_tuple:
                    #     sequence_move_cfg_tuple[key].extend(o_pose[key])

                    for key in o_pose1:
                        o_pose1[key] = [item for item in o_pose1[key]]
                    for key in sequence_move_tuple:
                        sequence_move_tuple[key].extend(o_pose1[key])
                    id1 +=1
                else:  # MoveJ
                    o_pose = {"left_arm": [], "right_arm": []}  # 初始化结果字典

                    for current_arm in arm:
                        pose_name = cfg["t"]
                        if isinstance(pose_name, str):  # 检查 pose_name 是否为字符串列表
                            if cfg["use_arm"] == "same" and not pose_name.startswith(("l_", "r_")):
                                pose_name = f"{'l_' if current_arm == 'left_arm' else 'r_'}{pose_name}"
                            elif cfg["use_arm"] == "dif" and not pose_name.startswith(("l_", "r_")):
                                pose_name = f"{'r_' if current_arm == 'left_arm' else 'l_'}{pose_name}"
                            elif cfg["use_arm"] == "all" and not pose_name.startswith(("l_", "r_")):
                                pose_name = f"{'l_' if current_arm == 'left_arm' else 'r_'}{pose_name}"
                            pose_value = self.init["Pose_List"]["basic_pose"].get(pose_name)
                        else:
                            pose_value = config_list[pose_cfg][0]
                        o_pose[current_arm].append(pose_value)


                    o_pose1 = deepcopy(o_pose)
                    # for key in o_pose:
                    #     o_pose[key] = [[id1] + cfg["gripper"] + item + ['movej'] for item in o_pose[key]]
                    # for key in sequence_move_cfg_tuple:
                    #     sequence_move_cfg_tuple[key].extend(o_pose[key])

                    for key in o_pose1:
                        o_pose1[key] = [item for item in o_pose1[key]]
                    for key in sequence_move_tuple:
                        sequence_move_tuple[key].extend(o_pose1[key])
                    id1 +=1

        # 循环打印信息
        # print("left_arm")
        # for i in range(len(sequence_move_cfg_tuple["left_arm"])):
        #     print(sequence_move_cfg_tuple["left_arm"][i])
        # print("right_arm")
        # for i in range(len(sequence_move_cfg_tuple["left_arm"])):
        #     print(sequence_move_cfg_tuple["right_arm"][i])   

        print("left_arm")
        for i in range(len(sequence_move_tuple["left_arm"])):
            print(sequence_move_tuple["left_arm"][i])
        print("right_arm")
        for i in range(len(sequence_move_tuple["left_arm"])):
            print(sequence_move_tuple["right_arm"][i])   


        # from IPython import embed; embed()
        self.save_point(sequence_move_tuple)
        # if self.log_msg["sequence_move"][1] == 1:logging.info(self.log_msg["sequence_move"][0].format(sequence_move_tuple["left_arm"],sequence_move_tuple["right_arm"]))
        # 执行任务
        success = await execute_move(sequence_move_tuple,sequence_move_cfg_tuple,gripper_list)
        # # 处理夹爪状态
        # time.sleep(robot_cfg[1])
        # self.w1_instruction.gripper(arm, robot_cfg[2])
        # time.sleep(robot_cfg[3])

        # 发送命令到 app
        if "app" in pose_cfg:
            command = config_list[pose_cfg][4][0]
            self.app.osend(command)


    def cal_userframe_xpos(self,o_pos: list, x_pos: list, xy_pos: list):
        """Calculate the User Frame transformation matrix.
        Args: 
            o_pos (list): Origin position of the User Frame [x, y, z].
            x_pos (list): A point on X-axis of the User Frame [x, y, z].
            xy_pos (list): A point on the XY plane of the User Frame [x, y, z].

        Returns:
            np.ndarray: 4x4 homogeneous transformation matrix representing the User Frame
        """
        # Convert input positions to numpy arrays

        o_pos = np.array(o_pos)
        x_pos = np.array(x_pos)
        xy_pos = np.array(xy_pos)
        # Calculate the X-axis direction vector
        x_axis = x_pos - o_pos
        x_axis /= np.linalg.norm(x_axis)
        # Calculate the vector in the XY plane
        xy_vector = xy_pos - o_pos
        # Calculate the Z-axis direction vector (cross product of X and XY vectors)
        z_axis = np.cross(x_axis, xy_vector)
        z_axis /= np.linalg.norm(z_axis)
        # Calculate the Y-axis direction vector (cross product of Z and X vectors)
        y_axis = np.cross(z_axis, x_axis)
        # Construct the User Frame transformation matrix
        userframe_xpos = np.eye(4)
        userframe_xpos[:3, 0] = x_axis  # X-axis
        userframe_xpos[:3, 1] = y_axis  # Y-axis
        userframe_xpos[:3, 2] = z_axis  # Z-axis
        userframe_xpos[:3, 3] = o_pos   # Origin position
        return userframe_xpos

    def transform_to_plane(self,pose, T):
        """
        将坐标系转换为平面坐标系
        参数：
            pose: 需要转换的基坐标系坐标，格式为 [x, y, z, roll, pitch, yaw]
            T: 变换矩阵，格式为 4x4 的 numpy 数组
        返回：
            pose: 转换后的平面坐标系坐标，格式为 [x, y, z, roll, pitch, yaw]
        """
        from scipy.spatial.transform import Rotation as R
        from scipy.linalg import pinv
        position = np.array(pose[:3])
        euler = pose[3:]
        # 计算转换矩阵
        pose_T = np.eye(4)
        pose_T[:3, :3] = R.from_euler('ZYX', euler, degrees=True).as_matrix()
        pose_T[:3, 3] = position
        new_T = pinv(T) @ pose_T
        new_pose = np.concatenate([new_T[:3, 3], R.from_matrix(new_T[:3, :3]).as_euler('xyz', degrees=False)])
        new_pose_list = np.round(new_pose, 2).tolist()
        return new_pose_list

    def inverse_transform_from_plane(self,plane_pose, T):
        """
        将平面坐标转换回原始坐标系
        参数：
            plane_pose: 平面坐标，格式为 [x, y, z, roll, pitch, yaw]
                        其中平面旋转角采用 'xyz' 顺序，单位为弧度
            T: 变换矩阵，格式为 4x4 的 numpy 数组
        返回：
            原始坐标系坐标，格式为 [x, y, z, roll, pitch, yaw]
            其中角度采用 'ZYX' 顺序，单位为°（两位小数）
        """
        from scipy.spatial.transform import Rotation as R
        import numpy as np
        # 由平面坐标构造变换矩阵（注意平面角度用 'xyz'，弧度）
        plane_position = np.array(plane_pose[:3])
        plane_euler = plane_pose[3:]
        plane_T = np.eye(4)
        plane_T[:3, :3] = R.from_euler('xyz', plane_euler, degrees=False).as_matrix()
        plane_T[:3, 3] = plane_position
        # 原始姿态的变换矩阵：pose_T = T @ plane_T
        pose_T = T @ plane_T
        # 提取原始平移和旋转。原始旋转由 transform_to_plane 使用 'ZYX'（角度制）构造
        original_position = pose_T[:3, 3]
        original_euler = R.from_matrix(pose_T[:3, :3]).as_euler('ZYX', degrees=True)
        original_pose = np.concatenate([original_position, original_euler])
        original_pose_list = np.round(original_pose, 2).tolist()
        return original_pose_list

    def grabpose_xyz_offset( self,pose, offset):
        """
        沿着抓取点自身坐标系的方向进行偏移
        
        参数:
            coord: 抓取点坐标和方向 [x, y, z, a, b, c] (单位: mm, 度)
            offset: 偏移量字典，如 {"x": 10, "y": 5, "z": 15} (单位: mm)
            euler_order: 欧拉角顺序，支持 "ZYX"（默认）或 "xyz"
            
        返回:
            偏移后的新坐标 [x', y', z', a, b, c]
        """
        x, y, z, a, b, c = pose
        
        # 如果RX和RY均为0，则直接处理Z偏移（无需旋转）
        if a == 0 and b == 0:
            new_pose = [
                x + offset.get("x", 0),
                y + offset.get("y", 0),
                z + offset.get("z", 0),
                a,
                b,
                c
            ]
            return new_pose
        
        # 否则计算旋转矩阵（xyz顺序：R = Rx(a) @ Ry(b) @ Rz(c)）
        rx = radians(a)
        ry = radians(b)
        rz = radians(c)
        
        Rx = np.array([
            [1, 0, 0],
            [0, cos(rx), -sin(rx)],
            [0, sin(rx), cos(rx)]
        ])
        
        Ry = np.array([
            [cos(ry), 0, sin(ry)],
            [0, 1, 0],
            [-sin(ry), 0, cos(ry)]
        ])
        
        Rz = np.array([
            [cos(rz), -sin(rz), 0],
            [sin(rz), cos(rz), 0],
            [0, 0, 1]
        ])
        
        R = np.dot(Rx, np.dot(Ry, Rz))
        
        # 局部偏移向量
        offset_vec = np.array([
            offset.get("x", 0),
            offset.get("y", 0),
            offset.get("z", 0)
        ])
        
        # 将局部偏移转换到世界坐标系
        world_offset = np.dot(R, offset_vec)
        
        # 返回新位姿（角度不变）
        new_pose = [
            x + world_offset[0],
            y + world_offset[1],
            z + world_offset[2],
            a,
            b,
            c
        ]
        
        return new_pose
        
    def move_fix_step(self,robot,step):
        """
        沿着基座标系的方向进行偏移
        
        参数:
            robot: r / l
            step: [dx,dy,dz] 单位m
        """
        if robot == "r": name = 'right_arm' 
        elif robot == "l": name = 'left_arm'
        self.w1_controller.translate_end_effector(name,deltaX=step[0], deltaY=step[1], deltaZ=step[2])
        self.w1_controller.translate_end_effector(name,deltaX=step[0], deltaY=step[1], deltaZ=step[2])

    def Use_arm(self,pose_tuple,left_arm_range,right_arm_range,axis,recv_num=2):
        """
        说明:
            使用那个手臂抓取,x为左右移动

        参数:
            pose_tuple: {"left_arm":robot1pose,"right_arm":robot2pose} 机器人坐标系坐标
            max_left_arm : 左手axis轴范围
            max_right_arm : 右手axis轴范围
            axis: 轴名称,0,1,2分别对应x,y,z轴
        """

        # if left_arm_range[0] < pose_tuple["left_arm"][0][axis] < left_arm_range[1]:
        #     use_arm = "left_arm"
        # if right_arm_range[0] < pose_tuple["right_arm"][0][axis] < right_arm_range[1]:  
        #     use_arm = "right_arm"
        # from IPython import embed; embed()

        # W1左右手的坐标系在各臂的原点，坐标可以根据X轴值的大小来判断
        use_arm=[0]*recv_num
        for i in range(recv_num):
            if pose_tuple["left_arm"][i][2] < pose_tuple["right_arm"][i][2]:
                use_arm[i] = "right_arm"
            else: use_arm[i] = "left_arm"
            logging.info(self.log_msg["use_arm"][0].format(use_arm))
        return use_arm

    def pose_deal(self, pose_tuple, work_name, index):
        """
        优化后的抓取坐标处理函数：
        1. 统一所有坐标的第四第五位元素为0
        2. 根据不同任务和索引设置最优抓放rz角度
        """
        def adjust_angle(a, b, toy_type):
            """
            通用角度调整函数
            参数:
                a: 初始角度[-180,180]
                b: 初始b值
                toy_type: 积木类型决定调整模式
            返回:
                调整后的(a, b)
            """
            # 定义各模式参数
            mode_params = {
                "12": {
                    "a_adjustments": [0, 90, -90, 180, -180],
                    "b_values": [0, 90, -90, 180, -180],
                    "sync_rotation": False
                },
                "3": {
                    "a_adjustments": [0, 180, -180],
                    "b_values": [90, -90],
                    "sync_rotation": False # 不需要保持抓放旋转一致
                },
                "5": {
                    "a_adjustments": [0, 180, -180],
                    "b_values": [-180, 180, 0],
                    "sync_rotation": False
                },
                "4": {
                    "a_adjustments": [0, 180, -180],
                    "b_values": [90, -90],
                    "sync_rotation": True # 需要保持抓放旋转一致
                },
                "6": {
                    "a_adjustments": [0, 180, -180],
                    "b_values": [-120, 60],
                    "sync_rotation": True
                },
                "7": {
                    "a_adjustments": [0, 180, -180],
                    "b_values": [120,-60],
                    "sync_rotation": True
                },
                "8": {
                    "a_adjustments": [0, 180, -180],
                    "b_values": [-180, 180,90,-90, 0],
                    "sync_rotation": False
                }
            }

            params = mode_params.get(toy_type)
            if not params:
                raise ValueError(f"无效的toy_type: {toy_type}")

            def normalize_angle(angle):
                """角度归一化到[-180,180]"""
                return (angle + 180) % 360 - 180

            if params["sync_rotation"]:
                # 同步旋转模式
                best_a, best_b = a, b
                min_distance = float('inf')

                for delta in params["a_adjustments"]:
                    new_a = normalize_angle(a + delta)
                    new_b = normalize_angle((b if b is not None else 0) + delta)
                    
                    if new_b not in params["b_values"]:
                        continue
                    
                    distance = abs(new_a)
                    if distance < min_distance or \
                    (distance == min_distance and abs(new_b - new_a) < abs(best_b - best_a)):
                        min_distance = distance
                        best_a, best_b = new_a, new_b
                
                return best_a, best_b
            else:
                # 非同步旋转模式
                adjusted_a = min(
                    [normalize_angle(a + delta) for delta in params["a_adjustments"]],
                    key=lambda x: abs(x)
                )
                adjusted_b = min(
                    params["b_values"],
                    key=lambda x: abs(x - adjusted_a)
                )
                return adjusted_a, adjusted_b


        def adjust_angle1(a, b):
            if abs(a - b) > 180:
                if a < b:
                    a = a - 180
                    b = b - 180
                else:
                    a = a + 180
                    b = b + 180
                # 将角度调整到 [-180, 180]
                a = round((a + 180) % 360 - 180,2)
                b = round((b + 180) % 360 - 180,2)
            return a, b
        

        # 1. 统一设置第四第五位元素为0
        for arm in pose_tuple:
            for pose in pose_tuple[arm]:
                pose[3] = pose[4] = 0

        # 2. 根据任务和索引设置最优角度
        # 定义任务索引与调整类型的映射
        index_mapping = {
            0: "12", 
            1: "12",
            2: "3",
            3: "4",
            4: "5",
            5: "6",
            6: "7",
            7: "8", 
            8: "8",
            9: "8",
            10: "8",
            11: "8"
        }
        
        if index in index_mapping:
            toy_type = index_mapping[index]
            for arm in pose_tuple:
                a, b = adjust_angle(
                    pose_tuple[arm][0][5],
                    pose_tuple[arm][1][5],
                    toy_type
                )
                a= round(a,2)
                b= round(b,2)
                if self.log_msg["pose_deal"][1]==1: logging.info(self.log_msg["pose_deal"][0].format(arm,a,b))
                if abs(a-b) > 180 and a<b:
                    a,b = adjust_angle1(a,b)
                    if self.log_msg["pose_deal2"][1]==1: logging.info(self.log_msg["pose_deal2"][0].format(arm,a,b))
                pose_tuple[arm][0][5] = a
                pose_tuple[arm][1][5] = b
        # if a or b ==0:
        #     from IPython import embed; embed()
        return pose_tuple

    def posetuning_calculate(self, robot_pose, shotpose, grabpose, euler):  
        """
        计算示教坐标
        参数:
            robot_pose: 当前坐标
            shotpose: 示教坐标
            grabpose: 示教的抓取坐标
            euler: 欧拉角顺序
        """
        def poseConvert(pose:list, rotation:str):
            rot_matrix = Rotation.from_euler(rotation, [pose[3],pose[4],pose[5]], degrees=True).as_matrix()
            pose_matrix = np.identity(4)
            pose_matrix[:3,:3] = rot_matrix
            pose_matrix[:3,3] = [pose[0],pose[1],pose[2]]
            return np.mat(pose_matrix)
        if self.log_msg["teach_offset"][1]==1: logging.info(self.log_msg["teach_offset"][0].format(robot_pose,shotpose,grabpose))
        if shotpose[5]=="n":
            return grabpose
        else:
            app_matrix = poseConvert(shotpose, euler)  
            teach_matrix = poseConvert(grabpose, euler)  
            matrix = np.linalg.inv(app_matrix) @ teach_matrix  
            CAM_BD = poseConvert(robot_pose, euler)  
            PUT3 = CAM_BD @ matrix  
            rxyz = Rotation.from_matrix(PUT3[:3, :3]).as_euler(euler, degrees=True)  
            rxyz = [round(num, 2) for num in rxyz]  
            x = round(PUT3[0, 3], 2)  
            y = round(PUT3[1, 3], 2)  
            z = round(PUT3[2, 3], 2)  
            return [x, y, z] + rxyz  
    