import time
import numpy as np
from APP import App  
import logging  
from Initialize import Initialize  
from threading import Thread, Event  
import threading
from copy import deepcopy
import numpy as np
from math import cos, sin, radians
from Function import Function 
from rich.logging import RichHandler
import asyncio  
import nest_asyncio



class TASK():  

    def __init__(self, init,log_msg):  
        self.init  = init
        self.log_msg = log_msg

    def perform_sequence(self):
        global ttt
        def execute_task(command, work_name=None, repeat=1):
            """
            执行任务的通用逻辑
            参数:
                command: 发送给 app 的指令
                work_name: 任务名称（可选）
                repeat: 重复执行的次数（默认为 1)
            """
            global ttt
            for i in range(repeat):
                pose_tuple, pose_param_list = app.send_recv2_num(command)
                # from IPython import embed; embed()
                if work_name == "把笔筒放回原位":
                    for left_data, right_data in zip(pose_tuple['left_arm'], pose_tuple['right_arm']):
                        left_data[3:] = [0, 0, 0]  
                        right_data[3:] = [0, 0, 0]  

                if pose_tuple["left_arm"] == [] and pose_tuple["right_arm"] == [] :
                    if self.log_msg["no pose"][1]==1: logging.info(self.log_msg["no pose"][0])
                else:
                    use_arm = function.Use_arm(pose_tuple, self.init["ROBOT1"]["move_limit"], 
                                            self.init["ROBOT2"]["move_limit"], 0)  # 选择使用哪个手臂
                    function.Move_posetuning_list(pose_tuple, work_name, index=ttt,arm=use_arm)
                    ttt+=1
                    if self.log_msg["Runs"][1]==1: logging.info(self.log_msg["Runs"][0].format(ttt)) 

        ttt=0
        while True:
            from IPython import embed; embed()
            # tasks2 = [
            #     ("d,0,0,0,0,0,0,9", "抓笔放笔2")
            # ]
            # for command, work_name in tasks2:
            execute_task("d,0,0,0,0,0,0,10", "把笔筒放回原位")
            execute_task("d,0,0,0,0,0,0,9", "抓笔放笔2")


if __name__ == '__main__':
    #———————————————————————————配置加载初始化——————————————————
    nest_asyncio.apply()  # 允许嵌套事件循环
    np.set_printoptions(7, suppress=True)
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(filename)s - Line: %(lineno)d - %(message)s')
    config_file_path = 'W1_config.json'
    init, log_msg = Initialize.load_config(config_file_path)
    use_robot = "W1" # 选择使用的机器人 W1 或 AGILE
    task = TASK(init,log_msg)
    global ttt
    #———————————————————————————视觉初始化——————————————————————————————#
    app = App(init, log_msg)
    app.connection()
    #———————————————————————————功能函数初始化————————————————————————————#
    function = Function(init, log_msg,use_robot,app)
    #———————————————————————————W1机器人初始化————————————————————————————#
    from Function import W1_Instruction
    w1 = W1_Instruction(log_msg,app)
    w1.initialize()
    # from IPython import embed; embed()
    w1.GoHome()
    time.sleep(3)


    # from scipy.spatial.transform import Rotation as R
    # right_tcp_xpos = np.eye(4)
    # right_tcp_xpos[2, 3] = 0.143
    # w1.w1_controller.set_tcp_xpos(name="right_arm", tcp_xpos=right_tcp_xpos)
    # left_tcp_xpos = np.eye(4)
    # left_tcp_xpos[2, 3] = 0.143
    # left_tcp_xpos[:3, :3] = R.from_rotvec([0.0, 0.0, 180.], degrees=True).as_matrix()
    # w1.w1_controller.set_tcp_xpos(name="left_arm", tcp_xpos=left_tcp_xpos)

    
    # w1.get_current_pose("l")
    # time.sleep(5)
    # w1.gripper("right_arm", 0)
    # app.send_recv("d,0,0,0,0,0,0,5")
    # w1.calibration("right_arm","getRobotPose",use_movepse = 1) # use_movepse = 1时。需要打断点
    #———————————————————————————进入线程作业——————————————————————————————#
    thread2 = Thread(target=task.perform_sequence)
    thread2.start()
    thread2.join()


'''
笔筒抓取点[-296.872, 331.647, 10.947, 138.521, -18.008, 178.044]
[-256.87, 511.649, -89.031, 138.519, -18.008, 178.047]


'''
'''
ssh grabotics@192.168.158.89
watch -n 1 "ethercat slaves"
cd PGC_gripper/server
python3 PGC_server.py

ssh -X grabotics@192.168.158.89
cd b1_control
./run_teaching_pendant.sh
'''


'''
left_arm
[-281.682, 271.109, 236.542, 132.362, -35.3, -167.744]
[-260.0, 411.0, 21.8, 119.0, -25.0, -170.0]
[-270.0, 395.0, 13.1, 119.0, -25.0, -170.0]
[-252.413, 507.188, -97.169, 122.1, -28.266, -174.731]
right_arm
[220.304, 389.767, 296.281, -152.372, 89.507, -134.026]
[220.304, 389.767, 296.281, -152.372, 89.507, -134.026]
[220.304, 389.767, 296.281, -152.372, 89.507, -134.026]
[220.304, 389.767, 296.281, -152.372, 89.507, -134.026]

'''