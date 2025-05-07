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
                repeat: 重复执行的次数（默认为 1）
            """
            global ttt
            for i in range(repeat):
                pose_tuple, pose_param_list = app.send_recv(command)
                use_arm = function.Use_arm(pose_tuple, self.init["ROBOT1"]["move_limit"], 
                                           self.init["ROBOT2"]["move_limit"], 0)  # 选择使用哪个手臂
                asyncio.run(function.Move_posetuning_list(pose_tuple, work_name, index=ttt,arm=use_arm))
                ttt+=1
                if self.log_msg["Runs"][1]==1: logging.info(self.log_msg["Runs"][0].format(ttt)) 
       
        ttt=0
        while True:
            tasks2 = [
                ("d,0,0,0,0,0,0,5", "抓笔放笔")
            ]
            for command, work_name in tasks2:
                execute_task(command, work_name)



if __name__ == '__main__':
    #———————————————————————————配置加载初始化——————————————————
    nest_asyncio.apply()  # 允许嵌套事件循环
    np.set_printoptions(7, suppress=True)
    logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(filename)s - Line: %(lineno)d - %(message)s')
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

    # tcp_xpos = np.eye(4)
    # tcp_xpos[2, 3] = 0.143
    # # tcp_xpos[2, 3] = 0
    # w1.w1_controller.set_tcp_xpos(name="left_arm", tcp_xpos=tcp_xpos)
    # w1.w1_controller.set_tcp_xpos(name="right_arm", tcp_xpos=tcp_xpos)
    # w1.get_current_pose("l")
    # w1.gripper("left_arm", 1)
    # app.send_recv("d,0,0,0,0,0,0,5")
    w1.GoHome()
    # w1.calibration("left_arm","getRobotPose",use_movepse = 1) # use_movepse = 1时。需要打断点
    #———————————————————————————进入线程作业——————————————————————————————#
    thread2 = Thread(target=task.perform_sequence)
    thread2.start()
    thread2.join()
