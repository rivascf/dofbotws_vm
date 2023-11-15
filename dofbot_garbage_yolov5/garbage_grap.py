#!/usr/bin/env python
# coding: utf-8
import Arm_Lib
from time import sleep


class garbage_grap_move:
    def __init__(self):
        # set move status
        # 设置移动状态
        self.move_status = True
        self.arm = Arm_Lib.Arm_Device()
        # Clamping jaw tightening angle
        # 夹爪加紧角度
        self.grap_joint = 142

    def move(self, joints, joints_down):
        '''
        移动过程 moving process
        :param joints: 移动到物体位置的各关节角度    The angle of each joint moving to the object position
        :param joints_down: 机械臂抬起各关节角度      The mechanical arm lifts each joint angle
        :param color_angle: 移动到对应垃圾桶的角度  Move to the angle corresponding to the trash can
        '''
        joints_0 = [90, 90, 15, 20, 90, 30]
        #joints = [90, 40, 30, 67, 265, 30]
        joints_uu = [90, 80, 50, 50, 265, self.grap_joint]
        # put up 抬起
        joints_up = [joints_down[0], 80, 50, 50, 265, 30]
        # Move over the object's position 移动至物体位置上方
        self.arm.Arm_serial_servo_write6_array(joints_uu, 1000)
        sleep(1)
 
        # Release the jaws 松开夹爪
        self.arm.Arm_serial_servo_write(6, 30, 500)
        sleep(0.5)
        # move to object position 移动至物体位置
        self.arm.Arm_serial_servo_write6_array(joints, 500)
        sleep(0.5)
        # gripping, clamping jaws 进行抓取,夹紧夹爪
        self.arm.Arm_serial_servo_write(6, self.grap_joint, 500)
        sleep(0.5)
        # set up 架起
        self.arm.Arm_serial_servo_write6_array(joints_uu, 1000)
        sleep(1)
        # Lift to the top of the corresponding position 抬起至对应位置上方
        self.arm.Arm_serial_servo_write(1, joints_down[0], 500)
        sleep(0.5)
        # Lift to the corresponding position 抬起至对应位置
        self.arm.Arm_serial_servo_write6_array(joints_down, 1000)
        sleep(1)
        # Release the object, release the gripper释放物体,松开夹爪
        self.arm.Arm_serial_servo_write(6, 30, 500)
        sleep(0.5)
        # put up 抬起
        self.arm.Arm_serial_servo_write6_array(joints_up, 1000)
        sleep(1)
        # move to initial position 移动至初始位置
        #self.arm.Arm_serial_servo_write6_array(joints_0, 1000)
       # sleep(1)

    def arm_run(self, name, joints):
        '''
        Manipulator movement function 机械臂移动函数
        :param name:识别的垃圾名称  Identified spam names
        :param joints: 反解求得的各关节角度 Angle of each joint obtained by inverse solution
        '''
        # Hazardous waste - red
        # 有害垃圾--红色
        if name == "Syringe" or name == "Used_batteries" or name == "Expired_cosmetics" or name == "Expired_tablets" and self.move_status == True:
            # It is set here. You can only run down after this operation
            # 此处设置,需执行完本次操作,才能向下运行
            self.move_status = False
            # print("Hazardous waste")
            # print(joints[0], joints[1], joints[2], joints[3], joints[4])
            # Obtain the target joint angle 获得目标关节角
            joints = [joints[0], joints[1], joints[2], joints[3]+5, 265, 30]
            # Attitude before moving to the target 移动到目标前的姿态
            joints_down = [45, 80, 35, 40, 265, self.grap_joint]
            # joints_down = [45, 50, 20, 60, 265, self.grap_joint]
            self.move(joints, joints_down)
            self.move_status = True
        # Recyclable waste - blue
        # 可回收垃圾--蓝色
        if name == "Zip_top_can" or name == "Newspaper" or name == "Old_school_bag" or name == "Book" and self.move_status == True:
            self.move_status = False
            # print("Recyclable waste")
            # print(joints[0], joints[1], joints[2], joints[3], joints[4])
            joints = [joints[0], joints[1], joints[2], joints[3]+5, 265, 30]
            joints_down = [27, 110, 0, 40, 265, self.grap_joint]
            # joints_down = [27, 75, 0, 50, 265, self.grap_joint]
            self.move(joints, joints_down)
            self.move_status = True
        # Kitchen waste - green
        # 厨余垃圾--绿色
        if name == "Fish_bone" or name == "Watermelon_rind" or name == "Apple_core" or name == "Egg_shell" and self.move_status == True:
            self.move_status = False
            # print("Kitchen waste")
            # print(joints[0], joints[1], joints[2], joints[3], joints[4])
            joints = [joints[0], joints[1], joints[2], joints[3]+5, 265, 30]
            joints_down = [152, 110, 0, 40, 265, self.grap_joint]
            # joints_down = [152, 75, 0, 50, 265, self.grap_joint]
            self.move(joints, joints_down)
            self.move_status = True

        # Other garbage--gray
        # 其他垃圾--灰色
        if name == "Cigarette_butts" or name == "Toilet_paper" or name == "Peach_pit" or name == "Disposable_chopsticks" and self.move_status == True:
            self.move_status = False
            # print("Other garbage")
            # print(joints[0], joints[1], joints[2], joints[3], joints[4])
            joints = [joints[0], joints[1], joints[2], joints[3]+5, 265, 30]
            joints_down = [137, 80, 35, 40, 265, self.grap_joint]
            # joints_down = [137, 50, 20, 60, 265, self.grap_joint]
            self.move(joints, joints_down)
            self.move_status = True
