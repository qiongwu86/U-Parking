#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
infocom，添加大模型，注册用的服务 服务端 ，用于返回规划好的路径, 并创建订阅者
"""

import numpy as np
import math
import rclpy                      # ROS2 Python接口库
from rclpy.node   import Node     # ROS2 节点类
from std_msgs.msg import String   # ROS2标准定义的String消息

from autoparking_interface.srv import RegisterCar
from geometry_msgs.msg import Pose2D
from nav_2d_msgs.msg import Path2D
from autoparking_interface.msg import ParkingPose
from std_msgs.msg import UInt16MultiArray
from .ParkingLot import *
from .parkpath_scout import *
# from Vehicle import *
# from parkpath_scout import *
from .Astar_my import *
from .paths_overlay import *
from datetime import datetime
import ament_index_python 
import os
import csv
from .LLMA.my_llma import *

class ServerNode(Node):

    def __init__(self, name, ox, oy):
        super().__init__(name)                             # ROS2节点父类初始化
        self.get_logger().info(f'The server has been started!') 
        # 1.停车场信息初始化
        self.parking_lot = ParkingLot(59)
        # 2.随机占用车辆
        # self.occupied_id_list = self.parking_lot.random_occupy()
        self.curren_num_car = 0                             # 统计车辆数
        self.parking_car = {}                               # 定义一个空字典来存储车辆对象 
        self.ox = ox
        self.oy = oy
        self.grid_size = 6.0                                # 小图的像素尺度下
        self.robot_radius = 30.0                            # 小图的像素尺度下
        self.subs = {}                                      # 以车名的形式存储相应的订阅者
        self.a_star = AStarPlanner2(self.ox, self.oy, self.grid_size, self.robot_radius) # 小图的像素尺度下 路径规划时都要转化为小图的像素尺度
        self.pixel_to_meter_m =0.0564

        self.path = self.test_show_path()                        # 测试画路径用 默认起始点：停车场入口，终点车位坐标：22，后续该车位将会被占用，不能继续选择

        self.srv = self.create_service(RegisterCar, 'RegisterCar', self.adder_callback)  # 创建服务器对象（接口类型、服务名、服务器回调函数）

        # 增加绘制叠加路径用
        self.pixel_to_meter_m=0.0564
        # 读取原始图像并转换为BGRA（添加Alpha通道）
        image = cv2.imread("/home/www/auto_parking/server_ws/src/server_python/server_python/ParkingMap_Line.png")
        image = cv2.cvtColor(image, cv2.COLOR_BGR2BGRA)  # 转换到BGRA格式
        alpha = 0.5  # 全局透明度系数（0=全透明, 1=不透明）
        self.paths_overlay = PathsOverlay(self.pixel_to_meter_m,image,alpha)

        self.info_pub_ = self.create_publisher(UInt16MultiArray, "/lot_info", 10)   # 创建发布者对象（消息类型、话题名、队列长度）
        self.info_pub_timer_ = self.create_timer(1.0, self.info_pub_callback)            # 创建一个定时器（单位为秒的周期，定时执行的回调函数）

    def info_pub_callback(self):
        msg = UInt16MultiArray()
        freelot_list = self.parking_lot.check_occupy()          # 当前空余车位id
        msg.data = freelot_list
        # self.get_logger().info(f'当前空余车位id{msg.data}') 
        self.info_pub_.publish(msg)

    def save_data(self,car_name,command,parking_id,uwb_id,path,park_path):
        # 查找已知功能包的位置 
        package_name = 'server_python'  # 替换为实际存在的功能包名称 
        package_path = ament_index_python.get_package_prefix(package_name) 
        # 假设功能包位于src目录下 
        src_path = os.path.dirname(os.path.dirname(package_path)) 
        print("src目录的完整路径:", src_path)
        # 构建目标目录的完整路径 
        directory = os.path.join(src_path,  'datalog')
        # 创建目录，如果目录不存在 
        os.makedirs(directory,  exist_ok=True)
        # 验证目录是否创建成功 
        if os.path.exists(directory): 
            print(f"'{directory}' 目录创建成功。")
        else:
            print(f"无法创建 '{directory}' 目录。")
        # 保存command
        system_time = datetime.now()
        full_datetime = datetime.combine(
            system_time.date(),
            system_time.time(),
            # tzinfo=rclpy_time.tzinfo
        )
        freelot_list = self.parking_lot.check_occupy()
        formatted_full_datetime = full_datetime.strftime("%Y-%m-%d %H:%M:%S")

        # 保存参考路径
        directory_reference_path = os.path.join(directory,  'reference_path.csv')
        reference_path = np.vstack([path,park_path])
        with open(directory_reference_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['reference_x', 'reference_y'])      # 写入表头
            writer.writerows(reference_path)         # 写入坐标行


        directory_command = os.path.join(directory,  'command.txt')
        # 保存到文件 
        try:
            with open(directory_command,  'a', encoding='utf-8') as file:
                
                file.write(f'当前空余车位id为{freelot_list}')
                file.write('\n')
                file.write(f'{car_name}的uwb_id:{uwb_id},目标车位:{parking_id}')                                      
                file.write('\n')
                file.write(f'{car_name}指令:')                                      
                file.write(command)
                file.write('    --')
                file.write(formatted_full_datetime) 
                file.write('\n')
                file.write('\n')
            print("文件保存成功！")
        except Exception as e:
            print(f"错误: {e}")

    def test_show_path(self):
        pix = 0.0564
        # 1.初始位姿 13.792321995350838,45.77100003051758
        initial_pose = Pose2D()
        initial_pose.x = 13.792321995350838
        initial_pose.y = 45.77100003051758        
        # initial_pose.x = 15.296
        # initial_pose.y = 29.989
        car_name = 'car1'
        uwb_id = 0
        # 随机占用车辆，然后输出空余车位id列表
        self.parking_lot.random_occupy(40)
        self.free_id_list = self.parking_lot.check_occupy()      

        api_key="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"
        model_id="meta/llama-3.3-70b-instruct"
        start_pose = [initial_pose.x, initial_pose.y]
        freelot_id = self.free_id_list

        reply = my_llama3(api_key, model_id, start_pose, freelot_id)
        parsed = parse_llm_output_robust(reply)

        target_slot_id = parsed['target_slot_id']               # 目标车位id
        target_slot_coord = parsed['target_slot_coord']         # 目标车位坐标
        key_path = parsed['key_path']                           # 关键点路径

        sx = start_pose[0]/self.pixel_to_meter_m                     # 像素坐标
        sy = start_pose[1]/self.pixel_to_meter_m
        full_path_x, full_path_y = [], []
        ex_path_x, ex_path_y = [], []
        key_points = key_path[1:]
        key_points[-1][0] -= 1.3
        key_points = [[x / pix, y / pix] for x, y in key_points]
        cur_x, cur_y = sx, sy
        # t0 = time.perf_counter()
        for gx, gy in key_points:
            print("cur_x, cur_y, gx, gy:\n",cur_x, cur_y, gx, gy)
            px, py, explored_x, explored_y = self.a_star.planning(cur_x, cur_y, gx, gy)
            full_path_x += px[:-1]
            full_path_y += py[:-1]
            ex_path_x += explored_x[:-1]
            ex_path_y += explored_y[:-1]
            cur_x, cur_y = gx, gy
        # t1 = time.perf_counter()
        # print(f"A* searching time: {t1 - t0:.6f} s")

        search_path = list(zip(full_path_x, full_path_y))   # 规划好的寻车路径 像素坐标
        explored_path = list(zip(ex_path_x, ex_path_y))    # 搜索路径
        path_list = [[x * pix, y * pix] for x, y in search_path]
        # 平滑
        path = np.array(path_list)                   # 现在是米坐标
        path,_ = path_by_bezier_2(path)
        parkstart_x = path[len(path)-1][0]
        parkstart_y = path[len(path)-1][1]
        if (target_slot_id%20) <= 9:
            park_path = parkpath(parkstart_x, parkstart_y)          # 向上泊车路径
        else:    
            park_path = down(parkstart_x, parkstart_y)          # 泊车路径

        plt_lot(freelot_id, start_pose, target_slot_id, key_points, path, park_path, explored_path)

        # # 绘制
        # plt.plot(self.ox, self.oy, ".k", label='Obstacle', markersize= 10)  # 黑色.       障碍物
        # plt.plot(sx, sy, "xr", markersize=12, label='Starting')  # 开始坐标
        # plt.grid(True)
        # plt.axis('equal')  # 保持栅格的横纵坐标刻度一致
        # x_coords = [point[0]/self.pixel_to_meter_m for point in path]
        # y_coords = [point[1]/self.pixel_to_meter_m for point in path]
        # plt.plot(x_coords, y_coords, 'ob', alpha=0.2, label='Search path')
        # plt.legend()
        # # 坐标轴同比例
        # ax = plt.gca()
        # ax.set_aspect(1)
        # plt.show()

    # 根据关键路径点产生轨迹
    def generate_path(self, points):
        pathx_list = []
        pathy_list = []
        # print('points:',points)
        # 拼接
        for i in range(len(points)-1):
            sxx = points[i][0]
            syy = points[i][1]
            gxx = points[i+1][0]
            gyy = points[i+1][1]
            pathx, pathy, explored_x, explored_y = self.a_star.planning(sxx, syy, gxx, gyy)
            pathx_list = pathx_list+pathx
            pathy_list = pathy_list+pathy
            pathx_list.pop()
            pathy_list.pop()

        # 合并
        path_all = []
        for i in range(len(pathx_list)):
            path_all.append((pathx_list[i], pathy_list[i]))
        return path_all

    def adder_callback(self, request, response):   # 创建回调函数，执行收到请求后对数据的处理
        """
        受到请求数据后,需返回规划好的路径,以及当前车辆在该停车场的id,以便于发布者发布
        """

        self.curren_num_car = self.curren_num_car + 1           # 注册的车辆数目加1
        response.car_id = self.curren_num_car

        # 接收数据
        car_name = request.car_name
        start_time = request.start_time
        initial_pose = request.initial_pose
        uwb_id = request.uwb_id
        parking_id_hope = request.parking_id_hope
        # self.get_logger().info(f'当前注册汽车的名字是{request.car_name}') 
        self.get_logger().info(f'The current name of the registered vehicle is {request.car_name}') 
        
        # 选择车位，并根据初始位姿来规划路径
        if parking_id_hope == 0:                        # 大模型
            # 随机占用车辆，然后输出空余车位id列表
            self.parking_lot.random_occupy(35)
            self.free_id_list = self.parking_lot.check_occupy()      

            api_key="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"
            model_id="meta/llama-3.3-70b-instruct"
            start_pose = [initial_pose.x, initial_pose.y]
            freelot_id = self.free_id_list

            reply = my_llama3(api_key, model_id, start_pose, freelot_id)
            parsed = parse_llm_output_robust(reply)

            target_slot_id = parsed['target_slot_id']               # 目标车位id
            response.parking_id = target_slot_id
            target_slot_coord = parsed['target_slot_coord']         # 目标车位坐标
            key_path = parsed['key_path']                           # 关键点路径

            sx = start_pose[0]/self.pixel_to_meter_m                     # 像素坐标
            sy = start_pose[1]/self.pixel_to_meter_m
            full_path_x, full_path_y = [], []
            key_points = key_path[1:]
            key_points[-1][0] -= 1.4
            key_points = [[x / self.pixel_to_meter_m, y / self.pixel_to_meter_m] for x, y in key_points]
            cur_x, cur_y = sx, sy
            # t0 = time.perf_counter()
            for gx, gy in key_points:
                print("cur_x, cur_y, gx, gy:\n",cur_x, cur_y, gx, gy)
                px, py, explored_x, explored_y = self.a_star.planning(cur_x, cur_y, gx, gy)
                full_path_x += px[:-1]
                full_path_y += py[:-1]
                cur_x, cur_y = gx, gy
            # t1 = time.perf_counter()
            # print(f"A* searching time: {t1 - t0:.6f} s")

            search_path = list(zip(full_path_x, full_path_y))   # 规划好的寻车路径 像素坐标
            path_list = [[x * self.pixel_to_meter_m, y * self.pixel_to_meter_m] for x, y in search_path]
            # 平滑
            path = np.array(path_list)                   # 现在是米坐标
            path,_ = path_by_bezier_2(path)
            path_2d = to_path2d(path)                               # 生成寻车路径
            parkstart_x = path[len(path)-1][0]
            parkstart_y = path[len(path)-1][1]
            if (target_slot_id%20) <= 9:
                park_path = parkpath(parkstart_x, parkstart_y)          # 向上泊车路径
            else:    
                park_path = down(parkstart_x, parkstart_y)          # 泊车路径
            park_path_2d = to_path2d(park_path)                     
            plt_lot(freelot_id, start_pose, target_slot_id, key_points, path, park_path)

            # 生成控制指令还没用
            command = "LLAMA-A"          
            # self.get_logger().info(f'控制指令为：{command}')
            self.get_logger().info(f'The control instruction: {command}')
            response.command = command

            # self.save_data(car_name,command,id,uwb_id)
            self.save_data(car_name,command,target_slot_id,uwb_id,path,park_path)
            
            # 创建车辆对象并添加到停车场列表中
            vehicle = Vehicle(car_name, str(start_time), initial_pose, target_slot_id, uwb_id) 
            self.parking_car[request.car_name] = vehicle             # 每次注册，将车辆记录在字典中

            topic_name = f'/{request.car_name}/parking'             # 设置订阅者的话题名字
            # self.get_logger().info(f'topic_name:{topic_name}') 
            # 创建订阅者
            subscription = self.create_subscription(ParkingPose,topic_name, self.subscriber_callback,10)
            self.subs[request.car_name]=subscription
            ########################################
            # 注销
            # subscription = self.subs.pop(request.car_name)          # 删除指定键并返回值 
            # subscription.destroy()                                  # 取消订阅坐标
            ########################################
            # 填充response
            response.path = path_2d
            response.park_path = park_path_2d

        else:                                           # 普通
            if parking_id_hope == -1:                      # 随机
                id, xy = self.parking_lot.random_choose()           # 获取停车位id与 车位坐标(基于A0的米坐标)
                self.get_logger().info(f'{request.car_name}的期望车位id为随机选择,随机选择空余车位:{id}') 
            else:
                xy = self.parking_lot.select_available_spot(parking_id_hope)
                if xy == 0: # 被占用了
                    id, xy = self.parking_lot.random_choose()           # 随机选择
                    self.get_logger().info(f'{request.car_name}的期望车位id为{parking_id_hope},但是被占用了，更改目标车位至:{id}') 
                elif xy == 1:# 所选车位无效
                    id, xy = self.parking_lot.random_choose()           # 随机选择
                    self.get_logger().info(f'{request.car_name}的期望车位id为{parking_id_hope},但是没有该车位，更改目标车位至:{id}') 
                else:
                    id = parking_id_hope                                # 指定车位
                    self.get_logger().info(f'{request.car_name}的期望车位id为{parking_id_hope},该车位空闲')        
            sx = initial_pose.x                                     # 基于map的真实坐标
            sy = initial_pose.y
            # self.get_logger().info(f'sx,sy:{sx,sy}') 
            sx = sx/self.pixel_to_meter_m                                      # 转换成用于路径规划的mmm像素坐标
            sy = sy/self.pixel_to_meter_m
            gx,gy = gxy_change(id, xy)                              
            response.parking_id = id
            pathx, pathy, explored_x, explored_y = self.a_star.planning(sx, sy, gx, gy)     # 规划路径
            JPs = findJPs(pathx, pathy)                             # 找出对应的岔路点
            points = np.vstack([[sx,sy],JPs,[gx,gy]])               # 合并关键路径点
            points = points.tolist()                                # 转成列表 mmm的像素坐标
            path = self.generate_path(points)                       # 根据岔路点生成路径，没有平滑后的
            path = path2map(path)                                   # 变成基于map的米坐标
            # 平滑
            path = np.array(path)
            # path = path_by_bezier(path)
            path,_ = path_by_bezier_2(path)
            path_2d = to_path2d(path)                               # 生成寻车路径
            parkstart_x = path[len(path)-1][0]
            parkstart_y = path[len(path)-1][1]
            if (id%20) <= 9:
                park_path = parkpath(parkstart_x, parkstart_y)          # 向上泊车路径
            else:    
                park_path = down(parkstart_x, parkstart_y)          # 泊车路径
            # park_path = parkpath(parkstart_x, parkstart_y)          # 泊车路径
            park_path_2d = to_path2d(park_path)

            # 生成控制指令还没用
            command = generate_path_instructions(points)            
            self.get_logger().info(f'控制指令为：{command}')
            response.command = command

            # self.save_data(car_name,command,id,uwb_id)
            self.save_data(car_name,command,id,uwb_id,path,park_path)
            
            # 创建车辆对象并添加到停车场列表中
            vehicle = Vehicle(car_name, str(start_time), initial_pose, id, uwb_id) 
            self.parking_car[request.car_name] = vehicle             # 每次注册，将车辆记录在字典中

            topic_name = f'/{request.car_name}/parking'             # 设置订阅者的话题名字
            # self.get_logger().info(f'topic_name:{topic_name}') 
            # 创建订阅者
            subscription = self.create_subscription(ParkingPose,topic_name, self.subscriber_callback,10)
            self.subs[request.car_name]=subscription
            ########################################
            # 注销
            # subscription = self.subs.pop(request.car_name)          # 删除指定键并返回值 
            # subscription.destroy()                                  # 取消订阅坐标
            ########################################
            # 填充response
            response.path = path_2d
            response.park_path = park_path_2d
            # self.get_logger().info(f'路径点为{response.path.poses}')   # 输出日志信息

            # path_2d = np.array(path_2d)
            # park_path_2d = np.array(park_path_2d)
            cv_paths = np.vstack([path,park_path])/self.pixel_to_meter_m       # TODO:绘制测试重叠路径
            self.paths_overlay.cvshow(cv_paths)

        return response                          # 反馈应答信息
        
    # 订阅车俩的位置信息
    def subscriber_callback(self,msg):
        car_name = msg.car_name
        self.parking_car[car_name].info_i += 1
        self.parking_car[car_name].update(msg.current_pose.x, msg.current_pose.y)       # 更新对应车辆的二维位姿
        if self.parking_car[car_name].info_i == 20:                                      # 每二十次打印一次
            self.get_logger().info(f"vehicle {car_name}\'s pose is {msg.current_pose.x,msg.current_pose.y}") 
            self.parking_car[car_name].info_i = 0
            
def main(args=None):                               # ROS2节点主入口main函数
    ox, oy = map_config()                          # 小图的像素尺度

    rclpy.init(args=args)                          # ROS2 Python接口初始化
    node = ServerNode("parking_server", ox, oy)  # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                               # 循环等待ROS2退出
    node.destroy_node()                            # 销毁节点对象
    rclpy.shutdown()                               # 关闭ROS2 Python接口

if __name__ == '__main__':
    main() 
