from collections import deque
import numpy as np
import matplotlib.pyplot  as plt 
import csv
import pandas as pd

import numpy as np
from collections import deque

from my_ekf import *
from moving_average import *
import ctypes
import os
from ukf import *
from ekf import *
import matplotlib.image as mpimg
from mpl_toolkits.axes_grid1.inset_locator import inset_axes
from matplotlib.patches import ConnectionPatch

from PIL import Image 

# dw1000
offset_x = 13.559322
offset_y = 10.153

# # 在泊车系统中的定位偏置 mk8000
# offset_x = 13.559322
# offset_y = 26.271186

"""
绘制各个算法的重叠曲线
"""

# date = "7-9-5"                                 # TODO: 选择数据
date = "5-9-1"                                 # TODO: 选择数据

# 1.读取数据文件：
# uwb_df = pd.read_csv(f'/home/www/auto_parking/auto_parking_ws/better/{date}/uwb_pos.csv')
uwb_df = pd.read_csv(f'/home/www/auto_parking/uwb_pos_with_noise.csv')
# ['uwb_x_noisy', 'uwb_y_noisy']
uwb_x = uwb_df['uwb_x_noisy'].tolist()           # 转成 Python 列表
uwb_y = uwb_df['uwb_y_noisy'].tolist() 
# uwb_x = uwb_df['uwb_x'].tolist()           # 转成 Python 列表
# uwb_y = uwb_df['uwb_y'].tolist() 
uwb_yaw = uwb_df['uwb_yaw'].tolist() 
linear = uwb_df['linear'].tolist() 
angular = uwb_df['angular'].tolist() 
init_x = uwb_x[0]
init_y = uwb_y[0]
init_yaw = uwb_yaw[0]

measurements = np.column_stack([uwb_x,uwb_y,uwb_yaw])
# measurements = np.column_stack([uwb_x,uwb_y])

class UWBMsg(ctypes.Structure):
    _fields_ = [
        ('x', ctypes.c_double),
        ('y', ctypes.c_double),
        ('z', ctypes.c_double),
    ]
UWBMsg = UWBMsg

# 初始化 anchor 与 distance 数组
anchor_array = (UWBMsg * 8)()
# 基站坐标      泊车版
anchor_array[0].x, anchor_array[0].y, anchor_array[0].z = 0.0, 0.0, 1.84
anchor_array[1].x, anchor_array[1].y, anchor_array[1].z = 14.25, 0.0, 1.84
anchor_array[2].x, anchor_array[2].y, anchor_array[2].z = 9.8, 13.73, 1.82

for i in range(3):
    # print(i)
    anchor_array[i].x += offset_x
    anchor_array[i].y += offset_y

folder_to_create = f"/home/www/auto_parking/auto_parking_ws/INFOCOM/LOCATION/{date}"    # 保存的文件路径

def create_folder(folder_path):
    try:
        # 检查路径是否已经存在
        if not os.path.exists(folder_path):
            # 创建文件夹（包括所有必要的父文件夹）
            os.makedirs(folder_path, exist_ok=True)
            print(f"文件夹已成功创建: {folder_path}")
        else:
            print(f"文件夹已存在: {folder_path}")
    except FileExistsError:
        print(f"错误: 指定的路径是一个文件，不是文件夹: {folder_path}")
    except PermissionError:
        print(f"错误: 没有权限创建文件夹: {folder_path}")
    except OSError as e:
        print(f"错误: 无法创建文件夹 {folder_path}. 原因: {e}")
    except Exception as e:
        print(f"错误: 发生了未知错误: {e}")

"""plt_all
def plt_all(date):
    # directory = "/home/www/auto_parking/auto_parking_ws/src/autoparking_python/autoparking_python"
    directory = folder_to_create
    directory_ukf = os.path.join(directory,  'ukf_path.csv')
    directory_ekf = os.path.join(directory,  'ekf_path.csv')                # aekf
    directory_e = os.path.join(directory,  'e_path.csv')                    # 初始ekf
    directory_odom = os.path.join(directory,  'odom_path.csv')

    ekf_df = pd.read_csv(directory_ekf)
    ukf_df = pd.read_csv(directory_ukf)
    e_df = pd.read_csv(directory_e)
    odom_df = pd.read_csv(directory_odom)

    ekf_x = ekf_df['ekf_x'].tolist()           # 转成 Python 列表
    ekf_y = ekf_df['ekf_y'].tolist() 
    ukf_x = ukf_df['ukf_x'].tolist()           # 转成 Python 列表
    ukf_y = ukf_df['ukf_y'].tolist() 
    e_x = e_df['e_x'].tolist()           # 转成 Python 列表
    e_y = e_df['e_y'].tolist() 
    odom_x = odom_df['odom_x'].tolist()           # 转成 Python 列表
    odom_y = odom_df['odom_y'].tolist() 

    ##########################################
    # 叠加地图
    # map_path = '/home/www/auto_parking/auto_parking_ws/better/EI/pdf/ei_map.png'    
    map_path = '/home/www/auto_parking/auto_parking_ws/src/autoparking_python/autoparking_python/Gray Boxed Image_screenshot_11.07.2025.png'   
    
    # map_path = '/home/www/auto_parking/auto_parking_ws/better/EI/pdf/map.png'    
    map_origin = [0, 0]  # 地图左下角对应的真实世界坐标（单位：m）
    resolution = 0.017  # 每个像素对应真实世界多少米
    # 图片大小（像素）
    # ==== 读取地图 ====
    map_img = mpimg.imread(map_path)
    
    # 地图图像 shape
    height, width = map_img.shape[:2]
    #####################################

    x_vals = uwb_x
    y_vals = uwb_y

    num = 20
    add_n = 1.67
    ekf_x_20 = ekf_x[-num:] if len(ekf_x) >=num else ekf_x.copy() 
    ekf_y_20 = ekf_y[-num:] if len(ekf_y) >=num else ekf_y.copy() 
    ekf_y_20 = [x + add_n for x in ekf_y_20]
    ukf_x_20 = ukf_x[-num:] if len(ukf_x) >=num else ukf_x.copy() 
    ukf_y_20 = ukf_y[-num:] if len(ukf_y) >=num else ukf_y.copy() 
    ukf_y_20 = [x + add_n for x in ukf_y_20]
    e_x_20 = e_x[-num:] if len(e_x) >=num else e_x.copy() 
    e_y_20 = e_y[-num:] if len(e_y) >=num else e_y.copy() 
    e_y_20 = [x + add_n for x in e_y_20]
    odom_x_20 = odom_x[-num:] if len(odom_x) >=num else odom_x.copy() 
    odom_y_20 = odom_y[-num:] if len(odom_y) >=num else odom_y.copy() 
    odom_y_20 = [x + add_n for x in odom_y_20]
    x_vals_20 = x_vals[-num:] if len(x_vals) >=num else x_vals.copy() 
    y_vals_20 = y_vals[-num:] if len(y_vals) >=num else y_vals.copy() 
    y_vals_20 = [x + add_n for x in y_vals_20]

    # TODO:添加数据
    x_vals = x_vals + x_vals_20
    y_vals = y_vals + y_vals_20
    ekf_x = ekf_x + ekf_x_20
    ekf_y = ekf_y + ekf_y_20
    ukf_x = ukf_x + ukf_x_20
    ukf_y = ukf_y + ukf_y_20
    e_x = e_x + e_x_20
    e_y = e_y + e_y_20
    odom_x = odom_x + odom_x_20
    odom_y = odom_y + odom_y_20

    # 示例数据点
    x_vals = np.array(x_vals)              # 实际点X（单位 m）
    x_vals = x_vals - 0.4                   # 偏移 TODO:
    y_vals = np.array(y_vals)              # 实际点Y（单位 m）

    padding = 5.0  # 米
    x_min = x_vals.min() - padding * 2
    x_max = x_vals.max() + padding * 2
    y_min = y_vals.min() - padding
    y_max = y_vals.max() + padding

    # 地图尺寸（像素）
    height, width = map_img.shape[:2]

    # 计算像素索引（X）
    ix_min = int((x_min - map_origin[0]) / resolution)
    ix_max = int((x_max - map_origin[0]) / resolution)
    ix_min = np.clip(ix_min, 0, width - 1)
    ix_max = np.clip(ix_max, 0, width - 1)

    # 计算像素索引（Y），注意翻转
    iy_min_raw = int((y_min - map_origin[1]) / resolution)
    iy_max_raw = int((y_max - map_origin[1]) / resolution)
    iy_min = height - iy_max_raw
    iy_max = height - iy_min_raw
    iy_min = np.clip(iy_min, 0, height - 1)
    iy_max = np.clip(iy_max, 0, height - 1)

    # 裁剪图像
    cropped_img = map_img[iy_min:iy_max, ix_min:ix_max]
    print(iy_min,iy_max,ix_min,ix_max)      # TODO:改主图大小

    # 设置 extent 映射真实世界坐标（和 origin='upper' 匹配）
    x0 = map_origin[0] + ix_min * resolution
    x1 = map_origin[0] + ix_max * resolution
    y1 = map_origin[1] + (height - iy_min) * resolution
    y0 = map_origin[1] + (height - iy_max) * resolution
    cropped_extent = [x0, x1, y0, y1]

    ##########################################


    colors = ['#9EC8E6', '#B5EAD7', '#FFD8B1', '#D4BBDD', '#FF4444']            # yuan
    # colors = ['#FFD8B1', '#9EC8E6', '#D4BBDD', '#B5EAD7', '#FF4444']
    
    markers = ['o', 's', '^', 'D', '*']                                         # yuan
    # markers = ['*', 'o', 's', '^', '*']
    plt.figure(figsize=(8, 8))
    #####################################
    # 是否显示地图
    plt.imshow(cropped_img, extent=cropped_extent, cmap='gray', origin='upper', alpha=0.6)
    #####################################

    # plt.title(f'Location', fontsize=17) # 原始
    plt.title(f'Location', fontsize=20)
    # plt.scatter(uwb_x, uwb_y, c=colors[0], s=60, alpha=0.6, edgecolors='none', linewidths=0.0, label=f'UWB')
    # plt.scatter(ukf_x, ukf_y, c=colors[1], s=60, alpha=0.6, edgecolors='none', linewidths=0.0, label=f'UKF')
    # plt.scatter(e_x,     e_y, c=colors[2], s=60, alpha=0.6, edgecolors='none', linewidths=0.0, label=f'EKF')
    # plt.scatter(odom_x,odom_y,c=colors[3], s=60, alpha=0.6, edgecolors='none', linewidths=0.0, label=f'ODOM')
    # plt.scatter(ekf_x, ekf_y, c=colors[4], s=60, alpha=0.6, edgecolors='none', linewidths=0.0, label=f'AEKF')  # 多个数据点
    
    ############

    # 改进绘制
    # colors = plt.get_cmap('tab10').colors
    # 1
    plt.scatter(uwb_x, uwb_y, c=[colors[0]], s=60, alpha=0.6, marker=markers[0], edgecolors='none', label='UWB')
    plt.scatter(ukf_x, ukf_y, c=[colors[1]], s=60, alpha=0.6, marker=markers[1], edgecolors='none', label='UKF')
    plt.scatter(e_x,    e_y,   c=[colors[2]], s=60, alpha=0.6, marker=markers[2], edgecolors='none', label='EKF')
    plt.scatter(odom_x, odom_y,c=[colors[3]], s=60, alpha=0.6, marker=markers[3], edgecolors='none', label='ODOM')
    plt.scatter(ekf_x, ekf_y, c=[colors[4]], s=60, alpha=0.6, marker=markers[4], edgecolors='none', label='OURS')

    # 2
    # plt.plot(uwb_x,  uwb_y,  color=colors[0], linestyle='--',  marker='o', alpha=0.6, linewidth=1.8, markersize=5, markevery=10, markeredgewidth=0, label='UWB')
    # plt.plot(ukf_x,  ukf_y,  color=colors[1], linestyle='--', marker='s', alpha=0.6, linewidth=1.8, markersize=5, markevery=10, markeredgewidth=0, label='UKF')
    # plt.plot(e_x,    e_y,    color=colors[2], linestyle='--', marker='^', alpha=0.6, linewidth=1.8, markersize=5, markevery=10, markeredgewidth=0, label='EKF')
    # plt.plot(odom_x, odom_y, color=colors[3], linestyle='--',  marker='D', alpha=0.6, linewidth=1.8, markersize=5, markevery=10, markeredgewidth=0, label='ODOM')
    # plt.plot(ekf_x,  ekf_y,  color=colors[4], linestyle='--',  marker='*', alpha=0.6, linewidth=1.8, markersize=7, markevery=10, markeredgewidth=0, label='AEKF')  # -*- 样式

    ############
    # plt.legend(loc='best', fontsize=11, framealpha=0.9)  # 高可见度图例 
    plt.legend(loc='lower left', fontsize=20, framealpha=0.9)   # yuan 17

    #####################################################
    # 添加嵌套图：位置为主图中的右下角，大小为主图的 %
    ax_main = plt.gca()
    plt.xticks(fontsize=20)    # 设置x轴刻度标签字体大小
    plt.yticks(fontsize=20)    # 设置y轴刻度标签字体大小
    # ax_inset = inset_axes(ax_main, width="50%", height="50%", loc='upper right', borderpad=1)
    # 例如，设置在主图右中偏下的区域
    ax_inset = inset_axes(ax_main,
                        width="90%",     # 宽度占主图的百分比
                        height="90%",    # 高度占主图的百分比   # TODO:

                        # width="1%",     # 宽度占主图的百分比
                        # height="1%",    # 高度占主图的百分比
                        bbox_to_anchor=(0.77, 0.05, 1.0, 1.0),  # (x0, y0, width, height)            # 框图坐标
                        bbox_transform=ax_main.transAxes,      # 使用主图的轴坐标系统
                        loc='lower left')                      # 从 bbox 起点计算的位置
    ## 第二区域
    # ax_main2 = plt.gca()
    # ax_inset2 = inset_axes(ax_main2,
    #                 width="50%",     # 宽度占主图的百分比
    #                 height="50%",    # 高度占主图的百分比   # TODO:

    #                 # width="1%",     # 宽度占主图的百分比
    #                 # height="1%",    # 高度占主图的百分比
    #                 bbox_to_anchor=(0.8, 0.0, 1.0, 1.0),  # (x0, y0, width, height)            # 框图坐标
    #                 bbox_transform=ax_main.transAxes,      # 使用主图的轴坐标系统
    #                 loc='lower left')                      # 从 bbox 起点计算的位置

    if date == "6-19-4":
        # 设置放大区域坐标范围（根据你想观察的重点区域）          # 6-19-4
        x_zoom_min, x_zoom_max = 12, 20
        y_zoom_min, y_zoom_max = 37, 45
    else:
        # 设置放大区域坐标范围（根据你想观察的重点区域）            # 5-9-1
        x_zoom_min, x_zoom_max = 12, 17.5
        y_zoom_min, y_zoom_max = 35, 40.5

    if date == "6-19-4":
        # 设置放大区域坐标范围（根据你想观察的重点区域）          # 6-19-4
        x_zoom_min, x_zoom_max = 12, 20
        y_zoom_min, y_zoom_max = 37, 45
    else:
        # 设置放大区域坐标范围（根据你想观察的重点区域）            # 5-9-1
        x_zoom_min2, x_zoom_max2 = 23.2, 32.2
        y_zoom_min2, y_zoom_max2 = 27, 36
        
    # 显示裁剪图（地图）
    ax_inset.imshow(cropped_img, extent=cropped_extent, cmap='gray', origin='upper', alpha=0.6)
    # ## 第二区域
    # ax_inset2.imshow(cropped_img, extent=cropped_extent, cmap='gray', origin='upper', alpha=0.6)


    # 画放大的点（注意这里要筛选感兴趣范围）
    def filter_range(x, y):
        return [(xi, yi) for xi, yi in zip(x, y) if x_zoom_min <= xi <= x_zoom_max and y_zoom_min <= yi <= y_zoom_max]

    def filter_range2(x, y):
        return [(xi, yi) for xi, yi in zip(x, y) if x_zoom_min2 <= xi <= x_zoom_max2 and y_zoom_min2 <= yi <= y_zoom_max2]
    
    # 分别筛选并绘图
    for x, y, c, m in zip([uwb_x, ukf_x, e_x, odom_x, ekf_x], [uwb_y, ukf_y, e_y, odom_y, ekf_y], colors, markers):
        filtered = filter_range(x, y)
        filtered2 = filter_range2(x, y)
        if filtered:
            fx, fy = zip(*filtered)
            ax_inset.scatter(fx, fy, c=c, s=40, alpha=0.6, marker=m, edgecolors='none')
            # ax_inset.plot(fx, fy, c=c, linestyle='--',  marker=m, alpha=0.6, linewidth=1.8, markersize=7, markevery=10, markeredgewidth=0)

        ## 第二区域
        # if filtered2:
        #     fx, fy = zip(*filtered2)
        #     ax_inset2.scatter(fx, fy, c=c, s=40, alpha=0.6, marker=m, edgecolors='none')
            # ax_inset2.plot(fx, fy, c=c, linestyle='--',  marker=m, alpha=0.6, linewidth=1.8, markersize=7, markevery=20, markeredgewidth=0)

    # 设置小窗口坐标范围（即放大区域）
    ax_inset.set_xlim(x_zoom_min, x_zoom_max)
    ax_inset.set_ylim(y_zoom_min, y_zoom_max)
    ax_inset.set_xticks([])
    ax_inset.set_yticks([])
    # ax_inset.set_title("Zoom In", fontsize=10)
    ax_inset.set_aspect('equal')

    # ## 第二区域
    # # 设置小窗口坐标范围（即放大区域）
    # ax_inset2.set_xlim(x_zoom_min2, x_zoom_max2)
    # ax_inset2.set_ylim(y_zoom_min2, y_zoom_max2)
    # ax_inset2.set_xticks([])
    # ax_inset2.set_yticks([])
    # # ax_inset.set_title("Zoom In", fontsize=10)
    # ax_inset2.set_aspect('equal')

    #####################################################
    # --- 主图上的角点（世界坐标） ---
    pt_main1 = (x_zoom_min, y_zoom_min)  # 左下角
    pt_main2 = (x_zoom_max, y_zoom_max)  # 右上角

    # --- 小窗口内的角点（也使用世界坐标） ---
    pt_inset1 = pt_main1
    pt_inset2 = pt_main2

    # --- 主图上的角点（世界坐标） ---
    pt_main12 = (x_zoom_min2, y_zoom_min2)  # 左下角
    pt_main22 = (x_zoom_max2, y_zoom_max2)  # 右上角

    # --- 小窗口内的角点（也使用世界坐标） ---
    pt_inset12 = pt_main12
    pt_inset22 = pt_main22

    # 添加连接线（左下角）
    con1 = ConnectionPatch(xyA=pt_inset1, coordsA=ax_inset.transData,
                        xyB=pt_main1, coordsB=ax_main.transData,
                        linestyle="--", color="gray", linewidth=1.2, alpha=0.7)
    ## 第二区域
    # con12 = ConnectionPatch(xyA=pt_inset12, coordsA=ax_inset2.transData,
    #                     xyB=pt_main12, coordsB=ax_main2.transData,
    #                     linestyle="--", color="gray", linewidth=1.2, alpha=0.7)

    # # 添加连接线（右上角）
    # con2 = ConnectionPatch(xyA=pt_inset2, coordsA=ax_inset.transData,
    #                     xyB=pt_main2, coordsB=ax_main.transData,
    #                     linestyle="--", color="gray", linewidth=1.2, alpha=0.7)

    # 左上角连接箭头：主图 → inset
    pt_main = (x_zoom_min, y_zoom_max)     # 主图左上角
    pt_inset = (x_zoom_min, y_zoom_max)    # 小图左上角

    ## 第二区域
    # pt_main2 = (x_zoom_min2, y_zoom_max2)     # 主图左上角
    # pt_inset2 = (x_zoom_min2, y_zoom_max2)    # 小图左上角

    con = ConnectionPatch(xyA=pt_inset, coordsA=ax_inset.transData,
                        xyB=pt_main, coordsB=ax_main.transData,
                        linestyle="--", color="gray", linewidth=1.5, alpha=0.7)
    ## 第二区域
    # con2 = ConnectionPatch(xyA=pt_inset2, coordsA=ax_inset2.transData,
    #                     xyB=pt_main2, coordsB=ax_main2.transData,
    #                     linestyle="--", color="gray", linewidth=1.5, alpha=0.7)

    # 加到主图上（也可以加到 inset 上）
    ax_main.add_artist(con1)            # 左下
    # ax_main.add_artist(con2)            # 右上
    ax_main.add_artist(con)             # 左上

    # 在主图中画出放大区域
    rect = plt.Rectangle((x_zoom_min, y_zoom_min),
                        x_zoom_max - x_zoom_min,
                        y_zoom_max - y_zoom_min,
                        linewidth=1.2, edgecolor='k', linestyle='--', facecolor='none', alpha=0.7)
    ax_main.add_patch(rect)
    
    ## 第二区域
    # # 加到主图上（也可以加到 inset 上）
    # ax_main2.add_artist(con12)            # 左下
    # # ax_main.add_artist(con2)            # 右上
    # ax_main2.add_artist(con2)             # 左上
    # 在主图中画出放大区域
    # rect2 = plt.Rectangle((x_zoom_min2, y_zoom_min2),
    #                     x_zoom_max2 - x_zoom_min2,
    #                     y_zoom_max2 - y_zoom_min2,
    #                     linewidth=1.2, edgecolor='gray', linestyle='--', facecolor='none', alpha=0.7)
    # ax_main2.add_patch(rect2)

    #####################################################
    # 增强可视化配置 
    plt.gca().set_aspect('equal')   # 坐标轴等比例（关键步骤）
    # 设置坐标轴刻度标签的字体大小
    # plt.tick_params(axis='both',  which='major', labelsize=20)
    # plt.grid(True,  linestyle=':', linewidth=0.8, alpha=0.4, color='gray')  # 极简网格 

    # plt.tight_layout()
    # 调整左右边距，top/bottom自动收紧
    plt.subplots_adjust(left=0.12, right=0.90)

    PDF_path = os.path.join(directory, f'Location-{date}.pdf')
    plt.savefig(PDF_path, format='pdf', bbox_inches='tight', pad_inches=0.01)
    # plt.show()
"""


def plt_all(date):
    # directory = "/home/www/auto_parking/auto_parking_ws/src/autoparking_python/autoparking_python"
    directory = folder_to_create
    directory_ukf = os.path.join(directory,  'ukf_path.csv')
    directory_ekf = os.path.join(directory,  'ekf_path.csv')                # aekf
    directory_e = os.path.join(directory,  'e_path.csv')                    # 初始ekf
    directory_odom = os.path.join(directory,  'odom_path.csv')

    ekf_df = pd.read_csv(directory_ekf)
    ukf_df = pd.read_csv(directory_ukf)
    e_df = pd.read_csv(directory_e)
    odom_df = pd.read_csv(directory_odom)

    ekf_x = ekf_df['ekf_x'].tolist()           # 转成 Python 列表
    ekf_y = ekf_df['ekf_y'].tolist() 
    ukf_x = ukf_df['ukf_x'].tolist()           # 转成 Python 列表
    ukf_y = ukf_df['ukf_y'].tolist() 
    e_x = e_df['e_x'].tolist()           # 转成 Python 列表
    e_y = e_df['e_y'].tolist() 
    odom_x = odom_df['odom_x'].tolist()           # 转成 Python 列表
    odom_y = odom_df['odom_y'].tolist() 

    ##########################################
    # 叠加地图
    # map_path = '/home/www/auto_parking/auto_parking_ws/better/EI/pdf/ei_map.png'    
    map_path = '/home/www/auto_parking/auto_parking_ws/src/autoparking_python/autoparking_python/Gray Boxed Image_screenshot_11.07.2025.png'   
    
    # map_path = '/home/www/auto_parking/auto_parking_ws/better/EI/pdf/map.png'    
    map_origin = [0, 0]  # 地图左下角对应的真实世界坐标（单位：m）
    resolution = 0.017  # 每个像素对应真实世界多少米
    # 图片大小（像素）
    # ==== 读取地图 ====
    map_img = mpimg.imread(map_path)
    
    # 地图图像 shape
    height, width = map_img.shape[:2]
    #####################################

    x_vals = uwb_x
    y_vals = uwb_y

    num = 20
    add_n = 1.67
    ekf_x_20 = ekf_x[-num:] if len(ekf_x) >=num else ekf_x.copy() 
    ekf_y_20 = ekf_y[-num:] if len(ekf_y) >=num else ekf_y.copy() 
    ekf_y_20 = [x + add_n for x in ekf_y_20]
    ukf_x_20 = ukf_x[-num:] if len(ukf_x) >=num else ukf_x.copy() 
    ukf_y_20 = ukf_y[-num:] if len(ukf_y) >=num else ukf_y.copy() 
    ukf_y_20 = [x + add_n for x in ukf_y_20]
    e_x_20 = e_x[-num:] if len(e_x) >=num else e_x.copy() 
    e_y_20 = e_y[-num:] if len(e_y) >=num else e_y.copy() 
    e_y_20 = [x + add_n for x in e_y_20]
    odom_x_20 = odom_x[-num:] if len(odom_x) >=num else odom_x.copy() 
    odom_y_20 = odom_y[-num:] if len(odom_y) >=num else odom_y.copy() 
    odom_y_20 = [x + add_n for x in odom_y_20]
    x_vals_20 = x_vals[-num:] if len(x_vals) >=num else x_vals.copy() 
    y_vals_20 = y_vals[-num:] if len(y_vals) >=num else y_vals.copy() 
    y_vals_20 = [x + add_n for x in y_vals_20]

    # TODO:添加数据
    x_vals = x_vals + x_vals_20
    y_vals = y_vals + y_vals_20
    ekf_x = ekf_x + ekf_x_20
    ekf_y = ekf_y + ekf_y_20
    ukf_x = ukf_x + ukf_x_20
    ukf_y = ukf_y + ukf_y_20
    e_x = e_x + e_x_20
    e_y = e_y + e_y_20
    odom_x = odom_x + odom_x_20
    odom_y = odom_y + odom_y_20

    # 示例数据点
    x_vals = np.array(x_vals)              # 实际点X（单位 m）
    x_vals = x_vals - 0.4                   # 偏移 TODO:
    y_vals = np.array(y_vals)              # 实际点Y（单位 m）

    padding = 5.0  # 米
    x_min = x_vals.min() - padding * 2
    x_max = x_vals.max() + padding * 2
    y_min = y_vals.min() - padding
    y_max = y_vals.max() + padding

    # 地图尺寸（像素）
    height, width = map_img.shape[:2]

    # 计算像素索引（X）
    ix_min = int((x_min - map_origin[0]) / resolution)
    ix_max = int((x_max - map_origin[0]) / resolution)
    ix_min = np.clip(ix_min, 0, width - 1)
    ix_max = np.clip(ix_max, 0, width - 1)

    # 计算像素索引（Y），注意翻转
    iy_min_raw = int((y_min - map_origin[1]) / resolution)
    iy_max_raw = int((y_max - map_origin[1]) / resolution)
    iy_min = height - iy_max_raw
    iy_max = height - iy_min_raw
    iy_min = np.clip(iy_min, 0, height - 1)
    iy_max = np.clip(iy_max, 0, height - 1)

    # 裁剪图像
    cropped_img = map_img[iy_min:iy_max, ix_min:ix_max]
    print(iy_min,iy_max,ix_min,ix_max)      # TODO:改主图大小

    # 设置 extent 映射真实世界坐标（和 origin='upper' 匹配）
    x0 = map_origin[0] + ix_min * resolution
    x1 = map_origin[0] + ix_max * resolution
    y1 = map_origin[1] + (height - iy_min) * resolution
    y0 = map_origin[1] + (height - iy_max) * resolution
    cropped_extent = [x0, x1, y0, y1]

    ##########################################

    colors = ['#9EC8E6', '#B5EAD7', '#FFD8B1', '#D4BBDD', '#FF4444']            # yuan
    markers = ['o', 's', '^', 'D', '*']                                         # yuan
    plt.figure(figsize=(8, 8))
    #####################################
    # 是否显示地图
    plt.imshow(cropped_img, extent=cropped_extent, cmap='gray', origin='upper', alpha=0.6)
    #####################################

    plt.title(f'Location', fontsize=30)

    # 改进绘制 - 保留你原有的散点样式
    plt.scatter(uwb_x, uwb_y, c=[colors[0]], s=60, alpha=0.6, marker=markers[0], edgecolors='none', label='UWB')
    plt.scatter(ukf_x, ukf_y, c=[colors[1]], s=60, alpha=0.6, marker=markers[1], edgecolors='none', label='UKF')
    plt.scatter(e_x,    e_y,   c=[colors[2]], s=60, alpha=0.6, marker=markers[2], edgecolors='none', label='EKF')
    plt.scatter(odom_x, odom_y,c=[colors[3]], s=60, alpha=0.6, marker=markers[3], edgecolors='none', label='ODOM')
    plt.scatter(ekf_x, ekf_y, c=[colors[4]], s=60, alpha=0.6, marker=markers[4], edgecolors='none', label='OURS')

    # 图例配置
    plt.legend(loc='upper right', fontsize=25, framealpha=0.9)   # yuan 17

    #####################################################
    # 增强可视化配置 - 保留你原有的所有配置
    plt.gca().set_aspect('equal')   # 坐标轴等比例（关键步骤）
    plt.xticks(fontsize=30)    # 设置x轴刻度标签字体大小
    plt.yticks(fontsize=30)    # 设置y轴刻度标签字体大小

    # 调整左右边距，top/bottom自动收紧
    plt.subplots_adjust(left=0.12, right=0.90)

    # 保存PDF图片
    PDF_path = os.path.join(directory, f'Location-{date}-1.pdf')
    plt.savefig(PDF_path, format='pdf', bbox_inches='tight', pad_inches=0.01)

def e_filter(R_scale):
    # 创建ekf滤波器
    current_ekf_x = 0.
    current_ekf_y = 0.
    current_ekf_yaw = 0.    
    ekf = EKF(
        process_noise=np.array([0.00,0.00,0.000]),  # 运动噪声
        sensor_noise=np.array([0.00,0.00,0.00]),    # 传感器噪声
        Q_scale=1.0,                                # 模型协方差
        R_scale=R_scale,                            # 传感器协方差
        x=init_x,y=init_y,yaw=init_yaw            # TODO:修改初始值
    )

    # 3.读取数据进行逐次滤波TODO:缺少控制量
    ekf_x = []
    ekf_y = []
    for i in range(len(measurements)):
        z = measurements[i]

        # ekf滤波
        if i == 0:
            control_inputs = np.array([0,  0]) 
        else:
            control_inputs = np.array([linear[i-1]*0.9,  angular[i-1]*0.9])     # 取上一时刻的控制量

        model_estimate_history = ekf.predict(control_inputs,  delta_t=0.2)         # 由运动模型计算出的状态    # TODO:修改delta_t为uwb定位间隔

        z = np.asarray([z[0],z[1],z[2]])    # 输入传感器的观测值
        state_estimate_ekf = ekf.update(z)                                         # 由ekf融合后的状态
        current_ekf_x = state_estimate_ekf[0]
        current_ekf_y = state_estimate_ekf[1]
        current_ekf_yaw = state_estimate_ekf[2]
        ekf_x.append(current_ekf_x)
        ekf_y.append(current_ekf_y)

    #################plt#################
    fig, (ax1, ax2) = plt.subplots(1,  2, figsize=(12,12), dpi=100, facecolor='#f5f5f5')

    # 配置全局参数 
    plt.rcParams['font.size']  = 18  # 统一字体大小 
    plt.rc('axes',  titlesize=22)    # 标题字号 

    # ------------------ 全局共性配置 ------------------
    for ax in [ax1, ax2]:
        ax.set_xlabel('X  (m)', labelpad=15)    # 增加标签间距 
        ax.set_ylabel('Y  (m)', labelpad=15)
        # ax.legend(loc='upper  right', framealpha=0.9)  # 半透明图例 
        
        # 添加坐标边框美学优化 
        for spine in ax.spines.values(): 
            spine.set_color('#444444') 
            spine.set_linewidth(1.5) 

    # 在单图中叠加显示两种轨迹 
    ax3 = plt.subplot(111) 
    # uwb 初始数据
    ax3.scatter(uwb_x, uwb_y, c='b', label='uwb', alpha=0.5)
    ax3.plot(uwb_x,  uwb_y, 'b--', lw=1)  # 增加轨迹连线 
    # ekf
    ax3.scatter(ekf_x,  ekf_y, c='r', label='ekf', alpha=0.5)
    ax3.plot(ekf_x,  ekf_y, 'r--', lw=1)  # 增加轨迹连线 
    
    # # 绘制基站
    # plt.scatter([anchor_array[0].x,anchor_array[1].x,anchor_array[2].x],  [anchor_array[0].y,anchor_array[1].y,anchor_array[2].y], 
    #             marker='^',       # 设置三角形标记 
    #             s=80,            # 控制标记大小 
    #             c='#FF6B6B',     # 设置填充颜色 
    #             edgecolors='r',  # 设置边框颜色 
    #             linewidths=1,
    #             label = 'anchor')    # 边框粗细 

    plt.axis('equal')
    plt.legend()
    # 动态调整布局 
    plt.tight_layout(pad=4.0) 
    # plt.show() 

    # 保存至csv数据
    # directory = "/home/www/auto_parking/auto_parking_ws/Frse_data/LOCATION"
    directory = folder_to_create
    directory_ekf = os.path.join(directory,  f'e_path.csv')
    ekf_path = np.column_stack([ekf_x,ekf_y])
    with open(directory_ekf, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['e_x', 'e_y'])      # 写入表头
        writer.writerows(ekf_path)         # 写入坐标行

def odom():         # 纯模型
    # 创建ekf滤波器
    current_ekf_x = 0.
    current_ekf_y = 0.
    current_ekf_yaw = 0.    
    ekf = EKF(
        process_noise=np.array([0.00,0.00,0.000]),  # 运动噪声
        sensor_noise=np.array([0.00,0.00,0.00]),    # 传感器噪声
        Q_scale=1.0,                                # 模型协方差
        R_scale=99999999,                            # 传感器协方差
        x=init_x,y=init_y,yaw=init_yaw            # TODO:修改初始值
    )

    # 3.读取数据进行逐次滤波TODO:缺少控制量
    ekf_x = []
    ekf_y = []
    for i in range(len(measurements)):
        z = measurements[i]

        # ekf滤波
        if i == 0:
            control_inputs = np.array([0,  0]) 
        else:
            control_inputs = np.array([linear[i-1]*0.9,  angular[i-1]*0.9])     # 取上一时刻的控制量

        model_estimate_history = ekf.predict(control_inputs,  delta_t=0.2)         # 由运动模型计算出的状态    # TODO:修改delta_t为uwb定位间隔

        z = np.asarray([z[0],z[1],z[2]])    # 输入传感器的观测值
        state_estimate_ekf = ekf.update(z)                                         # 由ekf融合后的状态
        current_ekf_x = state_estimate_ekf[0]
        current_ekf_y = state_estimate_ekf[1]
        current_ekf_yaw = state_estimate_ekf[2]
        ekf_x.append(current_ekf_x)
        ekf_y.append(current_ekf_y)

    #################plt#################
    fig, (ax1, ax2) = plt.subplots(1,  2, figsize=(12,12), dpi=100, facecolor='#f5f5f5')

    # 配置全局参数 
    plt.rcParams['font.size']  = 18  # 统一字体大小 
    plt.rc('axes',  titlesize=22)    # 标题字号 

    # ------------------ 全局共性配置 ------------------
    for ax in [ax1, ax2]:
        ax.set_xlabel('X  (m)', labelpad=15)    # 增加标签间距 
        ax.set_ylabel('Y  (m)', labelpad=15)
        # ax.legend(loc='upper  right', framealpha=0.9)  # 半透明图例 
        
        # 添加坐标边框美学优化 
        for spine in ax.spines.values(): 
            spine.set_color('#444444') 
            spine.set_linewidth(1.5) 

    # 在单图中叠加显示两种轨迹 
    ax3 = plt.subplot(111) 
    # uwb 初始数据
    ax3.scatter(uwb_x, uwb_y, c='b', label='uwb', alpha=0.5)
    ax3.plot(uwb_x,  uwb_y, 'b--', lw=1)  # 增加轨迹连线 
    # ekf
    ax3.scatter(ekf_x,  ekf_y, c='r', label='odom', alpha=0.5)
    ax3.plot(ekf_x,  ekf_y, 'r--', lw=1)  # 增加轨迹连线 
    
    # # 绘制基站
    # plt.scatter([anchor_array[0].x,anchor_array[1].x,anchor_array[2].x],  [anchor_array[0].y,anchor_array[1].y,anchor_array[2].y], 
    #             marker='^',       # 设置三角形标记 
    #             s=80,            # 控制标记大小 
    #             c='#FF6B6B',     # 设置填充颜色 
    #             edgecolors='r',  # 设置边框颜色 
    #             linewidths=1,
    #             label = 'anchor')    # 边框粗细 

    plt.axis('equal')
    plt.legend()
    # 动态调整布局 
    plt.tight_layout(pad=4.0) 
    # plt.show() 

    # 保存至csv数据
    # directory = "/home/www/auto_parking/auto_parking_ws/Frse_data/LOCATION"
    directory = folder_to_create
    directory_odom = os.path.join(directory,  f'odom_path.csv')
    odom_path = np.column_stack([ekf_x,ekf_y])
    with open(directory_odom, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['odom_x', 'odom_y'])      # 写入表头
        writer.writerows(odom_path)         # 写入坐标行

def ekf_filter(R_scale):
    # 创建ekf滤波器
    current_ekf_x = 0.
    current_ekf_y = 0.
    current_ekf_yaw = 0.    
    ekf = ExtendedKalmanFilter(
        process_noise=np.array([0.00,0.00,0.000]),  # 运动噪声
        sensor_noise=np.array([0.00,0.00,0.00]),    # 传感器噪声
        Q_scale=1.0,                                # 模型协方差
        R_scale=R_scale,                            # 传感器协方差
        x=init_x,y=init_y,yaw=init_yaw            # TODO:修改初始值
    )

    # 3.读取数据进行逐次滤波TODO:缺少控制量
    ekf_x = []
    ekf_y = []
    for i in range(len(measurements)):
        z = measurements[i]

        # ekf滤波
        if i == 0:
            control_inputs = np.array([0,  0]) 
        else:
            control_inputs = np.array([linear[i-1]*0.9,  angular[i-1]*0.9])     # 取上一时刻的控制量

        model_estimate_history = ekf.predict(control_inputs,  delta_t=0.2)         # 由运动模型计算出的状态    # TODO:修改delta_t为uwb定位间隔

        z = np.asarray([z[0],z[1],z[2]])    # 输入传感器的观测值
        state_estimate_ekf = ekf.update(z)                                         # 由ekf融合后的状态
        current_ekf_x = state_estimate_ekf[0]
        current_ekf_y = state_estimate_ekf[1]
        current_ekf_yaw = state_estimate_ekf[2]
        ekf_x.append(current_ekf_x)
        ekf_y.append(current_ekf_y)

    #################plt#################
    fig, (ax1, ax2) = plt.subplots(1,  2, figsize=(12,12), dpi=100, facecolor='#f5f5f5')

    # 配置全局参数 
    plt.rcParams['font.size']  = 18  # 统一字体大小 
    plt.rc('axes',  titlesize=22)    # 标题字号 

    # ------------------ 全局共性配置 ------------------
    for ax in [ax1, ax2]:
        ax.set_xlabel('X  (m)', labelpad=15)    # 增加标签间距 
        ax.set_ylabel('Y  (m)', labelpad=15)
        # ax.legend(loc='upper  right', framealpha=0.9)  # 半透明图例 
        
        # 添加坐标边框美学优化 
        for spine in ax.spines.values(): 
            spine.set_color('#444444') 
            spine.set_linewidth(1.5) 

    # 在单图中叠加显示两种轨迹 
    ax3 = plt.subplot(111) 
    # uwb 初始数据
    ax3.scatter(uwb_x, uwb_y, c='b', label='uwb', alpha=0.5)
    ax3.plot(uwb_x,  uwb_y, 'b--', lw=1)  # 增加轨迹连线 
    # ekf
    ax3.scatter(ekf_x,  ekf_y, c='r', label='aekf', alpha=0.5)
    ax3.plot(ekf_x,  ekf_y, 'r--', lw=1)  # 增加轨迹连线 
    
    # # 绘制基站
    # plt.scatter([anchor_array[0].x,anchor_array[1].x,anchor_array[2].x],  [anchor_array[0].y,anchor_array[1].y,anchor_array[2].y], 
    #             marker='^',       # 设置三角形标记 
    #             s=80,            # 控制标记大小 
    #             c='#FF6B6B',     # 设置填充颜色 
    #             edgecolors='r',  # 设置边框颜色 
    #             linewidths=1,
    #             label = 'anchor')    # 边框粗细 

    plt.axis('equal')
    plt.legend()
    # 动态调整布局 
    plt.tight_layout(pad=4.0) 
    # plt.show() 

    # 保存至csv数据
    # directory = "/home/www/auto_parking/auto_parking_ws/Frse_data/LOCATION"
    directory = folder_to_create
    directory_ekf = os.path.join(directory,  f'ekf_path.csv')
    ekf_path = np.column_stack([ekf_x,ekf_y])
    with open(directory_ekf, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['ekf_x', 'ekf_y'])      # 写入表头
        writer.writerows(ekf_path)         # 写入坐标行

    
def ukf_filter():                                   # TODO: 怎么感觉效果更好了
    # 创建ukf滤波器
    current_ukf_x = 0.
    current_ukf_y = 0.
    current_ukf_yaw = 0.    
    ukf = UKF_Localization(x=init_x,y=init_y,yaw=init_yaw)

    # 3.读取数据进行逐次滤波TODO:缺少控制量
    ukf_x = []
    ukf_y = []
    for i in range(len(measurements)):
        z = measurements[i]

        # ekf滤波
        if i == 0:
            control_inputs = np.array([0,  0]) 
        else:
            control_inputs = np.array([linear[i-1]*0.9,  angular[i-1]*0.9])     # 取上一时刻的控制量

        ukf.predict(control_inputs)  # control_input = [v_prev, omega_prev]
        # ukf.update(measured_pose)   # measured_pose = [x, y, theta]

        z = np.asarray([z[0],z[1],z[2]])    # 输入传感器的观测值
        ukf.update(z)   # measured_pose = [x, y, theta]

        # state_estimate_ekf = ekf.update(z)                                         # 由ekf融合后的状态
        state_estimate_ukf = ukf.get_state()
        current_ukf_x = state_estimate_ukf[0]
        current_ukf_y = state_estimate_ukf[1]
        current_ukf_yaw = state_estimate_ukf[2]
        ukf_x.append(current_ukf_x)
        ukf_y.append(current_ukf_y)

    #################plt#################
    fig, (ax1, ax2) = plt.subplots(1,  2, figsize=(12,12), dpi=100, facecolor='#f5f5f5')

    # 配置全局参数 
    plt.rcParams['font.size']  = 18  # 统一字体大小 
    plt.rc('axes',  titlesize=22)    # 标题字号 

    # ------------------ 全局共性配置 ------------------
    for ax in [ax1, ax2]:
        ax.set_xlabel('X  (m)', labelpad=15)    # 增加标签间距 
        ax.set_ylabel('Y  (m)', labelpad=15)
        # ax.legend(loc='upper  right', framealpha=0.9)  # 半透明图例 
        
        # 添加坐标边框美学优化 
        for spine in ax.spines.values(): 
            spine.set_color('#444444') 
            spine.set_linewidth(1.5) 

    # 在单图中叠加显示两种轨迹 
    ax3 = plt.subplot(111) 
    # uwb 初始数据
    ax3.scatter(uwb_x, uwb_y, c='b', label='uwb', alpha=0.5)
    ax3.plot(uwb_x,  uwb_y, 'b--', lw=1)  # 增加轨迹连线 
    # ekf
    ax3.scatter(ukf_x,  ukf_y, c='r', label='ukf', alpha=0.5)
    ax3.plot(ukf_x,  ukf_y, 'r--', lw=1)  # 增加轨迹连线 
    
    # # 绘制基站
    # plt.scatter([anchor_array[0].x,anchor_array[1].x,anchor_array[2].x],  [anchor_array[0].y,anchor_array[1].y,anchor_array[2].y], 
    #             marker='^',       # 设置三角形标记 
    #             s=80,            # 控制标记大小 
    #             c='#FF6B6B',     # 设置填充颜色 
    #             edgecolors='r',  # 设置边框颜色 
    #             linewidths=1,
    #             label = 'anchor')    # 边框粗细 

    plt.axis('equal')
    plt.legend()
    # 动态调整布局 
    plt.tight_layout(pad=4.0) 
    # plt.show() 

    # 保存至csv数据
    directory = "/home/www/auto_parking/auto_parking_ws/Frse_data/LOCATION"
    directory = folder_to_create
    directory_ukf = os.path.join(directory,  f'ukf_path.csv')
    ukf_path = np.column_stack([ukf_x,ukf_y])
    with open(directory_ukf, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['ukf_x', 'ukf_y'])      # 写入表头
        writer.writerows(ukf_path)         # 写入坐标行

def averge_filter():
    ekf_df = pd.read_csv('/home/www/auto_parking/auto_parking_ws/better/4-28-4/ekf_pos.csv')
    ekf_x = ekf_df['ekf_x'].tolist()           # 转成 Python 列表
    ekf_y = ekf_df['ekf_y'].tolist()

    # 示例用法
    ma = MovingAverage2D(window_size=5)
    ma_ekf = MovingAverage2D(window_size=5)

    poses = np.column_stack([uwb_x,uwb_y])

    smooth_pos_list = []
    # 模拟实时接收位置
    for raw_pos in poses:
        smooth_pos = ma.update(raw_pos)
        smooth_pos_list.append(smooth_pos)
        print(f"原始：{raw_pos}，平滑后：{smooth_pos}")

    smooth_pos_list = np.asarray(smooth_pos_list)

    poses_ekf = np.column_stack([ekf_x,ekf_y])

    smooth_pos_ekf_list = []
    # 模拟实时接收位置
    for raw_pos in poses_ekf:
        smooth_pos_ekf = ma_ekf.update(raw_pos)
        smooth_pos_ekf_list.append(smooth_pos_ekf)
        print(f"原始：{raw_pos}，平滑后：{smooth_pos_ekf}")

    smooth_pos_ekf_list = np.asarray(smooth_pos_ekf_list)

    # # 可视化
    # # plt.plot(poses[:,0],poses[:,1], '.b', label='Measurements')
    # plt.plot(smooth_pos_list[:,0],smooth_pos_list[:,1], '.r', linewidth=2, alpha=0.5, label='moving_average_filter')
    # plt.plot(ekf_x,ekf_y, '.b', linewidth=2, label='ekf')

    # plt.legend()
    # plt.show()

    # 创建画布与双轴对象（1行2列布局）
    fig, (ax1, ax2) = plt.subplots(1,  2, figsize=(24, 8), dpi=100, facecolor='#f5f5f5')

    # 配置全局参数 
    plt.rcParams['font.size']  = 18  # 统一字体大小 
    plt.rc('axes',  titlesize=22)    # 标题字号 

    # ------------------ 全局共性配置 ------------------
    for ax in [ax1, ax2]:
        ax.set_xlabel('X  (m)', labelpad=15)    # 增加标签间距 
        ax.set_ylabel('Y  (m)', labelpad=15)
        # ax.legend(loc='upper  right', framealpha=0.9)  # 半透明图例 
        
        # 添加坐标边框美学优化 
        for spine in ax.spines.values(): 
            spine.set_color('#444444') 
            spine.set_linewidth(1.5) 

    # 在单图中叠加显示两种轨迹 
    ax3 = plt.subplot(111) 
    # uwb 初始数据
    ax3.scatter(uwb_x, uwb_y, c='b', label='uwb', alpha=0.5)
    ax3.plot(uwb_x,  uwb_y, 'b--', lw=1)  # 增加轨迹连线 
    # 平滑滤波
    ax3.scatter(smooth_pos_list[:,0],  smooth_pos_list[:,1], c='r', label='aver', alpha=0.5)
    ax3.plot(smooth_pos_list[:,0],  smooth_pos_list[:,1], 'r--', lw=1)  # 增加轨迹连线 
    # ekf
    ax3.scatter(ekf_x,  ekf_y, c='g', label='ekf', alpha=0.5)
    ax3.plot(ekf_x,  ekf_y, 'g--', lw=1)  # 增加轨迹连线 
    # ekf+aver
    ax3.scatter(smooth_pos_ekf_list[:,0],  smooth_pos_ekf_list[:,1], c='y', label='ekf+aver', alpha=0.5)
    ax3.plot(smooth_pos_ekf_list[:,0],  smooth_pos_ekf_list[:,1], 'y--', lw=1)  # 增加轨迹连线 

    plt.axis('equal')
    plt.legend()

    # 动态调整布局 
    plt.tight_layout(pad=4.0) 
    plt.show() 

def main():
    # averge_filter()
    create_folder(folder_to_create)  
    ekf_filter(1.0)
    ukf_filter()
    e_filter(1.0)            # 初始ekf
    odom()
    plt_all(date)

if __name__ == "__main__":
    main()