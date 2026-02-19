import cv2
import numpy as np
import matplotlib.pyplot  as plt 
import matplotlib.colors  as mcolors 
import colorsys

# plt像素转化为cv像素
def plt2cv_coord(plt_point, img_height):                # 点迹；像素高度
    """坐标系转换核心逻辑"""
    x, y = plt_point 
    return (int(round(x)), int(round(img_height - y - 1)))

def generate_soft_colors(n=5, saturation=0.5, lightness=0.8):
    hues = [i*(360/n) for i in range(n)]
    hex_colors = []
    for h in hues:
        r, g, b = colorsys.hls_to_rgb(h/360,  lightness, saturation)
        hex_colors.append('#{:02X}{:02X}{:02X}'.format(int(r*255),  int(g*255), int(b*255)))
    return hex_colors

class PathsOverlay:
    def __init__(self, pixel_to_meter_m, image, alpha): 

        self.pixel_to_meter_m  = pixel_to_meter_m         
        self.image  = image      
        self.alpha  = alpha  
        self.height = self.image.shape[0]
        self.path = []
        self.cvshow_n = 0               # 记录执行次数
        
    def cvshow(self, points):
        # 第一次调用 原图像画
        if self.cvshow_n == 0:
            self.cvshow_n +=1
        # 在之前保存的图像上画
        else:
            # cv2.imshow("output_transparent.png", self.image)
            # cv2.waitKey(5000)
            self.image  = cv2.imread("/home/www/auto_parking/server_ws/output_transparent.png")
            self.cvshow_n +=1
        
        # 创建透明图层（与原始图像同尺寸）
        overlay = self.image.copy()  # 或 np.zeros_like(image)

        # colors = ['#9EC8E6', '#B5EAD7', '#FFD8B1', '#D4BBDD', '#FF4444']        # TODO:颜色修改
        # 生成4色方案 
        colors = generate_soft_colors(5)
        # print(colors)
        def hex_to_bgr(hex_str, alpha = 200):
            hex_str = hex_str.lstrip('#') 
            r = int(hex_str[0:2], 16)
            g = int(hex_str[2:4], 16)
            b = int(hex_str[4:6], 16)
            return (b, g, r, alpha)  # OpenCV使用BGR格式 
        point_color = []
        for hex_str in colors:
            color = hex_to_bgr(hex_str)
            point_color.append(color)

        # print(point_color)
        # 在图层上绘制半透明路径点
        for (x, y) in points:
            cv_pt = plt2cv_coord((x, y), self.height)
            cv2.circle(overlay, cv_pt, radius=4, color=point_color[self.cvshow_n-1], thickness=-1)

        # 混合图层与原始图像（叠加模式）
        cv2.addWeighted(overlay, self.alpha, self.image, 1 - self.alpha, 0, dst=self.image)
        cv2.imwrite("output_transparent.png", self.image)

# 绘画初始地图
def create_map():
    # 读取图像
    image = cv2.imread(r'/home/www/auto_parking/server_ws/src/server_python/server_python/map2.png')

    # 将非方框区域转为灰度
    gray_background = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray_background = cv2.cvtColor(gray_background, cv2.COLOR_GRAY2BGR)

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # 设定蓝色HSV范围
    lower_blue = np.array([100, 100, 50])
    upper_blue = np.array([140, 255, 255])

    # 生成蓝色掩码
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # 查找轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 创建拷贝用于绘制
    output = gray_background.copy()

    # 颜色定义
    PARKING_AVAILABLE_COLOR = (0, 180, 0)  # 深绿色
    PARKING_UNAVAILABLE_COLOR = (0, 0, 200)  # 深红色
    BORDER_COLOR = (128, 128, 128)  # 灰色边框
    FILL_COLOR_AVAILABLE = (200, 255, 200)  # 浅绿填充
    FILL_COLOR_UNAVAILABLE = (220, 220, 255)  # 浅红填充

    # 首先对所有车位进行排序（从左到右，从上到下）
    # 使用中心点坐标作为排序依据
    sorted_contours = sorted(contours, key=lambda c: (cv2.boundingRect(c)[1], cv2.boundingRect(c)[0]))

    # 初始化车位编号
    parking_number = 1

    # 处理方框
    for cnt in sorted_contours:
        x, y, w, h = cv2.boundingRect(cnt)  # _,_,38,90
        center_x = x + w // 2
        center_y = y + h // 2

        # 判断是否在指定区域
        in_special_area = center_x < 300 or center_x > 780
        is_available = not in_special_area  # 指定区域为不可用

        # 方框填充
        fill_color = (255, 255, 255) if is_available else FILL_COLOR_UNAVAILABLE
        output[y:y + h, x:x + w] = fill_color

        # 绘制虚线边框
        step = 10
        for i in range(x, x + w, step):
            cv2.line(output, (i, y), (i + 5, y), BORDER_COLOR, 1)
            cv2.line(output, (i, y + h), (i + 5, y + h), BORDER_COLOR, 1)
        for j in range(y, y + h, step):
            cv2.line(output, (x, j), (x, j + 5), BORDER_COLOR, 1)
            cv2.line(output, (x + w, j), (x + w, j + 5), BORDER_COLOR, 1)
        if not is_available:
            # 绘制斜线
            cv2.line(output, (x + 5, y + 5), (x + w - 5, y + h - 5), (0, 0, 200), 1)
            cv2.line(output, (x + w - 5, y + 5), (x + 5, y + h - 5), (0, 0, 200), 1)

    # # 显示图像
    # cv2.imshow('Parking  Status with Numbering', output)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    # 保存图像
    cv2.imwrite('/home/www/auto_parking/server_ws/src/server_python/server_python/parking_status_result.png', output)

def main():
    pixel_to_meter_m=0.0564
    # 读取原始图像并转换为BGRA（添加Alpha通道）
    # image = cv2.imread("/home/www/auto_parking/server_ws/src/server_python/server_python/ParkingMap_Line.png")
    image = cv2.imread('/home/www/auto_parking/server_ws/src/server_python/map/Gray Boxed Image_screenshot_29.07.2025.png')
    # # 检查图片是否成功加载
    # if image is not None:
    #     # 获取图片尺寸信息 
    #     height, width, channels = image.shape  
        
    #     # 打印图片信息 
    #     print(f"图片尺寸（宽×高）：{width} × {height} 像素")

    image = cv2.cvtColor(image, cv2.COLOR_BGR2BGRA)  # 转换到BGRA格式
    alpha = 0.7  # 全局透明度系数（0=全透明, 1=不透明）

    points = [(14,45),(16,45),(18,45)]
    points = np.array(points)/pixel_to_meter_m
    
    paths_overlay = PathsOverlay(pixel_to_meter_m,image,alpha)
    paths_overlay.cvshow(points)

    points = [(14,40),(16,40),(18,40)]
    points = np.array(points)/pixel_to_meter_m
    paths_overlay.cvshow(points)

if __name__ == '__main__':
    # main() 
    create_map()


