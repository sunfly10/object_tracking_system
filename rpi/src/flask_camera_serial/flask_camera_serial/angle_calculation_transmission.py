'''
파리미터를 통해 설정한 객체를 PID Control을 활용한 객체 추적 장치의 핵심 코드
최종 수정일: 26.2.16
'''

import rclpy as rp
from rclpy.node import Node

from rclpy.executors import MultiThreadedExecutor
from threading import Lock

import serial
import time

from object_option_msgs.msg import ObjectOption
from yolo_detection_msgs.msg import ObjectFeatureArray
from std_msgs.msg import Float32

class AngleCalcTrans(Node):
    def __init__(self):
        super().__init__('angle_calc_trans')
        #목표각도를 시리얼 통신으로 보내려고 함
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        except serial.SerialException as e:
            self.get_logger().error(f"시리얼 포트 연결 실패: {e}")
            rp.shutdown()

        self.lock_object = Lock()
        self.current_angle_lock = Lock()
        self.current_angle_x = 90
        self.current_angle_y = 90
        self.name = ''
        self.i = -1
        self.j = -1
        self.k = -1

        self.start_time_x = None
        self.start_time_y = None
        self.start_time = None
        ''' #PID 계수(파라미터로 변경하고 싶다면 사용)
        self.declare_parameter("Kp", 0.95)
        self.declare_parameter("Ki", 0.001)
        self.declare_parameter("Kd", 0.0003)'''
        
        self.error_angle_integral_x = 0
        self.error_angle_integral_y = 0
        self.prev_error_angle_x = 0
        self.prev_error_angle_y = 0
        self.prev_time = time.time()

        self.msg_error_x = Float32()
        self.msg_current_x = Float32()
        self.msg_expected_x = Float32()

        self.msg_error_y = Float32()
        self.msg_current_y = Float32()
        self.msg_expected_y= Float32()

        self.publisher_angle_error_graph_x = self.create_publisher(Float32, '/angle_error_graph_x', 10)
        self.publisher_current_angle_graph_x = self.create_publisher(Float32, '/current_angle_x', 10)
        self.publisher_expected_angle_graph_x = self.create_publisher(Float32, '/expected_angle_x', 10)

        self.publisher_angle_error_graph_y = self.create_publisher(Float32, '/angle_error_graph_y', 10)
        self.publisher_current_angle_graph_y = self.create_publisher(Float32, '/current_angle_y', 10)
        self.publisher_expected_angle_graph_y = self.create_publisher(Float32, '/expected_angle_y', 10)

        self.subscriber_object_feature_receive = self.create_subscription(ObjectFeatureArray, '/object_feature', self.angle_callback, 10)
        self.subscriber_option_receive = self.create_subscription(ObjectOption, '/option_trans', self.option_callback, 10)
    #선택한 옵션이 바뀔 경우 update
    def option_callback(self, msg):
        with self.lock_object:
            self.name = msg.name
            #0번이 가장 큰 것으로 했기 때문에 -1을 해준다
            self.i = msg.area_number-1
            self.j = msg.center_x_number-1
            self.k = msg.center_y_number-1
    #아두이노로 부터 시리얼 통신을 통해 현재 각도를 받기, x_y가 0: x축 움직이기 1: y축 움직이기 2: 동시에 움직이
    def current_angle_receive_callback(self, x_y):
        if x_y == 0:
            self.ser.write(b'receive_x\n')
            time.sleep(0.01)  #아두이노가 응답할 시간
            response = self.ser.readline().decode().strip()
            try:
                with self.current_angle_lock:
                    self.current_angle_x = float(response)
            except ValueError:
                return
            
        elif x_y == 1:
            self.ser.write(b'receive_y\n')
            time.sleep(0.01)  #아두이노가 응답할 시간
            response = self.ser.readline().decode().strip()
            try:
                with self.current_angle_lock:
                    self.current_angle_y = float(response)
            except ValueError:
                return
            
        elif x_y == 2:
            self.ser.write(b'receive_x\n')
            time.sleep(0.01)  #아두이노가 응답할 시간
            response = self.ser.readline().decode().strip()
            try:
                with self.current_angle_lock:
                    self.current_angle_x = float(response)
            except ValueError:
                return
            
            self.ser.write(b'receive_y\n')
            time.sleep(0.01)  #아두이노가 응답할 시간
            response = self.ser.readline().decode().strip()
            try:
                with self.current_angle_lock:
                    self.current_angle_y = float(response)
            except ValueError:
                return
    #현재 갖고 있는 데이터들중 파라미터 설정에 맞는 물체를 택하기
    def angle_callback(self, msg):
        with self.lock_object:
            name = self.name
            i = self.i
            j = self.j
            k = self.k
        if len(msg.objectarray) == 0:
            return
        width_size = msg.objectarray[0].width_size
        height_size = msg.objectarray[0].height_size
        frame_x_center = width_size / 2
        frame_y_center = height_size / 2

        object_center_x = []
        object_center_y = []
        selected_object_center_x = []
        selected_object_center_y = []
        selected_object_area = []
        selected_name = []
        object_name = []
        object_area = []
        
        if frame_x_center is None or frame_y_center is None or name == '':
            return

        for obj in msg.objectarray:
            object_name.append(obj.name) 
            object_center_x.append(obj.center_x_axis)
            object_area.append(obj.object_area)
            object_center_y.append(obj.center_y_axis)

        for obj_name, cen_x, cen_y, area in zip(object_name, object_center_x, object_center_y, object_area): # 두 개 이상의 리스트(또는 반복 가능한 객체)를 병렬로 묶는 함수
            if obj_name.strip().lower() == name.strip().lower():
                selected_object_area.append(area)
                selected_object_center_x.append(cen_x)

                selected_name.append(obj_name)
                selected_object_center_y.append(cen_y)
        
        sorted_area = sorted(selected_object_area, reverse=True) #내림차순 
        sorted_center_x = sorted(selected_object_center_x, reverse=True)
        sorted_center_y =sorted(selected_object_center_y, reverse=True)

        if i >= len(sorted_area) or i < 0:
            selected_area = None
        else:
            selected_area = sorted_area[i]

        if  j >= len(sorted_center_x) or j < 0:
            selected_center_x = None
        else:
            selected_center_x = sorted_center_x[j]
            
        if  k >= len(sorted_center_y) or k < 0:
            selected_center_y = None
        else:
            selected_center_y = sorted_center_y[k]

        if selected_area is None and selected_center_x is None and selected_center_y is None:
            return
        elif selected_area is None and selected_center_y is None:
            self.target_angle_transmit(frame_x_center=frame_x_center, frame_y_center=frame_y_center, selected_center_x = selected_center_x, selected_center_y = selected_center_y, x_y=0)
        elif selected_center_x is None and selected_center_y is None:
            for k in range(len(sorted_area)):
                if selected_object_area[k] == selected_area:
                    selected_center_x = selected_object_center_x[k]
                    selected_center_y = selected_object_center_y[k]
                    self.target_angle_transmit(frame_x_center=frame_x_center, frame_y_center=frame_y_center, selected_center_x = selected_center_x, selected_center_y = selected_center_y, x_y = 2)
                    break
        elif selected_center_x is None and selected_area is None:
            self.target_angle_transmit(frame_x_center=frame_x_center, frame_y_center=frame_y_center, selected_center_x = selected_center_x, selected_center_y = selected_center_y, x_y=1)            
        else:
            return
    #물체가 정해졌다면 그 물체가 가운데에 오도록 회전을 하게 각도를 계산 및 전송
    def target_angle_transmit(self, frame_x_center, frame_y_center, selected_center_x, selected_center_y, x_y):
        with self.lock_object:
            i = self.i
            j = self.j
            k = self.k
            display_name = self.name

        self.now_time = time.time()
        dt = max(self.now_time - self.prev_time, 0.001)
        if x_y == 0:
            self.error_angle_x = (frame_x_center - selected_center_x) / frame_x_center * 27 #rpi5camera 수평 시야각 54도, 수직 시야각 41도
            self.error_derivative_x = (self.error_angle_x - self.prev_error_angle_x) / dt
            self.error_angle_integral_x += self.error_angle_x * dt
            self.error_angle_integral_x = max(min(30, self.error_angle_integral_x), -30)
        elif x_y == 1:
            self.error_angle_y = (frame_y_center - selected_center_y) / frame_y_center * 21 #rpi5camera 수평 시야각 54도, 수직 시야각 41도
            self.error_derivative_y = (self.error_angle_y - self.prev_error_angle_y) / dt
            self.error_angle_integral_y += self.error_angle_y * dt
            self.error_angle_integral_y = max(min(30, self.error_angle_integral_y), -30)
        elif x_y == 2:
            self.error_angle_x = (frame_x_center - selected_center_x) / frame_x_center * 27 #rpi5camera 수평 시야각 54도, 수직 시야각 41도
            self.error_derivative_x = (self.error_angle_x - self.prev_error_angle_x) / dt
            self.error_angle_integral_x += self.error_angle_x * dt
            self.error_angle_integral_x = max(min(30, self.error_angle_integral_x), -30)

            self.error_angle_y = (frame_y_center - selected_center_y) / frame_y_center * 21 #rpi5camera 수평 시야각 54도, 수직 시야각 41도
            self.error_derivative_y = (self.error_angle_y - self.prev_error_angle_y) / dt
            self.error_angle_integral_y += self.error_angle_y * dt
            self.error_angle_integral_y = max(min(30, self.error_angle_integral_y), -30)

        tolerance = 0.5  # 허용 오차, 0.5/27*100 / 0.5/21*100 픽셀기준
        #오차가 클 때는 계수를 크게, 작을 때는 작게하여 조건적인 움직임을 보이게 함
        def select_pid_gains(error):
            if abs(error) >= 2:
                return 0.60, 0.003, 0.0006
            else:
                return 0.60, 0.001, 0.0001
            
        if x_y == 0:
            self.Kp, self.Ki, self.Kd = select_pid_gains(self.error_angle_x)
            self.angle_calc(x_y=0)

            if abs(self.error_angle_x) < tolerance:
                if self.start_time is None:
                    self.start_time = time.time()

                self.error_angle_integral_x = 0
                self.msg_error_x.data = float(self.error_angle_x)
                self.publisher_angle_error_graph_x.publish(self.msg_error_x) #허용 오차일 때도 확인하기 위해

                if time.time() - self.start_time >= 1: 
                    self.get_logger().info(f"tolerance 이내, (name, current, error, center_x 몇 번째) : ({display_name, self.current_angle_x, self.error_angle_x, j+1})")
                    self.start_time = time.time()                
                    return
            else:
                self.start_time = None

        elif x_y == 1:
            self.Kp, self.Ki, self.Kd = select_pid_gains(self.error_angle_y)
            self.angle_calc(x_y=1)

            if abs(self.error_angle_y) < tolerance:
                if self.start_time is None:
                    self.start_time = time.time()

                self.error_angle_integral_y = 0
                self.msg_error_y.data = float(self.error_angle_y)
                self.publisher_angle_error_graph_y.publish(self.msg_error_y) #허용 오차일 때도 확인하기 위해

                if time.time() - self.start_time >= 1: 
                    self.get_logger().info(f"tolerance 이내, (name, current, error, center_y 몇 번째) : ({display_name, self.current_angle_y, self.error_angle_y, k+1})")
                    self.start_time = time.time()                
                    return
            else:
                self.start_time = None

        elif x_y == 2:
            self.Kp, self.Ki, self.Kd = select_pid_gains(self.error_angle_x)
            if abs(self.error_angle_x) < tolerance:
                if self.start_time_x is None:
                    self.start_time_x = time.time()

                self.error_angle_integral_x = 0
                self.msg_error_x.data = float(self.error_angle_x)
                self.publisher_angle_error_graph_x.publish(self.msg_error_x) #허용 오차일 때도 확인하기 위해  
                if (time.time() - self.start_time_x) >= 1: 
                    self.get_logger().info(f"tolerance 이내(width), (name, area 몇 번째) : ({display_name, i+1})")
                    self.start_time_x = time.time()
                    return
            else:
                self.start_time_x = None     
            self.angle_calc(x_y=0) 

            self.Kp, self.Ki, self.Kd = select_pid_gains(self.error_angle_y)
            if abs(self.error_angle_y) < tolerance:
                if self.start_time_y is None:
                    self.start_time_y = time.time()

                self.error_angle_integral_y = 0
                self.msg_error_y.data = float(self.error_angle_y)
                self.publisher_angle_error_graph_y.publish(self.msg_error_y) #허용 오차일 때도 확인하기 위해

                if (time.time() - self.start_time_y) >= 1: 
                    self.get_logger().info(f"tolerance 이내(height), (name, area 몇 번째) : ({display_name, i+1})")
                    self.start_time_x = time.time()
                    self.start_time_y = time.time()                            
                    return
            else:
                self.start_time_y = None
                self.start_time_x = None
            self.angle_calc(x_y=1)
    #PID CONTROL을 활용한 각도 계산
    #수식: u(t) = Kp*e(t) + Ki*∫e(t)dt + Kd*de(t)/dt
    def angle_calc(self, x_y):
        if x_y == 0:
            pid_control_x = (self.Kp * self.error_angle_x + self.Kd * self.error_derivative_x + self.Ki * self.error_angle_integral_x) 
            pid_control_x = max(-32.5, min(32.5, pid_control_x))

            with self.current_angle_lock:
                current_angle_x = self.current_angle_x

            expected_angle_x = current_angle_x - pid_control_x 
            self.ser.write(f"x:{round(expected_angle_x)}\n".encode()) #문자열을 바이트로 변환해서 전송
            time.sleep(0.002 * abs(current_angle_x - int(expected_angle_x)))

            self.current_angle_receive_callback(x_y=x_y)

            self.prev_error_angle_x = self.error_angle_x

            self.msg_error_x.data = float(self.error_angle_x)
            self.msg_current_x.data = float(current_angle_x)
            self.msg_expected_x.data = float(expected_angle_x)
        elif x_y == 1:
            pid_control_y = (self.Kp * self.error_angle_y + self.Kd * self.error_derivative_y + self.Ki * self.error_angle_integral_y) 
            pid_control_y = max(-24.925, min(24.925, pid_control_y))
            with self.current_angle_lock:
                current_angle_y = self.current_angle_y
            expected_angle_y = current_angle_y - pid_control_y 
            self.ser.write(f"y:{round(expected_angle_y)}\n".encode()) #문자열을 바이트로 변환해서 전송
            time.sleep(0.002 * abs(current_angle_y - int(expected_angle_y)))
            self.current_angle_receive_callback(x_y=x_y)
            #self.get_logger().info(f"(전송, 오차)각도, pid계수: ({expected_angle}, {error_angle}), ({self.Kp}, {self.Ki}, {self.Kd})")
            self.prev_error_angle_y = self.error_angle_y

            self.prev_time = self.now_time

            self.msg_error_y.data = float(self.error_angle_y)
            self.msg_current_y.data = float(current_angle_y)
            self.msg_expected_y.data = float(expected_angle_y)
        
        self.publisher_angle_error_graph_x.publish(self.msg_error_x)
        self.publisher_current_angle_graph_x.publish(self.msg_current_x)
        self.publisher_expected_angle_graph_x.publish(self.msg_expected_x)

        self.publisher_angle_error_graph_y.publish(self.msg_error_y)
        self.publisher_current_angle_graph_y.publish(self.msg_current_y)
        self.publisher_expected_angle_graph_y.publish(self.msg_expected_y)

def main(args=None):
    rp.init(args=args)
    node = AngleCalcTrans()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rp.shutdown()

if __name__ == '__main__':
    main()
