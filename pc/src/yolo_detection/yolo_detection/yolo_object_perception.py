'''
YOLOV8를 활용해 데이터를 얻은 후 그 데이터를 활용해 필요로 하는 특정 데이터(X/Y중심값, 넓이, 이름, frame의 w/h 정보)를 얻는 코드이다
최종 수정일: 26.2.16
'''

import rclpy as rp
from rclpy.node import Node

from sensor_msgs.msg import Image #해상도, 인코딩 방식
from cv_bridge import CvBridge

from yolo_detection_msgs.msg import ObjectFeature, ObjectFeatureArray
from ultralytics import YOLO

from rclpy.executors import MultiThreadedExecutor
from threading import Lock

class YoloObjectPerception(Node):
    def __init__(self):
        super().__init__('object_perception')

        self.subscriber_rpicam2yolo=self.create_subscription(Image, '/image_raw', self.yolo_callback, 10)
        self.publisher_object_feature=self.create_publisher(ObjectFeatureArray, '/object_feature', 10) 

        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')

        self.frame_lock = Lock()
        self.frame = None

        self.create_timer(0.4, self.run_yolo)
    #image_raw를 받을 때마다 opencv용 이미지로 변환
    def yolo_callback(self, msg):
        with self.frame_lock:
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') 

    def run_yolo(self):
        with self.frame_lock:
            frame = self.frame

        if frame is None:
            return
        
        height, width, _ = frame.shape

        results = self.model(frame)
        objarray = ObjectFeatureArray()

        #감지한 물체 이름 및 중심 좌표
        for box in results[0].boxes:
            cls_id = int(box.cls[0]) #숫자만 꺼내는 작업
            cls_name = self.model.names[cls_id] #실제 name으로 변경
            
            obj = ObjectFeature()
            obj.width_size = width
            obj.height_size = height

            x1, y1, x2, y2 = box.xyxy[0]
            obj.center_x_axis = float((x1 + x2) / 2)
            obj.center_y_axis = float((y1 + y2) / 2)
            obj.name = cls_name
            obj.object_area = float(abs(((x2-x1)*(y2-y1)))) #크기로 물체 결정하기 위해

            objarray.objectarray.append(obj)
            self.get_logger().info(f'name, center axis: {obj.name, obj.center_x_axis, obj.center_y_axis}')
  
        self.publisher_object_feature.publish(objarray)


def main(args=None):
    rp.init(args=args)
    node = YoloObjectPerception()
    executor = MultiThreadedExecutor(num_threads = 2)
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
