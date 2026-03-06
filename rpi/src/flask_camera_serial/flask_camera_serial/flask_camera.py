"""
멀티쓰레딩을 활용하여 카메라의 시각 데이터(image_raw 토픽)와 웹 모니터링(Flask Streaming)을 동시에 처리하는 코드
최종 수정일: 26.2.13
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge

import threading
from flask import Flask, Response

from picamera2 import Picamera2

latest_frame = None #최근 카메라 프레임 저장 변수
lock = threading.Lock() #ROS와 Flask에서 모두 사용(멀티테스크) --> 한쪽이 사용하면 다른쪽은 기다리게 하기 위해 LOCK 사용
app = Flask(__name__)
@app.route('/image_stream') #주소만들어주기
#웹브라우저를 활용한 실시간 이미지 스트리밍(MJPEG)
def image_stream():
    def generate():
        global latest_frame
        while True:
            with lock:
                if latest_frame is None:
                    continue
                frame_rgb = cv2.cvtColor(latest_frame, cv2.COLOR_BGR2RGB) #BGR --> RGB 변환 후 Flask로 전송(open cv image는 bgr이므로 변환을 해줘야한다)
                ret, jpeg = cv2.imencode('.jpg', frame_rgb) #구현의 용이, 전송 효율을 위해 jpg 형식 사용
                if not ret:
                    continue
                frame_bytes = jpeg.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

class RpiCamPublisher(Node):
    #라즈베리파이 카메라에서 image_raw 토픽 발행
    def __init__(self):
        super().__init__('rpi_cam_pub')
        self.publisher = self.create_publisher(Image, '/image_raw', 10)
        self.bridge = CvBridge()
    
    def publish_frame(self, frame):
        global latest_frame
        #4채널(BGRA) → 3채널(BGR) 변환(open cv는 3채널 bgr 사용)
        if frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        
        img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.publisher.publish(img_msg)
        
        with lock:
            latest_frame = frame.copy() #최신 프레임 저장
            
def flask_thread():
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False) #host='0.0.0.0':외부접속허용 / use_reloader=False:스레드중복실행방지

def main():
    rclpy.init()
    node = RpiCamPublisher()
    #Flask 백그라운드 실행
    t = threading.Thread(target=flask_thread)
    t.daemon = True
    t.start()
    #Picamera2 설정
    cam = Picamera2()
    cam_config = cam.create_video_configuration({"size": (640, 480)})
    cam.configure(cam_config)
    cam.start()

    try:
        while rclpy.ok():
            frame = cam.capture_array()
            if frame is not None:
                node.publish_frame(frame)
    except KeyboardInterrupt:
        pass
    finally:
        cam.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
