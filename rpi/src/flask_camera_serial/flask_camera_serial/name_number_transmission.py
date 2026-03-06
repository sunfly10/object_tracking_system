"""
인식한 객체 중에서 내가 원하는 것(물체, 좌/우, 상/하, 앞/뒤)을 선택하는 코드
최종 수정일: 26.2.16
"""

import rclpy as rp
from rclpy.node import Node

from object_option_msgs.msg import ObjectOption

from rcl_interfaces.msg import SetParametersResult

class NameNumberTransmission(Node):
    def __init__(self):
        super().__init__('name_number_trans')

        self.publisher_option_trans = self.create_publisher(ObjectOption, '/option_trans', 10)
        #만들어준 파라미터(이름, 면적(앞/뒤), 좌/우)
        self.declare_parameter("name", '')
        self.declare_parameter("area_number", 0)
        self.declare_parameter("center_x_number", 0)
        self.declare_parameter("center_y_number", 0)

        self.add_on_set_parameters_callback(self.parameter_change_callback)
    #파라미터 변경 시 적용해주는 함수
    def parameter_change_callback(self, params):
        option = ObjectOption()

        for param in params:
            if param.name == "name":
                option.name = param.value
                option.area_number = self.get_parameter("area_number").value
                option.center_x_number = self.get_parameter("center_x_number").value
                option.center_y_number = self.get_parameter("center_y_number").value
            elif param.name == "area_number":
                option.area_number = param.value
                option.name = self.get_parameter("name").value
                option.center_x_number = self.get_parameter("center_x_number").value
                option.center_y_number = self.get_parameter("center_y_number").value  
            elif param.name == "center_x_number":
                option.center_x_number = param.value
                option.name = self.get_parameter("name").value
                option.area_number = self.get_parameter("area_number").value
                option.center_y_number = self.get_parameter("center_y_number").value
            elif param.name == "center_y_number":
                option.center_y_number = param.value
                option.area_number = self.get_parameter("area_number").value
                option.center_x_number = self.get_parameter("center_x_number").value
                option.name = self.get_parameter("name").value

        self.publisher_option_trans.publish(option)

        return SetParametersResult(successful=True)

def main(args=None):
    rp.init(args=args)
    node = NameNumberTransmission()
    try:
        rp.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rp.shutdown()

if __name__ == "__main__":
    main()
