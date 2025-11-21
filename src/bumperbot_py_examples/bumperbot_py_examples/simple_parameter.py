import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter

class SimpleParameter(Node):
    def __init__(self):
        super().__init__("simple_parameter")

        self.declare_parameter("param1", 32)
        self.declare_parameter("param2", "matan")

        self.add_on_set_parameters_callback(self.paramChangeCallback)

    def paramChangeCallback(self, params):
        result = SetParametersResult()

        for param in params:
            if (param.name == "param1" and param.type_ == Parameter.Type.INTEGER) or (param.name == "param2" and param.type_ == Parameter.Type.STRING):
                self.get_logger().info(f"'{param.name}' changed to {param.value}")
                result.successful = True

        return result


def main():
    rclpy.init()
    
    simple_parameter = SimpleParameter()
    rclpy.spin(simple_parameter)
    simple_parameter.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()