from .handle_uwb import NoopLoopUWB
import argparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import rclpy
from rclpy.node import Node
import argparse

class UWB2ROSNode(Node):
    def __init__(self, args):
        super().__init__('uwb2ros_node')
        print(args)
        # 获取参数
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=4
        )
        self.port = self.declare_parameter('uwb_port', '/dev/ttyS4').get_parameter_value().string_value
        self.uwb_raw_topic = self.declare_parameter('uwb_raw_topic', 'uwb_raw').get_parameter_value().string_value
        self.uwb_filtered_topic = self.declare_parameter('uwb_filtered_topic', 'uwb_filtered').get_parameter_value().string_value
        self.raw_dist_mtx_pub = self.create_publisher(Float32MultiArray, self.uwb_raw_topic, qos_profile)
        self.filtered_dist_mtx_pub = self.create_publisher(Float32MultiArray, self.uwb_filtered_topic, qos_profile)
        self.uwb = NoopLoopUWB(self.port, log_ON=False, max_neigh_num=10)
        self.args = args
        # 打印参数
        self.get_logger().info(f'Using port: {self.port}')
        self.get_logger().info(f'Publishing UWB raw data to: {self.uwb_raw_topic}')
        self.get_logger().info(f'Publishing UWB filtered data to: {self.uwb_filtered_topic}')
    
    def mat2ros_mat(self, matrix):
        # 创建Float32MultiArray消息
        matrix_msg = Float32MultiArray()

        # 设置矩阵维度信息（假设是n*n矩阵）
        dim = MultiArrayDimension()
        dim.label = "height"
        dim.size = len(matrix)
        dim.stride = len(matrix) * len(matrix[0])
        matrix_msg.layout.dim.append(dim)

        dim = MultiArrayDimension()
        dim.label = "width"
        dim.size = len(matrix[0])
        dim.stride = len(matrix[0])
        matrix_msg.layout.dim.append(dim)

        # 将矩阵数据转换为一维列表并赋值给消息的data字段
        # print([item for row in matrix for item in row])
        matrix_msg.data = [float(item) for row in matrix for item in row]
        
        return matrix_msg

    def run(self):
        while rclpy.ok():
            try:
                # 读取数据
                if self.uwb._id == -1:
                    time.sleep(0.1)
                    continue
                raw_dist_mtx = self.uwb.get_raw_dist_matrix()
                filtered_dist_mtx = self.uwb.get_filtered_dist_matrix()
                if raw_dist_mtx is None:
                    continue
                # 发布数据
                raw_dist_matrix_msg = self.mat2ros_mat(raw_dist_mtx)
                filtered_dist_matrix_msg = self.mat2ros_mat(filtered_dist_mtx)
                
                self.raw_dist_mtx_pub.publish(raw_dist_matrix_msg)
                self.filtered_dist_mtx_pub.publish(filtered_dist_matrix_msg)
                time.sleep(0.02)
            except KeyboardInterrupt:
                print("任务结束")
                break

def main(args=None):
    rclpy.init()

    # 设置命令行参数解析
    # parser = argparse.ArgumentParser()
    # parser.add_argument('--uwb_port', type=str,           default='/dev/ttyS4', help='串口号')
    # parser.add_argument('--uwb_raw_topic', type=str,      default='uwb_raw', help='UWB原始数据的topic名称')
    # parser.add_argument('--uwb_filtered_topic', type=str, default='uwb_filtered', help='UWB过滤后数据的topic名称')
    
    # 如果从终端启动，通过argparse获取参数
    # if args is None:
    #     args = parser.parse_args()
    # else:
    #     args = parser.parse_args(args[1:])

    uwb2ros_node = UWB2ROSNode(args)
    
    try:
        uwb2ros_node.run()
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
