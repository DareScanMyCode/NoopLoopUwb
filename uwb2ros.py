from handle_uwb import NoopLoopUWB
import argparse
import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

def mat2ros_mat(matrix):
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
    matrix_msg.data = [item for row in matrix for item in row]
    
    return matrix_msg


def uwv2ros(args):
    
    # 矩阵的publisher
    raw_dist_mtx_pub = rospy.Publisher(args.uwb_raw_topic, Float32MultiArray, queue_size=10)
    filtered_dist_mtx_pub = rospy.Publisher(args.uwb__filtered_topic, Float32MultiArray, queue_size=10)
    
    # 实例化类
    uwb = NoopLoopUWB(args.port, log_ON=True)
    
    while True:
        try:
            # 读取数据
            raw_dist_mtx = uwb.get_raw_dist_matrix()
            filtered_dist_mtx = uwb.get_filtered_dist_matrix()
            # 发布数据
            raw_dist_matrix_msg = mat2ros_mat(raw_dist_mtx)
            filtered_dist_matrix_msg = mat2ros_mat(filtered_dist_mtx)
            
            raw_dist_mtx_pub.publish(raw_dist_matrix_msg)
            filtered_dist_mtx_pub.publish(filtered_dist_matrix_msg)
        except KeyboardInterrupt:
            print("任务结束")
            break
        
        
if __name__ == "__main__":
    # 读取参数：串口号、topic名称
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=str, default='/dev/ttyS4', help='串口号')
    parser.add_argument('--uwb_raw_topic', type=str, default='uwb_raw', help='topic名称')
    parser.add_argument('--uwb__filtered_topic', type=str, default='uwb_filtered', help='topic名称')
    args = parser.parse_args()
    
    # initiate node
    rospy.init_node('uwb2ros', anonymous=True)
    
    uwv2ros(args)
    
    