VERSION = "0.1.0"
import warnings
warnings.warn(f"UWB version: {VERSION}, this version is not compatible with the previous version.")
import struct
import threading
import time
import serial
from functions import MovingAverageFilter

NODE = 0
ANCHOR = 1 
TAG = 2
CONSOLE = 3
MASTER = 4
SLAVE = 5
class UwbNighNode:
    def __init__(self, _id, role, filter_win_size=10) -> None:
        self._id = _id
        self.role = role
        self.raw_dis = -1
        self.filtered_dis = -1
        self.last_update_time = -1
        self.disFilter = MovingAverageFilter(filter_win_size)
        # self.history_raw_dis = Queue(maxsize=100)
        # self.history_filtered_dis = Queue(maxsize=100)
        pass
    
    def add_dis(self, dis):
        self.raw_dis = dis
        self.disFilter.add_sample(dis)
        self.filtered_dis = self.disFilter.get_value()
        self.last_update_time = time.time()
        # self.history_raw_dis.put(self.raw_dis)
        # self.history_filtered_dis.put(self.filtered_dis)
        
    def get_raw_dis(self):
        return self.raw_dis
    
    def get_filtered_dis(self):
        return self.filtered_dis
    
    
class LinkTrackAoaNode5:
    def __init__(self, role, _id, dis1, dis2, dis3, fp_rssi, rx_rssi):
        self.role = role
        self._id = _id
        self.dis1 = dis1
        self.dis2 = dis2
        self.dis3 = dis3
        # int32_t temp = (int32_t)(byte[0] << 8 | byte[1] << 16 | byte[2] << 24) / 256;
        temp = ((dis3 << 24) | (dis2 << 16) | (dis1 << 8))
        self.dis = temp/256.0/1000.0 # unit:m
        self.angle = None
        self.fp_rssi = fp_rssi
        self.rx_rssi = rx_rssi

    @classmethod
    def unpack(cls, data):
        """
        typedef struct {
            uint8_t role;
            uint32_t id;
            nint24_t dis;
            uint8_t fp_rssi;
            uint8_t rx_rssi;
        } nlt_nodeframe5_node_raw_t;
        """
        # print(len(data))
        unpacked_data = struct.unpack('<BIBBBBB', data)
        # print(unpacked_data)
        return cls(*unpacked_data)

class LinkTrackAoaNodeFrame5:
    def __init__(self, role, _id, local_time, system_time, reserved0, voltage, nodes):
        self.role = role
        self._id = _id
        self.local_time = local_time
        self.system_time = system_time
        self.reserved0 = reserved0
        self.voltage = voltage / 1000.
        self.nodes = nodes

    @classmethod
    def unpack(cls, data):
        """
        uint8_t role;
        uint32_t id;
        uint32_t local_time;
        uint32_t system_time;
        uint8_t reserved0[4];
        uint16_t voltage;
        uint8_t valid_node_count;
        """
        header_data = struct.unpack('<BIIIIHB', data[:20])
        role, _id, local_time, system_time,reserved0, voltage,valid_node_count = header_data
        # print(header_data)
        node_data = data[20:]

        nodes = []
        for i in range(valid_node_count):
            node, node_data = node_data[:10], node_data[10:]
            nodes.append(LinkTrackAoaNode5.unpack(node))

        return cls(role, _id, local_time, system_time, reserved0, voltage, nodes)

class UserPayload:
    MsgIDMap = {'neigh_dis': 0x01, 'str': 0x09}
    def __init__(self) -> None:
        pass

class UserPayloadNeighDis:
    MsgID = UserPayload.MsgIDMap['neigh_dis']
    PayloadLen = 10
    
    def __init__(self) -> None:
        # super().__init__()
        pass
    
    @classmethod
    def pack(cls, role_from, role_to, dis_raw, dis_filtered):
        payload = struct.pack('<BBBff', UserPayloadNeighDis.MsgID,role_from, role_to, dis_raw, dis_filtered)
        return payload
    
    @classmethod
    def unpack_true_payload(cls, true_payload):
        true_payload = struct.unpack('<BBff', true_payload)
        role_from, role_to, dis_raw, dis_filtered = true_payload
        return role_from, role_to, dis_raw, dis_filtered

class UserPayloadStr:
    MsgID = 0x09
    PayloadLen = 0
    
    def __init__(self) -> None:
        pass
    
    @classmethod
    def pack(cls, msg):
        payload = struct.pack('<B', UserPayloadStr.MsgID)
        payload += msg.encode('utf-8')
        return payload
    
    @classmethod
    def unpack_true_payload(cls, true_payload):
        return true_payload.decode('utf-8')
    
    
class LinkTrackUserNodeFrame1:
    """
    header   0x54
    marker   0xF1
    reserved 0x00 * 4
    remote_role u8
    remote_id u8
    datalen  u16
    data     u8*len
    checksum added
    """
    def __init__(self, role, _id, reserved0=0):
        self.header_byte = 0x54
        self.marker = 0xF1
        self.reserved = 0
        self.role = role
        self._id = _id
        self.reserved0 = reserved0
    
    def pack(self, neigh_node:UwbNighNode, remote_id):
        data_length = 6
        header = struct.pack('<BBfBBH', self.header_byte, self.marker, self.reserved, neigh_node.role, remote_id, UserPayloadNeighDis.PayloadLen)
        data = header
        data += UserPayloadNeighDis.pack(self.role,  neigh_node.role, neigh_node.get_raw_dis(), neigh_node.get_filtered_dis())
        checksum = sum(data) % 256
        data += struct.pack('<B', checksum)
        return data
    
    @classmethod
    def unpack(cls, data):
        header = struct.unpack('<BBfBBH', data[:10])
        header, marker, reserved, remote_role, remote_id, data_length = header
        role_from, role_to, dis = UserPayloadNeighDis.unpack_true_payload(data[10:10+data_length])
        return role_from, role_to, dis
        

    
class NoopLoopUWB:
    def __init__(self, dev='/dev/ttyUAB0', log_ON = False, max_neigh_num=5, send_dist=False,send_fps=20) -> None:
        self.dev = dev
        self.role = -1
        self._id = -1
        self.log_ON = log_ON
        self.send_dist = send_dist
        if self.send_dist:
            self.send_fps = send_fps
            self.time_delay_send = 1.0 / self.send_fps
        else:
            self.send_fps = None
            self.time_delay_send = None
        self.should_stop = False
        # 构造一个历史队列，用于存储历史数据
        self.node_num = -1
        self.neigh_nodes = {}
        self.ser = serial.Serial(dev, baudrate=921600, timeout=1)
        
        self.max_neigh_num = max_neigh_num
        self.adj_matrix_raw = [[0 for i in range(max_neigh_num)] for j in range(max_neigh_num)]
        self.adj_matrix_filtered = [[0 for i in range(max_neigh_num)] for j in range(max_neigh_num)] 
        
        # threads
        self.listen_t_thread = threading.Thread(target=self.listen_t)
        self.listen_t_thread.start()
        
        if self.send_dist:
            self.send_t_thread = threading.Thread(target=self.send_dist_t)
            self.send_t_thread.start()
        pass
    
    def update_dis_matrix(self, matrix, role_from, role_to, dis):
        matrix[role_from][role_to] = dis
        # matrix[role_to][role_from] = dis
        
    def send(self, msg):
        self.ser.write(msg)
    
    def send_dist_t(self):
        while self.role == -1:
            try:
                time.sleep(0.2)
                print("Waiting for nodes...")
            except KeyboardInterrupt:
                break
        self.user_frame = LinkTrackUserNodeFrame1(self.role, self._id)
        while not self.should_stop:
            try:
                for role in self.neigh_nodes:
                    self.send(self.user_frame.pack(self.neigh_nodes[role], NODE))
                    time.sleep(0.0006)
                # print(self.adj_matrix_raw)
                time.sleep(self.time_delay_send)
            except KeyboardInterrupt:
                print(f"UWB{self.role}任务结束...")
                break
        
    def listen_t(self):
        count = 0
        while not self.should_stop:
            try:
                # 读取两字节的帧头
                frame_header1 = self.ser.read(1)
                if frame_header1 == b'\x55':
                    frame_header2 = self.ser.read(1)
                    if frame_header2 == b'\x08':
                        # Frame5
                        # 读取两字节的帧长度
                        frame_length = struct.unpack('<H', self.ser.read(2))[0]
                        
                        # 读取整个帧
                        frame_data = self.ser.read(frame_length-4)
                        # print(f"frame_length: {frame_length}")
                        

                        # 解析帧
                        link_track_frame = LinkTrackAoaNodeFrame5.unpack(frame_data)
                        count = count + 1
                        
                        # 更新自身ID
                        self._id = link_track_frame._id
                        self.role = link_track_frame.role
                    
                elif frame_header1 == b'\x54':
                    # user defined msg
                    # print("User defined msg")
                    frame_header2 = self.ser.read(1)
                    if frame_header2 == b'\xF1':
                        # print("User defined msg chk2")
                        reserved = self.ser.read(4)
                        header_left = self.ser.read(4)
                        recv_role, recv_id, payload_len = struct.unpack('<BBH', header_left)
                        payload = self.ser.read(payload_len)
                        payload_id = struct.unpack('<B', payload[:1])[0]
                        if payload_id == UserPayloadNeighDis.MsgID:
                            from_role, to_role, dis_raw, dis_filtered = UserPayloadNeighDis.unpack_true_payload(payload)
                            self.update_dis_matrix(self.adj_matrix_raw, from_role, to_role, dis_raw)
                            self.update_dis_matrix(self.adj_matrix_filtered, from_role, to_role, dis_filtered)
                            if self.log_ON:
                                print(f"from_role: {from_role}, to_role: {to_role}, dis_raw: {dis_raw}, dis_filtered: {dis_filtered}")  
                        if payload_id == UserPayloadStr.MsgID:
                            msg = UserPayloadStr.unpack_true_payload(payload)
                            print(msg)
                    continue
                else:
                    continue
            except KeyboardInterrupt:
                print("任务结束...")
                break
            except: 
                print("在解析或读取时遇到错误！")
                continue
            
            try:
                # 检查是不是新的节点
                for i, node in enumerate(link_track_frame.nodes):
                    node:LinkTrackAoaNode5
                    if node.role not in self.neigh_nodes:
                        print(type(node.role))
                        self.neigh_nodes[node.role] = UwbNighNode(node._id, node.role)
                    self.neigh_nodes[node.role].add_dis(node.dis)
                    self.update_dis_matrix(self.adj_matrix_raw, self.role, node.role, node.dis)
                    self.update_dis_matrix(self.adj_matrix_filtered, self.role, node.role, self.neigh_nodes[node.role].get_filtered_dis())
                # print(count)
                if count > 5 and self.log_ON:
                    # 处理解析后的数据
                    count = 0
                    print(f"Role: {link_track_frame.role}")
                    print(f"ID: {link_track_frame._id}")
                    print(f"Local Time: {link_track_frame.local_time}")
                    print(f"System Time: {link_track_frame.system_time}")
                    print(f"Voltage: {link_track_frame.voltage}")

                    for i, node in enumerate(link_track_frame.nodes):
                        print(f"Node {i + 1} - Role: {node.role}, ID: {node._id}, Dis: {node.dis}, Angle: {node.angle}, FP RSSI: {node.fp_rssi}, RX RSSI: {node.rx_rssi}")
                    print()
            except KeyboardInterrupt:
                print('1111')
                break
            
def test_dis_filter():
    uwb = NoopLoopUWB(dev='COM3', log_ON = True)
    import matplotlib.pyplot as plt
    # 设置绘图
    plt.ion() # 打开交互模式
    fig, ax = plt.subplots()
    lengths = [] #
    while True:
        try:
            print(uwb.neigh_nodes)
            new_length = uwb.neigh_nodes[1].get_filtered_dis() # 从类中获取新的长度值
            lengths.append(new_length) # 将新的长度值添加到列表中
            
            ax.clear() # 清除之前的绘图
            ax.plot(lengths) # 绘制新的长度值
            plt.ylabel('Length')
            plt.xlabel('Time')
            plt.title('Length Over Time')
            plt.draw() # 更新绘图
            plt.pause(0.1) # 短暂暂停，以便我们可以看到绘图的更新
        except KeyboardInterrupt:
            print("---")
            break
            

    plt.ioff() # 关闭交互模式
    plt.show() # 显示最终的绘图
    pass

def test_data_trans():
    uwb1 = NoopLoopUWB('COM3', log_ON = False)
    uwb2 = NoopLoopUWB('COM5', log_ON = False)
    
    while len(uwb1.neigh_nodes) < 1:
        try:
            time.sleep(0.2)
            print("Waiting for nodes...")
        except KeyboardInterrupt:
            break
    while uwb2.role == -1:
        try:
            time.sleep(0.2)
            print("Waiting for nodes...")
        except KeyboardInterrupt:
            break
    uwb1_user_frame = LinkTrackUserNodeFrame1(uwb1.role, uwb1._id)
    
    for i in range(50):
        try:
            uwb1.send(uwb1_user_frame.pack(uwb1.neigh_nodes[uwb2.role], NODE, uwb2.role))
            # print(uwb2.adj_matrix_raw)
            time.sleep(0.2)
        except KeyboardInterrupt:
            print("任务结束...")
            break
    pass

if __name__ == "__main__":
    uwb1 = NoopLoopUWB('COM3', log_ON = False)
    uwb2 = NoopLoopUWB('COM5', log_ON = False)
    
    import tkinter as tk

    # 创建窗口
    root = tk.Tk()
    root.title("4x4 Number Matrix")

    # 初始化4x4的矩阵
    matrix = [[None for _ in range(4)] for _ in range(4)]

    # 创建一个函数来填充矩阵的值
    def fill_matrix(custom_values):
        for i in range(4):
            for j in range(4):
                if matrix[i][j] is not None:
                    matrix[i][j].delete(0, tk.END)  # 清空现有值
                else:
                    matrix[i][j] = tk.Entry(root, width=5)
                    matrix[i][j].grid(row=i, column=j)
                matrix[i][j].insert(0, str(custom_values[i][j]))

    # 创建一个函数来随机更新矩阵的值
    def update_values():
        # 这里仅为示例，实际更新逻辑根据需要实现
        fill_matrix(uwb1.adj_matrix_raw)
        root.after(200, update_values)
        
    update_values()
    # 启动事件循环
    root.mainloop()

    while True:
        try:
            time.sleep(0.2)
        except KeyboardInterrupt:
            uwb1.should_stop = True
            uwb2.should_stop = True
            break
    # test_data_trans()
    