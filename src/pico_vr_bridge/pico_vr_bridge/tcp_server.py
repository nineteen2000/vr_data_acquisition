"""TCP服务接收来自PICO App的数据,按行拆包后发布到/vr_raw_data."""

import socket
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VrSocketServer(Node):
    """把PICO App的TCP文本流桥接为ROS2 String话题."""

    def __init__(self):
        super().__init__('vr_socket_server')
        # PICO App当前已验证默认发往8018端口,每帧以换行符结束.
        self.declare_parameter('server_host', '0.0.0.0')
        self.declare_parameter('server_port', 8018)
        self.declare_parameter('raw_topic', '/vr_raw_data')

        self.server_host = self.get_parameter('server_host').get_parameter_value().string_value
        self.server_port = self.get_parameter('server_port').get_parameter_value().integer_value
        raw_topic = self.get_parameter('raw_topic').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(String, raw_topic, 10)
        self.stop_event = threading.Event()
        self.server_socket = None

        # socket.accept/recv是阻塞调用,放到后台线程避免阻塞rclpy.spin().
        self.start_server_thread()

    def start_server(self):
        """监听TCP客户端;客户端断开后回到accept等待下一次连接."""

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
            self.server_socket = server_socket
            server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server_socket.settimeout(1.0)
            server_socket.bind((self.server_host, self.server_port))
            server_socket.listen(1)
            self.get_logger().info(
                f'Listening for PICO VR data on {self.server_host}:{self.server_port}'
            )

            while not self.stop_event.is_set():
                try:
                    conn, addr = server_socket.accept()
                except socket.timeout:
                    continue
                except OSError:
                    break

                self.get_logger().info(f'PICO VR client connected: {addr}')
                self.handle_client(conn, addr)

    def handle_client(self, conn, addr):
        """接收字节流,只发布完整的换行分隔帧."""

        buffer = ''
        frame_count = 0
        with conn:
            conn.settimeout(1.0)
            while not self.stop_event.is_set():
                try:
                    data = conn.recv(4096)
                except socket.timeout:
                    continue
                except OSError:
                    break

                if not data:
                    break

                buffer += data.decode('utf-8', errors='replace')
                # TCP没有消息边界;这里把半包留在buffer里,只发布完整的一行.
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    if not line:
                        continue

                    msg = String()
                    msg.data = line
                    self.publisher_.publish(msg)
                    frame_count += 1

        self.get_logger().info(f'PICO VR client disconnected: {addr}, frames={frame_count}')

    def start_server_thread(self):
        """启动后台TCP服务线程."""

        server_thread = threading.Thread(target=self.start_server, daemon=True)
        server_thread.start()

    def stop_server(self):
        """请求线程退出,并关闭监听socket以打断accept等待."""

        self.stop_event.set()
        if self.server_socket is not None:
            self.server_socket.close()


def main(args=None):
    rclpy.init(args=args)
    vr_socket_server = VrSocketServer()

    try:
        rclpy.spin(vr_socket_server)
    except KeyboardInterrupt:
        pass
    finally:
        vr_socket_server.stop_server()
        vr_socket_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
