#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys
import select
import termios
import tty
import signal

# 速度配置
LINEAR_SPEED = 0.5    # m/s
ANGULAR_SPEED = 1.0   # rad/s
KEY_INTERVAL = 0.05   # 按键检测间隔(s)

# 按键映射: (linear_x, linear_y, angular_z)
KEY_MAP = {
    'w': (LINEAR_SPEED, 0.0, 0.0),
    's': (-LINEAR_SPEED, 0.0, 0.0),
    'a': (0.0, LINEAR_SPEED, 0.0),
    'd': (0.0, -LINEAR_SPEED, 0.0),
    'q': (0.0, 0.0, ANGULAR_SPEED),
    'e': (0.0, 0.0, -ANGULAR_SPEED),
    ' ': (0.0, 0.0, 0.0)
}

EXIT_FLAG = False
OLD_TERM = None
LAST_KEY = ''

def restore_terminal():
    """恢复终端属性"""
    if OLD_TERM and sys.stdin.isatty():
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, OLD_TERM)
    print("\n终端已恢复，节点退出")

def signal_handler(sig, frame):
    """信号处理: 响应退出"""
    global EXIT_FLAG
    EXIT_FLAG = True
    print("\n收到退出信号，清理资源中...")

def set_terminal_raw():
    """设置终端为cbreak模式"""
    if not sys.stdin.isatty():
        raise OSError("非TTY终端，不支持按键检测")
    
    global OLD_TERM
    OLD_TERM = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

def get_pressed_key():
    """非阻塞读取当前按键"""
    pressed = []
    while select.select([sys.stdin], [], [], 0)[0]:
        key = sys.stdin.read(1)
        if ord(key) == 3:
            global EXIT_FLAG
            EXIT_FLAG = True
            return ''
        pressed.append(key.lower())
    
    return pressed[-1] if pressed else ''

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_twist_keyboard')
        
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.lin_x = 0.0
        self.lin_y = 0.0
        self.ang_z = 0.0
        
        self.timer = self.create_timer(KEY_INTERVAL, self.timer_cb)
        self.print_usage()

    def print_usage(self):
        """打印控制说明"""
        print("="*50)
        print("ROS2 键盘控制节点")
        print(f"线速度: ±{LINEAR_SPEED}m/s, 角速度: ±{ANGULAR_SPEED}rad/s")
        print("控制键: W-前进 S-后退 A-左移 D-右移")
        print("        Q-左转 E-右转 空格-停止")
        print("        Ctrl+C-退出")
        print("="*50)

    def timer_cb(self):
        """定时器回调: 检测按键并发布速度"""
        global LAST_KEY
        key = get_pressed_key()

        if EXIT_FLAG:
            self.destroy_node()
            rclpy.shutdown()
            return

        if key in KEY_MAP:
            self.lin_x, self.lin_y, self.ang_z = KEY_MAP[key]
            if key != LAST_KEY:
                print(f"按键[{key}]: 线速度(x={self.lin_x:.2f}, y={self.lin_y:.2f}) 角速度(z={self.ang_z:.2f})")
                LAST_KEY = key
        else:
            self.lin_x = self.lin_y = self.ang_z = 0.0
            if LAST_KEY != '':
                print("无按键，速度清零")
                LAST_KEY = ''

        # 构造并发布消息
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        
        msg.twist.linear.x = self.lin_x
        msg.twist.linear.y = self.lin_y
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = self.ang_z
        
        self.pub.publish(msg)

    def destroy_node(self):
        """销毁节点前发布停止指令"""
        stop_msg = TwistStamped()
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        stop_msg.header.frame_id = "base_link"
        self.pub.publish(stop_msg)
        print("发布停止指令，节点销毁")
        super().destroy_node()

def main(args=None):
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    rclpy.init(args=args)

    try:
        set_terminal_raw()
        node = TeleopNode()
        
        while rclpy.ok() and not EXIT_FLAG:
            rclpy.spin_once(node, timeout_sec=KEY_INTERVAL)
        
    except Exception as e:
        print(f"\n运行异常: {e}")
    finally:
        restore_terminal()
        if 'node' in locals():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("节点退出完成")

if __name__ == '__main__':
    main()