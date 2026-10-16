"""Map VR controller buttons to LHandPro hand preset commands."""

import rclpy
from rclpy.node import Node
from vr_robot_interfaces.msg import HandCommand, VrControllerState


class VrToHandMapper(Node):
    """Publish one hand preset command on each configured button rising edge."""

    def __init__(self):
        super().__init__('vr_to_hand_mapper')
        self.declare_parameter('state_topic', '/vr_controller_state')
        self.declare_parameter('command_topic', '/hand_command')
        self.declare_parameter('button_x_hand_id', 0)
        self.declare_parameter('button_x_preset_name', 'open')
        self.declare_parameter('button_y_hand_id', 0)
        self.declare_parameter('button_y_preset_name', 'close')
        self.declare_parameter('button_a_hand_id', 1)
        self.declare_parameter('button_a_preset_name', 'open')
        self.declare_parameter('button_b_hand_id', 1)
        self.declare_parameter('button_b_preset_name', 'close')

        state_topic = self.get_parameter('state_topic').value
        command_topic = self.get_parameter('command_topic').value
        self.button_mappings = {
            'x': (
                'button_x',
                int(self.get_parameter('button_x_hand_id').value),
                str(self.get_parameter('button_x_preset_name').value),
            ),
            'y': (
                'button_y',
                int(self.get_parameter('button_y_hand_id').value),
                str(self.get_parameter('button_y_preset_name').value),
            ),
            'a': (
                'button_a',
                int(self.get_parameter('button_a_hand_id').value),
                str(self.get_parameter('button_a_preset_name').value),
            ),
            'b': (
                'button_b',
                int(self.get_parameter('button_b_hand_id').value),
                str(self.get_parameter('button_b_preset_name').value),
            ),
        }

        self.publisher_ = self.create_publisher(HandCommand, command_topic, 10)
        self.subscription = self.create_subscription(
            VrControllerState,
            state_topic,
            self.state_callback,
            10,
        )
        self.last_buttons = {name: False for name in self.button_mappings}

    def state_callback(self, msg):
        """Only publish on rising edge to avoid repeating commands at VR rate."""

        for name, (field_name, hand_id, preset_name) in self.button_mappings.items():
            self.publish_on_press(name, getattr(msg, field_name), hand_id, preset_name)

    def publish_on_press(self, name, pressed, hand_id, preset_name):
        if pressed and not self.last_buttons[name]:
            command = HandCommand()
            command.header.stamp = self.get_clock().now().to_msg()
            command.header.frame_id = 'pico_vr'
            command.hand_id = hand_id
            command.preset_name = preset_name
            command.enabled = True
            self.publisher_.publish(command)
            self.get_logger().info(f'VR {name.upper()} -> hand_id={hand_id} {preset_name}')

        self.last_buttons[name] = pressed


def main(args=None):
    rclpy.init(args=args)
    mapper = VrToHandMapper()

    try:
        rclpy.spin(mapper)
    except KeyboardInterrupt:
        pass
    finally:
        mapper.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
