from pathlib import Path
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def read(relative_path):
    return (PACKAGE_ROOT / relative_path).read_text(encoding="utf-8")


class LHandProServiceConfigTest(unittest.TestCase):
    def test_launch_exposes_channel_index_parameter(self):
        launch_file = read("launch/lhandpro.launch.py")

        self.assertIn("DeclareLaunchArgument", launch_file)
        self.assertIn("channel_index", launch_file)
        self.assertIn("LaunchConfiguration('channel_index')", launch_file)
        self.assertIn("interface_name", launch_file)
        self.assertIn("LaunchConfiguration('interface_name')", launch_file)
        self.assertIn("{'channel_index': channel_index}", launch_file)
        self.assertIn("{'interface_name': interface_name}", launch_file)

    def test_service_declares_channel_parameter_and_checks_bounds(self):
        source = read("src/hand_control_service.cpp")

        self.assertIn('declare_parameter<int>("channel_index", 1)', source)
        self.assertIn('declare_parameter<std::string>("interface_name", "")', source)
        self.assertIn("std::find(channels.begin(), channels.end(), current_interface_name_)", source)
        self.assertIn(
            "target < 0 || target >= static_cast<int>(channels.size())",
            source,
        )
        self.assertIn("channels[target]", source)

    def test_send_rpdo_callback_lifetime_is_owned_by_service(self):
        header = read("src/hand_control_service.hpp")
        source = read("src/hand_control_service.cpp")

        self.assertIn("send_rpdo_callback_", header)
        self.assertIn("send_rpdo_callback_ = [this]", source)
        self.assertIn("set_send_rpdo_callback_ex(&send_rpdo_callback_)", source)

    def test_lhandpro_library_is_resolved_from_thirdparty_first(self):
        cmake = read("CMakeLists.txt")

        self.assertIn("NO_DEFAULT_PATH", cmake)
        self.assertIn("PATHS ${THIRDPARTY_DIR}/lib", cmake)
        self.assertIn('INSTALL_RPATH', cmake)
        self.assertIn('$ORIGIN/..', cmake)
        self.assertIn('/opt/ros/humble/lib', cmake)
        self.assertIn('$ORIGIN/../../../lhandpro_interfaces/lib', cmake)

if __name__ == "__main__":
    unittest.main()
