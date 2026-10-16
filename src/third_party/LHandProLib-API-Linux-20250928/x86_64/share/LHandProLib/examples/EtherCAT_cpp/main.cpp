#include <algorithm>
#include <chrono>
#include <ctime>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <thread>
#include <vector>

#include "EthercatMaster.h"
#include "LHandProLib.hpp"

#ifdef _WIN32
#include <conio.h>
#else
#include <sys/select.h>
#include <sys/time.h>
#include <unistd.h>
#endif

// Esc 检测函数
bool is_escape_pressed() {
#ifdef _WIN32
  if (_kbhit()) {
    int ch = _getch();
    return ch == 27;
  }
#endif
  return false;
}

// 获取当前时间戳字符串
std::string getCurrentTimestamp() {
  // 获取当前时间点
  auto now = std::chrono::system_clock::now();
  auto duration = now.time_since_epoch();

  // 拆分秒和毫秒
  auto seconds =
      std::chrono::duration_cast<std::chrono::seconds>(duration).count();
  auto milliseconds =
      std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() %
      1000;

  // 转换为本地时间
  std::time_t now_time = static_cast<std::time_t>(seconds);
  std::tm local_tm = *std::localtime(&now_time);

  // 使用字符串流格式化
  std::ostringstream oss;
  oss << std::put_time(&local_tm, "[%Y-%m-%d %H:%M:%S") << "." << std::setw(3)
      << std::setfill('0') << milliseconds << "]";

  return oss.str();
}

void printHex(const std::vector<unsigned char>& data) {
  std::cout << "Data (hex): ";
  for (const auto& byte : data) {
    std::cout << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
              << static_cast<int>(byte) << " ";
  }
  std::cout << std::dec << std::endl;
}

int main() {
  auto lhp_lib_ = std::make_shared<lhplib::LHandProLib>();
  auto ec_master_ = std::make_shared<EthercatMaster>();

  /****** 通讯设备的初始化 ******/
  std::cout << "正在使用EtherCAT通讯(100M):\n" << std::endl;

  std::vector<std::string> names = ec_master_->scanNetworkInterfaces();
  // 获取通道
  std::cout << "找到网口数量：" << names.size() << std::endl << std::endl;
  if (names.size() == 0) {
#ifdef _WIN32
    system("pause");
#else
    std::cin.get();
#endif
    return 0;
  }
  for (int i = 0; i < names.size(); ++i) {
    std::cout << "输入 " << i << " ---- 网口 " << names[i] << std::endl;
  }
  std::cout << "请选择对应网口 " << "[0 - " << names.size() - 1 << "]"
            << std::endl;
  int channel_index = 0;
  while (true) {
    std::cin >> channel_index;
    if (channel_index < 0 || channel_index >= names.size()) {
      std::cout << "请输入正确的网口选择 " << "[0 - " << names.size() - 1 << "]"
                << std::endl;
      continue;
    }
    break;
  }

  // 初始化
  if (!ec_master_->init(channel_index)) {
    std::cout << "初始化失败" << std::endl;
#ifdef _WIN32
    system("pause");
#else
    std::cin.get();
#endif
    return -1;
  }
  std::cout << "连接成功" << std::endl;

  if (!ec_master_->start()) {
    std::cout << "启动设备失败" << std::endl;
    return -1;
  }

  // 启动后台数据交换通信（非阻塞）
  ec_master_->run();

  /****** LHandProLib的初始化 ******/
  // EtherCAT的发送函数
  auto send_func = [ec_master_](const unsigned char* data, unsigned int size) {
    return ec_master_->setOutputs(data, size);
  };
  // 使用std::function包装
  std::function<bool(const unsigned char*, unsigned int)> func = send_func;
  // 处理EtherCAT发送数据的回调
  lhp_lib_->set_send_rpdo_callback_ex(&func);

  // 监控线程, 刷新解析TPDO数据
  std::atomic<bool> stop{false};
  std::thread t_monitor([=, &stop]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    while (!stop) {
      // 获取TPDO数据, 并对TPDO数据进行数据的解析
      int input_size = ec_master_->getInputSize();
      std::vector<uint8_t> in_buffer(input_size);
      if (ec_master_->getInputs(in_buffer.data(), input_size)) {
        lhp_lib_->set_tpdo_data_decode(in_buffer.data(), input_size);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });

  // 初始化LHandProLib库
  if (lhp_lib_->initial(lhplib::LCN_ECAT) != lhplib::LER_NONE) {
    std::cerr << "LHandProLib 初始化失败！" << std::endl;
    return -1;
  }

  // 获取当前自由度数
  int dof_total = 0, dof_active = 0;
  lhp_lib_->get_dof(&dof_total, &dof_active);
  std::cout << "自由度: 总共 " << dof_total << ", 主动 " << dof_active
            << std::endl;

  // 设置控制模式
  lhp_lib_->set_control_mode(0, lhplib::LCM_POSITION);
  // 设置使能模式
  lhp_lib_->set_enable(0, 1);

  std::cout << "等待使能完成\n";
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  std::cout << "正在回零\n";
  lhp_lib_->home_motors(0);
  std::this_thread::sleep_for(std::chrono::milliseconds(5000));

  std::cout << "正在循环运动\n";
  std::vector<std::vector<int>> positions = {
      {10000, 0, 0, 0, 0, 0}, {0, 10000, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0},
      {0, 0, 10000, 0, 0, 0}, {0, 0, 0, 10000, 0, 0}, {0, 0, 0, 0, 10000, 0},
      {0, 0, 0, 0, 0, 10000}};

  while (true) {  // 执行测试数据
    for (int16_t i = 0; i < positions.size(); ++i) {
      // 检查键盘输入退出条件Esc
      if (is_escape_pressed()) {
        std::cout << "Esc pressed, exiting..." << std::endl;
        goto EscPressed;
      }
      for (int16_t j = 0; j < dof_active; ++j) {
        // 设置目标位置
        lhp_lib_->set_target_position(j + 1, positions[i][j]);
        // 设置速度
        lhp_lib_->set_position_velocity(j + 1, 20000);
        // 设置最大电流限制
        lhp_lib_->set_max_current(j + 1, 800);
      }
      // 驱动所有电机运动
      int retn = lhp_lib_->move_motors();

      std::cout << "line: " << i << " positions: ";
      for (int16_t j = 0; j < dof_active; ++j) {
        std::cout << positions[i][j] << " ";
      }
      std::cout << " retn: " << retn;
      std::cout << "\n";

      if (lhplib::LER_NONE != retn)
        goto EscPressed;

      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
  }

EscPressed:
  // 退出时：退出线程
  stop = true;
  t_monitor.join();

  lhp_lib_->close();
  ec_master_->stop();

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  return 0;
}