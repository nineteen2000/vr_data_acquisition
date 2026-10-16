"""
LHandProLib库 python 运行示例
"""

import sys
import time
import threading
import keyboard

from typing import Optional
from lhandprolib_wrapper import PyLHandProLib, LHandProLibError, LCM_POSITION, LCN_ECAT, LER_NONE
from ethercat_master import EthercatMaster

# 全局变量
g_ec_master: Optional[EthercatMaster] = None


def ec_send_callback(data: bytes) -> bool:
    """EtherCAT 发送回调函数"""
    global g_ec_master
    if g_ec_master:
        return g_ec_master.setOutputs(data, len(data))
    print("EC master not initialized!")
    return False


def main():
    global g_ec_master

    # 创建 PyLHandProLib 实例
    try:
        lhp = PyLHandProLib()
    except Exception as e:
        print(f"创建 LHandPro 失败: {e}")
        return -1

    g_ec_master = EthercatMaster()

    print("正在使用EtherCAT通讯(100M):\n")

    # 扫描网口 —— 函数内部已打印描述
    names = g_ec_master.scanNetworkInterfaces()
    print(f"找到网口数量：{len(names)}\n")

    if len(names) == 0:
        input("未找到网口，按回车退出...")
        lhp.close()
        return 0

    print(f"请选择对应网口 [0 - {len(names) - 1}]")

    channel_index = 0
    while True:
        try:
            channel_index = int(input(">>> "))
            if 0 <= channel_index < len(names):
                break
            else:
                print(f"请输入 [0 - {len(names) - 1}]")
        except ValueError:
            print("请输入数字")

    # 初始化 EtherCAT
    if not g_ec_master.init(channel_index, names):
        print("初始化失败")
        input("按回车退出...")
        lhp.close()
        return -1

    print("连接成功")

    if not g_ec_master.start():
        print("启动设备失败")
        lhp.close()
        return -1

    # 启动后台 IO
    g_ec_master.run()

    # 设置回调
    lhp.set_send_rpdo_callback(ec_send_callback)

    # 监控线程：刷新解析 TPDO 数据
    stop_flag = threading.Event()

    def monitor_thread():
        time.sleep(1.0)  # 等待初始化
        while not stop_flag.is_set():
            input_size = g_ec_master.getInputSize()
            inputs = g_ec_master.getInputs(input_size)
            if inputs is not None:
                lhp.set_tpdo_data_decode(inputs)
            time.sleep(0.01)  # 10ms

    t_monitor = threading.Thread(target=monitor_thread, daemon=True)
    t_monitor.start()

    # 初始化 LHandProLib
    try:
        lhp.initial(LCN_ECAT)
    except LHandProLibError as e:
        print(f"LHandProLib 初始化失败: {e}")
        stop_flag.set()
        t_monitor.join()
        lhp.close()
        return -1

    # 获取自由度
    dof_total, dof_active = lhp.get_dof()
    print(f"自由度: 总共 {dof_total}, 主动 {dof_active}")

    # 设置控制模式和使能
    lhp.set_control_mode(0, LCM_POSITION)
    lhp.set_enable(0, True)

    print("等待使能完成")
    time.sleep(1.0)

    print("正在回零")
    lhp.home_motors(0)
    time.sleep(5.0)

    print("正在循环运动")
    positions = [
        [10000, 0, 0, 0, 0, 0],
        [0, 10000, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 10000, 0, 0, 0],
        [0, 0, 0, 10000, 0, 0],
        [0, 0, 0, 0, 10000, 0],
        [0, 0, 0, 0, 0, 10000]
    ]

    try:
        while True:
            for i, pos_list in enumerate(positions):
                # 检查 Esc 键
                if keyboard.is_pressed('esc'):
                    print("Esc pressed, exiting...")
                    raise KeyboardInterrupt

                for j in range(dof_active):
                    motor_id = j + 1
                    lhp.set_target_position(motor_id, pos_list[j])
                    lhp.set_position_velocity(motor_id, 20000)
                    lhp.set_max_current(motor_id, 800)

                lhp.move_motors(0)
                print(f"line: {i} positions: {pos_list} ✅ 运动指令发送成功")
                time.sleep(1.0)

    except KeyboardInterrupt:
        print("程序被用户中断")

    finally:
        # 退出清理
        stop_flag.set()
        if t_monitor.is_alive():
            t_monitor.join(timeout=2.0)

        lhp.close()
        g_ec_master.stop()
        time.sleep(0.1)

    return 0


if __name__ == "__main__":
    sys.exit(main())