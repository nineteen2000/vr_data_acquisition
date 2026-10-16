#ifndef LHANDPROLIB_H
#define LHANDPROLIB_H

#ifdef __cplusplus
extern "C" {
#endif

/// @brief 定义导出/导入符号
#if defined(_WIN32) || defined(_WIN64)
#ifdef LHandProLib_EXPORTS
#define LHANDPRO_API __declspec(dllexport)
#else
#define LHANDPRO_API __declspec(dllimport)
#endif
#else
#define LHANDPRO_API __attribute__((visibility("default")))
#endif

#include <stdbool.h>
// 错误码
#define C_LER_NONE 0               ///< 执行成功
#define C_LER_PARAMETER 1          ///< 参数错误
#define C_LER_KEY_FUNC_UNINIT 2    ///< 关键函数未初始化
#define C_LER_GET_CONFIGURATION 3  ///< 读取配置失败
#define C_LER_DATA_ANOMALY 4       ///< 数据异常
#define C_LER_COMM_CONNECT 5       ///< 通讯连接错误
#define C_LER_COMM_SEND 6          ///< 通讯发送错误
#define C_LER_COMM_RECV 7          ///< 通讯接收错误
#define C_LER_COMM_DATA_FORMAT 8   ///< 通讯数据格式错误
#define C_LER_INVALID_PATH 9       ///< 无效的文件路径
#define C_LER_LOG_SAVE_FAIL 10     ///< 日志文件保存失败
#define C_LER_UNKNOWN 999          ///< 未知错误

// 自由度枚举
#define C_LAC_DOF_6 6    ///< 6自由度
#define C_LAC_DOF_15 15  ///< 15自由度

// 通讯类型枚举
#define C_LCN_ECAT 0   ///< EtherCAT
#define C_LCN_CAN 1    ///< CAN
#define C_LCN_CANFD 2  ///< CANFD
#define C_LCN_RS485 3  ///< RS485

// 控制模式枚举
#define C_LCM_POSITION 0  ///< 位置控制
#define C_LCM_VELOCITY 1  ///< 速度控制
#define C_LCM_TORQUE 2    ///< 力矩控制
#define C_LCM_VEL_TOR 3   ///< 速度+力矩混合控制
#define C_LCM_POS_TOR 4   ///< 位置+力矩混合控制
#define C_LCM_HOME 5      ///< 回零

// 运行状态枚举
#define C_LST_STOPPED 0     ///< 正常停止状态
#define C_LST_RUNNING 1     ///< 正常运行状态
#define C_LST_ALARM 2       ///< 报警停止状态
#define C_LST_POS_LIMIT 3   ///< 正限位状态
#define C_LST_NEG_LIMIT 4   ///< 负限位状态
#define C_LST_BOTH_LIMIT 5  ///< 正负限位状态
#define C_LST_EMG_STOP 6    ///< 急停状态
#define C_LST_HOMING 7      ///< 回零运行状态

// 传感器id枚举
#define C_LSS_FINGER_1_1 1   ///< 大拇指指尖
#define C_LSS_FINGER_1_2 2   ///< 大拇指指腹
#define C_LSS_FINGER_2_1 3   ///< 食指指尖
#define C_LSS_FINGER_2_2 4   ///< 食指指腹
#define C_LSS_FINGER_3_1 5   ///< 中指指尖
#define C_LSS_FINGER_3_2 6   ///< 中指指腹
#define C_LSS_FINGER_4_1 7   ///< 无名指指尖
#define C_LSS_FINGER_4_2 8   ///< 无名指指腹
#define C_LSS_FINGER_5_1 9   ///< 小拇指指尖
#define C_LSS_FINGER_5_2 10  ///< 小拇指指腹
#define C_LSS_HAND_PALM 11   ///< 手掌
#define C_LSS_MAX_COUNT 12   ///< 最大传感器数量

// 左右手枚举
#define C_LDR_HAND_RIGHT 0  ///< 右手
#define C_LDR_HAND_LEFT 1   ///< 左手

// 基本类型定义
typedef void* lhandprolib_handle;

// 回调函数类型定义
typedef void (*LogAddCallbackWrapper)(const char*);
typedef bool (*SendDataCallbackWrapper)(unsigned int, const unsigned char*,
                                        unsigned int);
typedef bool (*ECSendDataCallbackWrapper)(const unsigned char*, unsigned int);

// 创建和销毁
LHANDPRO_API lhandprolib_handle lhandprolib_create();
LHANDPRO_API void lhandprolib_destroy(lhandprolib_handle handle);

// 初始化和关闭
LHANDPRO_API int lhandprolib_initial(lhandprolib_handle handle, int mode);
LHANDPRO_API void lhandprolib_close(lhandprolib_handle handle);

// 回调设置
LHANDPRO_API void lhandprolib_set_send_data_callback(
    lhandprolib_handle handle, SendDataCallbackWrapper callback);
LHANDPRO_API void lhandprolib_set_send_rpdo_callback(
    lhandprolib_handle handle, ECSendDataCallbackWrapper callback);
LHANDPRO_API void lhandprolib_set_log_callback(lhandprolib_handle handle,
                                               LogAddCallbackWrapper callback);

// 数据接收处理
LHANDPRO_API void lhandprolib_set_recv_data_decode(lhandprolib_handle handle,
                                                   unsigned int id,
                                                   const unsigned char* data,
                                                   unsigned char len);
LHANDPRO_API int lhandprolib_set_tpdo_data_decode(lhandprolib_handle handle,
                                                  const unsigned char* data_ptr,
                                                  int data_size);

// RPDO数据处理
LHANDPRO_API int lhandprolib_get_pre_send_rpdo_data(lhandprolib_handle handle,
                                                    unsigned char* data_ptr,
                                                    int* io_size);

// 配置相关
LHANDPRO_API int lhandprolib_set_dof_type(lhandprolib_handle handle,
                                          int dof_type);
LHANDPRO_API int lhandprolib_get_dof(lhandprolib_handle handle, int* total,
                                     int* active);
LHANDPRO_API int lhandprolib_set_hand_direction(lhandprolib_handle handle,
                                                int dir);
LHANDPRO_API int lhandprolib_get_hand_direction(lhandprolib_handle handle,
                                                int* dir);

// 电机控制
LHANDPRO_API int lhandprolib_set_control_mode(lhandprolib_handle handle, int id,
                                              int mode);
LHANDPRO_API int lhandprolib_get_control_mode(lhandprolib_handle handle, int id,
                                              int* mode);
LHANDPRO_API int lhandprolib_set_enable(lhandprolib_handle handle, int id,
                                        int enable);
LHANDPRO_API int lhandprolib_get_enable(lhandprolib_handle handle, int id,
                                        int* enable);
LHANDPRO_API int lhandprolib_get_position_reached(lhandprolib_handle handle,
                                                  int id, int* reached);
LHANDPRO_API int lhandprolib_set_clear_alarm(lhandprolib_handle handle, int id);
LHANDPRO_API int lhandprolib_get_now_alarm(lhandprolib_handle handle, int id,
                                           int* alarm);
LHANDPRO_API int lhandprolib_home_motors(lhandprolib_handle handle, int id);

// 目标设置
LHANDPRO_API int lhandprolib_set_target_angle(lhandprolib_handle handle, int id,
                                              float angle);
LHANDPRO_API int lhandprolib_get_target_angle(lhandprolib_handle handle, int id,
                                              float* angle);
LHANDPRO_API int lhandprolib_set_target_position(lhandprolib_handle handle,
                                                 int id, int position);
LHANDPRO_API int lhandprolib_get_target_position(lhandprolib_handle handle,
                                                 int id, int* position);
LHANDPRO_API int lhandprolib_set_angular_velocity(lhandprolib_handle handle,
                                                  int id, float velocity);
LHANDPRO_API int lhandprolib_get_angular_velocity(lhandprolib_handle handle,
                                                  int id, float* velocity);
LHANDPRO_API int lhandprolib_set_position_velocity(lhandprolib_handle handle,
                                                   int id, int velocity);
LHANDPRO_API int lhandprolib_get_position_velocity(lhandprolib_handle handle,
                                                   int id, int* velocity);
LHANDPRO_API int lhandprolib_set_max_current(lhandprolib_handle handle, int id,
                                             int current);
LHANDPRO_API int lhandprolib_get_max_current(lhandprolib_handle handle, int id,
                                             int* current);

// 运动控制
LHANDPRO_API int lhandprolib_move_motors(lhandprolib_handle handle, int id);
LHANDPRO_API int lhandprolib_stop_motors(lhandprolib_handle handle, int id);

// 状态获取
LHANDPRO_API int lhandprolib_get_now_status(lhandprolib_handle handle, int id,
                                            int* status);
LHANDPRO_API int lhandprolib_get_now_angle(lhandprolib_handle handle, int id,
                                           float* angle);
LHANDPRO_API int lhandprolib_get_now_position(lhandprolib_handle handle, int id,
                                              int* position);
LHANDPRO_API int lhandprolib_get_now_angular_velocity(lhandprolib_handle handle,
                                                      int id, float* velocity);
LHANDPRO_API int lhandprolib_get_now_position_velocity(
    lhandprolib_handle handle, int id, int* velocity);
LHANDPRO_API int lhandprolib_get_now_current(lhandprolib_handle handle, int id,
                                             int* current);

// 触觉传感器
LHANDPRO_API int lhandprolib_get_finger_sensor_pos(lhandprolib_handle handle,
                                                   int sensor_id,
                                                   float** x_values,
                                                   float** y_values,
                                                   int* count);
LHANDPRO_API int lhandprolib_get_finger_pressure(lhandprolib_handle handle,
                                                 int sensor_id,
                                                 float** pressure_list,
                                                 int* count);
LHANDPRO_API int lhandprolib_set_finger_pressure_reset(
    lhandprolib_handle handle);
LHANDPRO_API int lhandprolib_get_finger_normal_force(lhandprolib_handle handle,
                                                     int sensor_id,
                                                     float* normal_force);
LHANDPRO_API int lhandprolib_get_finger_tangential_force(
    lhandprolib_handle handle, int sensor_id, float* tangential_force);
LHANDPRO_API int lhandprolib_get_finger_force_direction(
    lhandprolib_handle handle, int sensor_id, float* force_direction);
LHANDPRO_API int lhandprolib_get_finger_proximity(lhandprolib_handle handle,
                                                  int sensor_id,
                                                  float* proximity);

// 日志管理
LHANDPRO_API void lhandprolib_log_on(lhandprolib_handle handle, bool on,
                                     int maxsize);
LHANDPRO_API int lhandprolib_log_save(lhandprolib_handle handle,
                                      const char* file_name);
LHANDPRO_API void lhandprolib_log_clear(lhandprolib_handle handle);

#ifdef __cplusplus
}
#endif

#endif  // LHANDPROLIB_H