#ifndef H_LASER_DATA_JSON
#define H_LASER_DATA_JSON

#include "egsl/gsl_eigen.h"
#include "laser_data.h"
#include "algos.h"

/**
 * @brief 对laser_data进行赋值，用于线上机器人数据生成icp匹配需要的数据
 * @param[out]   ldp        指向laser_data的指针
 * @param[in]    range[]    每个点到激光中心的距离，即其读数
 * @param[in]    flags[]    每个点是否可用的flag
 * @param[in]    odom[]     里程计的值，这里设x,y为0, theta为imu的yaw角
 * @param[in]    nrays      激光每帧的点数
 * @param[in]    frame_id   帧数id
 */
void set_laser_frame(LDP ldp, const int* range, const char* flags,
                     const double* odom, int nrays, const unsigned long frame_id,
                     const long long time_stamp);

/**
 * @brief 对laser_data进行赋值，用于线下从数据集中读取数据
 * @param[out]  ldp         指向laser_data的指针
 * @param[in]   file_path   数据集路径，需以"/"结尾
 * @param[in]   nrays       激光每帧的点数
 * @param[in]   frame_id    帧数id
 *
 * @return  成功返回1，文件不存在会造成失败
 */
bool ld_from_yogo_stream(LDP ldp, const char* file_path,
                         const int nrays, const unsigned long frame_id);

/**
 * @brief 从关键帧中对laser_data进行赋值，用于图优化中提取新边时做icp匹配
 * @param[out]  ldp     指向laser_data的指针
 * @param[in]   laserD  从关键帧容器里传入的laser_data的引用
 */
void ld_from_keyframe(LDP ldp, const laser_data& laserD);

#endif
