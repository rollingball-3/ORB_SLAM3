/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <functional>
#include <limits>

// HDF5库
#include <H5Cpp.h>

// 添加OpenCV头文件用于图像处理
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

// ORB-SLAM3库
#include <System.h>
#include "ImuTypes.h"

using namespace std;
using namespace H5;

// 单个IMU测量数据结构
struct ImuMeasurement {
    double timestamp;  // 时间戳（秒）
    cv::Point3f gyro;  // 陀螺仪数据 (gx, gy, gz)
    cv::Point3f acc;   // 加速度计数据 (ax, ay, az)
};

// 图像帧数据结构
struct ImageFrame {
    double timestamp;  // 时间戳（秒）
    cv::Mat image;     // 图像数据
};

// 用于存储HDF5文件中数据的结构
struct OrbbecData {
    vector<ImageFrame> frames;    // 图像帧（含时间戳）
    vector<ImuMeasurement> imus;  // IMU测量（含时间戳）
};

// 添加插值函数
// 在两个值之间进行线性插值
double linearInterpolate(double x, double x1, double y1, double x2, double y2) {
    if (x2 == x1) return y1;  // 避免除以零
    return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
}

// 新增绘制原始IMU数据图表函数
void PlotRawIMUData(
    const vector<pair<double, array<double, 3>>>& gyro_measurements,
    const vector<pair<double, array<double, 3>>>& accel_measurements,
    const string& output_path) {
    
    if (gyro_measurements.empty() || accel_measurements.empty()) {
        cerr << "No raw IMU data to plot" << endl;
        return;
    }
    
    cout << "Generating raw IMU data plot to " << output_path << "..." << endl;
    
    // 定义图表尺寸和边距
    const int width = 1200;
    const int height = 900;
    const int margin = 50;
    const int plot_width = width - 2 * margin;
    const int plot_height = height - 2 * margin;
    
    // 创建白色背景图像
    cv::Mat plot(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
    
    // 确定时间范围 - 使用两组数据的最小和最大时间戳
    double gyro_min_time = gyro_measurements.front().first;
    double gyro_max_time = gyro_measurements.back().first;
    double accel_min_time = accel_measurements.front().first;
    double accel_max_time = accel_measurements.back().first;
    
    double min_time = std::min(gyro_min_time, accel_min_time);
    double max_time = std::max(gyro_max_time, accel_max_time);
    double time_range = max_time - min_time;
    
    // 找到陀螺仪和加速度计数据的最小最大值
    double min_gyro = std::numeric_limits<double>::max();
    double max_gyro = std::numeric_limits<double>::lowest();
    double min_acc = std::numeric_limits<double>::max();
    double max_acc = std::numeric_limits<double>::lowest();
    
    // 计算陀螺仪数据范围
    for (const auto& gyro : gyro_measurements) {
        // 更新陀螺仪最小值
        if (gyro.second[0] < min_gyro) min_gyro = gyro.second[0];
        if (gyro.second[1] < min_gyro) min_gyro = gyro.second[1];
        if (gyro.second[2] < min_gyro) min_gyro = gyro.second[2];
        
        // 更新陀螺仪最大值
        if (gyro.second[0] > max_gyro) max_gyro = gyro.second[0];
        if (gyro.second[1] > max_gyro) max_gyro = gyro.second[1];
        if (gyro.second[2] > max_gyro) max_gyro = gyro.second[2];
    }
    
    // 计算加速度计数据范围
    for (const auto& accel : accel_measurements) {
        // 更新加速度计最小值
        if (accel.second[0] < min_acc) min_acc = accel.second[0];
        if (accel.second[1] < min_acc) min_acc = accel.second[1];
        if (accel.second[2] < min_acc) min_acc = accel.second[2];
        
        // 更新加速度计最大值
        if (accel.second[0] > max_acc) max_acc = accel.second[0];
        if (accel.second[1] > max_acc) max_acc = accel.second[1];
        if (accel.second[2] > max_acc) max_acc = accel.second[2];
    }
    
    // 为陀螺仪和加速度计数据添加5%的边缘空间
    double gyro_range = max_gyro - min_gyro;
    min_gyro -= gyro_range * 0.05;
    max_gyro += gyro_range * 0.05;
    
    double acc_range = max_acc - min_acc;
    min_acc -= acc_range * 0.05;
    max_acc += acc_range * 0.05;
    
    // 防止数据范围为0（所有值相同）
    if (fabs(gyro_range) < 1e-6) {
        min_gyro -= 0.1;
        max_gyro += 0.1;
    }
    if (fabs(acc_range) < 1e-6) {
        min_acc -= 0.1;
        max_acc += 0.1;
    }
    
    // 绘制图表边框
    cv::rectangle(plot, cv::Point(margin, margin), cv::Point(width - margin, height - margin), 
                 cv::Scalar(0, 0, 0), 1);
    
    // 绘制图表标题
    std::string title = "Raw IMU Data Plot";
    int font_face = cv::FONT_HERSHEY_SIMPLEX;
    double font_scale = 0.8;
    int thickness = 2;
    cv::Size text_size = cv::getTextSize(title, font_face, font_scale, thickness, nullptr);
    cv::putText(plot, title, cv::Point((width - text_size.width) / 2, margin / 2 + text_size.height / 2),
               font_face, font_scale, cv::Scalar(0, 0, 0), thickness);
    
    // 分成两个子图：陀螺仪和加速度计
    int half_height = plot_height / 2;
    
    // 绘制陀螺仪子图标题
    std::string gyro_title = "Gyroscope Data (rad/s)";
    text_size = cv::getTextSize(gyro_title, font_face, font_scale * 0.7, thickness, nullptr);
    cv::putText(plot, gyro_title, cv::Point(margin, margin - 10),
               font_face, font_scale * 0.7, cv::Scalar(0, 0, 0), 1);
    
    // 绘制加速度计子图标题
    std::string acc_title = "Accelerometer Data (m/s^2)";
    text_size = cv::getTextSize(acc_title, font_face, font_scale * 0.7, thickness, nullptr);
    cv::putText(plot, acc_title, cv::Point(margin, margin + half_height - 10),
               font_face, font_scale * 0.7, cv::Scalar(0, 0, 0), 1);
    
    // 绘制时间轴标签
    std::string time_label = "Time (s)";
    text_size = cv::getTextSize(time_label, font_face, font_scale * 0.7, thickness, nullptr);
    cv::putText(plot, time_label, cv::Point((width - text_size.width) / 2, height - margin / 3),
               font_face, font_scale * 0.7, cv::Scalar(0, 0, 0), 1);
    
    // 绘制x轴（时间）刻度
    const int time_ticks = 10;
    for (int i = 0; i <= time_ticks; i++) {
        double tick_time = min_time + (time_range * i) / time_ticks;
        int x_pos = margin + (i * plot_width) / time_ticks;
        
        // 绘制刻度线
        cv::line(plot, cv::Point(x_pos, height - margin), cv::Point(x_pos, height - margin + 5), 
                cv::Scalar(0, 0, 0), 1);
        cv::line(plot, cv::Point(x_pos, margin + half_height), cv::Point(x_pos, margin + half_height + 5), 
                cv::Scalar(0, 0, 0), 1);
        
        // 绘制刻度值
        std::stringstream tick_label;
        tick_label.precision(1);
        tick_label << std::fixed << tick_time - min_time;
        text_size = cv::getTextSize(tick_label.str(), font_face, font_scale * 0.4, thickness, nullptr);
        cv::putText(plot, tick_label.str(), 
                   cv::Point(x_pos - text_size.width / 2, height - margin + 20),
                   font_face, font_scale * 0.4, cv::Scalar(0, 0, 0), 1);
    }
    
    // 绘制陀螺仪y轴刻度
    const int gyro_ticks = 5;
    for (int i = 0; i <= gyro_ticks; i++) {
        double tick_value = min_gyro + (max_gyro - min_gyro) * i / gyro_ticks;
        int y_pos = margin + half_height - (i * half_height) / gyro_ticks;
        
        // 绘制刻度线
        cv::line(plot, cv::Point(margin - 5, y_pos), cv::Point(margin, y_pos), 
                cv::Scalar(0, 0, 0), 1);
        
        // 绘制刻度值
        std::stringstream tick_label;
        tick_label.precision(2);
        tick_label << std::fixed << tick_value;
        text_size = cv::getTextSize(tick_label.str(), font_face, font_scale * 0.4, thickness, nullptr);
        cv::putText(plot, tick_label.str(), 
                   cv::Point(margin - text_size.width - 10, y_pos + text_size.height / 4),
                   font_face, font_scale * 0.4, cv::Scalar(0, 0, 0), 1);
    }
    
    // 绘制加速度计y轴刻度
    const int acc_ticks = 5;
    for (int i = 0; i <= acc_ticks; i++) {
        double tick_value = min_acc + (max_acc - min_acc) * i / acc_ticks;
        int y_pos = margin + half_height + half_height - (i * half_height) / acc_ticks;
        
        // 绘制刻度线
        cv::line(plot, cv::Point(margin - 5, y_pos), cv::Point(margin, y_pos), 
                cv::Scalar(0, 0, 0), 1);
        
        // 绘制刻度值
        std::stringstream tick_label;
        tick_label.precision(2);
        tick_label << std::fixed << tick_value;
        text_size = cv::getTextSize(tick_label.str(), font_face, font_scale * 0.4, thickness, nullptr);
        cv::putText(plot, tick_label.str(), 
                   cv::Point(margin - text_size.width - 10, y_pos + text_size.height / 4),
                   font_face, font_scale * 0.4, cv::Scalar(0, 0, 0), 1);
    }
    
    // 定义线条颜色
    cv::Scalar color_gx(255, 0, 0);   // 蓝色
    cv::Scalar color_gy(0, 255, 0);   // 绿色
    cv::Scalar color_gz(0, 0, 255);   // 红色
    cv::Scalar color_ax(255, 0, 255); // 紫色
    cv::Scalar color_ay(255, 255, 0); // 青色
    cv::Scalar color_az(0, 128, 255); // 橙色
    
    // 绘制图例
    const int legend_x = width - margin - 150;
    const int legend_y = margin + 20;
    const int legend_spacing = 25;
    
    // 陀螺仪图例
    cv::line(plot, cv::Point(legend_x, legend_y), cv::Point(legend_x + 20, legend_y), 
            color_gx, 2);
    cv::putText(plot, "Gyro X", cv::Point(legend_x + 25, legend_y + 5),
               font_face, font_scale * 0.5, color_gx, 1);
    
    cv::line(plot, cv::Point(legend_x, legend_y + legend_spacing), cv::Point(legend_x + 20, legend_y + legend_spacing), 
            color_gy, 2);
    cv::putText(plot, "Gyro Y", cv::Point(legend_x + 25, legend_y + legend_spacing + 5),
               font_face, font_scale * 0.5, color_gy, 1);
    
    cv::line(plot, cv::Point(legend_x, legend_y + 2 * legend_spacing), cv::Point(legend_x + 20, legend_y + 2 * legend_spacing), 
            color_gz, 2);
    cv::putText(plot, "Gyro Z", cv::Point(legend_x + 25, legend_y + 2 * legend_spacing + 5),
               font_face, font_scale * 0.5, color_gz, 1);
    
    // 加速度计图例
    cv::line(plot, cv::Point(legend_x, legend_y + half_height), cv::Point(legend_x + 20, legend_y + half_height), 
            color_ax, 2);
    cv::putText(plot, "Accel X", cv::Point(legend_x + 25, legend_y + half_height + 5),
               font_face, font_scale * 0.5, color_ax, 1);
    
    cv::line(plot, cv::Point(legend_x, legend_y + half_height + legend_spacing), 
            cv::Point(legend_x + 20, legend_y + half_height + legend_spacing), 
            color_ay, 2);
    cv::putText(plot, "Accel Y", cv::Point(legend_x + 25, legend_y + half_height + legend_spacing + 5),
               font_face, font_scale * 0.5, color_ay, 1);
    
    cv::line(plot, cv::Point(legend_x, legend_y + half_height + 2 * legend_spacing), 
            cv::Point(legend_x + 20, legend_y + half_height + 2 * legend_spacing), 
            color_az, 2);
    cv::putText(plot, "Accel Z", cv::Point(legend_x + 25, legend_y + half_height + 2 * legend_spacing + 5),
               font_face, font_scale * 0.5, color_az, 1);
    
    // 准备绘制数据点
    std::vector<cv::Point> points_gx, points_gy, points_gz;
    std::vector<cv::Point> points_ax, points_ay, points_az;
    
    // 计算陀螺仪数据点坐标
    for (const auto& gyro : gyro_measurements) {
        double normalized_time = (gyro.first - min_time) / time_range;
        int x = margin + static_cast<int>(normalized_time * plot_width);
        
        // 计算y坐标
        int gy_gx = margin + half_height - static_cast<int>((gyro.second[0] - min_gyro) / (max_gyro - min_gyro) * half_height);
        int gy_gy = margin + half_height - static_cast<int>((gyro.second[1] - min_gyro) / (max_gyro - min_gyro) * half_height);
        int gy_gz = margin + half_height - static_cast<int>((gyro.second[2] - min_gyro) / (max_gyro - min_gyro) * half_height);
        
        // 添加到点集
        points_gx.push_back(cv::Point(x, gy_gx));
        points_gy.push_back(cv::Point(x, gy_gy));
        points_gz.push_back(cv::Point(x, gy_gz));
    }
    
    // 计算加速度计数据点坐标
    for (const auto& accel : accel_measurements) {
        double normalized_time = (accel.first - min_time) / time_range;
        int x = margin + static_cast<int>(normalized_time * plot_width);
        
        // 计算y坐标
        int gy_ax = margin + half_height * 2 - static_cast<int>((accel.second[0] - min_acc) / (max_acc - min_acc) * half_height);
        int gy_ay = margin + half_height * 2 - static_cast<int>((accel.second[1] - min_acc) / (max_acc - min_acc) * half_height);
        int gy_az = margin + half_height * 2 - static_cast<int>((accel.second[2] - min_acc) / (max_acc - min_acc) * half_height);
        
        // 添加到点集
        points_ax.push_back(cv::Point(x, gy_ax));
        points_ay.push_back(cv::Point(x, gy_ay));
        points_az.push_back(cv::Point(x, gy_az));
    }
    
    // 绘制数据线条 - 使用polylines可以更高效地绘制多条线
    cv::polylines(plot, points_gx, false, color_gx, 1, cv::LINE_AA);
    cv::polylines(plot, points_gy, false, color_gy, 1, cv::LINE_AA);
    cv::polylines(plot, points_gz, false, color_gz, 1, cv::LINE_AA);
    
    cv::polylines(plot, points_ax, false, color_ax, 1, cv::LINE_AA);
    cv::polylines(plot, points_ay, false, color_ay, 1, cv::LINE_AA);
    cv::polylines(plot, points_az, false, color_az, 1, cv::LINE_AA);
    
    // 添加数据统计信息
    std::stringstream stats_text;
    stats_text.precision(2);
    stats_text << std::fixed
               << "Gyro data points: " << gyro_measurements.size()
               << ", Accel data points: " << accel_measurements.size()
               << " | Gyro range: [" << min_gyro << ", " << max_gyro << "]"
               << ", Accel range: [" << min_acc << ", " << max_acc << "]";
    
    text_size = cv::getTextSize(stats_text.str(), font_face, font_scale * 0.4, 1, nullptr);
    cv::putText(plot, stats_text.str(), 
               cv::Point((width - text_size.width) / 2, height - 10),
               font_face, font_scale * 0.4, cv::Scalar(0, 0, 0), 1);
    
    // 保存图表到文件
    try {
        cv::imwrite(output_path, plot);
        cout << "Raw IMU data plot saved to " << output_path << endl;
    } catch (const cv::Exception& e) {
        cerr << "Error saving IMU plot: " << e.what() << endl;
    }
} 


// 加载和解析HDF5文件
bool LoadOrbbecH5Data(const string &h5_path, OrbbecData &data)
{
    try {
        // 打开HDF5文件
        H5File file(h5_path, H5F_ACC_RDONLY);
        cout << "Successfully opened H5 file: " << h5_path << endl;
        
        // 首先读取所有时间戳数据，用于后续同步
        vector<int64_t> color_ts, gyro_ts, accel_ts;
        vector<float> gyro_data_raw, accel_data_raw;
        
        // 读取颜色图像时间戳 - 使用相机时间戳作为参考
        if (H5Lexists(file.getId(), "/orbbec/color_timestamp", H5P_DEFAULT) > 0) {
            DataSet timestamps_dataset = file.openDataSet("/orbbec/color_timestamp");
            
            DataSpace dataspace = timestamps_dataset.getSpace();
            hsize_t dims[2];
            dataspace.getSimpleExtentDims(dims, NULL);
            
            unsigned int num_timestamps = dims[0];
            color_ts.resize(num_timestamps);
            
            // 读取所有时间戳
            hsize_t offset[2] = {0, 0};
            hsize_t count[2] = {num_timestamps, 1};
            
            DataSpace memspace(2, count);
            DataSpace filespace = timestamps_dataset.getSpace();
            filespace.selectHyperslab(H5S_SELECT_SET, count, offset);
            
            timestamps_dataset.read(color_ts.data(), PredType::NATIVE_INT64, memspace, filespace);
            cout << "Read " << num_timestamps << " color image timestamps" << endl;
        }
        
        // 读取陀螺仪时间戳
        if (H5Lexists(file.getId(), "/orbbec/gyro_timestamp", H5P_DEFAULT) > 0) {
            DataSet gyro_ts_dataset = file.openDataSet("/orbbec/gyro_timestamp");
            
            DataSpace dataspace = gyro_ts_dataset.getSpace();
            hsize_t dims[2];
            dataspace.getSimpleExtentDims(dims, NULL);
            
            unsigned int num_timestamps = dims[0];
            gyro_ts.resize(num_timestamps);
            
            // 读取所有时间戳
            hsize_t offset[2] = {0, 0};
            hsize_t count[2] = {num_timestamps, 1};
            
            DataSpace memspace(2, count);
            DataSpace filespace = gyro_ts_dataset.getSpace();
            filespace.selectHyperslab(H5S_SELECT_SET, count, offset);
            
            gyro_ts_dataset.read(gyro_ts.data(), PredType::NATIVE_INT64, memspace, filespace);
            cout << "Read " << num_timestamps << " gyro timestamps" << endl;
            
            // 读取陀螺仪数据
            DataSet gyro_dataset = file.openDataSet("/orbbec/gyro_data");
            DataSpace gyro_dataspace = gyro_dataset.getSpace();
            hsize_t gyro_dims[2];
            gyro_dataspace.getSimpleExtentDims(gyro_dims, NULL);
            
            // 读取陀螺仪数据
            gyro_data_raw.resize(gyro_dims[0] * gyro_dims[1]);
            
            hsize_t gyro_offset[2] = {0, 0};
            hsize_t gyro_count[2] = {gyro_dims[0], gyro_dims[1]};
            
            DataSpace gyro_memspace(2, gyro_count);
            DataSpace gyro_filespace = gyro_dataset.getSpace();
            gyro_filespace.selectHyperslab(H5S_SELECT_SET, gyro_count, gyro_offset);
            
            gyro_dataset.read(gyro_data_raw.data(), PredType::NATIVE_FLOAT, gyro_memspace, gyro_filespace);
            cout << "Read " << gyro_dims[0] << " gyro measurements" << endl;
        }
        
        // 读取加速度计时间戳
        if (H5Lexists(file.getId(), "/orbbec/accel_timestamp", H5P_DEFAULT) > 0) {
            DataSet accel_ts_dataset = file.openDataSet("/orbbec/accel_timestamp");
            
            DataSpace dataspace = accel_ts_dataset.getSpace();
            hsize_t dims[2];
            dataspace.getSimpleExtentDims(dims, NULL);
            
            unsigned int num_timestamps = dims[0];
            accel_ts.resize(num_timestamps);
            
            // 读取所有时间戳
            hsize_t offset[2] = {0, 0};
            hsize_t count[2] = {num_timestamps, 1};
            
            DataSpace memspace(2, count);
            DataSpace filespace = accel_ts_dataset.getSpace();
            filespace.selectHyperslab(H5S_SELECT_SET, count, offset);
            
            accel_ts_dataset.read(accel_ts.data(), PredType::NATIVE_INT64, memspace, filespace);
            cout << "Read " << num_timestamps << " accel timestamps" << endl;
            
            // 读取加速度计数据
            DataSet accel_dataset = file.openDataSet("/orbbec/accel_data");
            DataSpace accel_dataspace = accel_dataset.getSpace();
            hsize_t accel_dims[2];
            accel_dataspace.getSimpleExtentDims(accel_dims, NULL);
            
            // 读取加速度计数据
            accel_data_raw.resize(accel_dims[0] * accel_dims[1]);
            
            hsize_t accel_offset[2] = {0, 0};
            hsize_t accel_count[2] = {accel_dims[0], accel_dims[1]};
            
            DataSpace accel_memspace(2, accel_count);
            DataSpace accel_filespace = accel_dataset.getSpace();
            accel_filespace.selectHyperslab(H5S_SELECT_SET, accel_count, accel_offset);
            
            accel_dataset.read(accel_data_raw.data(), PredType::NATIVE_FLOAT, accel_memspace, accel_filespace);
            cout << "Read " << accel_dims[0] << " accel measurements" << endl;
        }
        
        // 检查时间戳数据的有效性
        if (color_ts.empty() || gyro_ts.empty() || accel_ts.empty()) {
            cerr << "Error: Missing timestamp data" << endl;
            return false;
        }
        
        // 打印原始时间戳的范围信息
        cout << "\nRaw timestamp statistics:" << endl;
        cout << "First color timestamp: " << color_ts.front() << endl;
        cout << "Last color timestamp: " << color_ts.back() << endl;
        cout << "Color time span: " << (color_ts.back() - color_ts.front()) / 1.0e3 << " seconds" << endl;
        
        cout << "First gyro timestamp: " << gyro_ts.front() << endl;
        cout << "Last gyro timestamp: " << gyro_ts.back() << endl;
        cout << "Gyro time span: " << (gyro_ts.back() - gyro_ts.front()) / 1.0e3 << " seconds" << endl;
        
        cout << "First accel timestamp: " << accel_ts.front() << endl;
        cout << "Last accel timestamp: " << accel_ts.back() << endl;
        cout << "Accel time span: " << (accel_ts.back() - accel_ts.front()) / 1.0e3 << " seconds" << endl;
        
        
        // 明确将时间戳从毫秒转换为秒，不再进行自动检测
        double timestamp_divisor = 1000.0;  // 毫秒转为秒
        
        // 清空之前的数据结构
        data.frames.clear();
        data.imus.clear();
        
        // 现在，读取图像数据
        DataSet images_dataset = file.openDataSet("/orbbec/color_image");
        
        // 获取图像数据的维度信息
        DataSpace dataspace = images_dataset.getSpace();
        int ndims = dataspace.getSimpleExtentNdims();
        
        hsize_t dims[ndims];
        dataspace.getSimpleExtentDims(dims, NULL);
        
        cout << "Image dataset dimensions: " << ndims << " [";
        for (int i = 0; i < ndims; i++) {
            cout << dims[i];
            if (i < ndims - 1) cout << ", ";
        }
        cout << "]" << endl;
        
        // 预分配图像向量，减少重新分配的次数
        size_t num_images = std::min(static_cast<size_t>(dims[0]), color_ts.size());
        data.frames.reserve(num_images);
        
        // 计算单张图像大小
        size_t single_image_size = 1;
        for (int d = 1; d < ndims; d++) {
            single_image_size *= dims[d];
        }
        
        // 批量读取的图像数量，根据可用内存调整
        const size_t batch_size = 10; // 每次读取10张图像
        const size_t total_batches = (num_images + batch_size - 1) / batch_size;
        
        cout << "Reading " << num_images << " images in " << total_batches << " batches" << endl;
        
        // 批量读取图像
        for (size_t batch = 0; batch < total_batches; batch++) {
            // 计算当前批次的起始索引和数量
            size_t batch_start = batch * batch_size;
            size_t current_batch_size = std::min(batch_size, num_images - batch_start);
            
            if (current_batch_size == 0) break;
            
            // 为整个批次分配内存
            vector<unsigned char> batch_buffer(current_batch_size * single_image_size);
            
            // 设置读取批量图像的超立方体
            hsize_t offset[ndims];
            hsize_t count[ndims];
            
            offset[0] = batch_start;
            count[0] = current_batch_size;
            
            for (int d = 1; d < ndims; d++) {
                offset[d] = 0;
                count[d] = dims[d];
            }
            
            // 创建内存和文件空间
            hsize_t mem_dims[ndims];
            mem_dims[0] = current_batch_size;
            for (int d = 1; d < ndims; d++) {
                mem_dims[d] = dims[d];
            }
            
            DataSpace memspace(ndims, mem_dims);
            DataSpace filespace = images_dataset.getSpace();
            filespace.selectHyperslab(H5S_SELECT_SET, count, offset);
            
            // 读取整批图像数据
            images_dataset.read(batch_buffer.data(), PredType::NATIVE_UINT8, memspace, filespace);
            
            // 处理每个批次中的图像
            for (size_t i = 0; i < current_batch_size; i++) {
                size_t global_idx = batch_start + i;
                if (global_idx >= num_images) break;
                
                // 计算当前图像在批次buffer中的偏移量
                size_t img_offset = i * single_image_size;
                
                // 转换为OpenCV图像 - 直接使用数据而不复制
                if (ndims == 3) {
                    // 灰度图像
                    cv::Mat img(dims[1], dims[2], CV_8UC1, batch_buffer.data() + img_offset);
                    // 使用引用计数而不是clone，减少内存使用
                    data.frames.push_back({color_ts[global_idx] / timestamp_divisor, img});
                } else if (ndims == 4) {
                    // 彩色图像 - 转换为灰度
                    cv::Mat color_img(dims[1], dims[2], CV_8UC3, batch_buffer.data() + img_offset);
                    cv::Mat gray_img;
                    // 使用RGB2GRAY因为图像以RGB格式存储（numpy格式）
                    cv::cvtColor(color_img, gray_img, cv::COLOR_RGB2GRAY);
                    data.frames.push_back({color_ts[global_idx] / timestamp_divisor, gray_img});
                }
            }
            
            // 显示批次进度
            cout << "Read batch " << batch+1 << "/" << total_batches 
                    << " (" << (batch+1)*100/total_batches << "%)" << endl;
        }
        
        cout << "Successfully read " << data.frames.size() << " images" << endl;
        
        
        // 创建同步的IMU数据
        
        // 按时间戳分别存储陀螺仪和加速度计数据
        vector<pair<double, array<double, 3>>> gyro_measurements;  // (timestamp, [gx, gy, gz])
        vector<pair<double, array<double, 3>>> accel_measurements; // (timestamp, [ax, ay, az])
        
        // 收集所有陀螺仪数据
        for (size_t i = 0; i < gyro_ts.size(); i++) {
            double timestamp = gyro_ts[i] / timestamp_divisor;
            array<double, 3> gyro_data = {
                gyro_data_raw[i * 3 + 0],
                gyro_data_raw[i * 3 + 1],
                gyro_data_raw[i * 3 + 2]
            };
            gyro_measurements.push_back({timestamp, gyro_data});
        }
        
        // 收集所有加速度计数据
        for (size_t i = 0; i < accel_ts.size(); i++) {
            double timestamp = accel_ts[i] / timestamp_divisor;
            array<double, 3> accel_data = {
                accel_data_raw[i * 3 + 0],
                accel_data_raw[i * 3 + 1],
                accel_data_raw[i * 3 + 2]
            };
            accel_measurements.push_back({timestamp, accel_data});
        }
        
        // 确保数据按时间戳排序
        auto compareByTimestamp = [](const pair<double, array<double, 3>>& a, const pair<double, array<double, 3>>& b) {
            return a.first < b.first;
        };
        
        std::sort(gyro_measurements.begin(), gyro_measurements.end(), compareByTimestamp);
        std::sort(accel_measurements.begin(), accel_measurements.end(), compareByTimestamp);
        
        cout << "Collected gyro measurements: " << gyro_measurements.size() << endl;
        cout << "Collected accel measurements: " << accel_measurements.size() << endl;
        
        // 计算两个传感器的有效时间范围（重叠部分）
        double valid_start_time = std::max(gyro_measurements.front().first, accel_measurements.front().first);
        double valid_end_time = std::min(gyro_measurements.back().first, accel_measurements.back().first);
        
        cout << "Valid time range for IMU data: " << valid_start_time << " to " << valid_end_time 
             << " (" << valid_end_time - valid_start_time << " seconds)" << endl;
        
        
        vector<ImuMeasurement> synchronized_imus;
        int processed_points = 0;
        int skipped_points = 0;
        
        // 以陀螺仪为基准，对加速度计数据进行插值
        for (const auto& gyro : gyro_measurements) {
            double timestamp = gyro.first;
            
            // 跳过不在有效时间范围内的点
            if (timestamp < valid_start_time || timestamp > valid_end_time) {
                skipped_points++;
                continue;
            }
            
            // 寻找合适的加速度计数据点进行插值
            auto it = std::lower_bound(accel_measurements.begin(), accel_measurements.end(),
                                     pair<double, array<double, 3>>{timestamp, {}},
                                     compareByTimestamp);
            
            // 这些检查在时间范围截断后应该很少触发，但保留作为额外安全措施
            if (it == accel_measurements.begin() || it == accel_measurements.end()) {
                skipped_points++;
                continue;
            }
            
            // 检查时间戳是否恰好相等 - 如果相等就直接使用该点，无需插值
            if (std::abs(it->first - timestamp) < 1e-9) {  // 使用小误差范围来比较浮点数
                // 时间戳完全匹配，直接使用加速度计数据
                ImuMeasurement imu = {
                    timestamp,
                    cv::Point3f(gyro.second[0], gyro.second[1], gyro.second[2]),  // 陀螺仪数据
                    cv::Point3f(it->second[0], it->second[1], it->second[2])       // 精确匹配的加速度计数据
                };
                
                synchronized_imus.push_back(imu);
                processed_points++;
                continue;  // 跳过后续的插值步骤
            }
            
            // lower_bound返回的是大于等于的第一个元素
            auto next = it;
            auto prev = std::prev(it);
            
            // 插值计算加速度计数据
            double accel_x = linearInterpolate(timestamp, prev->first, prev->second[0], next->first, next->second[0]);
            double accel_y = linearInterpolate(timestamp, prev->first, prev->second[1], next->first, next->second[1]);
            double accel_z = linearInterpolate(timestamp, prev->first, prev->second[2], next->first, next->second[2]);
            
            // 创建同步的IMU测量
            ImuMeasurement imu = {
                timestamp,
                cv::Point3f(gyro.second[0], gyro.second[1], gyro.second[2]),  // 陀螺仪数据
                cv::Point3f(accel_x, accel_y, accel_z)                        // 插值的加速度计数据
            };
            
            synchronized_imus.push_back(imu);
            processed_points++;
        }
        
        cout << "IMU synchronization statistics:" << endl;
        cout << "- Total gyro points: " << gyro_measurements.size() << endl;
        cout << "- Processed points: " << processed_points << endl;
        cout << "- Skipped points: " << skipped_points << " (outside valid time range)" << endl;
        
        // 确保IMU数据按时间排序
        std::sort(synchronized_imus.begin(), synchronized_imus.end(), 
             [](const ImuMeasurement& a, const ImuMeasurement& b) {
                 return a.timestamp < b.timestamp;
             });
        
        // 将同步后的IMU数据存储到结果中
        data.imus = synchronized_imus;
        
        cout << "Created " << data.imus.size() << " synchronized IMU measurements" << endl;

        // 生成原始IMU数据图表
        PlotRawIMUData(gyro_measurements, accel_measurements, "imu_data_plot_raw.png");

        return true;
    }
    catch(Exception &error) {
        cerr << "HDF5 Error: " << error.getDetailMsg() << endl;
        return false;
    }
}


int main(int argc, char *argv[])
{
    // 确保有传入SLAM系统需要的配置文件
    if(argc < 4) {
        cerr << "Warning: No vocabulary or settings file provided. SLAM system will not be initialized." << endl;
        cerr << "Usage for SLAM: ./orbbec_inertial_euroc path_to_h5_file path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

    string h5_path = argv[1];
    cout << "Loading H5 file: " << h5_path << endl;

    // 加载和解析H5数据
    OrbbecData data;
    if (!LoadOrbbecH5Data(h5_path, data)) {
        cerr << "Failed to load H5 file: " << h5_path << endl;
        return 1;
    }

    // 打印数据统计信息
    cout << "\n=== Orbbec Camera Data Summary ===" << endl;
    cout << "Total image timestamps: " << data.frames.size() << endl;
    cout << "Total loaded images: " << data.frames.size() << endl;
    cout << "Total IMU measurements: " << data.imus.size() << endl;
    
    // 检查是否有足够的IMU数据
    if (data.imus.empty()) {
        cerr << "Error: No IMU measurements loaded" << endl;
        return 1;
    }

    // 准备数据用于SLAM
    cout << "\n=== 准备开始SLAM处理 ===" << endl;
    
    // 创建SLAM系统实例
    cout << "Creating SLAM system..." << endl;
    ORB_SLAM3::System SLAM(argv[2], argv[3], ORB_SLAM3::System::IMU_MONOCULAR, true);
    cout << "SLAM system created" << endl;

    // 用于跟踪时间统计
    vector<float> vTimesTrack;
    vTimesTrack.reserve(data.frames.size());

    // 处理所有图像和IMU数据
    // IMU处理相关变量
    vector<ORB_SLAM3::IMU::Point> vImuMeas;
    double t_imu = 0.0;
    double prev_tframe = -1.0;
    
    bool slamsuccess = false;
    int frameProcessed = 0;
    size_t last_imu_idx = 0;

        // 对每个图像帧执行处理
    for (size_t i = 0; i < data.frames.size(); ++i) {
        // 获取当前图像的时间戳
        double tframe = data.frames[i].timestamp;
        if (tframe < prev_tframe) {
            break;
        }
        prev_tframe = tframe;
        // 确保图像格式正确
        cv::Mat processImage = data.frames[i].image.clone();
        // 处理当前图像之前的所有IMU数据
        vImuMeas.clear();
        while(data.imus[last_imu_idx].timestamp <= tframe && tframe > 0)
        {
            vImuMeas.push_back(ORB_SLAM3::IMU::Point(data.imus[last_imu_idx].acc.x,data.imus[last_imu_idx].acc.y,data.imus[last_imu_idx].acc.z,
                                                    data.imus[last_imu_idx].gyro.x,data.imus[last_imu_idx].gyro.y,data.imus[last_imu_idx].gyro.z,
                                                    data.imus[last_imu_idx].timestamp));
            last_imu_idx++;
        }        
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        
        // 跟踪当前帧
        cout << "tframe = " << tframe << endl;
        // cout << "Tracking frame " << i << " with " << vImuMeas.size() << " IMU measurements" << endl;
        
        // // 详细打印TrackMonocular参数
        // cout << "=== ORBBEC_INERTIAL_EUROC PARAMETERS ===" << endl;
        // cout << "Image size: " << processImage.cols << "x" << processImage.rows << ", Type: " << processImage.type() << endl;
        // cout << "Timestamp: " << tframe << " seconds" << endl;
        // cout << "IMU measurements: " << vImuMeas.size() << endl;
        // cout << "===================================" << endl;
        
        SLAM.TrackMonocular(processImage, tframe, vImuMeas);
        frameProcessed++;
        
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
        vTimesTrack.push_back(ttrack);
        
        // 计算帧间时间差，模拟真实时间处理
        double T = 0;
        if (i < data.frames.size()-1)
            T = data.frames[i+1].timestamp - tframe;
        else if (i > 0)
            T = tframe - data.frames[i-1].timestamp;
        
        // 如果处理时间小于帧间间隔，则等待一段时间以模拟实时处理
        if (ttrack < T)
            usleep((T-ttrack)*1e6); // 1e6将秒转换为微秒
    }

    SLAM.Shutdown();

    SLAM.SaveTrajectoryTUM("Mono_inertial_CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("Mono_inertial_KeyFrameTrajectory.txt");

    return 0;
}
