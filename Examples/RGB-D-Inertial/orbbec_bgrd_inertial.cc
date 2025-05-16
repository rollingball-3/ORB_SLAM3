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

// HDF5 library
#include <H5Cpp.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

// ORB-SLAM3
#include <System.h>
#include "ImuTypes.h"

using namespace std;
using namespace H5;

// Data structure to store RGB-D frames
struct RgbdFrame {
    double timestamp;  // seconds
    cv::Mat rgb;
    cv::Mat depth;
};

// Single IMU measurement data structure
struct ImuMeasurement {
    double timestamp;  // seconds
    cv::Point3f gyro;  // gyroscope data (gx, gy, gz)
    cv::Point3f acc;   // accelerometer data (ax, ay, az)
};

// Structure to store all data from H5 file
struct OrbbecData {
    vector<RgbdFrame> frames;     // RGB-D frames with timestamps
    vector<ImuMeasurement> imus;  // IMU measurements with timestamps
};

// Linear interpolation between two values
double linearInterpolate(double x, double x1, double y1, double x2, double y2) {
    if (x2 == x1) return y1;  // avoid division by zero
    return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
}

// Load Orbbec RGB-D and IMU data from H5 file
bool LoadOrbbecH5Data(const string &h5_path, OrbbecData &data) {
    try {
        // Open HDF5 file
        H5File file(h5_path, H5F_ACC_RDONLY);
        cout << "Successfully opened H5 file: " << h5_path << endl;
        
        // Read timestamps
        vector<int64_t> color_ts, depth_ts, gyro_ts, accel_ts;
        vector<float> gyro_data_raw, accel_data_raw;
        
        // Read color image timestamps
        if (H5Lexists(file.getId(), "/orbbec/color_timestamp", H5P_DEFAULT) > 0) {
            DataSet timestamps_dataset = file.openDataSet("/orbbec/color_timestamp");
            
            DataSpace dataspace = timestamps_dataset.getSpace();
            hsize_t dims[2];
            dataspace.getSimpleExtentDims(dims, NULL);
            
            unsigned int num_timestamps = dims[0];
            color_ts.resize(num_timestamps);
            
            hsize_t offset[2] = {0, 0};
            hsize_t count[2] = {num_timestamps, 1};
            
            DataSpace memspace(2, count);
            DataSpace filespace = timestamps_dataset.getSpace();
            filespace.selectHyperslab(H5S_SELECT_SET, count, offset);
            
            timestamps_dataset.read(color_ts.data(), PredType::NATIVE_INT64, memspace, filespace);
            cout << "Read " << num_timestamps << " color image timestamps" << endl;
        } else {
            cerr << "Color timestamps not found in H5 file" << endl;
            return false;
        }
        
        // Read depth image timestamps
        if (H5Lexists(file.getId(), "/orbbec/depth_timestamp", H5P_DEFAULT) > 0) {
            DataSet timestamps_dataset = file.openDataSet("/orbbec/depth_timestamp");
            
            DataSpace dataspace = timestamps_dataset.getSpace();
            hsize_t dims[2];
            dataspace.getSimpleExtentDims(dims, NULL);
            
            unsigned int num_timestamps = dims[0];
            depth_ts.resize(num_timestamps);
            
            hsize_t offset[2] = {0, 0};
            hsize_t count[2] = {num_timestamps, 1};
            
            DataSpace memspace(2, count);
            DataSpace filespace = timestamps_dataset.getSpace();
            filespace.selectHyperslab(H5S_SELECT_SET, count, offset);
            
            timestamps_dataset.read(depth_ts.data(), PredType::NATIVE_INT64, memspace, filespace);
            cout << "Read " << num_timestamps << " depth image timestamps" << endl;
        } else {
            cerr << "Depth timestamps not found in H5 file" << endl;
            return false;
        }
        
        // Read gyroscope timestamps
        if (H5Lexists(file.getId(), "/orbbec/gyro_timestamp", H5P_DEFAULT) > 0) {
            DataSet gyro_ts_dataset = file.openDataSet("/orbbec/gyro_timestamp");
            
            DataSpace dataspace = gyro_ts_dataset.getSpace();
            hsize_t dims[2];
            dataspace.getSimpleExtentDims(dims, NULL);
            
            unsigned int num_timestamps = dims[0];
            gyro_ts.resize(num_timestamps);
            
            hsize_t offset[2] = {0, 0};
            hsize_t count[2] = {num_timestamps, 1};
            
            DataSpace memspace(2, count);
            DataSpace filespace = gyro_ts_dataset.getSpace();
            filespace.selectHyperslab(H5S_SELECT_SET, count, offset);
            
            gyro_ts_dataset.read(gyro_ts.data(), PredType::NATIVE_INT64, memspace, filespace);
            cout << "Read " << num_timestamps << " gyro timestamps" << endl;
            
            // Read gyroscope data
            DataSet gyro_dataset = file.openDataSet("/orbbec/gyro_data");
            DataSpace gyro_dataspace = gyro_dataset.getSpace();
            hsize_t gyro_dims[2];
            gyro_dataspace.getSimpleExtentDims(gyro_dims, NULL);
            
            gyro_data_raw.resize(gyro_dims[0] * gyro_dims[1]);
            
            hsize_t gyro_offset[2] = {0, 0};
            hsize_t gyro_count[2] = {gyro_dims[0], gyro_dims[1]};
            
            DataSpace gyro_memspace(2, gyro_count);
            DataSpace gyro_filespace = gyro_dataset.getSpace();
            gyro_filespace.selectHyperslab(H5S_SELECT_SET, gyro_count, gyro_offset);
            
            gyro_dataset.read(gyro_data_raw.data(), PredType::NATIVE_FLOAT, gyro_memspace, gyro_filespace);
            cout << "Read " << gyro_dims[0] << " gyro measurements" << endl;
        } else {
            cerr << "Gyro data not found in H5 file" << endl;
            return false;
        }
        
        // Read accelerometer timestamps
        if (H5Lexists(file.getId(), "/orbbec/accel_timestamp", H5P_DEFAULT) > 0) {
            DataSet accel_ts_dataset = file.openDataSet("/orbbec/accel_timestamp");
            
            DataSpace dataspace = accel_ts_dataset.getSpace();
            hsize_t dims[2];
            dataspace.getSimpleExtentDims(dims, NULL);
            
            unsigned int num_timestamps = dims[0];
            accel_ts.resize(num_timestamps);
            
            hsize_t offset[2] = {0, 0};
            hsize_t count[2] = {num_timestamps, 1};
            
            DataSpace memspace(2, count);
            DataSpace filespace = accel_ts_dataset.getSpace();
            filespace.selectHyperslab(H5S_SELECT_SET, count, offset);
            
            accel_ts_dataset.read(accel_ts.data(), PredType::NATIVE_INT64, memspace, filespace);
            cout << "Read " << num_timestamps << " accel timestamps" << endl;
            
            // Read accelerometer data
            DataSet accel_dataset = file.openDataSet("/orbbec/accel_data");
            DataSpace accel_dataspace = accel_dataset.getSpace();
            hsize_t accel_dims[2];
            accel_dataspace.getSimpleExtentDims(accel_dims, NULL);
            
            accel_data_raw.resize(accel_dims[0] * accel_dims[1]);
            
            hsize_t accel_offset[2] = {0, 0};
            hsize_t accel_count[2] = {accel_dims[0], accel_dims[1]};
            
            DataSpace accel_memspace(2, accel_count);
            DataSpace accel_filespace = accel_dataset.getSpace();
            accel_filespace.selectHyperslab(H5S_SELECT_SET, accel_count, accel_offset);
            
            accel_dataset.read(accel_data_raw.data(), PredType::NATIVE_FLOAT, accel_memspace, accel_filespace);
            cout << "Read " << accel_dims[0] << " accel measurements" << endl;
        } else {
            cerr << "Accelerometer data not found in H5 file" << endl;
            return false;
        }
        
        // Convert timestamps from milliseconds to seconds
        double timestamp_divisor = 1000.0;
        
        // Print timestamp statistics
        cout << "\nTimestamp statistics:" << endl;
        cout << "First color timestamp: " << color_ts.front() / timestamp_divisor << " s" << endl;
        cout << "Last color timestamp: " << color_ts.back() / timestamp_divisor << " s" << endl;
        cout << "Color time span: " << (color_ts.back() - color_ts.front()) / timestamp_divisor << " s" << endl;
        
        cout << "First depth timestamp: " << depth_ts.front() / timestamp_divisor << " s" << endl;
        cout << "Last depth timestamp: " << depth_ts.back() / timestamp_divisor << " s" << endl;
        cout << "Depth time span: " << (depth_ts.back() - depth_ts.front()) / timestamp_divisor << " s" << endl;
        
        cout << "First gyro timestamp: " << gyro_ts.front() / timestamp_divisor << " s" << endl;
        cout << "Last gyro timestamp: " << gyro_ts.back() / timestamp_divisor << " s" << endl;
        cout << "Gyro time span: " << (gyro_ts.back() - gyro_ts.front()) / timestamp_divisor << " s" << endl;
        
        cout << "First accel timestamp: " << accel_ts.front() / timestamp_divisor << " s" << endl;
        cout << "Last accel timestamp: " << accel_ts.back() / timestamp_divisor << " s" << endl;
        cout << "Accel time span: " << (accel_ts.back() - accel_ts.front()) / timestamp_divisor << " s" << endl;
        
        // Read RGB images
        DataSet rgb_dataset = file.openDataSet("/orbbec/color_image");
        DataSpace rgb_dataspace = rgb_dataset.getSpace();
        int rgb_ndims = rgb_dataspace.getSimpleExtentNdims();
        hsize_t rgb_dims[rgb_ndims];
        rgb_dataspace.getSimpleExtentDims(rgb_dims, NULL);
        
        // Read depth images
        DataSet depth_dataset = file.openDataSet("/orbbec/depth_image");
        DataSpace depth_dataspace = depth_dataset.getSpace();
        int depth_ndims = depth_dataspace.getSimpleExtentNdims();
        hsize_t depth_dims[depth_ndims];
        depth_dataspace.getSimpleExtentDims(depth_dims, NULL);
        
        cout << "RGB dataset dimensions: [";
        for (int i = 0; i < rgb_ndims; i++) {
            cout << rgb_dims[i];
            if (i < rgb_ndims - 1) cout << ", ";
        }
        cout << "]" << endl;
        
        cout << "Depth dataset dimensions: [";
        for (int i = 0; i < depth_ndims; i++) {
            cout << depth_dims[i];
            if (i < depth_ndims - 1) cout << ", ";
        }
        cout << "]" << endl;
        
        // Ensure color and depth have the same number of frames
        size_t num_frames = std::min(static_cast<size_t>(rgb_dims[0]), static_cast<size_t>(depth_dims[0]));
        num_frames = std::min(num_frames, color_ts.size());
        num_frames = std::min(num_frames, depth_ts.size());
        
        cout << "Processing " << num_frames << " synchronized frames" << endl;
        
        // Calculate single image sizes
        size_t rgb_image_size = 1;
        for (int d = 1; d < rgb_ndims; d++) {
            rgb_image_size *= rgb_dims[d];
        }
        
        size_t depth_image_size = 1;
        for (int d = 1; d < depth_ndims; d++) {
            depth_image_size *= depth_dims[d];
        }
        
        // Batch processing to avoid memory issues
        const size_t batch_size = 10; // Process 10 frames at a time
        const size_t total_batches = (num_frames + batch_size - 1) / batch_size;
        
        cout << "Reading " << num_frames << " frames in " << total_batches << " batches" << endl;
        
        // Reserve space for frames
        data.frames.clear();
        data.frames.reserve(num_frames);
        
        // Process frames in batches
        for (size_t batch = 0; batch < total_batches; batch++) {
            // Calculate batch parameters
            size_t batch_start = batch * batch_size;
            size_t current_batch_size = std::min(batch_size, num_frames - batch_start);
            
            if (current_batch_size == 0) break;
            
            // Allocate buffers for this batch
            vector<unsigned char> rgb_buffer(current_batch_size * rgb_image_size);
            vector<unsigned short> depth_buffer(current_batch_size * depth_image_size);
            
            // Set up hyperslab for RGB images
            hsize_t rgb_offset[rgb_ndims];
            hsize_t rgb_count[rgb_ndims];
            
            rgb_offset[0] = batch_start;
            rgb_count[0] = current_batch_size;
            
            for (int d = 1; d < rgb_ndims; d++) {
                rgb_offset[d] = 0;
                rgb_count[d] = rgb_dims[d];
            }
            
            // Set up memory space for RGB
            hsize_t rgb_mem_dims[rgb_ndims];
            rgb_mem_dims[0] = current_batch_size;
            for (int d = 1; d < rgb_ndims; d++) {
                rgb_mem_dims[d] = rgb_dims[d];
            }
            
            // Read RGB batch
            DataSpace rgb_memspace(rgb_ndims, rgb_mem_dims);
            DataSpace rgb_filespace = rgb_dataset.getSpace();
            rgb_filespace.selectHyperslab(H5S_SELECT_SET, rgb_count, rgb_offset);
            rgb_dataset.read(rgb_buffer.data(), PredType::NATIVE_UINT8, rgb_memspace, rgb_filespace);
            
            // Set up hyperslab for depth images
            hsize_t depth_offset[depth_ndims];
            hsize_t depth_count[depth_ndims];
            
            depth_offset[0] = batch_start;
            depth_count[0] = current_batch_size;
            
            for (int d = 1; d < depth_ndims; d++) {
                depth_offset[d] = 0;
                depth_count[d] = depth_dims[d];
            }
            
            // Set up memory space for depth
            hsize_t depth_mem_dims[depth_ndims];
            depth_mem_dims[0] = current_batch_size;
            for (int d = 1; d < depth_ndims; d++) {
                depth_mem_dims[d] = depth_dims[d];
            }
            
            // Read depth batch
            DataSpace depth_memspace(depth_ndims, depth_mem_dims);
            DataSpace depth_filespace = depth_dataset.getSpace();
            depth_filespace.selectHyperslab(H5S_SELECT_SET, depth_count, depth_offset);
            depth_dataset.read(depth_buffer.data(), PredType::NATIVE_UINT16, depth_memspace, depth_filespace);
            
            // Process each frame in the batch
            for (size_t i = 0; i < current_batch_size; i++) {
                size_t global_idx = batch_start + i;
                if (global_idx >= num_frames) break;
                
                // Get image offsets in buffers
                size_t rgb_offset = i * rgb_image_size;
                size_t depth_offset = i * depth_image_size;
                
                RgbdFrame frame;
                frame.timestamp = color_ts[global_idx] / timestamp_divisor;
                
                if (rgb_ndims == 3) { // Grayscale images
                    frame.rgb = cv::Mat(rgb_dims[1], rgb_dims[2], CV_8UC1, rgb_buffer.data() + rgb_offset).clone();
                } else if (rgb_ndims == 4) { // Color images
                    // RGB format in H5 file (height, width, channels)
                    frame.rgb = cv::Mat(rgb_dims[1], rgb_dims[2], CV_8UC3, rgb_buffer.data() + rgb_offset).clone();
                    // Convert from RGB to BGR for OpenCV
                    cv::cvtColor(frame.rgb, frame.rgb, cv::COLOR_RGB2BGR);
                }
                
                // Create depth image - typically 16-bit
                frame.depth = cv::Mat(depth_dims[1], depth_dims[2], CV_16UC1, depth_buffer.data() + depth_offset).clone();
                
                // Add frame to data collection
                data.frames.push_back(frame);
            }
            
            cout << "Processed batch " << batch+1 << "/" << total_batches 
                 << " (" << (batch+1)*100/total_batches << "%)" << endl;
        }
        
        cout << "Successfully loaded " << data.frames.size() << " RGB-D frames" << endl;
        
        // Process IMU data
        // Create collections for gyro and accel data
        vector<pair<double, array<double, 3>>> gyro_measurements;  // (timestamp, [gx, gy, gz])
        vector<pair<double, array<double, 3>>> accel_measurements; // (timestamp, [ax, ay, az])
        
        // Collect all gyroscope data
        for (size_t i = 0; i < gyro_ts.size(); i++) {
            double timestamp = gyro_ts[i] / timestamp_divisor;
            array<double, 3> gyro_data = {
                gyro_data_raw[i * 3 + 0],
                gyro_data_raw[i * 3 + 1],
                gyro_data_raw[i * 3 + 2]
            };
            gyro_measurements.push_back({timestamp, gyro_data});
        }
        
        // Collect all accelerometer data
        for (size_t i = 0; i < accel_ts.size(); i++) {
            double timestamp = accel_ts[i] / timestamp_divisor;
            array<double, 3> accel_data = {
                accel_data_raw[i * 3 + 0],
                accel_data_raw[i * 3 + 1],
                accel_data_raw[i * 3 + 2]
            };
            accel_measurements.push_back({timestamp, accel_data});
        }
        
        // Ensure data is sorted by timestamp
        auto compareByTimestamp = [](const pair<double, array<double, 3>>& a, 
                                     const pair<double, array<double, 3>>& b) {
            return a.first < b.first;
        };
        
        std::sort(gyro_measurements.begin(), gyro_measurements.end(), compareByTimestamp);
        std::sort(accel_measurements.begin(), accel_measurements.end(), compareByTimestamp);
        
        cout << "Collected " << gyro_measurements.size() << " gyro measurements" << endl;
        cout << "Collected " << accel_measurements.size() << " accel measurements" << endl;
        
        // Calculate valid time range (overlap)
        double valid_start_time = std::max(gyro_measurements.front().first, accel_measurements.front().first);
        double valid_end_time = std::min(gyro_measurements.back().first, accel_measurements.back().first);
        
        cout << "Valid time range for IMU data: " << valid_start_time << " to " << valid_end_time 
             << " (" << valid_end_time - valid_start_time << " seconds)" << endl;
        
        // Synchronize IMU data
        vector<ImuMeasurement> synchronized_imus;
        int processed_points = 0;
        int skipped_points = 0;
        
        // Use gyroscope as reference and interpolate accelerometer data
        for (const auto& gyro : gyro_measurements) {
            double timestamp = gyro.first;
            
            // Skip points outside valid time range
            if (timestamp < valid_start_time || timestamp > valid_end_time) {
                skipped_points++;
                continue;
            }
            
            // Find appropriate accelerometer data points for interpolation
            auto it = std::lower_bound(accel_measurements.begin(), accel_measurements.end(),
                                   pair<double, array<double, 3>>{timestamp, {}},
                                   compareByTimestamp);
            
            // These checks should rarely trigger after time range truncation
            if (it == accel_measurements.begin() || it == accel_measurements.end()) {
                skipped_points++;
                continue;
            }
            
            // Check if timestamp matches exactly
            if (std::abs(it->first - timestamp) < 1e-9) {
                // Exact match, use accelerometer data directly
                ImuMeasurement imu = {
                    timestamp,
                    cv::Point3f(gyro.second[0], gyro.second[1], gyro.second[2]),
                    cv::Point3f(it->second[0], it->second[1], it->second[2])
                };
                
                synchronized_imus.push_back(imu);
                processed_points++;
                continue;
            }
            
            // Interpolate accelerometer data
            auto next = it;
            auto prev = std::prev(it);
            
            double accel_x = linearInterpolate(timestamp, prev->first, prev->second[0], next->first, next->second[0]);
            double accel_y = linearInterpolate(timestamp, prev->first, prev->second[1], next->first, next->second[1]);
            double accel_z = linearInterpolate(timestamp, prev->first, prev->second[2], next->first, next->second[2]);
            
            // Create synchronized IMU measurement
            ImuMeasurement imu = {
                timestamp,
                cv::Point3f(gyro.second[0], gyro.second[1], gyro.second[2]),
                cv::Point3f(accel_x, accel_y, accel_z)
            };
            
            synchronized_imus.push_back(imu);
            processed_points++;
        }
        
        cout << "IMU synchronization statistics:" << endl;
        cout << "- Total gyro points: " << gyro_measurements.size() << endl;
        cout << "- Processed points: " << processed_points << endl;
        cout << "- Skipped points: " << skipped_points << " (outside valid time range)" << endl;
        
        // Sort IMU data by timestamp
        std::sort(synchronized_imus.begin(), synchronized_imus.end(), 
                [](const ImuMeasurement& a, const ImuMeasurement& b) {
                    return a.timestamp < b.timestamp;
                });
        
        // Store synchronized IMU data
        data.imus = synchronized_imus;
        
        cout << "Created " << data.imus.size() << " synchronized IMU measurements" << endl;
        
        return true;
    }
    catch(Exception &error) {
        cerr << "HDF5 Error: " << error.getDetailMsg() << endl;
        return false;
    }
}

int main(int argc, char **argv) {
    if (argc < 4) {
        cerr << endl << "Usage: ./orbbec_rgbd_inertial h5_file path_to_vocabulary path_to_settings [mask_file]" << endl;
        return 1;
    }

    // Parse command line arguments
    string h5_path = argv[1];
    string voc_path = argv[2];
    string settings_path = argv[3];
    
    string mask_path = "";
    bool use_mask = false;
    cv::Mat mask;    

    if (argc == 5) {
        mask_path = argv[4];
        use_mask = true;
        cout << "Loading mask from: " << mask_path << endl;
        mask = cv::imread(mask_path, cv::IMREAD_GRAYSCALE);
        if (mask.empty()) {
            cerr << "Failed to load mask image from: " << mask_path << endl;
            return 1;
        }
        cout << "Mask dimensions: " << mask.size() << endl;
        cout << "Mask type: " << mask.type() << endl;
        cout << "Mask channels: " << mask.channels() << endl;
        
        // Print some basic mask statistics
        int total_pixels = mask.rows * mask.cols;
        int white_pixels = cv::countNonZero(mask);
        int black_pixels = total_pixels - white_pixels;
        cout << "Total pixels: " << total_pixels << endl;
        cout << "White pixels (regions of interest): " << white_pixels << " (" << (white_pixels * 100.0 / total_pixels) << "%)" << endl;
        cout << "Black pixels (masked/ignored regions): " << black_pixels << " (" << (black_pixels * 100.0 / total_pixels) << "%)" << endl;
    }

    // 选择mask使用方式: 
    // true: 将mask直接应用于图像，然后将处理后的图像传给SLAM系统
    // false: 将mask直接传给SLAM系统，由系统内部处理mask（默认方式）
    bool apply_mask_to_image = true;
    
    cout << "Mask usage mode: " << (apply_mask_to_image ? "Apply to image before SLAM" : "Pass to SLAM system") << endl;

    // Load RGB-D and IMU data from H5 file
    OrbbecData data;
    cout << "Loading H5 file: " << h5_path << endl;
    
    if (!LoadOrbbecH5Data(h5_path, data)) {
        cerr << "Failed to load data from " << h5_path << endl;
        return 1;
    }

    // Print data statistics
    cout << "\n=== Orbbec Camera Data Summary ===" << endl;
    cout << "Total RGB-D frames: " << data.frames.size() << endl;
    cout << "Total IMU measurements: " << data.imus.size() << endl;
    
    // Check if we have sufficient data
    if (data.frames.empty() || data.imus.empty()) {
        cerr << "Error: No RGB-D frames or IMU measurements loaded" << endl;
        return 1;
    }

    // Create SLAM system
    cout << "Creating ORB-SLAM3 RGBD-Inertial system..." << endl;
    ORB_SLAM3::System SLAM(voc_path, settings_path, ORB_SLAM3::System::IMU_RGBD, false);
    // 设置Verbose级别为DEBUG
    ORB_SLAM3::Verbose::SetTh(ORB_SLAM3::Verbose::VERBOSITY_DEBUG);

    float imageScale = SLAM.GetImageScale();

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.reserve(data.frames.size());

    cout << "\nStarting RGBD-Inertial SLAM..." << endl;
    cout << "Number of frames: " << data.frames.size() << endl;

    // IMU processing variables
    vector<ORB_SLAM3::IMU::Point> vImuMeas;
    double prev_tframe = -1.0;
    size_t last_imu_idx = 0;

    // Process all frames
    for (size_t i = 0; i < data.frames.size(); ++i) {
        // Get current frame timestamp
        double tframe = data.frames[i].timestamp;
        
        // Ensure timestamps are monotonically increasing
        if (tframe < prev_tframe) {
            cerr << "Warning: Non-monotonic timestamp detected, skipping frame" << endl;
            continue;
        }
        prev_tframe = tframe;
        
        // Prepare RGB-D images
        cv::Mat im = data.frames[i].rgb.clone();
        cv::Mat depth = data.frames[i].depth.clone();

        // Resize images if needed
        if (imageScale != 1.0f) {
            int width = im.cols * imageScale;
            int height = im.rows * imageScale;
            cv::resize(im, im, cv::Size(width, height));
            cv::resize(depth, depth, cv::Size(width, height));
        }

        // Clear IMU vector for current frame
        vImuMeas.clear();
        
        // Process all IMU measurements until current frame
        while (last_imu_idx < data.imus.size() && data.imus[last_imu_idx].timestamp <= tframe) {
            const ImuMeasurement& imu = data.imus[last_imu_idx];
            
            vImuMeas.push_back(ORB_SLAM3::IMU::Point(
                imu.acc.x, imu.acc.y, imu.acc.z,
                imu.gyro.x, imu.gyro.y, imu.gyro.z,
                imu.timestamp));
                
            last_imu_idx++;
        }
        

        // Track current frame
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        
        if (i % 10 == 0)
            cout << "Processing frame " << i << "/" << data.frames.size() 
                 << " (t=" << tframe << "s, IMU: " << vImuMeas.size() << " measurements)" << endl;
        
        // Check if we have enough IMU measurements
        if (vImuMeas.size() < 5) {
            cerr << "Warning: Insufficient IMU measurements (" << vImuMeas.size() << "), break loop" << endl;
            break;
        }
        if (use_mask) {
            if (apply_mask_to_image) {
                // 方式1: 将mask直接应用于图像，然后将处理后的图像传给SLAM系统
                cv::Mat masked_im = im.clone();
                cv::Mat masked_depth = depth.clone();
                
                // 只保留mask中为白色(255)的部分，其余部分设为黑色(0)
                for(int r = 0; r < im.rows; r++) {
                    for(int c = 0; c < im.cols; c++) {
                        if(mask.at<uchar>(r, c) == 0) {
                            if (masked_im.channels() == 3)
                                masked_im.at<cv::Vec3b>(r, c) = cv::Vec3b(0, 0, 0);
                            else
                                masked_im.at<uchar>(r, c) = 0;
                            
                            masked_depth.at<ushort>(r, c) = 0;
                        }
                    }
                }
                
                // 将处理后的图像传给SLAM系统
                SLAM.TrackRGBD(masked_im, masked_depth, tframe, vImuMeas);
            } else {
                // 方式2: 将mask直接传给SLAM系统，由系统内部处理mask
                SLAM.TrackRGBD(im, depth, mask, tframe, vImuMeas);
            }
        } else {
            SLAM.TrackRGBD(im, depth, tframe, vImuMeas);
        }
        
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
        vTimesTrack.push_back(ttrack);

        // Print tracking statistics every 50 frames
        if (i % 50 == 0 && i > 0) {
            double avgTime = 0;
            for (size_t j = max(0, (int)vTimesTrack.size()-50); j < vTimesTrack.size(); j++) {
                avgTime += vTimesTrack[j];
            }
            avgTime /= min(50, (int)vTimesTrack.size());
            
            cout << "Average tracking time: " << avgTime << " s, " 
                 << "FPS: " << 1.0/avgTime << endl;
        }

        // Simulate real-time processing if needed
        double T = 0;
        if (i < data.frames.size()-1)
            T = data.frames[i+1].timestamp - tframe;
        else if (i > 0)
            T = tframe - data.frames[i-1].timestamp;

        if (ttrack < T)
            usleep((T-ttrack)*1e6); // Convert seconds to microseconds
    }

    // Shutdown SLAM system
    SLAM.Shutdown();
    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("RGBD-Inertial-CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("RGBD-Inertial-KeyFrameTrajectory.txt");
    cout << "Trajectory saved to RGBD-Inertial-CameraTrajectory.txt" << endl;
    cout << "Key frames saved to RGBD-Inertial-KeyFrameTrajectory.txt" << endl;

    cout << "Done!" << endl;
    return 0;
}