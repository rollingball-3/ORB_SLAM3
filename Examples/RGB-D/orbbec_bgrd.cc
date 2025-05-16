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

using namespace std;
using namespace H5;

// Data structure to store RGB and depth frames
struct RgbdFrame {
    double timestamp;  // seconds
    cv::Mat rgb;
    cv::Mat depth;
};

// Load Orbbec RGB-D data from H5 file
bool LoadOrbbecH5Data(const string &h5_path, vector<RgbdFrame> &data) {
    try {
        // Open HDF5 file
        H5File file(h5_path, H5F_ACC_RDONLY);
        cout << "Successfully opened H5 file: " << h5_path << endl;
        
        // Read timestamps
        vector<int64_t> color_ts, depth_ts;
        
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
        data.clear();
        data.reserve(num_frames);
        
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
                data.push_back(frame);
            }
            
            cout << "Processed batch " << batch+1 << "/" << total_batches 
                 << " (" << (batch+1)*100/total_batches << "%)" << endl;
        }
        
        cout << "Successfully loaded " << data.size() << " RGB-D frames" << endl;
        return true;
    }
    catch(Exception &error) {
        cerr << "HDF5 Error: " << error.getDetailMsg() << endl;
        return false;
    }
}

int main(int argc, char **argv) {
    if (argc < 4) {
        cerr << endl << "Usage: ./orbbec_bgrd h5_file path_to_vocabulary path_to_settings [mask_path]" << endl;
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
    // false: 将mask直接传给SLAM系统，由系统内部处理mask. #TODO need fix bug
    bool apply_mask_to_image = true;
    
    cout << "Mask usage mode: " << (apply_mask_to_image ? "Apply to image before SLAM" : "Pass to SLAM system") << endl;

    // Load RGB-D data from H5 file
    vector<RgbdFrame> frames;
    cout << "Loading H5 file: " << h5_path << endl;
    
    if (!LoadOrbbecH5Data(h5_path, frames)) {
        cerr << "Failed to load data from " << h5_path << endl;
        return 1;
    }

    // Create SLAM system
    cout << "Creating ORB-SLAM3 RGBD system..." << endl;
    ORB_SLAM3::System SLAM(voc_path, settings_path, ORB_SLAM3::System::RGBD, false);
    
    // 设置Verbose级别为DEBUG
    ORB_SLAM3::Verbose::SetTh(ORB_SLAM3::Verbose::VERBOSITY_DEBUG);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.reserve(frames.size());

    cout << "\nStarting RGBD SLAM..." << endl;
    cout << "Number of frames: " << frames.size() << endl;

    for (size_t i = 0; i < frames.size(); i++) {
        // Process images at original scale if imageScale=1, otherwise resize
        cv::Mat im = frames[i].rgb;
        cv::Mat depth = frames[i].depth;
        double timestamp = frames[i].timestamp;

        // Process RGB-D frame
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        
        // Pass the image to the SLAM system
        cout << "Processing frame " << i << "/" << frames.size() << " (t=" << timestamp << "s)" << endl;
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
                SLAM.TrackRGBD(masked_im, masked_depth, timestamp);
            } else {
                // 方式2: 将mask直接传给SLAM系统，由系统内部处理mask
                SLAM.TrackRGBD(im, depth, mask, timestamp);
            }
        } else {
            SLAM.TrackRGBD(im, depth, timestamp);
        }
        
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
        vTimesTrack.push_back(ttrack);

        // Print tracking info every 10 frames
        if (i % 10 == 0 && i > 0) {
            double avgTime = 0;
            for (size_t j = 0; j < vTimesTrack.size(); j++) {
                avgTime += vTimesTrack[j];
            }
            avgTime /= vTimesTrack.size();
            
            cout << "Average tracking time: " << avgTime << " s, " 
                 << "FPS: " << 1.0/avgTime << endl;
        }

        // Wait to simulate real-time processing if needed
        double T = 0;
        if (i < frames.size()-1)
            T = frames[i+1].timestamp - timestamp;
        else if (i > 0)
            T = timestamp - frames[i-1].timestamp;

        if (ttrack < T)
            usleep((T-ttrack)*1e6);
    }

    // Shutdown SLAM system
    cout << "Shutting down SLAM system..." << endl;
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("RGBD-CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("RGBD-KeyFrameTrajectory.txt");
    cout << "Trajectory saved to RGBD-CameraTrajectory.txt" << endl;
    cout << "Key frames saved to RGBD-KeyFrameTrajectory.txt" << endl;

    cout << "Done!" << endl;
    return 0;
}
