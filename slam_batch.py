#!/usr/bin/env python3
import os
import glob
import subprocess
import sys
import time
from datetime import datetime
import shutil

# Try to import tqdm for progress bar, but make it optional
try:
    from tqdm import tqdm
    TQDM_AVAILABLE = True
except ImportError:
    TQDM_AVAILABLE = False
    print("Warning: tqdm module not found. Progress bar will not be displayed.")
    print("To install tqdm, run: pip install tqdm")

def find_h5_files(base_dir):
    """Find all orbbec.h5 files in subdirectories"""
    h5_files = []
    subdirs = []
    
    # Walk through all subdirectories
    for root, dirs, files in os.walk(base_dir):
        for file in files:
            if file == "orbbec.h5":
                h5_files.append(os.path.join(root, file))
                # Get the subdirectory name (last part of the path)
                subdir = os.path.basename(root)
                subdirs.append(subdir)
    
    return h5_files, subdirs

def run_slam(h5_file, dataset_name, script_dir, use_mask=True):
    """Run ORB-SLAM3 on the given H5 file and return the log path"""
    
    # Paths
    voc_path = os.path.join(script_dir, "Vocabulary", "ORBvoc.txt")
    yaml_path = os.path.join(script_dir, "Examples", "RGB-D", "orbbec.yaml")
    mask_path = os.path.join(script_dir, "Examples", "RGB-D", "mask.png")
    
    # Create logs directory if it doesn't exist
    log_dir = os.path.join(script_dir, "logs")
    os.makedirs(log_dir, exist_ok=True)
    
    # Create a subfolder for this dataset
    dataset_dir = os.path.join(log_dir, dataset_name)
    os.makedirs(dataset_dir, exist_ok=True)
    
    # Generate log filename
    log_file = os.path.join(dataset_dir, f"{dataset_name}.log")
    
    # Build command
    slam_executable = os.path.join(script_dir, "Examples", "RGB-D", "orbbec_bgrd")
    
    command = [slam_executable, h5_file, voc_path, yaml_path]
    
    if use_mask and os.path.exists(mask_path):
        command.append(mask_path)
    
    print(f"Running SLAM on dataset: {dataset_name}")
    print(f"Command: {' '.join(command)}")
    print(f"Log file: {log_file}")
    
    # Run the command
    with open(log_file, 'w') as log:
        process = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
    
    if process.returncode != 0:
        print(f"Warning: SLAM process exited with code {process.returncode}")
    
    return log_file

def analyze_log(log_file, script_dir):
    """Run analysis script on the log file"""
    analysis_script = os.path.join(script_dir, "logs", "analyze_tracking.py")
    
    if not os.path.exists(analysis_script):
        print(f"Error: Analysis script not found at {analysis_script}")
        return
    
    print(f"Analyzing log file: {log_file}")
    
    # Run the analysis script
    process = subprocess.run(["python3", analysis_script, log_file], 
                             stdout=subprocess.PIPE, 
                             stderr=subprocess.STDOUT, 
                             universal_newlines=True)
    
    if process.returncode != 0:
        print(f"Warning: Analysis process exited with code {process.returncode}")
    
    # Print analysis output
    print(process.stdout)

def copy_trajectory_files(script_dir, dataset_name, log_file):
    """Copy trajectory files to the log directory"""
    # Source files
    camera_traj_file = os.path.join(script_dir, "RGBD-CameraTrajectory.txt")
    keyframe_traj_file = os.path.join(script_dir, "RGBD-KeyFrameTrajectory.txt")
    
    # Get the dataset directory from the log directory
    dataset_dir = os.path.dirname(log_file)
    
    # Destination files
    dest_camera_traj = os.path.join(dataset_dir, "RGBD-CameraTrajectory.txt")
    dest_keyframe_traj = os.path.join(dataset_dir, "RGBD-KeyFrameTrajectory.txt")
    
    files_copied = 0
    
    # Copy camera trajectory file if it exists
    if os.path.exists(camera_traj_file):
        shutil.copy2(camera_traj_file, dest_camera_traj)
        print(f"Copied camera trajectory to: {dest_camera_traj}")
        files_copied += 1
    else:
        print(f"Warning: Camera trajectory file not found at {camera_traj_file}")
    
    # Copy keyframe trajectory file if it exists
    if os.path.exists(keyframe_traj_file):
        shutil.copy2(keyframe_traj_file, dest_keyframe_traj)
        print(f"Copied keyframe trajectory to: {dest_keyframe_traj}")
        files_copied += 1
    else:
        print(f"Warning: Keyframe trajectory file not found at {keyframe_traj_file}")
    
    return files_copied

def main():
    # Get the script directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    if not script_dir:  # If the script is run from the current directory
        script_dir = os.getcwd()
    
    # Base directory containing datasets
    base_dir = "/data/public/datasets/tactile/anyskin_hand2"
    
    # Find all orbbec.h5 files
    h5_files, dataset_names = find_h5_files(base_dir)
    
    if not h5_files:
        print(f"No orbbec.h5 files found in subdirectories of {base_dir}")
        sys.exit(1)
    
    print(f"Found {len(h5_files)} datasets:")
    for i, (h5_file, name) in enumerate(zip(h5_files, dataset_names)):
        print(f"{i+1}. {name}: {h5_file}")
    
    # Process each dataset
    datasets = zip(h5_files, dataset_names)
    if TQDM_AVAILABLE:
        datasets = tqdm(list(datasets), desc="Processing datasets", unit="dataset")
    
    for i, (h5_file, dataset_name) in enumerate(datasets):
        print(f"\n[{i+1}/{len(h5_files)}] Processing dataset: {dataset_name}")
        
        # Run SLAM
        log_file = run_slam(h5_file, dataset_name, script_dir, use_mask=True)
        
        # Copy trajectory files
        files_copied = copy_trajectory_files(script_dir, dataset_name, log_file)
        
        # Analyze the log
        analyze_log(log_file, script_dir)
        
        print(f"Completed processing dataset: {dataset_name}")
        
    print("\nAll datasets processed.")

if __name__ == "__main__":
    main()
