# LiDAR Point Cloud Map Generator

## Overview
This project provides tools for generating and combining point cloud maps from LiDAR submap data. It processes multiple submaps with their corresponding trajectory and transformation information to create a unified, high-resolution point cloud map.

## Features
- High-resolution point cloud processing
- Accurate coordinate transformation using trajectory data
- Submap combination with initial transformation matrices
- Outlier filtering for noise reduction
- PCD file format output compatible with CloudCompare

## Directory Structure
```
./dump/
├── 000000/
│   ├── points_compact.bin  # Point cloud data
│   ├── covs_compact.bin    # Covariance data
│   ├── data.txt           # Transformation matrix
│   └── imu_rate.txt       # IMU data
├── ...
├── traj_lidar.txt         # LiDAR trajectory data
├── odom_lidar.txt         # LiDAR odometry data
└── graph.txt             # Graph structure information
```

## Requirements
- Python 3.x
- NumPy
- SciPy
- CloudCompare (for visualization)

## Installation
```bash
# Clone the repository
git clone https://github.com/H1romasa/GLIM_convert_bin_to_pcd.git

# Install required packages
pip3 install numpy scipy
```

## Usage
1. Prepare your data in the correct directory structure
2. Run the point cloud generator:
```bash
python3 transform_combine_submaps.py
```
3. The output file `city_hall_map_high_res.pcd` will be generated

## Data Format
### Trajectory Data
```
timestamp x y z qx qy qz qw
```

### Transformation Matrix
```
T_world_origin: 
   R11 R12 R13 tx
   R21 R22 R23 ty
   R31 R32 R33 tz
```

## Output
- A single PCD file containing the combined point cloud
- Maintains original point cloud density
- Preserves height information
- Suitable for CloudCompare visualization

## Notes
- Input data should maintain consistent coordinate systems
- The program preserves the original resolution of point clouds
- Outlier filtering can be adjusted via parameters

## Acknowledgments
- Thanks to the LiDAR SLAM community
- CloudCompare for visualization support
