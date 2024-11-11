#!/usr/bin/env python3
import numpy as np
import os
import glob
from scipy.spatial.transform import Rotation

class SubmapData:
    def __init__(self, submap_dir):
        self.dir = submap_dir
        self.id = int(os.path.basename(submap_dir))
        self.transform = self._read_transform()
        self.points = None
        
    def _read_transform(self):
        """サブマップの初期変換行列を読み込む"""
        transform = np.eye(4)
        with open(os.path.join(self.dir, 'data.txt'), 'r') as f:
            lines = f.readlines()
            if len(lines) >= 5:
                for i in range(3):
                    values = [float(x) for x in lines[i+2].strip().split()]
                    transform[i] = values
        return transform

def read_trajectory(traj_file):
    """トラジェクトリデータの読み込み"""
    trajectory = {}
    with open(traj_file, 'r') as f:
        for line in f:
            data = line.strip().split()
            if len(data) == 8:
                timestamp = float(data[0])
                trajectory[timestamp] = {
                    'position': np.array([float(data[1]), float(data[2]), float(data[3])]),
                    'quaternion': np.array([float(data[4]), float(data[5]), 
                                          float(data[6]), float(data[7])])
                }
    return trajectory

def read_bin_points(input_bin):
    """点群データの読み込み"""
    with open(input_bin, 'rb') as f:
        data = f.read()
    return np.frombuffer(data, dtype=np.float32).reshape(-1, 3)

def transform_points(points, position, quaternion, initial_transform=None):
    """点群の座標変換"""
    # 回転行列の作成
    R = Rotation.from_quat(quaternion).as_matrix()
    
    # 初期変換がある場合は適用
    if initial_transform is not None:
        points = np.dot(points, initial_transform[:3, :3].T) + initial_transform[:3, 3]
    
    # トラジェクトリによる変換
    transformed_points = np.dot(points, R.T) + position
    return transformed_points

def write_pcd(points, output_pcd):
    """PCDファイルの書き出し - 単色バージョン"""
    num_points = len(points)
    
    with open(output_pcd, 'w') as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z rgb\n")
        f.write("SIZE 4 4 4 4\n")
        f.write("TYPE F F F U\n")
        f.write("COUNT 1 1 1 1\n")
        f.write(f"WIDTH {num_points}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {num_points}\n")
        f.write("DATA ascii\n")
        
        # 固定色を使用（白色）
        rgb = 255 << 16 | 255 << 8 | 255
        
        for i in range(num_points):
            f.write(f"{points[i][0]:.6f} {points[i][1]:.6f} {points[i][2]:.6f} {rgb}\n")

def filter_outliers(points, std_dev_multiplier=3.0):
    """外れ値の除去"""
    mean = np.mean(points, axis=0)
    std_dev = np.std(points, axis=0)
    
    mask = np.all(np.abs(points - mean) <= std_dev_multiplier * std_dev, axis=1)
    return points[mask]

def main():
    base_dir = "./dump"
    output_pcd = "./city_hall_map_high_res.pcd"
    
    # トラジェクトリの読み込み
    trajectory = read_trajectory(os.path.join(base_dir, "traj_lidar.txt"))
    print(f"Loaded {len(trajectory)} trajectory points")
    
    # サブマップの処理
    submap_dirs = sorted(glob.glob(os.path.join(base_dir, "0*")))
    print(f"Found {len(submap_dirs)} submaps")
    
    all_points = []
    
    for submap_dir in submap_dirs:
        try:
            # サブマップデータの読み込み
            submap = SubmapData(submap_dir)
            points_file = os.path.join(submap_dir, "points_compact.bin")
            
            if os.path.exists(points_file):
                # 点群データの読み込みと変換
                points = read_bin_points(points_file)
                
                # 外れ値の除去（オプション）
                points = filter_outliers(points)
                
                # 対応するトラジェクトリの取得
                timestamps = sorted(trajectory.keys())
                traj_data = trajectory[timestamps[submap.id]]
                
                # 点群の変換（初期変換行列とトラジェクトリの両方を適用）
                transformed_points = transform_points(
                    points,
                    traj_data['position'],
                    traj_data['quaternion'],
                    submap.transform
                )
                
                all_points.append(transformed_points)
                print(f"Processed submap {submap.id}: {len(points)} points")
        
        except Exception as e:
            print(f"Error processing {submap_dir}: {str(e)}")
    
    # 全点群の結合
    combined_points = np.vstack(all_points)
    print(f"\nTotal points in combined map: {len(combined_points)}")
    
    # PCDファイルの書き出し
    write_pcd(combined_points, output_pcd)
    print(f"\nSuccessfully created high resolution PCD file: {output_pcd}")

if __name__ == "__main__":
    main()
