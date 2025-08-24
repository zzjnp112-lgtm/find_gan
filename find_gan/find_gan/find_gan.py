# 导入所需的库
import open3d as o3d
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import ColorRGBA
import time
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from collections import deque
import threading
import warnings

# 抑制数值计算警告
warnings.filterwarnings('ignore', category=RuntimeWarning)

class Mid360(Node):
    def __init__(self):
        super().__init__("mid360")
        qos_profile = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.lidar_sub = self.create_subscription(PointCloud2, '/cloud_registered', self.lidar_callback, qos_profile=qos_profile)
        self.lidar_x = self.create_subscription(Float64, '/pose_mid_x', self.call_back, 10)
        self.lidar_y = self.create_subscription(Float64, '/pose_mid_y', self.call_back2, 10)
        self.lidar_z = self.create_subscription(Float64, '/pose_mid_z', self.call_back3, 10)
        self.lidar_yaw = self.create_subscription(Float64, '/pose_mid_yaw', self.call_back4, 10)
        
        self.px = 0
        self.py = 0
        self.pz = 0
        self.yaw = 0
        self.point_cloud = []
        self.data_lock = threading.Lock()

    def read_points(self, msg, field_names=("x", "y", "z"), skip_nans=True):
        """改进的点云读取函数，增强数据清洗"""
        total_points = msg.width * msg.height
        field_offsets = {f.name: f.offset for f in msg.fields}
        
        for name in field_names:
            if name not in field_offsets:
                raise ValueError(f"Field {name} not found in point cloud")
        
        dtype = np.dtype([(name, np.float32) for name in field_names])
        
        try:
            data = np.frombuffer(msg.data, dtype=dtype, count=total_points)
            points = np.stack([data[name] for name in field_names], axis=1)
            
            if skip_nans:
                # 更严格的数据清洗
                # 1. 移除 NaN 和 Inf 值
                valid_mask = np.isfinite(points).all(axis=1)
                points = points[valid_mask]
                
                # 2. 移除异常值（距离过大或过小的点）
                distances = np.linalg.norm(points, axis=1)
                distance_mask = (distances > 0.01) & (distances < 8.0)  # 0.1m到50m范围内
                points = points[distance_mask]
                
                
                
            return points
            
        except Exception as e:
            self.get_logger().error(f"点云数据读取异常: {e}")
            return np.array([])
    
    def lidar_callback(self, msg):
        try:
            with self.data_lock:
                raw_points = self.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
                self.point_cloud = raw_points.tolist() if len(raw_points) > 0 else []
        except Exception as e:
            self.get_logger().error(f"LiDAR回调异常: {e}")
            with self.data_lock:
                self.point_cloud = []
    
    def call_back(self, msg): self.px = msg.data
    def call_back2(self, msg): self.py = msg.data
    def call_back3(self, msg): self.pz = msg.data
    def call_back4(self, msg): self.yaw = msg.data

class AdaptivePoleDetector:
    def __init__(self, max_frames=20):
        self.max_frames = max_frames
        # 修改：实际使用accumulated_points来存储多帧数据
        self.accumulated_points = deque(maxlen=max_frames)
        self.detection_history = deque(maxlen=40)
        
        # 调整参数设置
        self.min_diameter = 0.015 
        self.max_diameter = 0.06  
        self.min_height = 1.2   
        self.max_height = 2.5   
        
        # 聚类参数
        self.eps = 0.21        
        self.min_points = 1       
        
        # 动态调整参数
        self.adaptation_factor = 1.0
        self.detection_failure_count = 0
        
    def safe_computation(self, func, *args, **kwargs):
        """安全的数值计算包装器"""
        try:
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                result = func(*args, **kwargs)
                
                # 检查结果是否有效
                if hasattr(result, 'shape'):
                    if np.any(np.isnan(result)) or np.any(np.isinf(result)):
                        return None
                elif np.isnan(result) or np.isinf(result):
                    return None
                    
                return result
        except Exception:
            return None
    
    def clean_points(self, points):
        """清理点云数据，移除异常值"""
        if len(points) == 0:
            return np.array([])
        
        points = np.array(points)
        
        # 1. 移除 NaN 和 Inf
        valid_mask = np.isfinite(points).all(axis=1)
        points = points[valid_mask]
        
        if len(points) == 0:
            return np.array([])
        
        # 2. 使用IQR方法移除异常值
      
        # 3. 距离过滤
      
        
        return points
    
    def add_frame(self, current_points):
        """添加新的点云帧到累积队列"""
        if len(current_points) > 0:
            # 清理当前帧数据
            cleaned_points = self.clean_points(current_points)
            if len(cleaned_points) > 0:
                self.accumulated_points.append(cleaned_points)
                return True
        return False
    
    def get_accumulated_points(self):
        """获取累积的多帧点云数据"""
        if not self.accumulated_points:
            return np.array([])
        
        # 合并所有帧的点云数据
        all_points = []
        for frame_points in self.accumulated_points:
            all_points.extend(frame_points)
        
        return np.array(all_points) if all_points else np.array([])
        
    def adaptive_parameters(self):
        """根据检测失败次数动态调整参数"""
        if self.detection_failure_count > 10:
            self.min_diameter = 0.015
            self.max_diameter = 0.06
            self.min_height = 1.2
            self.eps =  0.21
            self.min_points = 1
        elif self.detection_failure_count > 5:
            self.min_diameter = 0.015
            self.max_diameter = 0.06
            self.min_height = 1.2
            self.eps =  0.21 
            self.min_points = 1
    
    def detect_poles(self, current_points):
        """改进的杆子检测 - 使用多帧累积数据"""
        # 添加当前帧到累积队列
        if not self.add_frame(current_points):
            self.detection_failure_count += 1
            self.adaptive_parameters()
            return []
        
        # 获取累积的多帧数据
        accumulated_data = self.get_accumulated_points()
        
        if len(accumulated_data) < 20:
            self.detection_failure_count += 1
            return []
        
       # print(f"使用累积数据进行检测: {len(self.accumulated_points)}帧, 总点数: {len(accumulated_data)}")
        
        # 尝试多种检测策略
        candidates = []
        
        try:
            thin_pole_candidates = self.thin_pole_detection(accumulated_data)
            candidates.extend(thin_pole_candidates)
        except Exception as e:
            print(f"细杆检测异常: {e}")

        # 策略1: 基于DBSCAN聚类
        try:
            dbscan_candidates = self.dbscan_detection(accumulated_data)
            candidates.extend(dbscan_candidates)
        except Exception as e:
            print(f"DBSCAN检测异常: {e}")
        
        # 策略2: 基于2D投影圆形检测
        try:
            projection_candidates = self.projection_circle_detection(accumulated_data)
            candidates.extend(projection_candidates)
        except Exception as e:
            print(f"投影检测异常: {e}")
        
        # 策略3: 基于垂直分层分析
        try:
            layer_candidates = self.vertical_layer_detection(accumulated_data)
            candidates.extend(layer_candidates)
        except Exception as e:
            print(f"分层检测异常: {e}")
        
        # 去重和评分
        final_candidates = self.merge_and_score_candidates(candidates)
        
        if final_candidates:
            self.detection_failure_count = 0
            return final_candidates
        else:
            self.detection_failure_count += 1
            self.adaptive_parameters()
            return []
    
    def dbscan_detection(self, points):
        """改进的DBSCAN检测"""
        if len(points) < self.min_points:
            return []
        
        try:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            
            # 使用当前参数进行聚类
            labels = np.array(pcd.cluster_dbscan(self.eps, self.min_points, print_progress=False))
            max_label = labels.max()
            
            if max_label < 0:
                return []
            
            candidates = []
            for label in range(max_label + 1):
                cluster_indices = np.where(labels == label)[0]
                if len(cluster_indices) < self.min_points:
                    continue
                
                cluster_points = points[cluster_indices]
                pole_info = self.analyze_cluster_geometry(cluster_points, "DBSCAN")
                if pole_info:
                    candidates.append(pole_info)
            
            return candidates
            
        except Exception as e:
            print(f"DBSCAN聚类异常: {e}")
            return []
    
    def projection_circle_detection(self, points):
        """改进的投影圆形检测"""
        if len(points) < 10:
            return []
        
        try:
            candidates = []
            
            # 安全的Z值计算
            z_values = points[:, 2]
            z_min, z_max = np.min(z_values), np.max(z_values)
            
            if z_max - z_min < self.min_height:
                return []
            
            # 分成多个高度层
            num_layers = max(3, int((z_max - z_min) / 0.3))
            layer_height = (z_max - z_min) / num_layers
            
            layer_centers = []
            for i in range(num_layers):
                z_start = z_min + i * layer_height
                z_end = z_start + layer_height
                
                layer_mask = (points[:, 2] >= z_start) & (points[:, 2] <= z_end)
                layer_points = points[layer_mask]
                
                if len(layer_points) < 5:
                    continue
                
                # 安全的2D投影分析
                xy_points = layer_points[:, [0, 1]]
                
                # 使用安全计算
                center_2d = self.safe_computation(np.mean, xy_points, axis=0)
                if center_2d is None:
                    continue
                
                # 计算到中心的距离
                distances = np.linalg.norm(xy_points - center_2d, axis=1)
                
                mean_radius = self.safe_computation(np.mean, distances)
                radius_std = self.safe_computation(np.std, distances)
                
                if mean_radius is None or radius_std is None or mean_radius == 0:
                    continue
                
                if radius_std / mean_radius < 0.8:
                    layer_centers.append({
                        'center_2d': center_2d,
                        'z_center': (z_start + z_end) / 2,
                        'radius': mean_radius,
                        'points': layer_points
                    })
            
            # 检查垂直一致性
            if len(layer_centers) >= 2:
                centers_2d = np.array([lc['center_2d'] for lc in layer_centers])
                center_std = self.safe_computation(np.std, centers_2d, axis=0)
                
                if center_std is not None and np.max(center_std) < 0.4:
                    all_layer_points = np.vstack([lc['points'] for lc in layer_centers])
                    pole_info = self.analyze_cluster_geometry(all_layer_points, "Projection")
                    if pole_info:
                        candidates.append(pole_info)
            
            return candidates
            
        except Exception as e:
            print(f"投影检测异常: {e}")
            return []
    
    def vertical_layer_detection(self, points):
        """改进的垂直分层检测"""
        if len(points) < 15:
            return []
        
        try:
            candidates = []
            
            # 按XY位置进行粗略分组
            xy_points = points[:, [0, 1]]
            
            # 使用较大的eps进行2D聚类
            pcd_2d = o3d.geometry.PointCloud()
            pcd_2d.points = o3d.utility.Vector3dVector(np.column_stack([xy_points, np.zeros(len(xy_points))]))
            
            labels_2d = np.array(pcd_2d.cluster_dbscan(0.3, 5, print_progress=False))
            max_label_2d = labels_2d.max()
            
            if max_label_2d < 0:
                return []
            
            for label in range(max_label_2d + 1):
                cluster_indices = np.where(labels_2d == label)[0]
                if len(cluster_indices) < 8:
                    continue
                
                cluster_points = points[cluster_indices]
                
                # 检查垂直分布
                z_values = cluster_points[:, 2]
                z_range = np.max(z_values) - np.min(z_values)
                
                if z_range >= self.min_height:
                    pole_info = self.analyze_cluster_geometry(cluster_points, "Vertical")
                    if pole_info:
                        candidates.append(pole_info)
            
            return candidates
            
        except Exception as e:
            print(f"垂直分层检测异常: {e}")
            return []
        

    def thin_pole_detection(self, points):
        """专门针对细杆的检测方法"""
        if len(points) < 5:
            return []
        
        try:
            # 高度分层分析
            z_values = points[:, 2]
            z_min, z_max = np.min(z_values), np.max(z_values)
            height = z_max - z_min
            
            # 忽略高度不足的点群
            if height < 0.8:
                return []
            
            # 按高度切片分析
            num_slices = int(height / 0.1)  # 每10cm一个切片
            if num_slices < 3:
                return []
            
            slice_centers = []
            for i in range(num_slices):
                z_start = z_min + i * 0.1
                z_end = z_start + 0.1
                
                slice_mask = (points[:, 2] >= z_start) & (points[:, 2] <= z_end)
                slice_points = points[slice_mask]
                
                if len(slice_points) < 1:  # 允许单个点
                    continue
                    
                # 计算切片中心
                center_2d = np.mean(slice_points[:, :2], axis=0)
                slice_centers.append({
                    'center': center_2d,
                    'z': (z_start + z_end) / 2,
                    'points': slice_points
                })
            
            # 检查垂直对齐度
            if len(slice_centers) < 3:
                return []
                
            # 计算中心点标准差
            centers_2d = np.array([sc['center'] for sc in slice_centers])
            center_std = np.std(centers_2d, axis=0)
            
            # 细杆应具有高度一致的垂直线
            if np.max(center_std) > 0.1:  # 5cm偏差阈值
                return []
                
            # 合并所有点作为候选杆
            all_points = np.vstack([sc['points'] for sc in slice_centers])
            pole_center = np.mean(centers_2d, axis=0)
            pole_z = (z_min + z_max) / 2
            
            return [{
                'center': np.array([pole_center[0], pole_center[1], pole_z]),
                'height': height,
                'diameter': 0.035,  # 预设直径
                'score': min(1.0, height / 1.9),  # 基于高度评分
                'method': 'ThinPole'
            }]
            
        except Exception as e:
            print(f"细杆检测异常: {e}")
            return []    
    
    def analyze_cluster_geometry(self, points, method="Unknown"):
        """改进的几何分析，增强数值稳定性"""
        if len(points) < 5:
            return None
        
        try:
            # 使用安全计算包装器
            bbox = o3d.geometry.AxisAlignedBoundingBox.create_from_points(o3d.utility.Vector3dVector(points))
            dimensions = bbox.get_extent()
            center = bbox.get_center()
            
            # 检查dimensions是否有效
            if np.any(np.isnan(dimensions)) or np.any(np.isinf(dimensions)):
                return None
            
            # 基本尺寸
            height = dimensions[2]
            width = dimensions[0]
            depth = dimensions[1]
            diameter = max(width, depth)
            
            # 尺寸约束检查
            if not (self.min_diameter <= diameter <= self.max_diameter and
                    self.min_height <= height <= self.max_height):
                return None
            
            # 形状分析
            xy_points = points[:, [0, 1]]
            z_points = points[:, 2]
            
            # 安全的圆形度分析
            xy_center = self.safe_computation(np.mean, xy_points, axis=0)
            if xy_center is None:
                return None
            
            distances_from_center = np.linalg.norm(xy_points - xy_center, axis=1)
            mean_radius = self.safe_computation(np.mean, distances_from_center)
            
            if mean_radius is None or mean_radius == 0:
                return None
            
            radius_std = self.safe_computation(np.std, distances_from_center)
            if radius_std is None:
                return None
            
            circularity = max(0, 1 - (radius_std / mean_radius))
            
            # 垂直分布连续性
            z_range = np.max(z_points) - np.min(z_points)
            
            # 长宽比
            aspect_ratio = height / diameter if diameter > 0 else 0
            
            # 点密度
            volume = width * depth * height
            density = len(points) / volume if volume > 0 else 0
            
            # 垂直性检查
            z_std = self.safe_computation(np.std, xy_points, axis=0)
            if z_std is None:
                verticality = 0.5
            else:
                verticality = 1.0 / (1.0 + np.max(z_std) * 5)
            
            # 评分计算
            score = (
                min(circularity * 2, 1.0) * 0.1 +
                min(z_range / self.min_height, 1.0) * 0.5 +
                min(aspect_ratio / 2.0, 1.0) * 0.3 +
                min(density / 50.0, 1.0) * 0.05 +
                verticality * 0.05
            )
            
            # 距离权重
            distance_to_origin = np.linalg.norm(center[:2])
            distance_weight = 1.0 / (1.0 + distance_to_origin * 0.02)
            final_score = score * distance_weight
            
            print(f"[{method}] 杆子候选 - 位置: ({center[0]:.2f}, {center[1]:.2f}), "
                  f"直径: {diameter:.3f}m, 高度: {height:.3f}m, "
                  f"圆形度: {circularity:.3f}, 长宽比: {aspect_ratio:.3f}, "
                  f"密度: {density:.1f}, 评分: {final_score:.3f}")
            
            # 检查是否满足阈值
            if (final_score > 0.02 and
                circularity > 0.01 and
                aspect_ratio > 15.0 and
                density > 0.05):
                
                return {
                    'center': center,
                    'dimensions': dimensions,
                    'score': final_score,
                    'bbox': bbox,
                    'points': points,
                    'diameter': diameter,
                    'height': height,
                    'circularity': circularity,
                    'verticality': verticality,
                    'aspect_ratio': aspect_ratio,
                    'method': method
                }
            
            return None
            
        except Exception as e:
            print(f"几何分析异常: {e}")
            return None
    
    def merge_and_score_candidates(self, candidates):
        """合并相近的候选并评分"""
        if not candidates:
            return []
        
        try:
            # 去重 - 合并位置相近的候选
            merged_candidates = []
            used_indices = set()
            
            for i, candidate in enumerate(candidates):
                if i in used_indices:
                    continue
                
                # 查找相近的候选
                similar_candidates = [candidate]
                for j, other in enumerate(candidates[i+1:], i+1):
                    if j in used_indices:
                        continue
                    
                    distance = np.linalg.norm(candidate['center'][:2] - other['center'][:2])
                    if distance < 1:
                        similar_candidates.append(other)
                        used_indices.add(j)
                
                # 选择评分最高的
                best_candidate = max(similar_candidates, key=lambda x: x['score'])
                merged_candidates.append(best_candidate)
            
            # 按评分排序
            merged_candidates.sort(key=lambda x: x['score'], reverse=True)
            
            return merged_candidates[:3]
            
        except Exception as e:
            print(f"候选合并异常: {e}")
            return []

class Find_gan(Node):
    def __init__(self, name):
        super().__init__(name)
        self.lidar_pub = self.create_publisher(ColorRGBA, '/gan', 20)
        self.mid360 = Mid360()
        
        self.mid_point = []
        self.px = 0
        self.py = 0
        self.pz = 0
        self.yaw = 0
        
        # 使用自适应杆子检测器
        self.pole_detector = AdaptivePoleDetector(max_frames=15)  # 明确设置为10帧
        
        # 检测统计
        self.detection_count = 0
        self.successful_detection_count = 0
        self.point_count_history = deque(maxlen=15)
        
        self.timer = self.create_timer(0.1, self.find)  # 10Hz
        
    def find(self):
        self.detection_count += 1
        
        # 更新位置信息
        self.px = self.mid360.px
        self.py = self.mid360.py
        self.pz = self.mid360.pz
        self.yaw = self.mid360.yaw
        
        # 获取点云数据
        with self.mid360.data_lock:
            self.mid_point = self.mid360.point_cloud.copy()
        
        # 记录点云数量
        self.point_count_history.append(len(self.mid_point))
        
        # 初始化检测结果
        self.r, self.g, self.b, self.a = 0, 0, 0, -1
        lidar_data = ColorRGBA()
        
        if len(self.mid_point) > 0:
            try:
                show = np.array(self.mid_point)
                
                # 输出点云统计信息
                if self.detection_count % 25 == 0:
                    avg_points = np.mean(self.point_count_history) if self.point_count_history else 0
                    frame_count = len(self.pole_detector.accumulated_points)
                    total_accumulated = len(self.pole_detector.get_accumulated_points())
                    
                    self.get_logger().info(f"点云统计 - 当前帧点数: {len(self.mid_point)}, "
                                         f"平均点数: {avg_points:.0f}, "
                                         f"累积帧数: {frame_count}/10, "
                                         f"累积总点数: {total_accumulated}, "
                                         f"点云范围: X[{np.min(show[:, 0]):.2f}, {np.max(show[:, 0]):.2f}], "
                                         f"Y[{np.min(show[:, 1]):.2f}, {np.max(show[:, 1]):.2f}], "
                                         f"Z[{np.min(show[:, 2]):.2f}, {np.max(show[:, 2]):.2f}]")
                
                # 杆子检测（使用累积的10帧数据）
                pole_candidates = self.pole_detector.detect_poles(show)
                
                # 处理检测结果
                if pole_candidates:
                    self.successful_detection_count += 1
                    best_pole = pole_candidates[0]
                    center = best_pole['center']
                    
                    self.r = center[0]  # 前方距离
                    self.g = center[1]  # 右方距离
                    self.a = 1
                    
                    # 发布检测结果
                    lidar_data.r = float(self.r)*100
                    lidar_data.g = float(self.g)*100
                    lidar_data.b = float(best_pole['score'])
                    lidar_data.a = float(self.a)
                    self.lidar_pub.publish(lidar_data)
                    
                    self.get_logger().info(
                        f"[{best_pole['method']}] 检测到杆子! 位置: 前方{self.r:.2f}m, 右方{self.g:.2f}m, "
                        f"直径: {best_pole['diameter']:.3f}m, 高度: {best_pole['height']:.3f}m, "
                        f"评分: {best_pole['score']:.3f}"
                    )
                    
            except Exception as e:
                self.get_logger().error(f"处理异常: {e}")
                import traceback
                traceback.print_exc()
                self.a = -1
        
        # 定期输出统计信息
        if self.detection_count % 50 == 0:
            success_rate = self.successful_detection_count / self.detection_count
            self.get_logger().info(f"检测统计 - 总计: {self.detection_count}, "
                                 f"成功: {self.successful_detection_count}, "
                                 f"成功率: {success_rate:.2%}")

def main(args=None):
    rclpy.init(args=args)
    node = Find_gan("find_gan")
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node.mid360)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()

if __name__ == '__main__':
    main()