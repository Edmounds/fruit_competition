"""
路点管理器 - 加载并管理YAML中的导航目标点

功能：
    - 从YAML文件加载路点配置
    - 解析并存储不同区域的目标位置
    - 提供路点查询接口
"""

import yaml
from typing import Dict, List, Optional, Tuple
import os
from dataclasses import dataclass


@dataclass
class Pose2D:
    """二维位姿数据类"""
    x: float
    y: float
    theta: float
    
    def __repr__(self) -> str:
        return f"Pose2D(x={self.x:.3f}, y={self.y:.3f}, theta={self.theta:.3f})"


@dataclass
class Waypoint:
    """路点数据类"""
    target_name: str
    pose: Pose2D
    zone: Optional[str] = None
    
    def __repr__(self) -> str:
        return f"Waypoint(name={self.target_name}, zone={self.zone}, {self.pose})"


class WaypointManager:
    """
    路点管理器类
    
    功能：
        - 从YAML加载路点配置
        - 支持查询单个路点、区域内所有路点
        - 提供路点列表和遍历接口
    """
    
    def __init__(self, yaml_file: str):
        """
        初始化路点管理器
        
        Args:
            yaml_file (str): YAML配置文件路径（绝对路径或相对于包目录）
        """
        self.yaml_file = yaml_file
        self._waypoints: Dict[str, Waypoint] = {}
        self._zones: Dict[str, List[Waypoint]] = {}
        self._start_location: Optional[Waypoint] = None
        self._collected_location: Optional[Waypoint] = None
        
        self._load_yaml()
    
    def _load_yaml(self) -> None:
        """
        从YAML文件加载路点配置
        
        YAML格式：
            start_location:
              pose: {x: 0.5, y: 0.5, theta: 0.785}
            
            collected_location:
              pose: {x: 4.5, y: 2.5, theta: 3.14}
            
            zones:
              A:
                - target_name: "A1"
                  pose: {x: 1.0, y: 2.5, theta: 0.0}
              B:
                qr_scanner_pose: {x: 2.5, y: 3.0, theta: 1.57}
                trees:
                  - target_name: "B1"
                    pose: {x: 2.8, y: 4.0, theta: 0.0}
        
        Raises:
            FileNotFoundError: 文件不存在时抛出
            yaml.YAMLError: YAML解析错误时抛出
        """
        # 检查文件是否存在
        if not os.path.exists(self.yaml_file):
            raise FileNotFoundError(f"路点配置文件不存在: {self.yaml_file}")
        
        # 加载YAML文件
        try:
            with open(self.yaml_file, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
        except yaml.YAMLError as e:
            raise yaml.YAMLError(f"YAML解析错误: {e}")
        
        if config is None:
            raise ValueError("YAML文件为空")
        
        # 解析起始位置
        if 'start_location' in config:
            start_data = config['start_location']
            if 'pose' in start_data:
                pose = self._parse_pose(start_data['pose'])
                self._start_location = Waypoint('start_location', pose)
        
        # 解析收集位置
        if 'collected_location' in config:
            collected_data = config['collected_location']
            if 'pose' in collected_data:
                pose = self._parse_pose(collected_data['pose'])
                self._collected_location = Waypoint('collected_location', pose)
        
        # 解析各个区域的路点
        if 'zones' in config:
            zones_config = config['zones']
            
            for zone_name, zone_data in zones_config.items():
                waypoints_in_zone: List[Waypoint] = []
                
                # 处理不同区域的数据结构
                if isinstance(zone_data, list):
                    # 区域数据是直接列表 (如A区)
                    for item in zone_data:
                        if 'target_name' in item and 'pose' in item:
                            pose = self._parse_pose(item['pose'])
                            waypoint = Waypoint(item['target_name'], pose, zone=zone_name)
                            waypoints_in_zone.append(waypoint)
                            self._waypoints[item['target_name']] = waypoint
                
                elif isinstance(zone_data, dict):
                    # 区域数据是字典，可能包含qr_scanner_pose和子列表 (如B、C区)
                    
                    # 处理扫描二维码位置
                    if 'qr_scanner_pose' in zone_data:
                        qr_pose = self._parse_pose(zone_data['qr_scanner_pose'])
                        qr_waypoint = Waypoint(f'{zone_name}_qr_scanner', qr_pose, zone=zone_name)
                        waypoints_in_zone.append(qr_waypoint)
                        self._waypoints[f'{zone_name}_qr_scanner'] = qr_waypoint
                    
                    # 处理具体的采摘点
                    for key in ['trees', 'targets']:  # B区用trees，C区用targets
                        if key in zone_data and isinstance(zone_data[key], list):
                            for item in zone_data[key]:
                                if 'target_name' in item and 'pose' in item:
                                    pose = self._parse_pose(item['pose'])
                                    waypoint = Waypoint(item['target_name'], pose, zone=zone_name)
                                    waypoints_in_zone.append(waypoint)
                                    self._waypoints[item['target_name']] = waypoint
                
                # 将区域路点列表存储
                if waypoints_in_zone:
                    self._zones[zone_name] = waypoints_in_zone
    
    @staticmethod
    def _parse_pose(pose_dict: Dict) -> Pose2D:
        """
        从字典解析位姿
        
        Args:
            pose_dict (Dict): 包含x, y, theta的字典
            
        Returns:
            Pose2D: 解析后的位姿对象
        """
        return Pose2D(
            x=float(pose_dict.get('x', 0.0)),
            y=float(pose_dict.get('y', 0.0)),
            theta=float(pose_dict.get('theta', 0.0))
        )
    
    def get_waypoint(self, target_name: str) -> Optional[Waypoint]:
        """
        获取单个路点
        
        Args:
            target_name (str): 目标名称 (如 "A1", "B1", "C_qr_scanner" 等)
            
        Returns:
            Optional[Waypoint]: 找到的路点，或None
        """
        return self._waypoints.get(target_name)
    
    def get_zone_waypoints(self, zone_name: str) -> List[Waypoint]:
        """
        获取某个区域的所有路点
        
        Args:
            zone_name (str): 区域名称 (如 "A", "B", "C")
            
        Returns:
            List[Waypoint]: 该区域的路点列表
        """
        return self._zones.get(zone_name, [])
    
    def get_start_location(self) -> Optional[Waypoint]:
        """
        获取起始位置
        
        Returns:
            Optional[Waypoint]: 起始位置路点，或None
        """
        return self._start_location
    
    def get_collected_location(self) -> Optional[Waypoint]:
        """
        获取收集位置
        
        Returns:
            Optional[Waypoint]: 收集位置路点，或None
        """
        return self._collected_location
    
    def list_all_waypoints(self) -> Dict[str, Waypoint]:
        """
        列出所有路点
        
        Returns:
            Dict[str, Waypoint]: 所有路点字典
        """
        return self._waypoints.copy()
    
    def list_all_zones(self) -> List[str]:
        """
        列出所有区域
        
        Returns:
            List[str]: 区域名称列表
        """
        return list(self._zones.keys())
    
    def print_summary(self) -> str:
        """
        打印路点配置摘要
        
        Returns:
            str: 摘要信息
        """
        summary = "=== 路点配置摘要 ===\n"
        
        if self._start_location:
            summary += f"起始位置: {self._start_location}\n"
        
        if self._collected_location:
            summary += f"收集位置: {self._collected_location}\n"
        
        summary += f"\n区域数量: {len(self._zones)}\n"
        for zone_name in self._zones:
            summary += f"  - {zone_name}区: {len(self._zones[zone_name])}个路点\n"
            for wp in self._zones[zone_name]:
                summary += f"      {wp}\n"
        
        return summary
