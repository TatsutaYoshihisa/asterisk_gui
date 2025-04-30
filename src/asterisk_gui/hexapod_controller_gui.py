"""
Hexapod Controller GUI module for Asterisk robot.
This module provides a GUI interface for controlling all six legs of the Asterisk hexapod robot.
"""

import os
import rospy
import rospkg
import numpy as np
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import (QWidget, QVBoxLayout, QLabel, QSlider, QPushButton, 
                                          QDoubleSpinBox, QTabWidget, QComboBox, QGridLayout)
from python_qt_binding.QtCore import Qt

class HexapodControllerGUI:
    """
    GUI class for controlling all six legs of the Asterisk hexapod robot.
    This class is used by the RQT plugin implementation in scripts/hexapod_controller_gui.py
    """
    
    def __init__(self):
        """Initialize the Hexapod controller GUI class"""
        # 脚のID一覧
        self.leg_ids = ['RF', 'LF', 'LM', 'LB', 'RB', 'RM']
        self.leg_names = {
            'RF': '右前脚', 'LF': '左前脚', 
            'LM': '左中脚', 'LB': '左後脚',
            'RB': '右後脚', 'RM': '右中脚'
        }
        
        # 各脚のデフォルト位置
        self.default_positions = {
            'RF': {'x': 150, 'y': -100, 'z': -150},
            'LF': {'x': 150, 'y': 100, 'z': -150},
            'LM': {'x': 0, 'y': 150, 'z': -150},
            'LB': {'x': -150, 'y': 100, 'z': -150},
            'RB': {'x': -150, 'y': -100, 'z': -150},
            'RM': {'x': 0, 'y': -150, 'z': -150}
        }
        
        # 歩行パターン設定
        self.gait_patterns = ['トライポッド歩行', 'ウェーブ歩行', 'リップル歩行']
        self.cycle_time = 0.0
        self.cycle_period = 2.0  # 1サイクルの時間（秒）
        self.step_height = 50.0  # 歩行時の脚の上げ高さ（mm）
        self.stride_length = 100.0  # 一歩の歩幅（mm）
        
        # トライポッド歩行のグループ
        self.tripod_groups = [
            ['RF', 'LM', 'RB'],  # グループ1
            ['LF', 'RM', 'LB']   # グループ2
        ]
        
        # ウェーブ歩行の順序
        self.wave_sequence = ['RF', 'RB', 'LB', 'LF', 'RM', 'LM']
        
        # リップル歩行のグループ
        self.ripple_groups = [
            ['RF', 'LB'],  # グループ1
            ['RM', 'LF'],  # グループ2
            ['RB', 'LM']   # グループ3
        ]
        
        rospy.loginfo("HexapodControllerGUI モジュールが初期化されました")
    
    def generate_tripod_gait(self, forward, lateral, turn, phase):
        """トライポッド歩行パターンの生成"""
        positions = {}
        
        # 位相によってグループを入れ替え
        swing_group = self.tripod_groups[0] if phase < 0.5 else self.tripod_groups[1]
        stance_group = self.tripod_groups[1] if phase < 0.5 else self.tripod_groups[0]
        
        # 位相を0-1の範囲に正規化
        if phase < 0.5:
            norm_phase = phase * 2.0  # 0-0.5 → 0-1
        else:
            norm_phase = (phase - 0.5) * 2.0  # 0.5-1 → 0-1
        
        # スイング脚（上げて前に出す脚）の位置計算
        for leg_id in swing_group:
            base_pos = self.default_positions[leg_id]
            
            # 前後移動
            dx = forward * self.stride_length * (norm_phase * 2.0 - 1.0)
            
            # 左右移動
            dy = lateral * self.stride_length * (norm_phase * 2.0 - 1.0)
            
            # 旋回（回転中心からの距離に比例）
            dtheta = np.radians(turn * 20.0 * (norm_phase * 2.0 - 1.0))
            x_rot = base_pos['x'] * np.cos(dtheta) - base_pos['y'] * np.sin(dtheta) - base_pos['x']
            y_rot = base_pos['x'] * np.sin(dtheta) + base_pos['y'] * np.cos(dtheta) - base_pos['y']
            
            # 脚を上げる（sinカーブで）
            dz = self.step_height * np.sin(norm_phase * np.pi)
            
            # 合計の移動量
            positions[leg_id] = {
                'x': base_pos['x'] + dx + x_rot,
                'y': base_pos['y'] + dy + y_rot,
                'z': base_pos['z'] - dz  # 下が負なので注意
            }
        
        # スタンス脚（地面を押す脚）の位置計算
        for leg_id in stance_group:
            base_pos = self.default_positions[leg_id]
            
            # 前後移動（逆向き）
            dx = forward * self.stride_length * (1.0 - norm_phase * 2.0)
            
            # 左右移動（逆向き）
            dy = lateral * self.stride_length * (1.0 - norm_phase * 2.0)
            
            # 旋回（逆向き）
            dtheta = np.radians(turn * 20.0 * (1.0 - norm_phase * 2.0))
            x_rot = base_pos['x'] * np.cos(dtheta) - base_pos['y'] * np.sin(dtheta) - base_pos['x']
            y_rot = base_pos['x'] * np.sin(dtheta) + base_pos['y'] * np.cos(dtheta) - base_pos['y']
            
            # 地面に接地
            dz = 0.0
            
            # 合計の移動量
            positions[leg_id] = {
                'x': base_pos['x'] + dx + x_rot,
                'y': base_pos['y'] + dy + y_rot,
                'z': base_pos['z'] - dz
            }
        
        return positions
    
    def generate_wave_gait(self, forward, lateral, turn, phase):
        """ウェーブ歩行パターンの生成"""
        positions = {}
        num_legs = len(self.wave_sequence)
        phase_per_leg = 1.0 / num_legs
        
        # 各脚のパターン生成
        for i, leg_id in enumerate(self.wave_sequence):
            # 脚ごとの位相をずらす
            leg_phase = (phase + i * phase_per_leg) % 1.0
            base_pos = self.default_positions[leg_id]
            
            # スイングフェーズ（脚を上げて前に出す）か判定
            is_swing = leg_phase < 0.25  # 1/4の時間でスイング
            
            if is_swing:
                # スイングフェーズの正規化位相（0-1）
                norm_phase = leg_phase / 0.25
                
                # 前後移動
                dx = forward * self.stride_length * (norm_phase * 2.0 - 1.0)
                
                # 左右移動
                dy = lateral * self.stride_length * (norm_phase * 2.0 - 1.0)
                
                # 旋回
                dtheta = np.radians(turn * 20.0 * (norm_phase * 2.0 - 1.0))
                x_rot = base_pos['x'] * np.cos(dtheta) - base_pos['y'] * np.sin(dtheta) - base_pos['x']
                y_rot = base_pos['x'] * np.sin(dtheta) + base_pos['y'] * np.cos(dtheta) - base_pos['y']
                
                # 脚を上げる
                dz = self.step_height * np.sin(norm_phase * np.pi)
            else:
                # スタンスフェーズの正規化位相（0-1）
                norm_phase = (leg_phase - 0.25) / 0.75
                
                # 前後移動（逆向き・ゆっくり）
                dx = forward * self.stride_length * (1.0 - norm_phase)
                
                # 左右移動（逆向き・ゆっくり）
                dy = lateral * self.stride_length * (1.0 - norm_phase)
                
                # 旋回（逆向き・ゆっくり）
                dtheta = np.radians(turn * 20.0 * (1.0 - norm_phase))
                x_rot = base_pos['x'] * np.cos(dtheta) - base_pos['y'] * np.sin(dtheta) - base_pos['x']
                y_rot = base_pos['x'] * np.sin(dtheta) + base_pos['y'] * np.cos(dtheta) - base_pos['y']
                
                # 地面に接地
                dz = 0.0
            
            # 合計の移動量
            positions[leg_id] = {
                'x': base_pos['x'] + dx + x_rot,
                'y': base_pos['y'] + dy + y_rot,
                'z': base_pos['z'] - dz
            }
        
        return positions
    
    def generate_ripple_gait(self, forward, lateral, turn, phase):
        """リップル歩行パターンの生成"""
        positions = {}
        num_groups = len(self.ripple_groups)
        phase_per_group = 1.0 / num_groups
        
        # 各グループのパターン生成
        for i, group in enumerate(self.ripple_groups):
            # グループごとの位相をずらす
            group_phase = (phase + i * phase_per_group) % 1.0
            
            # スイングフェーズ（脚を上げて前に出す）か判定
            is_swing = group_phase < 0.33  # 1/3の時間でスイング
            
            for leg_id in group:
                base_pos = self.default_positions[leg_id]
                
                if is_swing:
                    # スイングフェーズの正規化位相（0-1）
                    norm_phase = group_phase / 0.33
                    
                    # 前後移動
                    dx = forward * self.stride_length * (norm_phase * 2.0 - 1.0)
                    
                    # 左右移動
                    dy = lateral * self.stride_length * (norm_phase * 2.0 - 1.0)
                    
                    # 旋回
                    dtheta = np.radians(turn * 20.0 * (norm_phase * 2.0 - 1.0))
                    x_rot = base_pos['x'] * np.cos(dtheta) - base_pos['y'] * np.sin(dtheta) - base_pos['x']
                    y_rot = base_pos['x'] * np.sin(dtheta) + base_pos['y'] * np.cos(dtheta) - base_pos['y']
                    
                    # 脚を上げる
                    dz = self.step_height * np.sin(norm_phase * np.pi)
                else:
                    # スタンスフェーズの正規化位相（0-1）
                    norm_phase = (group_phase - 0.33) / 0.67
                    
                    # 前後移動（逆向き・ゆっくり）
                    dx = forward * self.stride_length * (1.0 - norm_phase)
                    
                    # 左右移動（逆向き・ゆっくり）
                    dy = lateral * self.stride_length * (1.0 - norm_phase)
                    
                    # 旋回（逆向き・ゆっくり）
                    dtheta = np.radians(turn * 20.0 * (1.0 - norm_phase))
                    x_rot = base_pos['x'] * np.cos(dtheta) - base_pos['y'] * np.sin(dtheta) - base_pos['x']
                    y_rot = base_pos['x'] * np.sin(dtheta) + base_pos['y'] * np.cos(dtheta) - base_pos['y']
                    
                    # 地面に接地
                    dz = 0.0
                
                # 合計の移動量
                positions[leg_id] = {
                    'x': base_pos['x'] + dx + x_rot,
                    'y': base_pos['y'] + dy + y_rot,
                    'z': base_pos['z'] - dz
                }
        
        return positions
    
    def calculate_leg_positions_for_posture(self, roll, pitch, yaw, height):
        """ロール・ピッチ・ヨーに基づいた脚位置の計算"""
        positions = {}
        
        # 角度をラジアンに変換
        roll_rad = np.radians(roll)
        pitch_rad = np.radians(pitch)
        yaw_rad = np.radians(yaw)
        
        # 回転行列の作成
        # ロール（X軸周り）
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(roll_rad), -np.sin(roll_rad)],
            [0, np.sin(roll_rad), np.cos(roll_rad)]
        ])
        
        # ピッチ（Y軸周り）
        Ry = np.array([
            [np.cos(pitch_rad), 0, np.sin(pitch_rad)],
            [0, 1, 0],
            [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]
        ])
        
        # ヨー（Z軸周り）
        Rz = np.array([
            [np.cos(yaw_rad), -np.sin(yaw_rad), 0],
            [np.sin(yaw_rad), np.cos(yaw_rad), 0],
            [0, 0, 1]
        ])
        
        # 合成回転行列
        R = Rz @ Ry @ Rx
        
        # 各脚の位置を計算
        for leg_id in self.leg_ids:
            # デフォルト位置（ロボット座標系）
            base_pos = np.array([
                self.default_positions[leg_id]['x'],
                self.default_positions[leg_id]['y'],
                0  # Z=0を基準に回転
            ])
            
            # 回転適用
            rotated_pos = R @ base_pos
            
            # 高さを加算
            rotated_pos[2] += height
            
            # 結果を格納
            positions[leg_id] = {
                'x': rotated_pos[0],
                'y': rotated_pos[1],
                'z': rotated_pos[2]
            }
        
        return positions