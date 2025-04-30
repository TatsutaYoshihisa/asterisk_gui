#!/usr/bin/env python3

import os
import rospy
import rospkg
import numpy as np
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import (QWidget, QVBoxLayout, QLabel, QSlider, QPushButton, 
                                         QDoubleSpinBox, QTabWidget, QComboBox, QGridLayout)
from python_qt_binding.QtCore import Qt, QTimer
from dual_leg_controller.msg import LegCommand, LegPosition

class HexapodControllerGUI:
    """
    GUI class for controlling all six legs of the Asterisk hexapod robot.
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


class HexapodControllerGUIPlugin(Plugin):
    """RQTプラグインとしての六脚ロボット制御GUI"""
    
    def __init__(self, context):
        """GUIプラグインの初期化"""
        super(HexapodControllerGUIPlugin, self).__init__(context)
        self.setObjectName('HexapodControllerGUIPlugin')

        # コアロジックのインスタンス化
        self.core = HexapodControllerGUI()

        # メインウィジェットの作成
        self._widget = QWidget()
        self._widget.setWindowTitle('Asterisk ヘキサポッド コントローラー')
        
        # メインレイアウト
        main_layout = QVBoxLayout(self._widget)
        
        # タブウィジェットの作成
        self.tab_widget = QTabWidget()
        main_layout.addWidget(self.tab_widget)
        
        # 脚のID一覧
        self.leg_ids = self.core.leg_ids
        self.leg_names = self.core.leg_names
        
        # 各脚ごとのGUIウィジェットを作成
        self.leg_widgets = {}
        self.leg_publishers = {}
        self.leg_subscribers = {}
        
        # 単脚制御タブの作成
        self.create_single_leg_tab()
        
        # 全脚制御タブの作成
        self.create_all_legs_tab()
        
        # 歩行パターンタブの作成
        self.create_gait_control_tab()
        
        # ウィジェットを追加
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)
        
        # 歩行用タイマー
        self.walking = False
        self.gait_timer = QTimer(self._widget)
        self.gait_timer.timeout.connect(self.update_gait)
        self.gait_timer.setInterval(50)  # 20Hzで更新
        self.cycle_phase = 0.0
        
        # 初期化完了メッセージ
        rospy.loginfo("Asterisk六脚ロボットコントローラGUIが初期化されました")
    
    def create_single_leg_tab(self):
        """単脚制御タブの作成"""
        single_leg_tab = QWidget()
        layout = QVBoxLayout(single_leg_tab)
        
        # 脚選択用コンボボックス
        leg_selection_layout = QGridLayout()
        leg_selection_layout.addWidget(QLabel("制御する脚を選択:"), 0, 0)
        
        self.leg_selector = QComboBox()
        for leg_id in self.leg_ids:
            self.leg_selector.addItem(f"{self.leg_names[leg_id]} ({leg_id})")
        
        leg_selection_layout.addWidget(self.leg_selector, 0, 1)
        layout.addLayout(leg_selection_layout)
        
        # 各脚の制御UI作成
        for leg_id in self.leg_ids:
            leg_widget = QWidget()
            leg_layout = QVBoxLayout(leg_widget)
            
            # 位置制御グループ
            position_group = QGridLayout()
            position_group.addWidget(QLabel(f"X位置 (mm):"), 0, 0)
            position_group.addWidget(QLabel(f"Y位置 (mm):"), 1, 0)
            position_group.addWidget(QLabel(f"Z位置 (mm):"), 2, 0)
            
            # 位置入力スピンボックス
            x_input = QDoubleSpinBox()
            x_input.setRange(-200, 200)
            x_input.setValue(0)
            x_input.setSingleStep(5)
            position_group.addWidget(x_input, 0, 1)
            
            y_input = QDoubleSpinBox()
            y_input.setRange(-200, 200)
            y_input.setValue(0)
            y_input.setSingleStep(5)
            position_group.addWidget(y_input, 1, 1)
            
            z_input = QDoubleSpinBox()
            z_input.setRange(-200, 0)
            z_input.setValue(-150)
            z_input.setSingleStep(5)
            position_group.addWidget(z_input, 2, 1)
            
            # 移動ボタン
            move_button = QPushButton("この位置に移動")
            move_button.clicked.connect(lambda _, l=leg_id: self.send_position_command(l))
            position_group.addWidget(move_button, 3, 0, 1, 2)
            
            leg_layout.addLayout(position_group)
            
            # 関節角度制御グループ
            joint_group = QGridLayout()
            joint_group.addWidget(QLabel(f"Coxa角度:"), 0, 0)
            joint_group.addWidget(QLabel(f"Femur角度:"), 1, 0)
            joint_group.addWidget(QLabel(f"Tibia角度:"), 2, 0)
            
            # 関節スライダー
            coxa_slider = QSlider(Qt.Horizontal)
            coxa_slider.setRange(-90, 90)
            coxa_slider.setValue(0)
            coxa_slider.setTickPosition(QSlider.TicksBelow)
            coxa_slider.setTickInterval(10)
            joint_group.addWidget(coxa_slider, 0, 1)
            
            coxa_value = QLabel("0°")
            coxa_slider.valueChanged.connect(lambda v, lbl=coxa_value: lbl.setText(f"{v}°"))
            joint_group.addWidget(coxa_value, 0, 2)
            
            femur_slider = QSlider(Qt.Horizontal)
            femur_slider.setRange(-90, 90)
            femur_slider.setValue(0)
            femur_slider.setTickPosition(QSlider.TicksBelow)
            femur_slider.setTickInterval(10)
            joint_group.addWidget(femur_slider, 1, 1)
            
            femur_value = QLabel("0°")
            femur_slider.valueChanged.connect(lambda v, lbl=femur_value: lbl.setText(f"{v}°"))
            joint_group.addWidget(femur_value, 1, 2)
            
            tibia_slider = QSlider(Qt.Horizontal)
            tibia_slider.setRange(-90, 90)
            tibia_slider.setValue(0)
            tibia_slider.setTickPosition(QSlider.TicksBelow)
            tibia_slider.setTickInterval(10)
            joint_group.addWidget(tibia_slider, 2, 1)
            
            tibia_value = QLabel("0°")
            tibia_slider.valueChanged.connect(lambda v, lbl=tibia_value: lbl.setText(f"{v}°"))
            joint_group.addWidget(tibia_value, 2, 2)
            
            # 速度スライダー
            joint_group.addWidget(QLabel(f"速度:"), 3, 0)
            velocity_slider = QSlider(Qt.Horizontal)
            velocity_slider.setRange(0, 100)
            velocity_slider.setValue(50)
            velocity_slider.setTickPosition(QSlider.TicksBelow)
            velocity_slider.setTickInterval(10)
            joint_group.addWidget(velocity_slider, 3, 1)
            
            velocity_value = QLabel("50%")
            velocity_slider.valueChanged.connect(lambda v, lbl=velocity_value: lbl.setText(f"{v}%"))
            joint_group.addWidget(velocity_value, 3, 2)
            
            # 関節コマンド送信ボタン
            joint_cmd_button = QPushButton("関節角度を設定")
            joint_cmd_button.clicked.connect(lambda _, l=leg_id: self.send_joint_command(l))
            joint_group.addWidget(joint_cmd_button, 4, 0, 1, 3)
            
            leg_layout.addLayout(joint_group)
            
            # ホームポジションボタン
            home_button = QPushButton("ホームポジションに移動")
            home_button.clicked.connect(lambda _, l=leg_id: self.move_to_home_position(l))
            leg_layout.addWidget(home_button)
            
            # 現在位置表示
            status_group = QGridLayout()
            
            current_x_label = QLabel("X位置: 0.0 mm")
            status_group.addWidget(current_x_label, 0, 0)
            
            current_y_label = QLabel("Y位置: 0.0 mm")
            status_group.addWidget(current_y_label, 1, 0)
            
            current_z_label = QLabel("Z位置: 0.0 mm")
            status_group.addWidget(current_z_label, 2, 0)
            
            current_coxa_label = QLabel("Coxa角度: 0.0°")
            status_group.addWidget(current_coxa_label, 0, 1)
            
            current_femur_label = QLabel("Femur角度: 0.0°")
            status_group.addWidget(current_femur_label, 1, 1)
            
            current_tibia_label = QLabel("Tibia角度: 0.0°")
            status_group.addWidget(current_tibia_label, 2, 1)
            
            leg_layout.addLayout(status_group)
            
            # ウィジェットを非表示にしておく（最初のもの以外）
            leg_widget.setVisible(leg_id == self.leg_ids[0])
            
            # レイアウトに追加
            layout.addWidget(leg_widget)
            
            # ウィジェット参照を保存
            self.leg_widgets[leg_id] = {
                'widget': leg_widget,
                'x_input': x_input,
                'y_input': y_input,
                'z_input': z_input,
                'coxa_slider': coxa_slider,
                'femur_slider': femur_slider,
                'tibia_slider': tibia_slider,
                'velocity_slider': velocity_slider,
                'current_x_label': current_x_label,
                'current_y_label': current_y_label,
                'current_z_label': current_z_label,
                'current_coxa_label': current_coxa_label,
                'current_femur_label': current_femur_label,
                'current_tibia_label': current_tibia_label
            }
            
            # ROSインターフェースの初期化
            self.leg_publishers[leg_id] = {
                'command': rospy.Publisher(
                    f'/dual_leg_controller/{leg_id}/command', 
                    LegCommand, 
                    queue_size=1
                ),
                'position': rospy.Publisher(
                    f'/dual_leg_controller/{leg_id}/position_command',
                    LegPosition,
                    queue_size=1
                )
            }
            
            self.leg_subscribers[leg_id] = {
                'position': rospy.Subscriber(
                    f'/dual_leg_controller/{leg_id}/foot_position', 
                    LegPosition, 
                    lambda msg, l=leg_id: self.position_callback(msg, l),
                    queue_size=1
                ),
                'joints': rospy.Subscriber(
                    f'/dual_leg_controller/{leg_id}/joint_angles',
                    LegCommand,
                    lambda msg, l=leg_id: self.joint_angles_callback(msg, l),
                    queue_size=1
                )
            }
        
        # コンボボックスの変更時に表示を切り替え
        self.leg_selector.currentIndexChanged.connect(self.change_leg_display)
        
        self.tab_widget.addTab(single_leg_tab, "単脚制御")

    def create_all_legs_tab(self):
        """全脚制御タブの作成"""
        all_legs_tab = QWidget()
        layout = QVBoxLayout(all_legs_tab)
        
        # 全脚の高さ制御
        height_control_group = QGridLayout()
        height_control_group.addWidget(QLabel("全脚の高さ (mm):"), 0, 0)
        
        self.all_legs_height = QDoubleSpinBox()
        self.all_legs_height.setRange(-200, 0)
        self.all_legs_height.setValue(-150)
        self.all_legs_height.setSingleStep(5)
        height_control_group.addWidget(self.all_legs_height, 0, 1)
        
        self.set_all_legs_height_button = QPushButton("全脚の高さを設定")
        self.set_all_legs_height_button.clicked.connect(self.set_all_legs_height)
        height_control_group.addWidget(self.set_all_legs_height_button, 0, 2)
        
        layout.addLayout(height_control_group)
        
        # 全脚のホームポジション
        self.all_legs_home_button = QPushButton("全脚をホームポジションに移動")
        self.all_legs_home_button.clicked.connect(self.all_legs_home)
        layout.addWidget(self.all_legs_home_button)
        
        # 全脚の姿勢制御
        posture_control_group = QGridLayout()
        
        # ロール
        posture_control_group.addWidget(QLabel("ロール (度):"), 0, 0)
        self.roll_slider = QSlider(Qt.Horizontal)
        self.roll_slider.setRange(-30, 30)
        self.roll_slider.setValue(0)
        self.roll_slider.setTickPosition(QSlider.TicksBelow)
        self.roll_slider.setTickInterval(5)
        posture_control_group.addWidget(self.roll_slider, 0, 1)
        self.roll_value = QLabel("0°")
        self.roll_slider.valueChanged.connect(lambda v: self.update_posture_value(self.roll_value, v))
        posture_control_group.addWidget(self.roll_value, 0, 2)
        
        # ピッチ
        posture_control_group.addWidget(QLabel("ピッチ (度):"), 1, 0)
        self.pitch_slider = QSlider(Qt.Horizontal)
        self.pitch_slider.setRange(-30, 30)
        self.pitch_slider.setValue(0)
        self.pitch_slider.setTickPosition(QSlider.TicksBelow)
        self.pitch_slider.setTickInterval(5)
        posture_control_group.addWidget(self.pitch_slider, 1, 1)
        self.pitch_value = QLabel("0°")
        self.pitch_slider.valueChanged.connect(lambda v: self.update_posture_value(self.pitch_value, v))
        posture_control_group.addWidget(self.pitch_value, 1, 2)
        
        # ヨー
        posture_control_group.addWidget(QLabel("ヨー (度):"), 2, 0)
        self.yaw_slider = QSlider(Qt.Horizontal)
        self.yaw_slider.setRange(-30, 30)
        self.yaw_slider.setValue(0)
        self.yaw_slider.setTickPosition(QSlider.TicksBelow)
        self.yaw_slider.setTickInterval(5)
        posture_control_group.addWidget(self.yaw_slider, 2, 1)
        self.yaw_value = QLabel("0°")
        self.yaw_slider.valueChanged.connect(lambda v: self.update_posture_value(self.yaw_value, v))
        posture_control_group.addWidget(self.yaw_value, 2, 2)
        
        # 姿勢適用ボタン
        self.apply_posture_button = QPushButton("姿勢を適用")
        self.apply_posture_button.clicked.connect(self.apply_robot_posture)
        posture_control_group.addWidget(self.apply_posture_button, 3, 0, 1, 3)
        
        layout.addLayout(posture_control_group)
        
        self.tab_widget.addTab(all_legs_tab, "全脚制御")

    def create_gait_control_tab(self):
        """歩行パターン制御タブの作成"""
        gait_tab = QWidget()
        layout = QVBoxLayout(gait_tab)
        
        # 歩行パターン選択
        gait_selection_layout = QGridLayout()
        gait_selection_layout.addWidget(QLabel("歩行パターン:"), 0, 0)
        
        self.gait_selector = QComboBox()
        for gait in self.core.gait_patterns:
            self.gait_selector.addItem(gait)
        
        gait_selection_layout.addWidget(self.gait_selector, 0, 1)
        layout.addLayout(gait_selection_layout)
        
        # 歩行速度
        speed_layout = QGridLayout()
        speed_layout.addWidget(QLabel("歩行速度:"), 0, 0)
        
        self.gait_speed_slider = QSlider(Qt.Horizontal)
        self.gait_speed_slider.setRange(0, 100)
        self.gait_speed_slider.setValue(50)
        self.gait_speed_slider.setTickPosition(QSlider.TicksBelow)
        self.gait_speed_slider.setTickInterval(10)
        speed_layout.addWidget(self.gait_speed_slider, 0, 1)
        
        self.speed_value = QLabel("50%")
        self.gait_speed_slider.valueChanged.connect(
            lambda v: self.speed_value.setText(f"{v}%"))
        speed_layout.addWidget(self.speed_value, 0, 2)
        
        layout.addLayout(speed_layout)
        
        # 歩行方向制御
        direction_layout = QGridLayout()
        
        # 前進・後退
        direction_layout.addWidget(QLabel("前進/後退:"), 0, 0)
        self.forward_slider = QSlider(Qt.Horizontal)
        self.forward_slider.setRange(-100, 100)
        self.forward_slider.setValue(0)
        self.forward_slider.setTickPosition(QSlider.TicksBelow)
        self.forward_slider.setTickInterval(25)
        direction_layout.addWidget(self.forward_slider, 0, 1)
        
        self.forward_value = QLabel("0%")
        self.forward_slider.valueChanged.connect(
            lambda v: self.forward_value.setText(f"{v}%"))
        direction_layout.addWidget(self.forward_value, 0, 2)
        
        # 左右移動
        direction_layout.addWidget(QLabel("左右移動:"), 1, 0)
        self.lateral_slider = QSlider(Qt.Horizontal)
        self.lateral_slider.setRange(-100, 100)
        self.lateral_slider.setValue(0)
        self.lateral_slider.setTickPosition(QSlider.TicksBelow)
        self.lateral_slider.setTickInterval(25)
        direction_layout.addWidget(self.lateral_slider, 1, 1)
        
        self.lateral_value = QLabel("0%")
        self.lateral_slider.valueChanged.connect(
            lambda v: self.lateral_value.setText(f"{v}%"))
        direction_layout.addWidget(self.lateral_value, 1, 2)
        
        # 旋回
        direction_layout.addWidget(QLabel("旋回:"), 2, 0)
        self.turn_slider = QSlider(Qt.Horizontal)
        self.turn_slider.setRange(-100, 100)
        self.turn_slider.setValue(0)
        self.turn_slider.setTickPosition(QSlider.TicksBelow)
        self.turn_slider.setTickInterval(25)
        direction_layout.addWidget(self.turn_slider, 2, 1)
        
        self.turn_value = QLabel("0%")
        self.turn_slider.valueChanged.connect(
            lambda v: self.turn_value.setText(f"{v}%"))
        direction_layout.addWidget(self.turn_value, 2, 2)
        
        layout.addLayout(direction_layout)
        
        # 開始・停止ボタン
        button_layout = QGridLayout()
        
        self.start_walking_button = QPushButton("歩行開始")
        self.start_walking_button.clicked.connect(self.start_walking)
        button_layout.addWidget(self.start_walking_button, 0, 0)
        
        self.stop_walking_button = QPushButton("歩行停止")
        self.stop_walking_button.clicked.connect(self.stop_walking)
        button_layout.addWidget(self.stop_walking_button, 0, 1)
        
        layout.addLayout(button_layout)
        
        self.tab_widget.addTab(gait_tab, "歩行制御")

    def change_leg_display(self, index):
        """脚選択コンボボックスが変更されたときの処理"""
        # すべての脚ウィジェットを非表示
        for leg_data in self.leg_widgets.values():
            leg_data['widget'].setVisible(False)
        
        # 選択された脚を表示
        selected_leg_id = self.leg_ids[index]
        self.leg_widgets[selected_leg_id]['widget'].setVisible(True)

    def joint_angles_callback(self, msg, leg_id):
        """関節角度データを受信したときのコールバック"""
        try:
            # 対応する脚ウィジェットを取得
            leg_data = self.leg_widgets[leg_id]
            
            # ラジアンから度に変換
            coxa_deg = int(np.degrees(msg.coxa_angle))
            femur_deg = int(np.degrees(msg.femur_angle))
            tibia_deg = int(np.degrees(msg.tibia_angle))
            
            # スライダーを更新（ValueChangedシグナルを発生させないためにブロック）
            leg_data['coxa_slider'].blockSignals(True)
            leg_data['femur_slider'].blockSignals(True)
            leg_data['tibia_slider'].blockSignals(True)
            
            leg_data['coxa_slider'].setValue(coxa_deg)
            leg_data['femur_slider'].setValue(femur_deg)
            leg_data['tibia_slider'].setValue(tibia_deg)
            
            # シグナルブロックを解除
            leg_data['coxa_slider'].blockSignals(False)
            leg_data['femur_slider'].blockSignals(False)
            leg_data['tibia_slider'].blockSignals(False)
            
            # 表示を更新
            leg_data['current_coxa_label'].setText(f"Coxa角度: {coxa_deg}°")
            leg_data['current_femur_label'].setText(f"Femur角度: {femur_deg}°")
            leg_data['current_tibia_label'].setText(f"Tibia角度: {tibia_deg}°")

        except Exception as e:
            rospy.logerr(f"関節角度コールバックでエラー [{leg_id}]: {str(e)}")

    def position_callback(self, msg, leg_id):
        """足先位置データを受信したときのコールバック"""
        try:
            # 対応する脚ウィジェットを取得
            leg_data = self.leg_widgets[leg_id]
            
            # 表示を更新
            leg_data['current_x_label'].setText(f"X位置: {msg.x:.1f} mm")
            leg_data['current_y_label'].setText(f"Y位置: {msg.y:.1f} mm")
            leg_data['current_z_label'].setText(f"Z位置: {msg.z:.1f} mm")

        except Exception as e:
            rospy.logerr(f"位置コールバックでエラー [{leg_id}]: {str(e)}")

    def send_joint_command(self, leg_id):
        """選択した脚の関節角度コマンドを送信"""
        try:
            # 対応する脚ウィジェットを取得
            leg_data = self.leg_widgets[leg_id]
            
            # コマンドメッセージを作成
            cmd = LegCommand()
            
            # 度からラジアンに変換
            cmd.coxa_angle = np.radians(float(leg_data['coxa_slider'].value()))
            cmd.femur_angle = np.radians(float(leg_data['femur_slider'].value()))
            cmd.tibia_angle = np.radians(float(leg_data['tibia_slider'].value()))
            
            # 速度（0-1の範囲に変換）
            cmd.velocity = float(leg_data['velocity_slider'].value()) / 100.0
            
            # コマンドを送信
            self.leg_publishers[leg_id]['command'].publish(cmd)
            
            rospy.loginfo(f"脚 {leg_id} の関節角度コマンド送信 - Coxa: {leg_data['coxa_slider'].value()}°, " +
                          f"Femur: {leg_data['femur_slider'].value()}°, " +
                          f"Tibia: {leg_data['tibia_slider'].value()}°, " +
                          f"速度: {leg_data['velocity_slider'].value()}%")

        except Exception as e:
            rospy.logerr(f"関節コマンド送信でエラー [{leg_id}]: {str(e)}")

    def move_to_home_position(self, leg_id):
        """選択した脚をホームポジションに移動"""
        try:
            # 対応する脚ウィジェットを取得
            leg_data = self.leg_widgets[leg_id]
            
            # ホームポジションコマンドの作成
            cmd = LegCommand()
            cmd.coxa_angle = 0.0
            cmd.femur_angle = 0.0
            cmd.tibia_angle = 0.0
            cmd.velocity = float(leg_data['velocity_slider'].value()) / 100.0
            
            # スライダーを0に設定
            leg_data['coxa_slider'].setValue(0)
            leg_data['femur_slider'].setValue(0)
            leg_data['tibia_slider'].setValue(0)
            
            # コマンドを送信
            self.leg_publishers[leg_id]['command'].publish(cmd)
            
            rospy.loginfo(f"脚 {leg_id} をホームポジションに移動")

        except Exception as e:
            rospy.logerr(f"ホームポジション移動でエラー [{leg_id}]: {str(e)}")

    def send_position_command(self, leg_id):
        """選択した脚の位置コマンドを送信"""
        try:
            # 対応する脚ウィジェットを取得
            leg_data = self.leg_widgets[leg_id]
            
            # 位置コマンドの作成
            pos_cmd = LegPosition()
            pos_cmd.x = leg_data['x_input'].value()
            pos_cmd.y = leg_data['y_input'].value()
            pos_cmd.z = leg_data['z_input'].value()
            
            # コマンドを送信
            self.leg_publishers[leg_id]['position'].publish(pos_cmd)
            
            rospy.loginfo(f"脚 {leg_id} の位置コマンド送信 - X: {pos_cmd.x:.1f} mm, " +
                         f"Y: {pos_cmd.y:.1f} mm, Z: {pos_cmd.z:.1f} mm")

        except Exception as e:
            rospy.logerr(f"位置コマンド送信でエラー [{leg_id}]: {str(e)}")

    def set_all_legs_height(self):
        """全脚の高さを設定"""
        try:
            height = self.all_legs_height.value()
            
            for leg_id in self.leg_ids:
                # 対応する脚ウィジェットを取得
                leg_data = self.leg_widgets[leg_id]
                
                # 現在のXY値を維持して高さだけ変更
                pos_cmd = LegPosition()
                pos_cmd.x = leg_data['x_input'].value()
                pos_cmd.y = leg_data['y_input'].value()
                pos_cmd.z = height
                
                # 入力値を更新
                leg_data['z_input'].setValue(height)
                
                # コマンドを送信
                self.leg_publishers[leg_id]['position'].publish(pos_cmd)
            
            rospy.loginfo(f"全脚の高さを {height:.1f} mmに設定")

        except Exception as e:
            rospy.logerr(f"全脚の高さ設定でエラー: {str(e)}")

    def all_legs_home(self):
        """全脚をホームポジションに移動"""
        try:
            for leg_id in self.leg_ids:
                self.move_to_home_position(leg_id)
            
            rospy.loginfo("全脚をホームポジションに移動")

        except Exception as e:
            rospy.logerr(f"全脚ホームポジション移動でエラー: {str(e)}")

    def update_posture_value(self, label, value):
        """姿勢制御スライダーの値を表示に反映"""
        label.setText(f"{value}°")

    def apply_robot_posture(self):
        """ロボット全体の姿勢を適用"""
        try:
            # スライダーから姿勢角度を取得
            roll = self.roll_slider.value()
            pitch = self.pitch_slider.value()
            yaw = self.yaw_slider.value()
            height = self.all_legs_height.value()
            
            # 姿勢に基づいて脚位置を計算
            positions = self.core.calculate_leg_positions_for_posture(roll, pitch, yaw, height)
            
            # 各脚に位置コマンドを送信
            for leg_id, position in positions.items():
                pos_cmd = LegPosition()
                pos_cmd.x = position['x']
                pos_cmd.y = position['y']
                pos_cmd.z = position['z']
                
                # コマンドを送信
                self.leg_publishers[leg_id]['position'].publish(pos_cmd)
                
                # 入力値を更新
                leg_data = self.leg_widgets[leg_id]
                leg_data['x_input'].setValue(position['x'])
                leg_data['y_input'].setValue(position['y'])
                leg_data['z_input'].setValue(position['z'])
            
            rospy.loginfo(f"ロボット姿勢を適用 - ロール: {roll}°, ピッチ: {pitch}°, ヨー: {yaw}°")

        except Exception as e:
            rospy.logerr(f"姿勢適用でエラー: {str(e)}")

    def start_walking(self):
        """歩行開始"""
        try:
            if self.walking:
                rospy.logwarn("すでに歩行中です")
                return
            
            # 歩行パラメータを設定
            self.gait_pattern = self.gait_selector.currentText()
            self.walk_speed = self.gait_speed_slider.value() / 100.0
            self.forward_ratio = self.forward_slider.value() / 100.0
            self.lateral_ratio = self.lateral_slider.value() / 100.0
            self.turn_ratio = self.turn_slider.value() / 100.0
            
            # サイクル位相初期化
            self.cycle_phase = 0.0
            
            # 歩行フラグを設定
            self.walking = True
            
            # タイマーの間隔を歩行速度に基づいて設定
            cycle_time = self.core.cycle_period / self.walk_speed
            update_interval = int(cycle_time * 1000 / 20)  # 20ステップで1サイクル
            self.gait_timer.setInterval(max(50, min(200, update_interval)))
            
            # タイマー開始
            self.gait_timer.start()
            
            rospy.loginfo(f"歩行開始 - パターン: {self.gait_pattern}, 速度: {self.walk_speed*100:.0f}%, " +
                         f"前進: {self.forward_ratio*100:.0f}%, 横移動: {self.lateral_ratio*100:.0f}%, " +
                         f"旋回: {self.turn_ratio*100:.0f}%")

        except Exception as e:
            rospy.logerr(f"歩行開始でエラー: {str(e)}")

    def stop_walking(self):
        """歩行停止"""
        try:
            if not self.walking:
                rospy.logwarn("歩行していません")
                return
            
            # タイマー停止
            self.gait_timer.stop()
            
            # 歩行フラグをクリア
            self.walking = False
            
            # 現在の位置で停止
            rospy.loginfo("歩行停止")

        except Exception as e:
            rospy.logerr(f"歩行停止でエラー: {str(e)}")

    def update_gait(self):
        """歩行パターンの更新（タイマーから定期的に呼び出される）"""
        try:
            if not self.walking:
                return
            
            # サイクル位相を更新
            self.cycle_phase = (self.cycle_phase + 0.05) % 1.0
            
            # 歩行パターンに基づいて脚位置を計算
            if self.gait_pattern == "トライポッド歩行":
                positions = self.core.generate_tripod_gait(
                    self.forward_ratio, self.lateral_ratio, self.turn_ratio, self.cycle_phase)
            elif self.gait_pattern == "ウェーブ歩行":
                positions = self.core.generate_wave_gait(
                    self.forward_ratio, self.lateral_ratio, self.turn_ratio, self.cycle_phase)
            elif self.gait_pattern == "リップル歩行":
                positions = self.core.generate_ripple_gait(
                    self.forward_ratio, self.lateral_ratio, self.turn_ratio, self.cycle_phase)
            else:
                rospy.logerr(f"未知の歩行パターン: {self.gait_pattern}")
                return
            
            # 各脚に位置コマンドを送信
            for leg_id, position in positions.items():
                pos_cmd = LegPosition()
                pos_cmd.x = position['x']
                pos_cmd.y = position['y']
                pos_cmd.z = position['z']
                
                # コマンドを送信
                self.leg_publishers[leg_id]['position'].publish(pos_cmd)

        except Exception as e:
            rospy.logerr(f"歩行パターン更新でエラー: {str(e)}")

    def shutdown_plugin(self):
        """プラグイン終了時の処理"""
        try:
            # 歩行停止
            if self.walking:
                self.stop_walking()
            
            # サブスクライバーとパブリッシャーの登録解除
            for leg_id in self.leg_ids:
                for sub in self.leg_subscribers[leg_id].values():
                    sub.unregister()
                for pub in self.leg_publishers[leg_id].values():
                    pub.unregister()
            
            rospy.loginfo("六脚ロボットコントローラGUIを終了")

        except Exception as e:
            rospy.logerr(f"プラグイン終了処理でエラー: {str(e)}")

    def save_settings(self, plugin_settings, instance_settings):
        """設定の保存"""
        # 将来的な機能拡張のための空実装
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        """設定の復元"""
        # 将来的な機能拡張のための空実装
        pass

if __name__ == '__main__':
    import sys
    from rqt_gui.main import Main
    
    # ROSノードはRQTプラグインマネージャーによって自動的に初期化されるため、
    # ここでは初期化しません。スタンドアロンモード時のみ初期化します。
    
    main = Main(filename='hexapod_controller_gui')
    sys.exit(main.main(standalone='hexapod_controller_gui.HexapodControllerGUIPlugin'))