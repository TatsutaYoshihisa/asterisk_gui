#!/usr/bin/env python3

import os
import rospy
import rospkg
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QLabel, QSlider, QPushButton, QDoubleSpinBox
from python_qt_binding.QtCore import Qt
from single_leg_controller.msg import LegCommand, LegPosition
import numpy as np

class LegControllerGUI(Plugin):
    def __init__(self, context):
        """Initialize the GUI plugin"""
        super(LegControllerGUI, self).__init__(context)
        self.setObjectName('LegControllerGUI')

        # Create QWidget
        self._widget = QWidget()
        
        # Get path to UI file
        ui_file = os.path.join(rospkg.RosPack().get_path('single_leg_control_gui'), 
                              'resource', 'leg_controller.ui')
        
        # Load UI file
        loadUi(ui_file, self._widget)
        
        # Add widget to the user interface
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

        # Initialize ROS Subscribers
        self.current_pos_sub = rospy.Subscriber(
            '/single_leg_controller/foot_position', 
            LegPosition, 
            self.position_callback,
            queue_size=1
        )

        # 現在の関節角度を購読 - メッセージ型を修正
        self.joint_angles_sub = rospy.Subscriber(
            '/single_leg_controller/joint_angles',
            LegCommand,  # Float64MultiArrayからLegCommandに変更
            self.joint_angles_callback,
            queue_size=1
        )
        
        # Initialize ROS Publishers
        self.target_pos_pub = rospy.Publisher(
            '/single_leg_controller/command', 
            LegCommand, 
            queue_size=1
        )
        
        self.position_pub = rospy.Publisher(
            '/single_leg_controller/position_command',
            LegPosition,
            queue_size=1
        )

        # Connect UI elements for joint control
        self._widget.coxa_slider.valueChanged.connect(self.send_joint_command)
        self._widget.femur_slider.valueChanged.connect(self.send_joint_command)
        self._widget.tibia_slider.valueChanged.connect(self.send_joint_command)
        self._widget.velocity_slider.valueChanged.connect(self.send_joint_command)

        # Connect home position button
        self._widget.home_position_button.clicked.connect(self.move_to_home_position)

        # Connect UI elements for position control
        self._widget.move_to_position_button.clicked.connect(self.send_position_command)

        # Set initial slider ranges
        self._widget.coxa_slider.setRange(-90, 90)   # ±90度
        self._widget.coxa_slider.setTickPosition(QSlider.TicksBelow)
        self._widget.coxa_slider.setTickInterval(10)
        
        self._widget.femur_slider.setRange(-90, 90)  # ±90度
        self._widget.femur_slider.setTickPosition(QSlider.TicksBelow)
        self._widget.femur_slider.setTickInterval(10)
        
        self._widget.tibia_slider.setRange(-90, 90)  # ±90度
        self._widget.tibia_slider.setTickPosition(QSlider.TicksBelow)
        self._widget.tibia_slider.setTickInterval(10)
        
        self._widget.velocity_slider.setRange(0, 100)  # 0-100%
        self._widget.velocity_slider.setTickPosition(QSlider.TicksBelow)
        self._widget.velocity_slider.setTickInterval(10)
        self._widget.velocity_slider.setValue(50)  # デフォルト値を50%に設定

        # Set initial position input ranges
        self._widget.x_input.setRange(-200, 200)  # ±200mm
        self._widget.y_input.setRange(-200, 200)  # ±200mm
        self._widget.z_input.setRange(-200, 200)  # ±200mm
        
        # Initialize values
        self._widget.x_input.setValue(0)
        self._widget.y_input.setValue(0)
        self._widget.z_input.setValue(-150)  # デフォルトの高さ

        # Initialize update timer (10Hz)
        self._update_timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

        # Initialize message objects
        self._current_position = None
        self._last_command = None
        self._current_joint_angles = None

        # Wait for initial joint angles
        rospy.sleep(0.5)  # 初期データを待つ

        # Log initialization
        rospy.loginfo("LegControllerGUI initialized")
        rospy.loginfo("Publishing to: /single_leg_controller/command")
        rospy.loginfo("Publishing to: /single_leg_controller/position_command")
        rospy.loginfo("Subscribing to: /single_leg_controller/foot_position")
        rospy.loginfo("Subscribing to: /single_leg_controller/joint_angles")

    def joint_angles_callback(self, msg):
        """
        Update slider positions when new joint angle data is received
        Args:
            msg (LegCommand): Message containing current joint angles in radians
        """
        try:
            # LegCommandメッセージからデータを取得
            self._current_joint_angles = [msg.coxa_angle, msg.femur_angle, msg.tibia_angle]
            
            # スライダーの値を更新（ValueChangedシグナルを発生させないように一時的にブロック）
            self._widget.coxa_slider.blockSignals(True)
            self._widget.femur_slider.blockSignals(True)
            self._widget.tibia_slider.blockSignals(True)
            
            # ラジアンから度に変換してスライダーを設定
            self._widget.coxa_slider.setValue(int(np.degrees(msg.coxa_angle)))
            self._widget.femur_slider.setValue(int(np.degrees(msg.femur_angle)))
            self._widget.tibia_slider.setValue(int(np.degrees(msg.tibia_angle)))
            
            # シグナルブロックを解除
            self._widget.coxa_slider.blockSignals(False)
            self._widget.femur_slider.blockSignals(False)
            self._widget.tibia_slider.blockSignals(False)
            
            rospy.logdebug(f"Received joint angles - Coxa: {np.degrees(msg.coxa_angle):.1f}°, " +
                          f"Femur: {np.degrees(msg.femur_angle):.1f}°, " +
                          f"Tibia: {np.degrees(msg.tibia_angle):.1f}°")

        except Exception as e:
            rospy.logerr(f"Error in joint_angles_callback: {str(e)}")

    def move_to_home_position(self):
        """Move all joints to their home position (0 degrees)"""
        try:
            # Create command for home position
            cmd = LegCommand()
            cmd.coxa_angle = 0.0
            cmd.femur_angle = 0.0
            cmd.tibia_angle = 0.0
            cmd.velocity = float(self._widget.velocity_slider.value()) / 100.0

            # Update sliders to 0 position
            self._widget.coxa_slider.setValue(0)
            self._widget.femur_slider.setValue(0)
            self._widget.tibia_slider.setValue(0)

            # Publish command
            self.target_pos_pub.publish(cmd)

            rospy.loginfo("Moving to home position")

        except Exception as e:
            rospy.logerr(f"Error in move_to_home_position: {str(e)}")

    def send_joint_command(self):
        """Send joint angle command when sliders are moved"""
        try:
            # Create new command message
            cmd = LegCommand()
            
            # Convert degrees to radians for joint angles
            cmd.coxa_angle = np.radians(float(self._widget.coxa_slider.value()))
            cmd.femur_angle = np.radians(float(self._widget.femur_slider.value()))
            cmd.tibia_angle = np.radians(float(self._widget.tibia_slider.value()))
            
            # Convert percentage to 0-1 range for velocity
            cmd.velocity = float(self._widget.velocity_slider.value()) / 100.0
            
            # Store command for reference
            self._last_command = cmd
            
            # Publish command
            self.target_pos_pub.publish(cmd)

            # Update target angle display
            self._widget.target_coxa_label.setText(
                f"Target Coxa: {self._widget.coxa_slider.value()}°")
            self._widget.target_femur_label.setText(
                f"Target Femur: {self._widget.femur_slider.value()}°")
            self._widget.target_tibia_label.setText(
                f"Target Tibia: {self._widget.tibia_slider.value()}°")
            self._widget.velocity_label.setText(
                f"Velocity: {self._widget.velocity_slider.value()}%")

            rospy.logdebug(f"Sent joint command - Coxa: {cmd.coxa_angle:.2f} rad, " +
                          f"Femur: {cmd.femur_angle:.2f} rad, " +
                          f"Tibia: {cmd.tibia_angle:.2f} rad, " +
                          f"Velocity: {cmd.velocity:.2f}")

        except Exception as e:
            rospy.logerr(f"Error in send_joint_command: {str(e)}")

    def send_position_command(self):
        """Send position command when the move button is clicked"""
        try:
            # Create position command message
            pos_cmd = LegPosition()
            pos_cmd.x = self._widget.x_input.value()
            pos_cmd.y = self._widget.y_input.value()
            pos_cmd.z = self._widget.z_input.value()

            # Publish position command
            self.position_pub.publish(pos_cmd)

            rospy.loginfo(f"Sent position command - X: {pos_cmd.x:.1f} mm, " +
                         f"Y: {pos_cmd.y:.1f} mm, " +
                         f"Z: {pos_cmd.z:.1f} mm")

        except Exception as e:
            rospy.logerr(f"Error in send_position_command: {str(e)}")

    def position_callback(self, msg):
        """
        Update current position display when new position data is received
        Args:
            msg (LegPosition): Message containing current end-effector position
        """
        try:
            # Store current position
            self._current_position = msg

            # Update position labels
            self._widget.current_x_label.setText(f"X Position: {msg.x:.1f} mm")
            self._widget.current_y_label.setText(f"Y Position: {msg.y:.1f} mm")
            self._widget.current_z_label.setText(f"Z Position: {msg.z:.1f} mm")

            rospy.logdebug(f"Received position - X: {msg.x:.1f} mm, " +
                          f"Y: {msg.y:.1f} mm, " +
                          f"Z: {msg.z:.1f} mm")

        except Exception as e:
            rospy.logerr(f"Error in position_callback: {str(e)}")

    def timer_callback(self, event):
        """
        Periodic update function for monitoring communication health
        Args:
            event: Timer event information (not used)
        """
        try:
            # Check if we're receiving position updates
            if self._current_position is not None:
                time_since_last_update = rospy.Time.now() - rospy.Time(0)
                if time_since_last_update.to_sec() > 1.0:  # No updates for 1 second
                    rospy.logwarn("No position updates received for 1 second")

        except Exception as e:
            rospy.logerr(f"Error in timer_callback: {str(e)}")

    def shutdown_plugin(self):
        """Clean up subscribers and publishers when plugin is shut down"""
        try:
            # Unregister all subscribers and publishers
            self.current_pos_sub.unregister()
            self.joint_angles_sub.unregister()
            self.target_pos_pub.unregister()
            self.position_pub.unregister()
            
            # Stop the update timer
            self._update_timer.shutdown()

            rospy.loginfo("LegControllerGUI shutdown successfully")

        except Exception as e:
            rospy.logerr(f"Error in shutdown_plugin: {str(e)}")

    def save_settings(self, plugin_settings, instance_settings):
        """
        Save GUI settings
        Currently not implemented, but required by Plugin interface
        """
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        """
        Restore GUI settings
        Currently not implemented, but required by Plugin interface
        """
        pass
