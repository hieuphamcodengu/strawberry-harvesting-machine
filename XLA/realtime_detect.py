import cv2
from ultralytics import YOLO
import time
import numpy as np
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import threading
import serial
import serial.tools.list_ports
import json
import os

class StrawberryDetectorApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Strawberry Detection System - YOLOv8")
        self.root.geometry("1200x700")
        self.root.configure(bg='#2b2b2b')
        
        # Load model
        self.model = YOLO('best.pt')
        
        # Class names và colors
        self.class_names = {0: 'Ripe', 1: 'Unripe'}
        self.colors = {
            0: (0, 255, 0),    # Ripe - xanh lá
            1: (0, 0, 255)     # Unripe - đỏ
        }
        
        # Biến trạng thái
        self.is_running = False
        self.cap = None
        self.current_camera = 0
        self.conf_threshold = 0.5
        self.iou_threshold = 0.45
        self.brightness = 0
        self.fps = 0
        self.total_objects = 0
        self.frame_count = 0
        
        # Distance calculation parameters
        self.show_distance = True
        self.show_coordinates = True  # Hiển thị tọa độ 3D
        self.focal_length = 615  # Focal length pixel (cần calibrate)
        self.real_width = 3.0    # Chiều rộng thực của dâu tây (cm) - có thể điều chỉnh
        self.calibration_distance = 30.0  # Khoảng cách calibration (cm)
        self.flip_horizontal = True  # Flip camera horizontally (mirror mode)
        self.image_width = 640   # Chiều rộng ảnh
        self.image_height = 480  # Chiều cao ảnh
        
        # Object tracking
        self.tracking_enabled = True  # Bật tracking để bám đối tượng
        
        # Serial communication
        self.serial_port = None
        self.serial_connected = False
        self.serial_window = None
        self.test_mode_active = False  # Test mode: gửi T# liên tục
        self.test_thread = None
        self.auto_stop_sent = False     # Đã gửi D# khi dâu vào zone
        self.coord_send_time = 0        # Thời điểm gửi D# để delay 1s
        self.coord_to_send = None       # Tọa độ từ detection (để hiển thị)
        self.saved_coord_for_auto = None  # Tọa độ đã save từ input để gửi auto
        
        # Config file
        self.config_file = "strawberry_config.txt"
        
        # Target zone lines (X coordinates - vertical lines)
        self.x_line_left = 250     # Đường trái (pixel)
        self.x_line_right = 390    # Đường phải (pixel)
        self.show_target_zone = True
        self.auto_stop_enabled = False  # Tự động dừng khi dâu vào vùng
        
        # Load config from file
        self.load_config()
        
        # Setup GUI
        self.setup_gui()
        
        # Update loop
        self.update_frame()
        
    def setup_gui(self):
        # Main container
        main_frame = tk.Frame(self.root, bg='#2b2b2b')
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Left panel - Video display
        left_panel = tk.Frame(main_frame, bg='#2b2b2b')
        left_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))
        
        # Title
        title_label = tk.Label(left_panel, text="🍓 STRAWBERRY DETECTION", 
                               font=('Arial', 16, 'bold'), bg='#2b2b2b', fg='#00ff00')
        title_label.pack(pady=10)
        
        # Video frame
        self.video_label = tk.Label(left_panel, bg='black')
        self.video_label.pack(fill=tk.BOTH, expand=True)
        
        # Status bar
        status_frame = tk.Frame(left_panel, bg='#1e1e1e')
        status_frame.pack(fill=tk.X, pady=(10, 0))
        
        self.status_label = tk.Label(status_frame, text="Status: STOPPED", 
                                     font=('Arial', 10), bg='#1e1e1e', fg='#ff4444')
        self.status_label.pack(side=tk.LEFT, padx=10, pady=5)
        
        self.fps_label = tk.Label(status_frame, text="FPS: 0.0", 
                                  font=('Arial', 10), bg='#1e1e1e', fg='#00ffff')
        self.fps_label.pack(side=tk.LEFT, padx=10)
        
        self.objects_label = tk.Label(status_frame, text="Objects: 0", 
                                      font=('Arial', 10), bg='#1e1e1e', fg='#00ffff')
        self.objects_label.pack(side=tk.LEFT, padx=10)
        
        # Right panel - Controls với Scrollbar
        right_container = tk.Frame(main_frame, bg='#1e1e1e', width=350)
        right_container.pack(side=tk.RIGHT, fill=tk.Y)
        right_container.pack_propagate(False)
        
        # Canvas và Scrollbar
        canvas = tk.Canvas(right_container, bg='#1e1e1e', highlightthickness=0)
        scrollbar = tk.Scrollbar(right_container, orient="vertical", command=canvas.yview)
        right_panel = tk.Frame(canvas, bg='#1e1e1e')
        
        right_panel.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=right_panel, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        # Bind mouse wheel
        def _on_mousewheel(event):
            canvas.yview_scroll(int(-1*(event.delta/120)), "units")
        canvas.bind_all("<MouseWheel>", _on_mousewheel)
        
        # Control panel title
        control_title = tk.Label(right_panel, text="⚙️ CONTROL PANEL", 
                                font=('Arial', 14, 'bold'), bg='#1e1e1e', fg='#ffffff')
        control_title.pack(pady=15)
        
        # Start/Stop button
        self.start_button = tk.Button(right_panel, text="▶ START DETECTION", 
                                      command=self.toggle_detection,
                                      font=('Arial', 12, 'bold'),
                                      bg='#00aa00', fg='white',
                                      activebackground='#00ff00',
                                      relief=tk.RAISED, bd=3,
                                      height=2, cursor='hand2')
        self.start_button.pack(pady=10, padx=20, fill=tk.X)
        
        # Serial Control button
        serial_button = tk.Button(right_panel, text="📡 SERIAL CONTROL", 
                                 command=self.open_serial_window,
                                 font=('Arial', 11, 'bold'),
                                 bg='#0066cc', fg='white',
                                 activebackground='#0088ff',
                                 relief=tk.RAISED, bd=3,
                                 height=1, cursor='hand2')
        serial_button.pack(pady=10, padx=20, fill=tk.X)
        
        # Camera selection
        self.create_control_group(right_panel, "📷 Camera Selection", 
                                 self.create_camera_selector)
        
        # Confidence slider
        self.create_control_group(right_panel, "🎯 Confidence Threshold", 
                                 lambda p: self.create_slider(p, "Confidence:", 0, 100, 50, 
                                                              lambda v: setattr(self, 'conf_threshold', float(v)/100)))
        
        # IOU slider
        self.create_control_group(right_panel, "🔲 IOU Threshold", 
                                 lambda p: self.create_slider(p, "IOU:", 0, 100, 45, 
                                                              lambda v: setattr(self, 'iou_threshold', float(v)/100)))
        
        # Brightness slider
        self.create_control_group(right_panel, "💡 Brightness", 
                                 lambda p: self.create_slider(p, "Brightness:", 0, 200, 100, 
                                                              lambda v: setattr(self, 'brightness', int(v)-100)))
        
        # Distance settings
        self.create_control_group(right_panel, "📏 Distance Settings", 
                                 self.create_distance_controls)
        
        # Target Zone controls
        self.create_control_group(right_panel, "🎯 Target Zone (X)", 
                                 self.create_target_zone_controls)
        
        # Save button
        save_button = tk.Button(right_panel, text="💾 Save Current Frame", 
                               command=self.save_frame,
                               font=('Arial', 10),
                               bg='#0066cc', fg='white',
                               activebackground='#0088ff',
                               relief=tk.RAISED, bd=2,
                               cursor='hand2')
        save_button.pack(pady=10, padx=20, fill=tk.X)
        
        # Save config button
        config_button = tk.Button(right_panel, text="⚙️ SAVE ALL CONFIG", 
                                 command=self.save_all_config,
                                 font=('Arial', 11, 'bold'),
                                 bg='#00aa00', fg='white',
                                 activebackground='#00cc00',
                                 relief=tk.RAISED, bd=3,
                                 cursor='hand2')
        config_button.pack(pady=10, padx=20, fill=tk.X)
        
        # Info section
        info_frame = tk.Frame(right_panel, bg='#2b2b2b', relief=tk.SUNKEN, bd=2)
        info_frame.pack(pady=10, padx=20, fill=tk.BOTH, expand=True)
        
        info_title = tk.Label(info_frame, text="ℹ️ Information", 
                             font=('Arial', 11, 'bold'), bg='#2b2b2b', fg='#ffffff')
        info_title.pack(pady=5)
        
        info_text = tk.Label(info_frame, 
                            text="• Ripe: Green box\n• Unripe: Red box\n\nCalibration:\n1. Place at 30cm\n2. Set width\n3. Click Calibrate\n\nCoordinates:\nX: Left(-)/Right(+)\nY: Up(+)/Down(-)\nZ: Depth (cm)",
                            font=('Arial', 8), bg='#2b2b2b', fg='#cccccc',
                            justify=tk.LEFT)
        info_text.pack(pady=5, padx=10)
        
    def create_control_group(self, parent, title, content_creator):
        frame = tk.LabelFrame(parent, text=title, font=('Arial', 10, 'bold'),
                             bg='#1e1e1e', fg='#ffffff', bd=2, relief=tk.GROOVE)
        frame.pack(pady=10, padx=20, fill=tk.X)
        content_creator(frame)
        
    def create_camera_selector(self, parent):
        cam_frame = tk.Frame(parent, bg='#1e1e1e')
        cam_frame.pack(pady=10, padx=10, fill=tk.X)
        
        tk.Label(cam_frame, text="Camera Index:", font=('Arial', 9),
                bg='#1e1e1e', fg='#cccccc').pack(side=tk.LEFT, padx=5)
        
        self.camera_var = tk.StringVar(value="0")
        camera_combo = ttk.Combobox(cam_frame, textvariable=self.camera_var,
                                   values=["0", "1", "2", "3"], width=5, state='readonly')
        camera_combo.pack(side=tk.LEFT, padx=5)
        camera_combo.bind('<<ComboboxSelected>>', self.on_camera_change)
        
    def create_distance_controls(self, parent):
        dist_frame = tk.Frame(parent, bg='#1e1e1e')
        dist_frame.pack(pady=10, padx=10, fill=tk.X)
        
        # Toggle distance display
        self.distance_var = tk.BooleanVar(value=True)
        distance_check = tk.Checkbutton(dist_frame, text="Show Distance", 
                                       variable=self.distance_var,
                                       command=lambda: setattr(self, 'show_distance', self.distance_var.get()),
                                       bg='#1e1e1e', fg='#cccccc', 
                                       selectcolor='#2b2b2b', font=('Arial', 9))
        distance_check.pack(anchor=tk.W, pady=5)
        
        # Toggle flip horizontal (mirror mode)
        self.flip_var = tk.BooleanVar(value=True)
        flip_check = tk.Checkbutton(dist_frame, text="🔄 Mirror Mode", 
                                   variable=self.flip_var,
                                   command=lambda: setattr(self, 'flip_horizontal', self.flip_var.get()),
                                   bg='#1e1e1e', fg='#cccccc', 
                                   selectcolor='#2b2b2b', font=('Arial', 9))
        flip_check.pack(anchor=tk.W, pady=5)
        
        # Toggle coordinates display
        self.coord_var = tk.BooleanVar(value=True)
        coord_check = tk.Checkbutton(dist_frame, text="📍 Show Coordinates (X,Y,Z)", 
                                    variable=self.coord_var,
                                    command=lambda: setattr(self, 'show_coordinates', self.coord_var.get()),
                                    bg='#1e1e1e', fg='#cccccc', 
                                    selectcolor='#2b2b2b', font=('Arial', 9))
        coord_check.pack(anchor=tk.W, pady=5)
        
        # Toggle tracking
        self.tracking_var = tk.BooleanVar(value=True)
        tracking_check = tk.Checkbutton(dist_frame, text="🎯 Enable Tracking (ByteTrack)", 
                                       variable=self.tracking_var,
                                       command=lambda: setattr(self, 'tracking_enabled', self.tracking_var.get()),
                                       bg='#1e1e1e', fg='#cccccc', 
                                       selectcolor='#2b2b2b', font=('Arial', 9))
        tracking_check.pack(anchor=tk.W, pady=5)
        
        # Real width input
        width_frame = tk.Frame(dist_frame, bg='#1e1e1e')
        width_frame.pack(fill=tk.X, pady=5)
        tk.Label(width_frame, text="Object Width (cm):", font=('Arial', 8),
                bg='#1e1e1e', fg='#cccccc').pack(side=tk.LEFT)
        self.width_entry = tk.Entry(width_frame, width=8, bg='#2b2b2b', fg='#ffffff')
        self.width_entry.insert(0, "3.0")
        self.width_entry.pack(side=tk.LEFT, padx=5)
        self.width_entry.bind('<Return>', lambda e: setattr(self, 'real_width', float(self.width_entry.get())))
        
        # Calibration button
        calib_button = tk.Button(dist_frame, text="📐 Calibrate", 
                                command=self.calibrate_camera,
                                font=('Arial', 8),
                                bg='#ff6600', fg='white',
                                cursor='hand2')
        calib_button.pack(pady=5, fill=tk.X)
        
        # Calibration info
        self.calib_label = tk.Label(dist_frame, 
                                   text=f"Focal: {self.focal_length:.0f}px",
                                   font=('Arial', 8), bg='#1e1e1e', fg='#ffaa00')
        self.calib_label.pack(pady=2)
    
    def create_target_zone_controls(self, parent):
        zone_frame = tk.Frame(parent, bg='#1e1e1e')
        zone_frame.pack(pady=10, padx=10, fill=tk.X)
        
        # Toggle show target zone
        self.zone_var = tk.BooleanVar(value=True)
        zone_check = tk.Checkbutton(zone_frame, text="Show Target Zone", 
                                    variable=self.zone_var,
                                    command=lambda: setattr(self, 'show_target_zone', self.zone_var.get()),
                                    bg='#1e1e1e', fg='#cccccc', 
                                    selectcolor='#2b2b2b', font=('Arial', 9))
        zone_check.pack(anchor=tk.W, pady=5)
        
        # Auto stop when target in zone
        self.auto_stop_var = tk.BooleanVar(value=False)
        auto_check = tk.Checkbutton(zone_frame, text="🚦 Auto Stop in Zone", 
                                   variable=self.auto_stop_var,
                                   command=lambda: setattr(self, 'auto_stop_enabled', self.auto_stop_var.get()),
                                   bg='#1e1e1e', fg='#cccccc', 
                                   selectcolor='#2b2b2b', font=('Arial', 9))
        auto_check.pack(anchor=tk.W, pady=5)
        
        # X Left line input
        left_frame = tk.Frame(zone_frame, bg='#1e1e1e')
        left_frame.pack(fill=tk.X, pady=5)
        tk.Label(left_frame, text="X Left (px):", font=('Arial', 8),
                bg='#1e1e1e', fg='#cccccc').pack(side=tk.LEFT)
        self.x_left_entry = tk.Entry(left_frame, width=8, bg='#2b2b2b', fg='#ffffff')
        self.x_left_entry.insert(0, str(self.x_line_left))
        self.x_left_entry.pack(side=tk.LEFT, padx=5)
        self.x_left_entry.bind('<Return>', lambda e: self.update_target_zone())
        
        # X Right line input
        right_frame = tk.Frame(zone_frame, bg='#1e1e1e')
        right_frame.pack(fill=tk.X, pady=5)
        tk.Label(right_frame, text="X Right (px):", font=('Arial', 8),
                bg='#1e1e1e', fg='#cccccc').pack(side=tk.LEFT)
        self.x_right_entry = tk.Entry(right_frame, width=8, bg='#2b2b2b', fg='#ffffff')
        self.x_right_entry.insert(0, str(self.x_line_right))
        self.x_right_entry.pack(side=tk.LEFT, padx=5)
        self.x_right_entry.bind('<Return>', lambda e: self.update_target_zone())
        
        # Apply button
        apply_btn = tk.Button(zone_frame, text="✓ Apply Zone", 
                             command=self.update_target_zone,
                             font=('Arial', 8),
                             bg='#00aa00', fg='white',
                             cursor='hand2')
        apply_btn.pack(pady=5, fill=tk.X)
        
        # Info label
        info_label = tk.Label(zone_frame, 
                             text="Image size: 640x480",
                             font=('Arial', 7), bg='#1e1e1e', fg='#888888')
        info_label.pack(pady=2)
    
    def update_target_zone(self):
        """Update target zone lines từ entry"""
        try:
            self.x_line_left = int(self.x_left_entry.get())
            self.x_line_right = int(self.x_right_entry.get())
            
            # Validate
            if self.x_line_left >= self.x_line_right:
                print("Warning: X Left must be < X Right")
                return
            
            print(f"Target zone updated: X={self.x_line_left} to {self.x_line_right}")
            
            # Lưu tất cả config
            self.save_all_config()
        except ValueError:
            print("Invalid input! Use numbers only.")
    
    def save_config(self):
        """Lưu config ra file txt (chỉ cơ bản, cho auto-save)"""
        config = {
            "real_width": self.real_width,
            "focal_length": self.focal_length,
            "x_line_left": self.x_line_left,
            "x_line_right": self.x_line_right
        }
        try:
            with open(self.config_file, 'w') as f:
                json.dump(config, f, indent=4)
            print(f"Config saved to {self.config_file}")
        except Exception as e:
            print(f"Error saving config: {e}")
    
    def save_all_config(self):
        """Lưu tất cả cài đặt ra file txt"""
        config = {
            "real_width": self.real_width,
            "focal_length": self.focal_length,
            "x_line_left": self.x_line_left,
            "x_line_right": self.x_line_right,
            "conf_threshold": self.conf_threshold,
            "iou_threshold": self.iou_threshold,
            "brightness": self.brightness,
            "show_distance": self.show_distance,
            "show_coordinates": self.show_coordinates,
            "flip_horizontal": self.flip_horizontal,
            "tracking_enabled": self.tracking_enabled,
            "show_target_zone": self.show_target_zone,
            "auto_stop_enabled": self.auto_stop_enabled,
            "current_camera": self.current_camera
        }
        try:
            with open(self.config_file, 'w') as f:
                json.dump(config, f, indent=4)
            print(f"\n=== ALL CONFIG SAVED ===")
            print(f"File: {self.config_file}")
            print(f"  Detection: conf={self.conf_threshold:.2f}, iou={self.iou_threshold:.2f}")
            print(f"  Brightness: {self.brightness}")
            print(f"  Width: {self.real_width}cm, Focal: {self.focal_length}px")
            print(f"  Zone: X={self.x_line_left} to {self.x_line_right}")
            print(f"  Camera: {self.current_camera}")
            print(f"========================\n")
            
            # Hiển thị thông báo trên GUI
            if hasattr(self, 'status_label'):
                self.status_label.config(text="✅ All config saved successfully!")
                self.root.after(3000, lambda: self.status_label.config(text="Ready"))
                
        except Exception as e:
            print(f"Error saving config: {e}")
    
    def load_config(self):
        """Đọc config từ file txt"""
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r') as f:
                    config = json.load(f)
                
                # Load các giá trị cơ bản
                self.real_width = config.get("real_width", 3.0)
                self.focal_length = config.get("focal_length", 615)
                self.x_line_left = config.get("x_line_left", 250)
                self.x_line_right = config.get("x_line_right", 390)
                
                # Load các cài đặt khác (nếu có)
                self.conf_threshold = config.get("conf_threshold", 0.5)
                self.iou_threshold = config.get("iou_threshold", 0.45)
                self.brightness = config.get("brightness", 0)
                self.show_distance = config.get("show_distance", True)
                self.show_coordinates = config.get("show_coordinates", True)
                self.flip_horizontal = config.get("flip_horizontal", True)
                self.tracking_enabled = config.get("tracking_enabled", True)
                self.show_target_zone = config.get("show_target_zone", True)
                self.auto_stop_enabled = config.get("auto_stop_enabled", False)
                self.current_camera = config.get("current_camera", 0)
                
                print(f"Config loaded from {self.config_file}")
                print(f"  Width: {self.real_width}cm, Focal: {self.focal_length}px")
                print(f"  Zone: X={self.x_line_left} to {self.x_line_right}")
                print(f"  Detection: conf={self.conf_threshold:.2f}, iou={self.iou_threshold:.2f}")
                print(f"  Brightness: {self.brightness}")
            except Exception as e:
                print(f"Error loading config: {e}")
        else:
            print("No config file found. Using default values.")
    
    def open_serial_window(self):
        """Mở cửa sổ Serial Control"""
        if self.serial_window is not None and self.serial_window.winfo_exists():
            self.serial_window.lift()
            return
        
        self.serial_window = tk.Toplevel(self.root)
        self.serial_window.title("📡 Serial Control - ESP32 Communication")
        self.serial_window.geometry("600x500")
        self.serial_window.configure(bg='#2b2b2b')
        
        # Title
        title = tk.Label(self.serial_window, text="📡 ESP32 SERIAL CONTROL",
                        font=('Arial', 14, 'bold'), bg='#2b2b2b', fg='#00ff00')
        title.pack(pady=10)
        
        # Connection frame
        conn_frame = tk.LabelFrame(self.serial_window, text="Connection Settings",
                                  font=('Arial', 10, 'bold'), bg='#1e1e1e', fg='#ffffff')
        conn_frame.pack(pady=10, padx=20, fill=tk.X)
        
        # COM Port
        port_frame = tk.Frame(conn_frame, bg='#1e1e1e')
        port_frame.pack(pady=10, padx=10, fill=tk.X)
        
        tk.Label(port_frame, text="COM Port:", font=('Arial', 10),
                bg='#1e1e1e', fg='#cccccc').pack(side=tk.LEFT, padx=5)
        
        self.com_var = tk.StringVar()
        self.com_combo = ttk.Combobox(port_frame, textvariable=self.com_var, width=15)
        self.com_combo.pack(side=tk.LEFT, padx=5)
        
        refresh_btn = tk.Button(port_frame, text="🔄", command=self.refresh_ports,
                               bg='#0066cc', fg='white', cursor='hand2')
        refresh_btn.pack(side=tk.LEFT, padx=5)
        
        # Baudrate
        baud_frame = tk.Frame(conn_frame, bg='#1e1e1e')
        baud_frame.pack(pady=5, padx=10, fill=tk.X)
        
        tk.Label(baud_frame, text="Baudrate:", font=('Arial', 10),
                bg='#1e1e1e', fg='#cccccc').pack(side=tk.LEFT, padx=5)
        
        self.baud_var = tk.StringVar(value="115200")
        baud_combo = ttk.Combobox(baud_frame, textvariable=self.baud_var,
                                 values=["9600", "57600", "115200", "230400"],
                                 width=12, state='readonly')
        baud_combo.pack(side=tk.LEFT, padx=5)
        
        # Connect/Disconnect buttons
        btn_frame = tk.Frame(conn_frame, bg='#1e1e1e')
        btn_frame.pack(pady=10, padx=10, fill=tk.X)
        
        self.connect_btn = tk.Button(btn_frame, text="🔌 CONNECT",
                                    command=self.connect_serial,
                                    font=('Arial', 10, 'bold'),
                                    bg='#00aa00', fg='white',
                                    cursor='hand2', width=15)
        self.connect_btn.pack(side=tk.LEFT, padx=5, expand=True, fill=tk.X)
        
        self.disconnect_btn = tk.Button(btn_frame, text="🔌 DISCONNECT",
                                       command=self.disconnect_serial,
                                       font=('Arial', 10, 'bold'),
                                       bg='#aa0000', fg='white',
                                       cursor='hand2', width=15, state=tk.DISABLED)
        self.disconnect_btn.pack(side=tk.LEFT, padx=5, expand=True, fill=tk.X)
        
        # Test commands frame
        test_frame = tk.LabelFrame(self.serial_window, text="Test Commands",
                                  font=('Arial', 10, 'bold'), bg='#1e1e1e', fg='#ffffff')
        test_frame.pack(pady=10, padx=20, fill=tk.X)
        
        test_btn_frame = tk.Frame(test_frame, bg='#1e1e1e')
        test_btn_frame.pack(pady=10, padx=10)
        
        tk.Button(test_btn_frame, text="▶ TIEN (T#)", command=lambda: self.send_command("T#"),
                 bg='#00aa00', fg='white', font=('Arial', 9, 'bold'),
                 cursor='hand2', width=12).pack(side=tk.LEFT, padx=5)
        
        tk.Button(test_btn_frame, text="⏸ DUNG (D#)", command=lambda: self.send_command("D#"),
                 bg='#aa0000', fg='white', font=('Arial', 9, 'bold'),
                 cursor='hand2', width=12).pack(side=tk.LEFT, padx=5)
        
        # Coordinate input frame
        coord_frame = tk.LabelFrame(self.serial_window, text="📍 Manual Coordinate Send",
                                   font=('Arial', 10, 'bold'), bg='#1e1e1e', fg='#ffffff')
        coord_frame.pack(pady=10, padx=20, fill=tk.X)
        
        coord_info = tk.Label(coord_frame,
                             text="Gửi tọa độ thủ công để test tool (format: Gz,y#)",
                             font=('Arial', 8), bg='#1e1e1e', fg='#cccccc')
        coord_info.pack(pady=5)
        
        # Z coordinate input
        z_input_frame = tk.Frame(coord_frame, bg='#1e1e1e')
        z_input_frame.pack(pady=5, padx=10, fill=tk.X)
        tk.Label(z_input_frame, text="Z (cm):", font=('Arial', 9),
                bg='#1e1e1e', fg='#cccccc', width=8).pack(side=tk.LEFT)
        self.z_coord_entry = tk.Entry(z_input_frame, width=10, bg='#2b2b2b', fg='#ffffff')
        self.z_coord_entry.insert(0, "50.0")
        self.z_coord_entry.pack(side=tk.LEFT, padx=5)
        
        # Y coordinate input
        y_input_frame = tk.Frame(coord_frame, bg='#1e1e1e')
        y_input_frame.pack(pady=5, padx=10, fill=tk.X)
        tk.Label(y_input_frame, text="Y (cm):", font=('Arial', 9),
                bg='#1e1e1e', fg='#cccccc', width=8).pack(side=tk.LEFT)
        self.y_coord_entry = tk.Entry(y_input_frame, width=10, bg='#2b2b2b', fg='#ffffff')
        self.y_coord_entry.insert(0, "10.0")
        self.y_coord_entry.pack(side=tk.LEFT, padx=5)
        
        # Button frame for Save and Send
        btn_frame = tk.Frame(coord_frame, bg='#1e1e1e')
        btn_frame.pack(pady=10, padx=10, fill=tk.X)
        
        # Save button - lưu tọa độ để gửi auto khi detection
        save_coord_btn = tk.Button(btn_frame, text="💾 Save for Auto",
                                  command=self.save_coord_for_auto,
                                  font=('Arial', 9, 'bold'),
                                  bg='#00aa00', fg='white',
                                  cursor='hand2')
        save_coord_btn.pack(side=tk.LEFT, padx=5, expand=True, fill=tk.X)
        
        # Send coordinate button
        send_coord_btn = tk.Button(btn_frame, text="📤 Send Coord (Gz,y#)",
                                  command=self.send_manual_coord,
                                  font=('Arial', 9, 'bold'),
                                  bg='#ff6600', fg='white',
                                  cursor='hand2')
        send_coord_btn.pack(side=tk.LEFT, padx=5, expand=True, fill=tk.X)
        
        # Test mode frame
        test_mode_frame = tk.LabelFrame(self.serial_window, text="🧪 Test Mode (Auto)",
                                       font=('Arial', 10, 'bold'), bg='#1e1e1e', fg='#ffffff')
        test_mode_frame.pack(pady=10, padx=20, fill=tk.X)
        
        test_mode_info = tk.Label(test_mode_frame, 
                                 text="Gửi T# liên tục mỗi 50ms để test auto stop",
                                 font=('Arial', 8), bg='#1e1e1e', fg='#cccccc')
        test_mode_info.pack(pady=5)
        
        test_mode_btn_frame = tk.Frame(test_mode_frame, bg='#1e1e1e')
        test_mode_btn_frame.pack(pady=10, padx=10)
        
        self.start_test_btn = tk.Button(test_mode_btn_frame, text="▶ START TEST",
                                       command=self.start_test_mode,
                                       font=('Arial', 10, 'bold'),
                                       bg='#00aa00', fg='white',
                                       cursor='hand2', width=15)
        self.start_test_btn.pack(side=tk.LEFT, padx=5)
        
        self.stop_test_btn = tk.Button(test_mode_btn_frame, text="⏹ STOP TEST",
                                      command=self.stop_test_mode,
                                      font=('Arial', 10, 'bold'),
                                      bg='#aa0000', fg='white',
                                      cursor='hand2', width=15, state=tk.DISABLED)
        self.stop_test_btn.pack(side=tk.LEFT, padx=5)
        
        # Debug log frame
        log_frame = tk.LabelFrame(self.serial_window, text="📋 Debug Log",
                                 font=('Arial', 10, 'bold'), bg='#1e1e1e', fg='#ffffff')
        log_frame.pack(pady=10, padx=20, fill=tk.BOTH, expand=True)
        
        # Text widget với scrollbar
        log_scroll = tk.Scrollbar(log_frame)
        log_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        
        self.log_text = tk.Text(log_frame, height=10, bg='#0a0a0a', fg='#00ff00',
                               font=('Consolas', 9), yscrollcommand=log_scroll.set)
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        log_scroll.config(command=self.log_text.yview)
        
        # Clear log button
        tk.Button(log_frame, text="🗑️ Clear Log", command=self.clear_log,
                 bg='#666666', fg='white', cursor='hand2').pack(pady=5)
        
        # Refresh ports khi mở cửa sổ
        self.refresh_ports()
        self.log_message("Serial window opened. Select COM port and click CONNECT.")
        
        # Start reading thread nếu đã kết nối
        if self.serial_connected:
            self.start_serial_read_thread()
    
    def refresh_ports(self):
        """Refresh danh sách COM ports"""
        ports = serial.tools.list_ports.comports()
        port_list = [port.device for port in ports]
        
        if hasattr(self, 'com_combo'):
            self.com_combo['values'] = port_list
            if port_list:
                self.com_combo.current(0)
                self.log_message(f"Found {len(port_list)} COM port(s): {', '.join(port_list)}")
            else:
                self.log_message("No COM ports found!")
    
    def connect_serial(self):
        """Kết nối với ESP32 qua Serial"""
        try:
            port = self.com_var.get()
            baud = int(self.baud_var.get())
            
            if not port:
                self.log_message("[ERROR] Please select a COM port!", "red")
                return
            
            self.serial_port = serial.Serial(port, baud, timeout=1)
            self.serial_connected = True
            
            self.connect_btn.config(state=tk.DISABLED)
            self.disconnect_btn.config(state=tk.NORMAL)
            
            self.log_message(f"[SUCCESS] Connected to {port} @ {baud} baud", "green")
            
            # Start reading thread
            self.start_serial_read_thread()
            
        except Exception as e:
            self.log_message(f"[ERROR] Connection failed: {str(e)}", "red")
            self.serial_connected = False
    
    def disconnect_serial(self):
        """Ngắt kết nối Serial"""
        try:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
            
            self.serial_connected = False
            self.serial_port = None
            
            self.connect_btn.config(state=tk.NORMAL)
            self.disconnect_btn.config(state=tk.DISABLED)
            
            self.log_message("[INFO] Disconnected from serial port", "yellow")
            
        except Exception as e:
            self.log_message(f"[ERROR] Disconnect failed: {str(e)}", "red")
    
    def send_command(self, cmd):
        """Gửi lệnh đến ESP32"""
        if not self.serial_connected or not self.serial_port:
            self.log_message("[ERROR] Not connected! Click CONNECT first.", "red")
            return
        
        try:
            self.serial_port.write(cmd.encode())
            self.log_message(f"[SENT] {cmd}", "cyan")
        except Exception as e:
            self.log_message(f"[ERROR] Send failed: {str(e)}", "red")
    
    def start_serial_read_thread(self):
        """Bắt đầu thread đọc dữ liệu từ ESP32"""
        def read_serial():
            while self.serial_connected and self.serial_port and self.serial_port.is_open:
                try:
                    if self.serial_port.in_waiting > 0:
                        data = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                        if data:
                            # Kiểm tra emergency stop từ ESP32
                            if data == "STOP":
                                self.log_message("[ESP32] EMERGENCY STOP received!", "red")
                                # Tự động dừng test mode
                                if self.test_mode_active:
                                    self.root.after(0, self.stop_test_mode)
                            # Kiểm tra harvest complete từ ESP32
                            elif data == "HARVEST_DONE#":
                                self.log_message("✅ [HARVEST] COMPLETE! Strawberry harvested successfully!", "green")
                                # Có thể thêm âm thanh hoặc notification ở đây
                            else:
                                self.log_message(f"[ESP32] {data}", "white")
                    time.sleep(0.05)
                except Exception as e:
                    self.log_message(f"[ERROR] Read error: {str(e)}", "red")
                    break
        
        thread = threading.Thread(target=read_serial, daemon=True)
        thread.start()
    
    def log_message(self, message, color="white"):
        """Thêm message vào debug log"""
        if hasattr(self, 'log_text'):
            timestamp = time.strftime("%H:%M:%S")
            
            # Color mapping (không dùng trong Text widget, chỉ để tham khảo)
            self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
            
            # Tự động scroll xuống cuối
            self.log_text.see(tk.END)
    
    def clear_log(self):
        """Xóa debug log"""
        if hasattr(self, 'log_text'):
            self.log_text.delete(1.0, tk.END)
            self.log_message("Log cleared.")
    
    def save_coord_for_auto(self):
        """Lưu tọa độ từ input boxes để gửi tự động khi detection dừng bánh xe"""
        try:
            z_val = float(self.z_coord_entry.get())
            y_val = float(self.y_coord_entry.get())
            
            self.saved_coord_for_auto = (z_val, y_val)
            self.log_message(f"[SAVED] Auto-send coord: Z={z_val:.1f}, Y={y_val:.1f}", "green")
            self.log_message("[INFO] This coord will be sent after D# + 1s in auto mode", "cyan")
            
        except ValueError:
            self.log_message("[ERROR] Invalid coordinate values!", "red")
    
    def send_manual_coord(self):
        """Gửi tọa độ thủ công từ input boxes"""
        if not self.serial_connected or not self.serial_port:
            self.log_message("[ERROR] Not connected! Click CONNECT first.", "red")
            return
        
        try:
            z_val = float(self.z_coord_entry.get())
            y_val = float(self.y_coord_entry.get())
            
            coord_cmd = f"G{z_val:.1f},{y_val:.1f}#"
            self.send_command(coord_cmd)
            self.log_message(f"[MANUAL] Sent coordinates: {coord_cmd}", "cyan")
            
        except ValueError:
            self.log_message("[ERROR] Invalid coordinate values!", "red")
    
    def start_test_mode(self):
        """Bắt đầu test mode - gửi T# 1 lần duy nhất"""
        if not self.serial_connected or not self.serial_port:
            self.log_message("[ERROR] Not connected! Click CONNECT first.", "red")
            return
        
        self.test_mode_active = True
        self.auto_stop_sent = False  # Reset cờ
        self.start_test_btn.config(state=tk.DISABLED)
        self.stop_test_btn.config(state=tk.NORMAL)
        
        # Gửi lệnh T# 1 lần duy nhất
        try:
            self.serial_port.write("T#".encode())
            self.log_message("[TEST MODE] Started - Sent T# (ESP32 will handle continuous forward)", "green")
        except Exception as e:
            self.log_message(f"[ERROR] Failed to send T#: {str(e)}", "red")
            self.test_mode_active = False
            self.start_test_btn.config(state=tk.NORMAL)
            self.stop_test_btn.config(state=tk.DISABLED)
    
    def stop_test_mode(self):
        """Dừng test mode"""
        self.test_mode_active = False
        self.start_test_btn.config(state=tk.NORMAL)
        self.stop_test_btn.config(state=tk.DISABLED)
        
        self.log_message("[TEST MODE] Stopped", "yellow")
        
        # Gửi lệnh dừng
        if self.serial_port and self.serial_port.is_open:
            self.send_command("D#")
    
    def calibrate_camera(self):
        """Calibrate focal length using current detected object"""
        if hasattr(self, 'last_pixel_width') and self.last_pixel_width > 0:
            # Formula: Focal_Length = (Pixel_Width * Distance) / Real_Width
            self.real_width = float(self.width_entry.get())
            self.focal_length = (self.last_pixel_width * self.calibration_distance) / self.real_width
            self.calib_label.config(text=f"Focal: {self.focal_length:.0f}px")
            print(f"Calibrated! Focal Length: {self.focal_length:.2f}px")
            print(f"Place object at {self.calibration_distance}cm and click Calibrate")
            
            # Lưu tất cả config
            self.save_all_config()
        else:
            print("No object detected for calibration!")
    
    def calculate_distance(self, pixel_width):
        """Calculate distance using: Distance = (Real_Width * Focal_Length) / Pixel_Width"""
        if pixel_width > 0 and self.focal_length > 0:
            distance = (self.real_width * self.focal_length) / pixel_width
            return distance
        return 0
    
    def calculate_3d_coordinates(self, center_x, center_y, distance):
        """Calculate 3D coordinates (X, Y, Z) in cm
        - Z: depth (distance from camera)
        - X: horizontal position (negative = left, positive = right)
        - Y: vertical position (NEGATIVE = down, POSITIVE = up)
        Origin is at camera center
        """
        if distance > 0 and self.focal_length > 0:
            # Tâm ảnh
            img_center_x = self.image_width / 2
            img_center_y = self.image_height / 2
            
            # Tính offset từ tâm (pixel)
            offset_x = center_x - img_center_x
            offset_y = center_y - img_center_y
            
            # Chuyển đổi sang tọa độ thực (cm)
            # Công thức: Real_Coordinate = (Pixel_Offset * Distance) / Focal_Length
            X = (offset_x * distance) / self.focal_length
            # ĐẢO DẤU Y: trong ảnh Y tăng khi đi xuống, nhưng thực tế Y dương là đi lên
            Y = -(offset_y * distance) / self.focal_length
            Z = distance
            
            return X, Y, Z
        return 0, 0, 0
    
    def create_slider(self, parent, label, from_, to, default, command):
        slider_frame = tk.Frame(parent, bg='#1e1e1e')
        slider_frame.pack(pady=10, padx=10, fill=tk.X)
        
        label_widget = tk.Label(slider_frame, text=label, font=('Arial', 9),
                               bg='#1e1e1e', fg='#cccccc')
        label_widget.pack(anchor=tk.W)
        
        value_label = tk.Label(slider_frame, text=str(default), font=('Arial', 9, 'bold'),
                              bg='#1e1e1e', fg='#00ff00')
        value_label.pack(anchor=tk.E)
        
        slider = tk.Scale(slider_frame, from_=from_, to=to, orient=tk.HORIZONTAL,
                         bg='#2b2b2b', fg='#ffffff', troughcolor='#404040',
                         highlightthickness=0, bd=0, length=280,
                         command=lambda v: [command(v), value_label.config(text=f"{float(v):.2f}" if '.' in str(float(v)) else v)])
        slider.set(default)
        slider.pack(fill=tk.X)
        
        return slider
        
    def toggle_detection(self):
        self.is_running = not self.is_running
        
        if self.is_running:
            self.start_button.config(text="⏸ STOP DETECTION", bg='#aa0000', activebackground='#ff0000')
            self.status_label.config(text="Status: RUNNING", fg='#00ff00')
            
            # Mở camera
            if self.cap is None or not self.cap.isOpened():
                self.cap = cv2.VideoCapture(self.current_camera)
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        else:
            self.start_button.config(text="▶ START DETECTION", bg='#00aa00', activebackground='#00ff00')
            self.status_label.config(text="Status: STOPPED", fg='#ff4444')
            
    def on_camera_change(self, event=None):
        new_camera = int(self.camera_var.get())
        
        if new_camera != self.current_camera:
            if self.cap is not None:
                self.cap.release()
            
            self.current_camera = new_camera
            
            if self.is_running:
                self.cap = cv2.VideoCapture(self.current_camera)
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                
    def save_frame(self):
        if hasattr(self, 'current_frame') and self.current_frame is not None:
            filename = f'capture_{self.frame_count}.jpg'
            cv2.imwrite(filename, self.current_frame)
            self.frame_count += 1
            print(f"Đã lưu ảnh: {filename}")
            
            # Hiển thị thông báo
            self.status_label.config(text=f"Saved: {filename}", fg='#ffff00')
            self.root.after(2000, lambda: self.status_label.config(
                text="Status: RUNNING" if self.is_running else "Status: STOPPED",
                fg='#00ff00' if self.is_running else '#ff4444'))
        
    def update_frame(self):
        start_time = time.time()
        
        if self.is_running and self.cap is not None and self.cap.isOpened():
            ret, frame = self.cap.read()
            
            if ret:
                # Flip horizontal (mirror mode) nếu được bật
                if self.flip_horizontal:
                    frame = cv2.flip(frame, 1)
                
                # Điều chỉnh độ sáng
                if self.brightness != 0:
                    frame = cv2.convertScaleAbs(frame, alpha=1, beta=self.brightness)
                
                # Detect với tracking nếu enabled
                if self.tracking_enabled:
                    results = self.model.track(frame, 
                                             imgsz=640,
                                             conf=self.conf_threshold,
                                             iou=self.iou_threshold,
                                             persist=True,  # Giữ track ID giữa các frame
                                             tracker="bytetrack.yaml",  # ByteTrack tracker
                                             verbose=False)
                else:
                    results = self.model.predict(frame, 
                                               imgsz=640,
                                               conf=self.conf_threshold, 
                                               iou=self.iou_threshold, 
                                               verbose=False)
                
                # Vẽ target zone lines nếu được bật
                if self.show_target_zone:
                    # Đường trái (xanh lá) - vertical line
                    cv2.line(frame, (self.x_line_left, 0), (self.x_line_left, self.image_height), 
                            (0, 255, 0), 2)
                    cv2.putText(frame, f"X_LEFT: {self.x_line_left}", (self.x_line_left + 5, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    # Đường phải (vàng) - vertical line
                    cv2.line(frame, (self.x_line_right, 0), (self.x_line_right, self.image_height), 
                            (0, 255, 255), 2)
                    cv2.putText(frame, f"X_RIGHT: {self.x_line_right}", (self.x_line_right - 100, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                    
                    # Vẽ vùng target (semi-transparent)
                    overlay = frame.copy()
                    cv2.rectangle(overlay, (self.x_line_left, 0), 
                                (self.x_line_right, self.image_height), 
                                (0, 255, 0), -1)
                    cv2.addWeighted(overlay, 0.1, frame, 0.9, 0, frame)
                
                # Biến check xem có dâu trong zone không
                target_in_zone = False
                
                # Vẽ bounding boxes
                for result in results:
                    boxes = result.boxes
                    
                    for box in boxes:
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                        
                        conf = float(box.conf[0])
                        cls = int(box.cls[0])
                        
                        # Lấy track ID nếu có
                        track_id = None
                        if hasattr(box, 'id') and box.id is not None:
                            track_id = int(box.id[0])
                        
                        class_name = self.class_names.get(cls, 'Unknown')
                        color = self.colors.get(cls, (255, 255, 255))
                        
                        # Tính tâm của bounding box
                        center_x = int((x1 + x2) / 2)
                        center_y = int((y1 + y2) / 2)
                        
                        # Check xem tâm có nằm trong target zone không (theo trục X)
                        in_zone = self.x_line_left <= center_x <= self.x_line_right
                        if in_zone:
                            target_in_zone = True
                            # Đổi màu box thành màu cam nếu trong zone
                            color = (0, 165, 255)  # Orange
                            print(f"[DEBUG] Object in zone! center_x={center_x}, zone=[{self.x_line_left}, {self.x_line_right}]")
                        
                        # Vẽ box với màu đã xác định
                        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                        
                        # Hiển thị class name, confidence và track ID
                        label = f"{class_name} {conf:.2f}"
                        if track_id is not None:
                            label += f" ID:{track_id}"
                        cv2.putText(frame, label, (x1, y1 - 10),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                        
                        # Tính khoảng cách và tọa độ 3D
                        pixel_width = x2 - x1
                        self.last_pixel_width = pixel_width  # Lưu để calibrate
                        
                        distance = self.calculate_distance(pixel_width)
                        
                        if self.show_coordinates and distance > 0:
                            # Tính tọa độ 3D
                            X, Y, Z = self.calculate_3d_coordinates(center_x, center_y, distance)
                            
                            # Vẽ tọa độ bên dưới box (2 dòng)
                            coord_text1 = f"X:{X:+.1f} Y:{Y:+.1f}"
                            coord_text2 = f"Z:{Z:.1f}cm"
                            
                            # Dòng 1: X, Y
                            cv2.putText(frame, coord_text1, (x1, y2 + 18),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                            # Dòng 2: Z
                            cv2.putText(frame, coord_text2, (x1, y2 + 38),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                        elif self.show_distance and distance > 0:
                            # Chỉ hiển thị khoảng cách nếu không hiển thị tọa độ
                            distance_text = f"{distance:.1f}cm"
                            cv2.putText(frame, distance_text, (center_x - 30, y2 + 20),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                        
                        # Vẽ dấu chấm màu đỏ ở giữa box
                        cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)  # Chấm đỏ
                        cv2.circle(frame, (center_x, center_y), 6, (255, 255, 255), 1)  # Viền trắng
                        
                        # Vẽ dấu * ở trên cùng box (giữa theo chiều ngang)
                        cv2.putText(frame, '*', (center_x - 8, y1 - 10), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)  # Dấu * màu đỏ
                        cv2.putText(frame, '*', (center_x - 8, y1 - 10), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 1)  # Viền trắng
                
                # Auto stop nếu có dâu trong zone (chỉ trong test mode)
                # Debug: In ra các điều kiện
                if target_in_zone:
                    print(f"[DEBUG] Target in zone detected!")
                    print(f"  auto_stop_enabled: {self.auto_stop_enabled}")
                    print(f"  test_mode_active: {self.test_mode_active}")
                    print(f"  auto_stop_sent: {self.auto_stop_sent}")
                    print(f"  serial_connected: {self.serial_connected}")
                
                if self.auto_stop_enabled and target_in_zone and self.test_mode_active:
                    if not self.auto_stop_sent:  # Chỉ gửi 1 lần
                        if self.serial_connected and self.serial_port:
                            print("[DEBUG] Sending D# command...")
                            self.send_command("D#")  # Gửi lệnh dừng
                            self.auto_stop_sent = True  # Đánh dấu đã gửi
                            self.test_mode_active = False  # Tắt test mode
                            self.start_test_btn.config(state=tk.NORMAL)
                            self.stop_test_btn.config(state=tk.DISABLED)
                            self.log_message("[AUTO STOP] Target in zone - Sent D#", "yellow")
                            
                            # Lưu thời điểm để gửi tọa độ sau 1s
                            self.coord_send_time = time.time()
                            # Ghi log tọa độ sẽ gửi (từ saved_coord_for_auto)
                            if self.saved_coord_for_auto:
                                z_val, y_val = self.saved_coord_for_auto
                                print(f"[DEBUG] Will send SAVED coordinates after 1s: Z={z_val:.1f}, Y={y_val:.1f}")
                            else:
                                print("[DEBUG] No saved coordinates - will NOT send coord after 1s")
                            
                            print("[DEBUG] D# sent successfully!")
                        else:
                            print("[DEBUG] Serial not connected!")
                    else:
                        print("[DEBUG] D# already sent (auto_stop_sent=True)")
                    
                    # Hiển thị thông báo
                    cv2.putText(frame, "TARGET IN ZONE - STOPPED", (150, 50),
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
                
                # Cập nhật thông tin
                self.total_objects = len(results[0].boxes) if len(results) > 0 else 0
                self.current_frame = frame.copy()
                
                # Kiểm tra xem đã đến lúc gửi tọa độ chưa (sau 1s kể từ D#)
                if self.saved_coord_for_auto is not None and self.coord_send_time > 0:
                    if time.time() - self.coord_send_time >= 1.0:
                        # Gửi lệnh tọa độ Gz,y# (từ saved_coord_for_auto)
                        z_val, y_val = self.saved_coord_for_auto
                        coord_cmd = f"G{z_val:.1f},{y_val:.1f}#"
                        self.send_command(coord_cmd)
                        self.log_message(f"[AUTO COORD] Sent saved coord: {coord_cmd}", "cyan")
                        print(f"[DEBUG] Coordinate command sent: {coord_cmd}")
                        
                        # Reset
                        self.coord_to_send = None
                        self.coord_send_time = 0
                
                # Tính FPS
                self.fps = 1 / (time.time() - start_time)
                self.fps_label.config(text=f"FPS: {self.fps:.1f}")
                self.objects_label.config(text=f"Objects: {self.total_objects}")
                
                # Hiển thị frame
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(frame_rgb)
                img = img.resize((800, 600), Image.Resampling.LANCZOS)
                imgtk = ImageTk.PhotoImage(image=img)
                
                self.video_label.imgtk = imgtk
                self.video_label.configure(image=imgtk)
        else:
            # Hiển thị màn hình đen khi dừng
            blank = np.zeros((600, 800, 3), dtype=np.uint8)
            cv2.putText(blank, "STOPPED - Click START to begin", (150, 300),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            
            img = Image.fromarray(blank)
            imgtk = ImageTk.PhotoImage(image=img)
            self.video_label.imgtk = imgtk
            self.video_label.configure(image=imgtk)
        
        # Lặp lại
        self.root.after(10, self.update_frame)
        
    def on_closing(self):
        self.is_running = False
        self.test_mode_active = False  # Dừng test mode
        
        if self.cap is not None:
            self.cap.release()
        
        # Đóng serial port
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        
        # Lưu config cuối cùng
        self.save_config()
        
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = StrawberryDetectorApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()

