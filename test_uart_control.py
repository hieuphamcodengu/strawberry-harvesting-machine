import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports
import threading
import time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation
from collections import deque

class MotorControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Arduino Motor Control & Monitor")
        self.root.geometry("1000x700")
        
        self.ser = None
        self.is_connected = False
        self.is_reading = False
        self.is_sending = False  # Cờ để gửi liên tục
        
        # Data buffers cho plotting (lưu 200 điểm)
        self.time_data = deque(maxlen=200)
        self.target_L_data = deque(maxlen=200)
        self.actual_L_data = deque(maxlen=200)
        self.target_R_data = deque(maxlen=200)
        self.actual_R_data = deque(maxlen=200)
        self.start_time = time.time()
        
        self.create_widgets()
        self.setup_plot()
        
        # Bind sự kiện đóng cửa sổ
        self.root.protocol("WM_DELETE_WINDOW", self.exit_app)
        
    def create_widgets(self):
        # Frame kết nối COM
        connect_frame = tk.LabelFrame(self.root, text="COM Port Connection", padx=10, pady=10)
        connect_frame.pack(padx=10, pady=10, fill="x")
        
        tk.Label(connect_frame, text="COM Port:").grid(row=0, column=0, padx=5)
        self.com_combo = ttk.Combobox(connect_frame, width=15, state="readonly")
        self.com_combo.grid(row=0, column=1, padx=5)
        self.refresh_ports()
        
        tk.Button(connect_frame, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=5)
        self.connect_btn = tk.Button(connect_frame, text="Connect", command=self.toggle_connection, bg="green", fg="white")
        self.connect_btn.grid(row=0, column=3, padx=5)
        
        self.status_label = tk.Label(connect_frame, text="Disconnected", fg="red", font=("Arial", 10, "bold"))
        self.status_label.grid(row=0, column=4, padx=10)
        
        tk.Button(connect_frame, text="Exit", command=self.exit_app, bg="red", fg="white", font=("Arial", 10, "bold")).grid(row=0, column=5, padx=5)
        
        # Frame điều khiển động cơ
        control_frame = tk.LabelFrame(self.root, text="Motor Control", padx=10, pady=10)
        control_frame.pack(padx=10, pady=10, fill="x")
        
        # Motor Left
        tk.Label(control_frame, text="Left Motor", font=("Arial", 12, "bold")).grid(row=0, column=0, columnspan=3, pady=5)
        
        tk.Label(control_frame, text="Direction:").grid(row=1, column=0, sticky="w")
        self.L_dir_var = tk.IntVar(value=0)
        tk.Radiobutton(control_frame, text="Stop", variable=self.L_dir_var, value=0).grid(row=1, column=1)
        tk.Radiobutton(control_frame, text="Forward", variable=self.L_dir_var, value=1).grid(row=1, column=2)
        tk.Radiobutton(control_frame, text="Backward", variable=self.L_dir_var, value=2).grid(row=1, column=3)
        
        tk.Label(control_frame, text="RPM (0-250):").grid(row=2, column=0, sticky="w")
        self.L_rpm_scale = tk.Scale(control_frame, from_=0, to=250, orient="horizontal", length=300, command=lambda x: self.update_rpm_labels())
        self.L_rpm_scale.grid(row=2, column=1, columnspan=3)
        self.L_rpm_label = tk.Label(control_frame, text="0", font=("Arial", 12))
        self.L_rpm_label.grid(row=2, column=4, padx=10)
        
        # Separator
        ttk.Separator(control_frame, orient="horizontal").grid(row=3, column=0, columnspan=5, sticky="ew", pady=10)
        
        # Motor Right
        tk.Label(control_frame, text="Right Motor", font=("Arial", 12, "bold")).grid(row=4, column=0, columnspan=3, pady=5)
        
        tk.Label(control_frame, text="Direction:").grid(row=5, column=0, sticky="w")
        self.R_dir_var = tk.IntVar(value=0)
        tk.Radiobutton(control_frame, text="Stop", variable=self.R_dir_var, value=0).grid(row=5, column=1)
        tk.Radiobutton(control_frame, text="Forward", variable=self.R_dir_var, value=1).grid(row=5, column=2)
        tk.Radiobutton(control_frame, text="Backward", variable=self.R_dir_var, value=2).grid(row=5, column=3)
        
        tk.Label(control_frame, text="RPM (0-250):").grid(row=6, column=0, sticky="w")
        self.R_rpm_scale = tk.Scale(control_frame, from_=0, to=250, orient="horizontal", length=300, command=lambda x: self.update_rpm_labels())
        self.R_rpm_scale.grid(row=6, column=1, columnspan=3)
        self.R_rpm_label = tk.Label(control_frame, text="0", font=("Arial", 12))
        self.R_rpm_label.grid(row=6, column=4, padx=10)
        
        # Control buttons frame
        button_frame = tk.Frame(control_frame)
        button_frame.grid(row=7, column=0, columnspan=5, pady=10)
        
        self.start_btn = tk.Button(button_frame, text="Start Sending", command=self.start_sending, bg="blue", fg="white", font=("Arial", 11, "bold"), width=15)
        self.start_btn.pack(side="left", padx=5)
        
        self.stop_btn = tk.Button(button_frame, text="Stop Sending", command=self.stop_sending, bg="orange", fg="white", font=("Arial", 11, "bold"), width=15, state="disabled")
        self.stop_btn.pack(side="left", padx=5)
        
        # Frame cho plot
        plot_frame = tk.LabelFrame(self.root, text="Real-time Monitor (Serial Plotter)", padx=5, pady=5)
        plot_frame.pack(padx=10, pady=10, fill="both", expand=True)
        
        self.plot_canvas_widget = tk.Frame(plot_frame)
        self.plot_canvas_widget.pack(fill="both", expand=True)
        
    def setup_plot(self):
        # Tạo figure và axes cho matplotlib
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 5))
        self.fig.tight_layout(pad=3.0)
        
        # Left motor plot
        self.line_target_L, = self.ax1.plot([], [], 'r--', label='Target L', linewidth=2)
        self.line_actual_L, = self.ax1.plot([], [], 'b-', label='Actual L', linewidth=2)
        self.ax1.set_xlabel('Time (s)')
        self.ax1.set_ylabel('RPM')
        self.ax1.set_title('Left Motor')
        self.ax1.legend(loc='upper right')
        self.ax1.grid(True)
        self.ax1.set_ylim(0, 300)
        
        # Right motor plot
        self.line_target_R, = self.ax2.plot([], [], 'r--', label='Target R', linewidth=2)
        self.line_actual_R, = self.ax2.plot([], [], 'g-', label='Actual R', linewidth=2)
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('RPM')
        self.ax2.set_title('Right Motor')
        self.ax2.legend(loc='upper right')
        self.ax2.grid(True)
        self.ax2.set_ylim(0, 300)
        
        # Embed vào tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_canvas_widget)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        
        # Animation
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=100, blit=False, cache_frame_data=False)
        
    def update_plot(self, frame):
        if len(self.time_data) > 0:
            # Update data
            self.line_target_L.set_data(list(self.time_data), list(self.target_L_data))
            self.line_actual_L.set_data(list(self.time_data), list(self.actual_L_data))
            self.line_target_R.set_data(list(self.time_data), list(self.target_R_data))
            self.line_actual_R.set_data(list(self.time_data), list(self.actual_R_data))
            
            # Auto-scale x-axis
            if len(self.time_data) > 0:
                min_time = min(self.time_data)
                max_time = max(self.time_data)
                self.ax1.set_xlim(min_time, max_time + 1)
                self.ax2.set_xlim(min_time, max_time + 1)
        
        return self.line_target_L, self.line_actual_L, self.line_target_R, self.line_actual_R
        
    def refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        port_list = [port.device for port in ports]
        self.com_combo['values'] = port_list
        if port_list:
            self.com_combo.current(0)
            
    def toggle_connection(self):
        if not self.is_connected:
            self.connect()
        else:
            self.disconnect()
            
    def connect(self):
        port = self.com_combo.get()
        if not port:
            tk.messagebox.showerror("Error", "Please select a COM port")
            return
            
        try:
            self.ser = serial.Serial(port, 9600, timeout=0.1)
            time.sleep(2)  # Đợi Arduino reset
            self.is_connected = True
            self.status_label.config(text=f"Connected to {port}", fg="green")
            self.connect_btn.config(text="Disconnect", bg="red")
            
            # Start reading thread
            self.is_reading = True
            self.read_thread = threading.Thread(target=self.read_serial, daemon=True)
            self.read_thread.start()
            
            # Enable start button
            self.start_btn.config(state="normal")
            
        except Exception as e:
            tk.messagebox.showerror("Error", f"Cannot connect: {str(e)}")
            
    def disconnect(self):
        self.is_reading = False
        self.is_sending = False  # Dừng gửi khi disconnect
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.is_connected = False
        self.status_label.config(text="Disconnected", fg="red")
        self.connect_btn.config(text="Connect", bg="green")
        self.start_btn.config(state="disabled")
        self.stop_btn.config(state="disabled")
        
    def update_rpm_labels(self):
        """Cập nhật label hiển thị RPM"""
        self.L_rpm_label.config(text=str(self.L_rpm_scale.get()))
        self.R_rpm_label.config(text=str(self.R_rpm_scale.get()))
        
    def start_sending(self):
        """Bắt đầu gửi liên tục"""
        if not self.is_connected:
            return
        
        self.is_sending = True
        self.start_btn.config(state="disabled")
        self.stop_btn.config(state="normal")
        
        # Bắt đầu thread gửi liên tục
        self.send_thread = threading.Thread(target=self.continuous_send, daemon=True)
        self.send_thread.start()
        
    def stop_sending(self):
        """Dừng gửi liên tục"""
        self.is_sending = False
        self.start_btn.config(state="normal")
        self.stop_btn.config(state="disabled")
        
    def continuous_send(self):
        """Thread gửi dữ liệu liên tục"""
        while self.is_sending:
            try:
                L_dir = self.L_dir_var.get()
                L_rpm = self.L_rpm_scale.get()
                R_dir = self.R_dir_var.get()
                R_rpm = self.R_rpm_scale.get()
                
                # Tạo chuỗi gửi: "L_dir,L_rpm,R_dir,R_rpm#"
                command = f"{L_dir},{L_rpm},{R_dir},{R_rpm}#\n"
                
                if self.ser and self.ser.is_open:
                    self.ser.write(command.encode('utf-8'))
                    
                time.sleep(0.05)  # Gửi mỗi 50ms (20Hz) giống nano_1
            except Exception as e:
                print(f"Send error: {e}")
                break
                
    def exit_app(self):
        """Thoát chương trình an toàn"""
        print("Closing application...")
        
        # Dừng gửi dữ liệu
        self.is_sending = False
        
        # Dừng đọc dữ liệu
        self.is_reading = False
        
        # Đóng cổng serial
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
                print("Serial port closed")
            except Exception as e:
                print(f"Error closing serial: {e}")
        
        # Dừng animation
        if hasattr(self, 'ani'):
            self.ani.event_source.stop()
        
        # Đợi một chút để threads kết thúc
        time.sleep(0.2)
        
        # Thoát chương trình
        self.root.quit()
        self.root.destroy()
        
    def read_serial(self):
        while self.is_reading:
            try:
                if self.ser and self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        # Parse data: "target_L actual_L actual_R"
                        parts = line.split()
                        if len(parts) >= 3:
                            try:
                                # Bỏ qua target từ Arduino, lấy target từ GUI
                                actual_L = float(parts[1])
                                actual_R = float(parts[2])
                                
                                # Lấy target từ slider GUI
                                target_L = float(self.L_rpm_scale.get())
                                target_R = float(self.R_rpm_scale.get())
                                
                                # Update data
                                current_time = time.time() - self.start_time
                                self.time_data.append(current_time)
                                self.target_L_data.append(target_L)
                                self.actual_L_data.append(actual_L)
                                self.target_R_data.append(target_R)
                                self.actual_R_data.append(actual_R)
                                
                            except ValueError:
                                pass
                time.sleep(0.01)
            except Exception as e:
                print(f"Read error: {e}")
                break
                
    def send_control(self):
        """Gửi một lần (dùng cho test hoặc manual)"""
        if not self.is_connected or not self.ser:
            return
            
        L_dir = self.L_dir_var.get()
        L_rpm = self.L_rpm_scale.get()
        R_dir = self.R_dir_var.get()
        R_rpm = self.R_rpm_scale.get()
        
        # Update labels
        self.L_rpm_label.config(text=str(L_rpm))
        self.R_rpm_label.config(text=str(R_rpm))
        
        # Tạo chuỗi gửi: "L_dir,L_rpm,R_dir,R_rpm#"
        command = f"{L_dir},{L_rpm},{R_dir},{R_rpm}#\n"
        
        try:
            self.ser.write(command.encode('utf-8'))
            print(f"Sent: {command.strip()}")
        except Exception as e:
            print(f"Send error: {e}")

def main():
    root = tk.Tk()
    app = MotorControlApp(root)
    root.mainloop()

if __name__ == "__main__":
    main()
