#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import ttk
import yaml
import os
from rcl_interfaces.msg import Parameter
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import ParameterType

class ToolTip:
    def __init__(self, widget, text):
        self.widget = widget
        self.text = text
        self.tipwindow = None
        widget.bind("<Enter>", self.show_tip)
        widget.bind("<Leave>", self.hide_tip)
    def show_tip(self, event=None):
        if self.tipwindow or not self.text:
            return
        x, y, _, cy = self.widget.bbox("insert") if hasattr(self.widget, "bbox") else (0,0,0,0)
        x = x + self.widget.winfo_rootx() + 25
        y = y + cy + self.widget.winfo_rooty() + 20
        self.tipwindow = tw = tk.Toplevel(self.widget)
        tw.wm_overrideredirect(True)
        tw.wm_geometry(f"+{x}+{y}")
        label = tk.Label(tw, text=self.text, justify=tk.LEFT,
                         background="#ffffe0", relief=tk.SOLID, borderwidth=1,
                         font=("tahoma", "8", "normal"))
        label.pack(ipadx=4)
    def hide_tip(self, event=None):
        tw = self.tipwindow
        self.tipwindow = None
        if tw:
            tw.destroy()

class CropBoxGUI(Node):
    def __init__(self):
        super().__init__('crop_box_gui')
        self.declare_parameter('config_path', '')
        self.config_path = self.get_parameter('config_path').get_parameter_value().string_value

        self.root = tk.Tk()
        self.root.title("🖌️ Wall Detection Crop Box Control")
        self.root.configure(bg='#e9ecef')
        style = ttk.Style(self.root)
        style.theme_use('clam')
        style.configure('TFrame', background='#e9ecef')
        style.configure('TLabel', background='#e9ecef', font=('Segoe UI', 10))
        style.configure('Header.TLabel', font=('Segoe UI', 13, 'bold'), background='#343a40', foreground='#fff')
        style.configure('Section.TLabelframe', background='#f8f9fa', borderwidth=2, relief='groove')
        style.configure('Section.TLabelframe.Label', font=('Segoe UI', 11, 'bold'), background='#f8f9fa')
        style.configure('TButton', font=('Segoe UI', 10, 'bold'), background='#007bff', foreground='#fff')
        style.map('TButton', background=[('active', '#0056b3')])
        style.configure('Status.TLabel', font=('Segoe UI', 10, 'bold'), background='#e9ecef')
        style.configure('TScale', background='#e9ecef')

        wall_config = self.load_config().get('wall_detection', {})
        min_x = wall_config.get('crop_min_x', -0.7)
        max_x = wall_config.get('crop_max_x', 0.7)
        min_y = wall_config.get('crop_min_y', -1.5)
        max_y = wall_config.get('crop_max_y', 0.0)
        min_z = wall_config.get('crop_min_z', 0.05)
        max_z = wall_config.get('crop_max_z', 1.5)
        min_wall_points = wall_config.get('min_wall_points', 50)
        self.default_values = {
            'min_x': min_x, 'max_x': max_x,
            'min_y': min_y, 'max_y': max_y,
            'min_z': min_z, 'max_z': max_z,
            'min_wall_points': min_wall_points
        }

        main_frame = ttk.Frame(self.root, padding="15 15 15 15")
        main_frame.pack(fill=tk.BOTH, expand=True)

        title_bar = ttk.Label(main_frame, text="Wall Detection Crop Box Parameters", style='Header.TLabel', anchor='center')
        title_bar.pack(fill='x', pady=(0, 12))

        axis_frame = ttk.Frame(main_frame)
        axis_frame.pack(fill='x', pady=(0, 10))
        self.create_axis_section(axis_frame, "X-Axis Crop Box", min_x, max_x, "min_x_var", "max_x_var", 0, "#e3f2fd", "Set the minimum and maximum X values for the crop box.")
        self.create_axis_section(axis_frame, "Y-Axis Crop Box", min_y, max_y, "min_y_var", "max_y_var", 1, "#fff3e0", "Set the minimum and maximum Y values for the crop box.")
        self.create_axis_section(axis_frame, "Z-Axis Crop Box", min_z, max_z, "min_z_var", "max_z_var", 2, "#e8f5e9", "Set the minimum and maximum Z values for the crop box.")

        wall_points_frame = ttk.Labelframe(main_frame, text="Wall Detection", style='Section.TLabelframe', padding="10 8 10 8")
        wall_points_frame.pack(fill='x', pady=(0, 10))
        ttk.Label(wall_points_frame, text="Min Wall Points:").grid(row=0, column=0, padx=5, sticky='w')
        self.min_wall_points_var = tk.IntVar(value=min_wall_points)
        min_wall_points_slider = ttk.Scale(wall_points_frame, from_=10, to=1000, orient=tk.HORIZONTAL,
                                           variable=self.min_wall_points_var, length=220)
        min_wall_points_slider.grid(row=0, column=1, padx=5)
        min_wall_points_entry = ttk.Entry(wall_points_frame, textvariable=self.min_wall_points_var, width=7)
        min_wall_points_entry.grid(row=0, column=2, padx=5)
        ToolTip(min_wall_points_slider, "Minimum number of points to consider a wall detected.")
        ToolTip(min_wall_points_entry, "Minimum number of points to consider a wall detected.")
        self.min_wall_points_var.trace_add("write", lambda *args: self.on_slider_change(self.min_wall_points_var))
        min_wall_points_slider.configure(command=lambda v: self.on_slider_change(self.min_wall_points_var))

        bottom_frame = ttk.Frame(main_frame)
        bottom_frame.pack(fill='x', pady=(5, 0))
        reset_button = ttk.Button(bottom_frame, text="Reset to Defaults", command=self.reset_to_defaults)
        reset_button.pack(side='left', padx=(0, 10))
        ToolTip(reset_button, "Reset all crop box and wall detection parameters to their default values.")

        self.status_var = tk.StringVar(value="Ready")
        self.status_label = ttk.Label(bottom_frame, textvariable=self.status_var, style='Status.TLabel', anchor='center')
        self.status_label.pack(side='left', fill='x', expand=True, padx=5)

        self.last_values = {}
        self.timer = self.create_timer(0.3, self.update_parameters)
        self.root.bind_class("TEntry", "<FocusOut>", self.validate_entry)
        self.get_logger().info(f'Crop Box GUI started from {self.config_path}')
        self.root.update_idletasks()
        self.root.minsize(self.root.winfo_reqwidth(), self.root.winfo_reqheight())

    def create_axis_section(self, parent, title, min_val, max_val, min_var_name, max_var_name, row, bg_color, tooltip):
        lf = ttk.Labelframe(parent, text=title, style='Section.TLabelframe', padding="10 8 10 8")
        lf.grid(row=row, column=0, sticky='ew', pady=4)
        lf.configure()
        lf.grid_columnconfigure(1, weight=1)
        ttk.Label(lf, text="Min:").grid(row=0, column=0, padx=3, sticky='e')
        setattr(self, min_var_name, tk.DoubleVar(value=min_val))
        min_var = getattr(self, min_var_name)
        min_slider = ttk.Scale(lf, from_=-3.0, to=3.0, orient=tk.HORIZONTAL, variable=min_var, length=180)
        min_slider.grid(row=0, column=1, padx=3)
        min_entry = ttk.Entry(lf, textvariable=min_var, width=7)
        min_entry.grid(row=0, column=2, padx=3)
        ToolTip(min_slider, f"{tooltip} (Min)")
        ToolTip(min_entry, f"{tooltip} (Min)")
        min_var.trace_add("write", lambda *args: self.on_slider_change(min_var))
        min_slider.configure(command=lambda v: self.on_slider_change(min_var))
        ttk.Label(lf, text="Max:").grid(row=1, column=0, padx=3, sticky='e')
        setattr(self, max_var_name, tk.DoubleVar(value=max_val))
        max_var = getattr(self, max_var_name)
        max_slider = ttk.Scale(lf, from_=-3.0, to=3.0, orient=tk.HORIZONTAL, variable=max_var, length=180)
        max_slider.grid(row=1, column=1, padx=3)
        max_entry = ttk.Entry(lf, textvariable=max_var, width=7)
        max_entry.grid(row=1, column=2, padx=3)
        ToolTip(max_slider, f"{tooltip} (Max)")
        ToolTip(max_entry, f"{tooltip} (Max)")
        max_var.trace_add("write", lambda *args: self.on_slider_change(max_var))
        max_slider.configure(command=lambda v: self.on_slider_change(max_var))

    def validate_entry(self, event):
        try:
            widget = event.widget
            value = float(widget.get())
            if value < -3.0:
                widget.delete(0, tk.END)
                widget.insert(0, "-3.0")
            elif value > 3.0:
                widget.delete(0, tk.END)
                widget.insert(0, "3.0")
        except ValueError:
            widget.delete(0, tk.END)
            widget.insert(0, "0.0")
            
    def reset_to_defaults(self):
        self.min_x_var.set(self.default_values['min_x'])
        self.max_x_var.set(self.default_values['max_x'])
        self.min_y_var.set(self.default_values['min_y'])
        self.max_y_var.set(self.default_values['max_y'])
        self.min_z_var.set(self.default_values['min_z'])
        self.max_z_var.set(self.default_values['max_z'])
        self.min_wall_points_var.set(self.default_values['min_wall_points'])
        self.status_var.set("Reset to default values")
        self.update_status_color("blue")
    
    def update_status_color(self, color):
        self.status_label.configure(foreground=color)
        
    def on_slider_change(self, var):
        try:
            current = var.get()
            rounded = round(current * 20) / 20
            if current != rounded:
                var.set(rounded)
        except:
            pass
    
    def load_config(self):
        try:
            if os.path.exists(self.config_path):
                with open(self.config_path, 'r') as f:
                    return yaml.safe_load(f)
            self.get_logger().error(f'Config file not found: {self.config_path}')
        except Exception as e:
            self.get_logger().error(f'Error loading config: {e}')
        return {}
    
    def update_parameters(self):
        try:
            current = {
                'crop_min_x': self.min_x_var.get(),
                'crop_max_x': self.max_x_var.get(),
                'crop_min_y': self.min_y_var.get(),
                'crop_max_y': self.max_y_var.get(),
                'crop_min_z': self.min_z_var.get(),
                'crop_max_z': self.max_z_var.get(),
                'min_wall_points': int(self.min_wall_points_var.get())
            }
            if current == self.last_values:
                return
            self.last_values = current.copy()
            client = self.create_client(SetParameters, '/wall_detector/set_parameters')
            if not client.wait_for_service(timeout_sec=0.1):
                self.status_var.set("Waiting for wall_detector...")
                self.update_status_color("orange")
                return
            request = SetParameters.Request()
            for name, value in current.items():
                param = Parameter()
                param.name = name
                param.value.type = ParameterType.PARAMETER_DOUBLE
                param.value.double_value = value
                request.parameters.append(param)
            future = client.call_async(request)
            future.add_done_callback(self.param_callback)
            self.status_var.set("Updating parameters...")
            self.update_status_color("blue")
        except Exception as e:
            self.status_var.set(f"Error: {str(e)}")
            self.update_status_color("red")
            self.get_logger().error(f'Error updating parameters: {e}')
    
    def param_callback(self, future):
        try:
            response = future.result()
            if all(result.successful for result in response.results):
                self.status_var.set("Parameters updated")
                self.update_status_color("green")
            else:
                self.status_var.set("Update failed")
                self.update_status_color("red")
        except Exception as e:
            self.status_var.set(f"Error: {str(e)}")
            self.update_status_color("red")
            
    def spin(self):
        while rclpy.ok():
            self.root.update()
            rclpy.spin_once(self, timeout_sec=0.01)
            
    def shutdown(self):
        self.root.quit()
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    gui = CropBoxGUI()
    try:
        gui.spin()
    except KeyboardInterrupt:
        pass
    except tk.TclError:
        pass
    finally:
        gui.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
