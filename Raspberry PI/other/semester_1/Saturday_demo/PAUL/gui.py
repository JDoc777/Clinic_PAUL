import tkinter as tk
from tkinter import ttk
import threading
import time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


class RobotGUI:
    """Simple Tkinter GUI to display shared_data, encoder, sonar and pose info.

    The GUI polls provided sources periodically and updates labels. It does not
    modify robot state except via a small command panel for sending motor and
    flag values through shared_data.set_command_params().
    """

    def __init__(self, root, shared_data, encoder_processor, local_map, poll=0.000002):
        self.root = root
        self.shared = shared_data
        self.enc = encoder_processor
        self.lmap = local_map
        self.poll = float(poll)

        self._running = True

        root.title("Robot Monitor")

        # Adjust grid configuration for layout
        root.columnconfigure(2, weight=1)  # Add weight to the third column for the local map
        root.rowconfigure(3, weight=1)

        # Top frame: statuses partitioned into labeled sections
        top = ttk.Frame(root, padding=(6, 6))
        top.grid(row=0, column=0, sticky="nsew")

        # Motor control entries and actions (spanning two columns)
        ctrl = ttk.LabelFrame(root, text="Controls", padding=(4, 4))
        ctrl.grid(row=0, column=0, columnspan=2, sticky="ew", padx=2, pady=2)
        ttk.Label(ctrl, text="Motors (FL FR RL RR):").grid(row=0, column=0)
        self.motor_entries = [ttk.Entry(ctrl, width=4) for _ in range(4)]
        for i, e in enumerate(self.motor_entries):
            e.grid(row=0, column=1 + i)
            e.insert(0, "0")
        ttk.Button(ctrl, text="Send Motors", command=self._send_motors).grid(row=0, column=5, padx=2)

        ttk.Label(ctrl, text="Flags:").grid(row=1, column=0)
        self.flags_entry = ttk.Entry(ctrl, width=4)
        self.flags_entry.grid(row=1, column=1)
        self.flags_entry.insert(0, "0")
        ttk.Button(ctrl, text="Send Flags", command=self._send_flags).grid(row=1, column=2, padx=2)

        # Button to set all wheels to 1 m/s
        ttk.Button(ctrl, text="Set wheels to 1 m/s", command=self._set_wheels_one).grid(row=1, column=3, padx=6)

        # Encoder Velocity Display
        enc_vel_frame = ttk.LabelFrame(root, text="Encoder Velocities (m/s)", padding=(4, 4))
        enc_vel_frame.grid(row=1, column=0, sticky="nsew", padx=2, pady=2)
        self.enc_vel_vars = {w: tk.StringVar(value=f"{w}: 0.000") for w in ("FR", "FL", "RR", "RL")}
        for w in ("FR", "FL", "RR", "RL"):
            ttk.Label(enc_vel_frame, textvariable=self.enc_vel_vars[w]).pack(anchor="w")

        # Sonar Data Display
        sonar_frame = ttk.LabelFrame(root, text="Sonar Data (cm)", padding=(4, 4))
        sonar_frame.grid(row=1, column=1, sticky="nsew", padx=2, pady=2)
        self.sonar_vars = {d: tk.StringVar(value=f"{d}: ?") for d in ("F", "B", "L", "R")}
        for d in ("F", "B", "L", "R"):
            ttk.Label(sonar_frame, textvariable=self.sonar_vars[d]).pack(anchor="w")

        # General log (spanning two columns)
        logf = ttk.Frame(root, padding=(4, 4))
        logf.grid(row=2, column=0, columnspan=2, sticky="nsew", padx=2, pady=2)
        root.rowconfigure(2, weight=1)
        general_log_frame = ttk.Labelframe(logf, text="General Log")
        self.log = tk.Text(general_log_frame, height=8, wrap="none")
        self.log.pack(fill="both", expand=True)
        general_log_frame.pack(fill="both", expand=True)

        # Local Map Visualization
        map_frame = ttk.LabelFrame(root, text="Local Map", padding=(4, 4))
        map_frame.grid(row=0, column=2, rowspan=3, sticky="nsew", padx=2, pady=2)
        self.fig, self.ax = plt.subplots(figsize=(6, 3))
        self.ax.set_title("Robot Local Map")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.grid(True)
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.set_xlim(-3, 3)
        self.ax.set_ylim(-3, 3)
        self.path_line, = self.ax.plot([], [], lw=1, label="Path")
        self.pose_dot, = self.ax.plot([], [], "o", ms=3, label="Current Pose")
        self.ax.legend(fontsize="small")

        self.canvas = FigureCanvasTkAgg(self.fig, master=map_frame)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.pack(fill="both", expand=True)

        self.map_data = {"xs": [], "ys": []}

        # Velocity Plots for Each Wheel
        plot_frame = ttk.LabelFrame(root, text="Wheel Velocities", padding=(4, 4))
        plot_frame.grid(row=3, column=0, columnspan=3, sticky="nsew", padx=2, pady=2)
        self.fig_vel, self.axes = plt.subplots(2, 2, figsize=(6, 4))
        self.fig_vel.tight_layout(pad=2.0)
        self.canvas_vel = FigureCanvasTkAgg(self.fig_vel, master=plot_frame)
        self.canvas_vel_widget = self.canvas_vel.get_tk_widget()
        self.canvas_vel_widget.pack(fill="both", expand=True)

        self.wheel_lines = {}
        self.wheel_data = {w: {"time": [], "velocity": []} for w in ("FR", "FL", "RR", "RL")}
        self.last_plot_update = time.time()  # Track the last plot update time
        self.plot_update_interval = 0.5  # Update plots every 0.5 seconds
        wheels = ["FR", "FL", "RR", "RL"]

        for ax, wheel in zip(self.axes.flatten(), wheels):
            ax.set_title(f"{wheel} Velocity", fontsize=8)
            ax.set_xlabel("Time (s)", fontsize=8)
            ax.set_ylabel("Velocity (m/s)", fontsize=8)
            ax.tick_params(axis="both", labelsize=6)
            ax.grid(True)
            self.wheel_lines[wheel], = ax.plot([], [], lw=1, label=f"{wheel} Velocity")
            ax.legend(fontsize="x-small")

        # Ensure proper shutdown on window close
        root.protocol("WM_DELETE_WINDOW", self._on_close)

        # Start polling thread
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

        # Wheel Control Buttons (2x3 grid, smaller size)
        wheel_ctrl_frame = ttk.LabelFrame(root, text="Wheel Controls", padding=(4, 4))
        wheel_ctrl_frame.grid(row=4, column=0, columnspan=2, sticky="nsew", padx=2, pady=2)

        ttk.Button(wheel_ctrl_frame, text="Fwd", width=8, command=lambda: self._set_wheel_action((100, 100, 100, 100), 2)).grid(row=0, column=0, padx=2, pady=2)
        ttk.Button(wheel_ctrl_frame, text="Back", width=8, command=lambda: self._set_wheel_action((-100, -100, -100, -100), 2)).grid(row=1, column=0, padx=2, pady=2)
        ttk.Button(wheel_ctrl_frame, text="CW", width=8, command=lambda: self._set_wheel_action((255, -255, 255, -255), 1.5)).grid(row=0, column=1, padx=2, pady=2)
        ttk.Button(wheel_ctrl_frame, text="CCW", width=8, command=lambda: self._set_wheel_action((-255, 255, -255, 255), 1.5)).grid(row=1, column=1, padx=2, pady=2)
        ttk.Button(wheel_ctrl_frame, text="Crab L", width=8, command=lambda: self._set_wheel_action((-255, 255, 255, -255), 2)).grid(row=0, column=2, padx=2, pady=2)
        ttk.Button(wheel_ctrl_frame, text="Crab R", width=8, command=lambda: self._set_wheel_action((255, -255, -255, 255), 2)).grid(row=1, column=2, padx=2, pady=2)

        # Bind resize event to dynamically update the local map size
        root.bind("<Configure>", self._on_resize)

    def _log(self, msg, kind="general"):
        """Log encoder data only."""
        ts = time.strftime("%H:%M:%S")
        self.log.insert("end", f"[{ts}] {msg}\n")
        self.log.see("end")

    def _on_resize(self, event):
        """Handle window resize events to update the local map size."""
        try:
            # Dynamically adjust the figure size based on the canvas size
            canvas_width = self.canvas_widget.winfo_width()
            canvas_height = self.canvas_widget.winfo_height()
            self.fig.set_size_inches(canvas_width / self.fig.dpi, canvas_height / self.fig.dpi)

            # Redraw the local map
            self.ax.relim()
            self.ax.autoscale_view()
            self.canvas.draw()
        except Exception as exc:
            self._log(f"Error resizing map: {exc}")

    def _update_map(self):
        """Update the local map with the latest pose."""
        try:
            pose = self.lmap.get_pose()
            x, y = float(pose["x"]), float(pose["y"])

            self.map_data["xs"].append(x)
            self.map_data["ys"].append(y)

            # Limit history length
            if len(self.map_data["xs"]) > 500:
                self.map_data["xs"] = self.map_data["xs"][-500:]
                self.map_data["ys"] = self.map_data["ys"][-500:]

            self.path_line.set_data(self.map_data["xs"], self.map_data["ys"])
            self.pose_dot.set_data([x], [y])

            self.ax.relim()
            self.ax.autoscale_view()
            self.canvas.draw()
        except Exception as exc:
            self._log(f"Error updating map: {exc}")

    def _update_plots(self):
        """Update the velocity plots for each wheel."""
        current_time = time.time()
        if current_time - self.last_plot_update < self.plot_update_interval:
            return  # Skip update if interval has not elapsed

        for wheel in ("FR", "FL", "RR", "RL"):
            try:
                velocity = self.enc.get_velocity(wheel)
                self.wheel_data[wheel]["time"].append(current_time)
                self.wheel_data[wheel]["velocity"].append(velocity)

                # Limit history length to fit on the plot
                if len(self.wheel_data[wheel]["time"]) > 100:
                    self.wheel_data[wheel]["time"] = self.wheel_data[wheel]["time"][-100:]
                    self.wheel_data[wheel]["velocity"] = self.wheel_data[wheel]["velocity"][-100:]

                # Smooth the data using interpolation
                if len(self.wheel_data[wheel]["time"]) > 1:
                    smoothed_time = self.wheel_data[wheel]["time"]
                    smoothed_velocity = self.wheel_data[wheel]["velocity"]
                else:
                    smoothed_time = self.wheel_data[wheel]["time"]
                    smoothed_velocity = self.wheel_data[wheel]["velocity"]

                # Update plot data
                self.wheel_lines[wheel].set_data(smoothed_time, smoothed_velocity)
                ax = self.wheel_lines[wheel].axes
                ax.relim()
                ax.autoscale_view()
            except Exception as exc:
                self._log(f"Error updating plot for {wheel}: {exc}")

        self.canvas_vel.draw()
        self.last_plot_update = current_time  # Update the last plot update time

    def _loop(self):
        while self._running:
            try:
                # Update encoder data and log it
                for w in ("FR", "FL", "RR", "RL"):
                    try:
                        enc_value = self.enc.get_velocity(w)  # Fetch encoder velocity
                        self.enc_vel_vars[w].set(f"{w}: {enc_value:.3f}")  # Update GUI
                        self._log(f"{w}: {enc_value:.3f} m/s")  # Log to general log
                    except KeyError:
                        self._log(f"Error: Wheel '{w}' not found in encoder data.")
                    except Exception as exc:
                        self._log(f"Error fetching encoder data for {w}: {exc}")

                # Update sonar data
                try:
                    sonar_data = self.shared.get_received_data().get("sonar", {})
                    for d in ("F", "B", "L", "R"):
                        value = sonar_data.get(d, "?")
                        self.sonar_vars[d].set(f"{d}: {value}")
                except Exception as exc:
                    self._log(f"Error fetching sonar data: {exc}")

                # Update the local map
                self._update_map()

                # Update the velocity plots
                self._update_plots()

            except Exception as exc:
                self._log(f"Poll error: {exc}")

            time.sleep(self.poll)

    def _on_close(self):
        # stop thread
        self._running = False
        try:
            # give it a moment to exit
            self._thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            self.root.destroy()
        except Exception:
            pass

    def _send_motors(self):
        try:
            vals = tuple(int(e.get()) for e in self.motor_entries)
            # order in Payload.set_motors is (fl, fr, rl, rr)
            self.shared.set_command_params(motors=vals)
            self._log(f"Sent motors: {vals}", kind="general")
        except Exception as exc:
            self._log(f"Bad motor values: {exc}", kind="general")

    def _send_flags(self):
        try:
            f = int(self.flags_entry.get(), 0)
            self.shared.set_command_params(flags=f)
            self._log(f"Sent flags: {f}", kind="general")
        except Exception as exc:
            self._log(f"Bad flags value: {exc}", kind="general")

    def _set_wheels_one(self):
        """Set all wheels to target speed 1.0 m/s by using motor.calculate_pwm()
        to compute PWMs and sending them to shared_data as motors.
        Requires a motor instance passed to the GUI at creation.
        """
        if not hasattr(self, 'motor') or self.motor is None:
            self._log("No motor controller available to set wheel speeds.", kind="general")
            return
        try:
            target_speeds = {'set_FR': 1.0, 'set_FL': 1.0, 'set_RR': 1.0, 'set_RL': 1.0}
            pwm = self.motor.calculate_pwm(target_speeds)
            # pwm dict keys: PWM_FR, PWM_FL, PWM_RR, PWM_RL
            fl = pwm.get('PWM_FL', 0)
            fr = pwm.get('PWM_FR', 0)
            rl = pwm.get('PWM_RL', 0)
            rr = pwm.get('PWM_RR', 0)
            self.shared.set_command_params(motors=(fl, fr, rl, rr))
            self._log(f"Set wheels to 1 m/s -> motors(PWM): FL={fl} FR={fr} RL={rl} RR={rr}", kind="general")
        except Exception as exc:
            self._log(f"Failed to set wheels to 1 m/s: {exc}", kind="general")

    def _set_wheel_action(self, speeds, duration):
        """
        Set motor speeds for a specific duration.
        :param speeds: Tuple of motor speeds (fl, fr, rl, rr).
        :param duration: Duration in seconds to maintain the speeds.
        """
        self.shared.set_command_params(motors=speeds)
        self._log(f"Set motors to {speeds} for {duration} seconds.", kind="general")

        def reset_motors():
            time.sleep(duration)
            self.shared.set_command_params(motors=(0, 0, 0, 0))
            self._log("Motors reset to 0.", kind="general")

        threading.Thread(target=reset_motors, daemon=True).start()

    def _move_forward(self):
        """Set wheels to move forward."""
        self.shared.set_command_params(motors=(1, 1, 1, 1))
        self._log("Wheels moving forward.", kind="general")

    def _move_backward(self):
        """Set wheels to move backward."""
        self.shared.set_command_params(motors=(-1, -1, -1, -1))
        self._log("Wheels moving backward.", kind="general")

    def _rotate_clockwise(self):
        """Set wheels to rotate clockwise."""
        self.shared.set_command_params(motors=(1, -1, 1, -1))
        self._log("Wheels rotating clockwise.", kind="general")

    def _rotate_counterclockwise(self):
        """Set wheels to rotate counterclockwise."""
        self.shared.set_command_params(motors=(-1, 1, -1, 1))
        self._log("Wheels rotating counterclockwise.", kind="general")

    def _crab_left(self):
        """Set wheels to crab walk left."""
        self.shared.set_command_params(motors=(-1, 1, 1, -1))
        self._log("Wheels crab walking left.", kind="general")

    def _crab_right(self):
        """Set wheels to crab walk right."""
        self.shared.set_command_params(motors=(1, -1, -1, 1))
        self._log("Wheels crab walking right.", kind="general")


def start_gui(shared_data, encoder_processor, local_map, motor=None, poll=0.00002):
    root = tk.Tk()
    gui = RobotGUI(root, shared_data, encoder_processor, local_map, poll=poll)
    # attach motor if provided so GUI actions can use it
    gui.motor = motor
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    return gui
