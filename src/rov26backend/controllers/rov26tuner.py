import json
import os
import queue
import threading
import tkinter as tk

from PIL import Image, ImageTk

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# ================= CONFIGURATION =================
IMAGE_PATH = os.path.join(SCRIPT_DIR, "vectored-frame.png")
SAVE_FILE = "pwm_settings.json"

MOTOR_COORDS = {
    1: ("right", 100),  # Top Right
    2: ("left", 100),  # Top Left
    5: ("right", 380),  # Mid Right
    6: ("left", 380),  # Mid Left
    3: ("right", 660),  # Bottom Right
    4: ("left", 660),  # Bottom Left
}

# Per-motor parameters
PARAMS_CONFIG = [
    ("MIN", 1000, 1500, 1, int),
    ("MAX", 1500, 2000, 1, int),
    ("THROTTLE", -100.0, 100.0, 0.01, float),
    ("YAW", -1000.0, 200.0, 0.01, float),
    ("FORWARD", -100.0, 100.0, 0.01, float),
    ("LATERAL", -100.0, 100.0, 0.01, float),
    ("ROLL", -100.0, 100.0, 0.01, float),
    ("PITCH", -100.0, 100.0, 0.01, float),
]

# Global PID parameters
GLOBAL_PARAMS_CONFIG = [
    ("ATC_RAT_YAW_P", 0.0, 1.0, 0.01, float),
    ("ATC_RAT_YAW_I", 0.0, 0.1, 0.001, float),
    ("ATC_RAT_YAW_D", 0.0, 0.05, 0.001, float),
    ("ATC_ANG_YAW_P", 0.0, 10.0, 0.1, float),
]
# =================================================


class LivePWMOverlayTuner:
    def __init__(self, param_queue: queue.Queue):
        self.update_timers = {}
        self.param_queue = param_queue
        self.entry_widgets = {}
        self.root = None
        self.pwm_data = self._load_config()
        self._thread = None
        self._is_running = threading.Event()

    def start(self):
        if self._thread is None:
            self._is_running.set()
            self._thread = threading.Thread(target=self._run_ui, daemon=True)
            self._thread.start()

    def _run_ui(self):
        self.root = tk.Tk()
        self.root.title("Live ArduSub Motor & PID Tuner")
        self._build_connection_frame()
        self._build_canvas_frame()
        self.root.mainloop()

    def stop(self):
        self._is_running.clear()
        if self.root:
            self.root.quit()
        if self._thread:
            self._thread.join()

    def _load_config(self):
        defaults = {
            m: {p[0]: (1500 if p[4] == int else 0.0) for p in PARAMS_CONFIG}
            for m in MOTOR_COORDS
        }

        # Add Global parameter defaults
        defaults["GLOBAL"] = {p[0]: 0.0 for p in GLOBAL_PARAMS_CONFIG}

        if os.path.exists(SAVE_FILE):
            try:
                with open(SAVE_FILE, "r") as f:
                    data = json.load(f)
                    for k, v in data.items():
                        if k == "GLOBAL":
                            defaults["GLOBAL"].update(v)
                        else:
                            try:
                                motor_id = int(k)
                                if motor_id in defaults:
                                    defaults[motor_id].update(v)
                            except ValueError:
                                pass  # Ignore invalid keys
            except Exception as e:
                print(f"Warning: Failed to load {SAVE_FILE}: {e}")

        return defaults

    def _save_config(self):
        try:
            with open(SAVE_FILE, "w") as f:
                json.dump(self.pwm_data, f, indent=4)
        except Exception as e:
            print(f"Error: Failed to save to {SAVE_FILE}: {e}")

    def _build_connection_frame(self):
        frame = tk.Frame(self.root, pady=10, padx=10)
        frame.pack(side=tk.TOP, fill=tk.X)
        self.send_all_btn = tk.Button(
            frame,
            text="Send All to FC",
            bg="blue",
            fg="white",
            command=self.send_all_params,
        )
        self.send_all_btn.pack(side=tk.LEFT, padx=10)

    def _build_canvas_frame(self):
        canvas_frame = tk.Frame(self.root)
        canvas_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        try:
            self.bg_image = Image.open(IMAGE_PATH)
            self.bg_photo = ImageTk.PhotoImage(self.bg_image)
            img_width, img_height = self.bg_image.size
        except FileNotFoundError:
            img_width, img_height = 400, 700
            self.bg_photo = tk.PhotoImage(width=img_width, height=img_height)

        side_panel_width = 250
        canvas_width = img_width + (side_panel_width * 2)
        canvas_height = max(img_height, 900)

        self.canvas = tk.Canvas(canvas_frame, width=canvas_width, height=canvas_height)
        self.canvas.pack()
        self.canvas.create_image(side_panel_width, 0, image=self.bg_photo, anchor=tk.NW)

        # Build Global PID Overlay at top center
        self._build_param_overlay(
            "GLOBAL", canvas_width // 2, 20, tk.N, GLOBAL_PARAMS_CONFIG, "PID Tuning"
        )

        # Build Motor Overlays
        for motor_id, (side, y) in MOTOR_COORDS.items():
            x = 10 if side == "left" else canvas_width - 10
            anchor = tk.W if side == "left" else tk.E
            self._build_param_overlay(
                motor_id, x, y, anchor, PARAMS_CONFIG, f"Motor {motor_id}"
            )

    def _build_param_overlay(self, group_id, x, y, anchor, config_list, title):
        box = tk.Frame(self.canvas, bg="white", padx=4, pady=4, bd=1, relief=tk.SOLID)
        if group_id not in self.entry_widgets:
            self.entry_widgets[group_id] = {}

        tk.Label(box, text=title, font=("Arial", 9, "bold"), bg="white").grid(
            row=0, column=0, columnspan=3
        )

        row_idx = 1
        for param_name, min_v, max_v, res, type_cast in config_list:
            saved_val = self.pwm_data[group_id][param_name]

            tk.Label(box, text=f"{param_name}:", font=("Arial", 8), bg="white").grid(
                row=row_idx, column=0, sticky="e"
            )

            entry = tk.Entry(box, width=8, justify="center")
            entry.insert(0, str(saved_val))
            entry.grid(row=row_idx, column=1, padx=2)
            self.entry_widgets[group_id][param_name] = entry

            scale = tk.Scale(
                box,
                from_=min_v,
                to=max_v,
                resolution=res,
                orient=tk.HORIZONTAL,
                showvalue=False,
                length=120,
                bg="white",
            )
            scale.set(saved_val)
            scale.grid(row=row_idx, column=2)

            def make_slider_cb(e_ref, g_id, p_name, caster):
                def cb(val):
                    if self.root.focus_get() != e_ref:
                        e_ref.delete(0, tk.END)
                        e_ref.insert(0, val)
                    self.on_value_change(g_id, p_name, val, caster)

                return cb

            def make_entry_cb(s_ref, e_ref, g_id, p_name, caster):
                def cb(event):
                    try:
                        val = caster(e_ref.get())
                        s_ref.set(val)
                    except ValueError:
                        e_ref.delete(0, tk.END)
                        e_ref.insert(0, str(s_ref.get()))

                return cb

            scale.config(command=make_slider_cb(entry, group_id, param_name, type_cast))
            enter_cb = make_entry_cb(scale, entry, group_id, param_name, type_cast)

            entry.bind("<Return>", enter_cb)
            entry.bind("<FocusOut>", enter_cb)

            row_idx += 1

        self.canvas.create_window(x, y, window=box, anchor=anchor)

    def on_value_change(self, group_id, param_name, value, type_cast):
        val = type_cast(value)
        if self.pwm_data[group_id][param_name] != val:
            self.pwm_data[group_id][param_name] = val
            self._save_config()

        timer_key = f"{group_id}_{param_name}"
        if timer_key in self.update_timers:
            self.root.after_cancel(self.update_timers[timer_key])

        self.update_timers[timer_key] = self.root.after(
            150, self.send_param_update, group_id, param_name, val
        )

    def send_all_params(self):
        # FIX: Force focus out of any active Entry box so its value commits to self.pwm_data
        self.root.focus_set()

        for group_id, limits in self.pwm_data.items():
            for param_name, val in limits.items():
                self.send_param_update(group_id, param_name, val)

    def send_param_update(self, group_id, param_name, value):
        if group_id == "GLOBAL":
            param_id = param_name  # Send ATC_RAT_YAW_P directly
        else:
            param_id = f"MOT_{group_id}_{param_name}"

        self.param_queue.put((param_id, value))
        self._flash_entry(group_id, param_name)

    def _flash_entry(self, group_id, param_name):
        entry = self.entry_widgets.get(group_id, {}).get(param_name)
        if entry:
            original_bg = entry.cget("background")
            entry.config(bg="lightgreen")
            self.root.after(300, lambda: entry.config(bg=original_bg))
