import os
import json
import tkinter as tk
import queue
import threading
from PIL import Image, ImageTk

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# ================= CONFIGURATION =================
IMAGE_PATH = os.path.join(SCRIPT_DIR, "vectored-frame.png")
SAVE_FILE = "pwm_settings.json"  # JSON config file name

# Coordinates for the input fields on your specific image (x, y)
MOTOR_COORDS = {
    1: ("right", 70),  # Top Right
    2: ("left", 70),  # Top Left
    5: ("right", 210),  # Mid Right (Upper)
    6: ("left", 210),  # Mid Left  (Upper)
    3: ("right", 370),  # Bottom Right
    4: ("left", 370),  # Bottom Left
}
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
        """Creates the UI and runs the mainloop within the SAME thread."""
        self.root = tk.Tk()
        self.root.title("Live ArduSub PWM Overlay Tuner")
        self._build_connection_frame()
        self._build_canvas_frame()
        self.root.mainloop()

    def stop(self):
        self._is_running.clear()
        if self.root:
            self.root.quit()
        if self._thread:
            self._thread.join()

    # Configuration Save/Load Methods
    def _load_config(self):
        """Reads the JSON file if it exists, otherwise loads defaults."""
        if os.path.exists(SAVE_FILE):
            try:
                with open(SAVE_FILE, "r") as f:
                    data = json.load(f)
                    # JSON keys are always strings, so we cast the motor IDs back to integers
                    return {int(k): v for k, v in data.items()}
            except Exception as e:
                print(f"Warning: Failed to load {SAVE_FILE}: {e}")

        return {m: {"MIN": 1100, "MAX": 1900} for m in MOTOR_COORDS.keys()}

    def _save_config(self):
        """Saves the current state to the JSON file."""
        try:
            with open(SAVE_FILE, "w") as f:
                json.dump(self.pwm_data, f, indent=4)
        except Exception as e:
            print(f"Error: Failed to save to {SAVE_FILE}: {e}")

    def _build_connection_frame(self):
        """Builds the top bar for port selection and MAVLink connection."""
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
        """Builds the background image canvas and populates the motor controls."""
        canvas_frame = tk.Frame(self.root)
        canvas_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        try:
            self.bg_image = Image.open(IMAGE_PATH)
            self.bg_photo = ImageTk.PhotoImage(self.bg_image)
            img_width, img_height = self.bg_image.size
        except FileNotFoundError:
            img_width, img_height = 400, 500
            self.bg_photo = tk.PhotoImage(width=img_width, height=img_height)
            print(f"Warning: {IMAGE_PATH} not found. Using blank background.")

        side_panel_width = 250
        canvas_width = img_width + (side_panel_width * 2)
        canvas_height = img_height

        self.canvas = tk.Canvas(canvas_frame, width=canvas_width, height=canvas_height)
        self.canvas.pack()

        # Place the image offset by the side panel width so it sits in the middle
        self.canvas.create_image(side_panel_width, 0, image=self.bg_photo, anchor=tk.NW)

        for motor_id, (side, y) in MOTOR_COORDS.items():
            if side == "left":
                x = 10
                anchor = tk.W
            else:
                x = canvas_width - 10
                anchor = tk.E

            self._build_motor_overlay(motor_id, x, y, anchor)

    def _build_motor_overlay(self, motor_id, x, y, anchor):
        """Constructs a small control panel for a motor and places it on the canvas."""
        box = tk.Frame(self.canvas, bg="white", padx=4, pady=4, bd=1, relief=tk.SOLID)

        # Ensure dictionary keys exist for this motor
        if motor_id not in self.entry_widgets:
            self.entry_widgets[motor_id] = {}

        # Retrieve saved settings for this specific motor
        saved_min = self.pwm_data[motor_id]["MIN"]
        saved_max = self.pwm_data[motor_id]["MAX"]

        # Motor Title
        tk.Label(
            box, text=f"Motor {motor_id}", font=("Arial", 9, "bold"), bg="white"
        ).grid(row=0, column=0, columnspan=3)

        # --- MINIMUM CONTROLS ---
        tk.Label(box, text="Min:", font=("Arial", 8), bg="white").grid(
            row=1, column=0, sticky="e"
        )
        min_entry = tk.Entry(box, width=5, justify="center")
        min_entry.insert(0, str(saved_min))
        min_entry.grid(row=1, column=1, padx=2)

        # Store entry reference for flashing
        self.entry_widgets[motor_id]["MIN"] = min_entry

        min_scale = tk.Scale(
            box,
            from_=1000,
            to=1500,
            orient=tk.HORIZONTAL,
            showvalue=False,
            length=120,
            bg="white",
        )
        min_scale.set(saved_min)
        min_scale.grid(row=1, column=2)

        # --- MAXIMUM CONTROLS ---
        tk.Label(box, text="Max:", font=("Arial", 8), bg="white").grid(
            row=2, column=0, sticky="e"
        )
        max_entry = tk.Entry(box, width=5, justify="center")
        max_entry.insert(0, str(saved_max))
        max_entry.grid(row=2, column=1, padx=2)

        # Store entry reference for flashing
        self.entry_widgets[motor_id]["MAX"] = max_entry

        max_scale = tk.Scale(
            box,
            from_=1500,
            to=2000,
            orient=tk.HORIZONTAL,
            showvalue=False,
            length=120,
            bg="white",
        )
        max_scale.set(saved_max)
        max_scale.grid(row=2, column=2)

        self.canvas.create_window(x, y, window=box, anchor=anchor)

        def on_slider_move(val, entry, m=motor_id, t="MIN"):
            if self.root.focus_get() != entry:
                entry.delete(0, tk.END)
                entry.insert(0, val)
            self.on_value_change(m, t, val)

        def on_entry_commit(event, scale, entry, m=motor_id, t="MIN"):
            try:
                val = int(entry.get())
                scale.set(val)
            except ValueError:
                entry.delete(0, tk.END)
                entry.insert(0, str(scale.get()))

        # Bind Sliders
        min_scale.config(
            command=lambda val, e=min_entry: on_slider_move(val, e, motor_id, "MIN")
        )
        max_scale.config(
            command=lambda val, e=max_entry: on_slider_move(val, e, motor_id, "MAX")
        )

        # Bind Entries (Enter key and click-away)
        min_entry.bind(
            "<Return>",
            lambda event, s=min_scale, e=min_entry: on_entry_commit(
                event, s, e, motor_id, "MIN"
            ),
        )
        max_entry.bind(
            "<Return>",
            lambda event, s=max_scale, e=max_entry: on_entry_commit(
                event, s, e, motor_id, "MAX"
            ),
        )
        min_entry.bind(
            "<FocusOut>",
            lambda event, s=min_scale, e=min_entry: on_entry_commit(
                event, s, e, motor_id, "MIN"
            ),
        )
        max_entry.bind(
            "<FocusOut>",
            lambda event, s=max_scale, e=max_entry: on_entry_commit(
                event, s, e, motor_id, "MAX"
            ),
        )

    def on_value_change(self, motor_id, limit_type, value):
        """Updates internal state, auto-saves to JSON, and rate-limits serial updates."""
        val = int(value)

        if self.pwm_data[motor_id][limit_type] != val:
            self.pwm_data[motor_id][limit_type] = val
            self._save_config()

        timer_key = f"{motor_id}_{limit_type}"

        if timer_key in self.update_timers:
            self.root.after_cancel(self.update_timers[timer_key])

        self.update_timers[timer_key] = self.root.after(
            150, self.send_param_update, motor_id, limit_type, val
        )

    def send_all_params(self):
        """Iterates through all saved data and sends it to the flight controller."""
        for motor_id, limits in self.pwm_data.items():
            for limit_type, val in limits.items():
                self.send_param_update(motor_id, limit_type, val)

    def send_param_update(self, motor_id, limit_type, value):
        """Constructs and sends the MAVLink parameter setting message."""
        param_id = f"MOT_{motor_id}_{limit_type}"

        self.param_queue.put((param_id, value))

        self._flash_entry(motor_id, limit_type)

    def _flash_entry(self, motor_id, limit_type):
        """Temporarily changes the background color of an entry to signify an update."""
        entry = self.entry_widgets.get(motor_id, {}).get(limit_type)
        if entry:
            original_bg = entry.cget("background")
            entry.config(bg="lightgreen")
            self.root.after(300, lambda: entry.config(bg=original_bg))
