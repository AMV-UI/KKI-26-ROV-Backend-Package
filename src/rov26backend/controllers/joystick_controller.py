import os
# Force Pygame to run headless so it doesn't crash on a Raspberry Pi / Jetson without a monitor
os.environ["SDL_VIDEODRIVER"] = "dummy"

import pygame
import threading
import logging
import time

from rov26backend.models.button import ModeButton
from rov26backend.models.simul_press_button import SimulPressButton

logger = logging.getLogger("ROV.joystick")


class JoystickController:
    def __init__(self):
        # Initialize only the modules we need
        pygame.init()
        pygame.joystick.init()
        
        self.joystick = None
        self._connect_joystick()

        # Target and Current States
        self.current_manual_control = [1500.0, 1500.0, 1500.0, 1500.0]
        self.target_manual_control = [1500.0, 1500.0, 1500.0, 1500.0]
        
        # Button States
        self.btn_states = {
            "BTN_SOUTH": 0,  # A Button
            "BTN_EAST": 0,   # B Button
            "BTN_WEST": 0,   # X Button
            "BTN_NORTH": 0   # Y Button
        }
        self.lb_btn = False
        self.rb_btn = False
        self.servo_btn = 0
        
        # Button Logic Handlers
        self.arm_btn = SimulPressButton()
        self.btn_ctrl = {
            "BTN_EAST": ModeButton("ALT_HOLD"),
            "BTN_WEST": ModeButton("STABILIZE"),
            "BTN_SOUTH": ModeButton("MANUAL"),
            "BTN_NORTH": ModeButton("HOLD"),
        }

        # PWM configurations
        self.smoothing_factor = 0.2
        self.pwm_center = 1500
        self.pwm_range = 400
        self.pwm_min = 1300
        self.pwm_max = 1700

        self.MAX_SLEW_PER_SEC = 400
        self.last_servo_time = time.time()
        self.servo_pwm = 2500

        self.lock = threading.Lock()

    def _connect_joystick(self):
        """Attempts to connect to the first available Pygame joystick."""
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            logger.info(f"Gamepad connected: {self.joystick.get_name()}")
        else:
            logger.warning("No gamepad detected! Waiting for connection...")

    def monitor(self):
        """
        Runs continuously in the joystick_loop thread. 
        Pumps the Pygame event queue and reads the raw hardware states.
        """
        try:
            # Pygame requires the event queue to be pumped to update hardware states
            pygame.event.pump()
            
            # Auto-reconnect logic
            if pygame.joystick.get_count() == 0 and self.joystick is not None:
                self.joystick = None
                logger.warning("Gamepad disconnected.")
            elif pygame.joystick.get_count() > 0 and self.joystick is None:
                self._connect_joystick()

            # If no joystick is found, yield the thread to prevent CPU pegging
            if self.joystick is None:
                time.sleep(0.5)
                return

            self._calculate_targets()

            # ~50Hz polling rate to match your control loop
            time.sleep(0.02) 

        except Exception as e:
            logger.warning(f"Error reading gamepad: {e}")
            time.sleep(1.0)
            pygame.joystick.quit()
            pygame.joystick.init()
            self.joystick = None

    def _apply_deadzone(self, val, deadzone=0.1):
        """Zeroes out tiny stick drifts."""
        return val if abs(val) > deadzone else 0.0

    def _calculate_targets(self):
        """Reads raw states from Pygame and translates them to PWM values."""
        
        # Pygame Standard Xbox Mappings:
        # Axis 0: Left Stick X (-1.0 Left to 1.0 Right)
        # Axis 1: Left Stick Y (-1.0 Up to 1.0 Down)
        # Axis 2: Right Stick X (-1.0 Left to 1.0 Right)
        # Axis 4: Left Trigger (-1.0 Released to 1.0 Pressed)
        # Axis 5: Right Trigger (-1.0 Released to 1.0 Pressed)
        
        raw_lateral = self._apply_deadzone(self.joystick.get_axis(0))
        # Multiply by -1 because pushing forward on the stick gives a negative value in Pygame
        raw_forward = self._apply_deadzone(self.joystick.get_axis(1) * -1.0) 
        raw_yaw = self._apply_deadzone(self.joystick.get_axis(2))
        
        # Normalize triggers from (-1.0 to 1.0) into (0.0 to 1.0)
        num_axes = self.joystick.get_numaxes()
        raw_down_trigger = (self.joystick.get_axis(4) + 1.0) / 2.0 if num_axes > 4 else 0.0
        raw_up_trigger = (self.joystick.get_axis(5) + 1.0) / 2.0 if num_axes > 5 else 0.0
        
        raw_vertical = self._apply_deadzone(raw_up_trigger - raw_down_trigger) * -1

        forward = max(self.pwm_min, min(self.pwm_max, int(self.pwm_center + (raw_forward * self.pwm_range))))
        lateral = max(self.pwm_min, min(self.pwm_max, int(self.pwm_center + (raw_lateral * self.pwm_range))))
        vertical = max(self.pwm_min, min(self.pwm_max, int(self.pwm_center + (raw_vertical * self.pwm_range))))
        yaw = max(self.pwm_min, min(self.pwm_max, int(self.pwm_center + (raw_yaw * self.pwm_range))))

        with self.lock:
            self.target_manual_control = [forward, lateral, vertical, yaw]
            
            # Map standard Xbox buttons
            self.btn_states["BTN_SOUTH"] = self.joystick.get_button(0)  # A
            self.btn_states["BTN_EAST"] = self.joystick.get_button(1)   # B
            self.btn_states["BTN_WEST"] = self.joystick.get_button(2)   # X
            self.btn_states["BTN_NORTH"] = self.joystick.get_button(3)  # Y
            
            self.lb_btn = self.joystick.get_button(4) == 1
            self.rb_btn = self.joystick.get_button(5) == 1
            
            # Map D-PAD (HAT) for servo control
            if self.joystick.get_numhats() > 0:
                hat_x, hat_y = self.joystick.get_hat(0)
                self.servo_btn = hat_y

    def update_smoothed_controls(self, control_state):
        with self.lock:
            target_snapshot = self.target_manual_control[:]
            btn_snapshot = self.btn_states.copy()
            current_servo_btn = self.servo_btn
            lb = self.lb_btn
            rb = self.rb_btn

        for i in range(4):
            self.current_manual_control[i] += self.smoothing_factor * (
                target_snapshot[i] - self.current_manual_control[i]
            )

        logger.debug(f"""SENDING RC:
                     forward: {int(self.current_manual_control[0])}
                     lateral: {int(self.current_manual_control[1])}
                     vertical:{int(self.current_manual_control[2])}
                     yaw:     {int(self.current_manual_control[3])}""")

        control_state.update(
            forward=int(self.current_manual_control[0]),
            lateral=int(self.current_manual_control[1]),
            vertical=int(self.current_manual_control[2]),
            yaw=int(self.current_manual_control[3]),
        )

        for name, ctrl in self.btn_ctrl.items():
            if ctrl.toggle(btn_snapshot[name]):
                control_state.update(target_mode=ctrl.mode_name)

        if self.arm_btn.toggle(lb, rb):
            control_state.update(arm_toggle=True)

        now = time.time()
        dt = now - self.last_servo_time
        self.last_servo_time = now

        delta = current_servo_btn * self.MAX_SLEW_PER_SEC * dt

        # Servo boundaries
        self.servo_pwm = max(1700, min(2500, self.servo_pwm + delta))

        logger.debug(f"SERVO: {self.servo_pwm}")
        control_state.update(servo=int(self.servo_pwm))

        return control_state
