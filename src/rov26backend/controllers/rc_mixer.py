from rov26backend.models.button import PressButton


class RcMixer:
    def __init__(self):
        self.current_manual_control = {
            "forward": 1500.0,
            "lateral": 1500.0,
            "vertical": 1500.0,
            "yaw": 1500.0,
        }
        self.target_manual_control = {
            "forward": 1500.0,
            "lateral": 1500.0,
            "vertical": 1500.0,
            "yaw": 1500.0,
        }
        self.target_manual_control = [1500.0, 1500.0, 1500.0, 1500.0]

        self.depth_hold_btn = PressButton()
        self.manual_btn = PressButton()
        self.stabilize_btn = PressButton()
        self.autonomous_btn = PressButton()

    def update_inputs(
        self,
        l_analog_x=0,
        l_analog_y=0,
        r_analog_x=0,
        r_analog_y=0,
        rt_analog=0,
        lt_analog=0,
        rb=False,
        lb=False,
        btn_up=False,
        btn_right=False,
        btn_left=False,
        btn_down=False,
    ):
        pass
