class ModeButton:
    def __init__(self, mode_name):
        self.pressed = False
        self.mode_name = mode_name

    def toggle(self, state):
        if state and not self.pressed:
            self.pressed = state
            return True
        else:
            self.pressed = state
            return False
