class PressButton:
    def __init__(self):
        self.pressed = False

    def toggle(self, state):
        if state and not self.pressed:
            self.pressed = state
            return True
        else:
            self.pressed = state
            return False
