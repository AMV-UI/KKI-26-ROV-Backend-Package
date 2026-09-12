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


class PressButtonTarget:
    def __init__(self, target):
        self.target = target
        self.pressed = False

    def toggle(self, target):
        state = target == self.target
        if state and not self.pressed:
            self.pressed = state
            return True
        else:
            self.pressed = state
            return False
