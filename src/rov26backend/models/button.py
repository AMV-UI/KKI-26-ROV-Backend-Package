import time
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

class DoubleButton:
    def __init__(self):
        self.pressed = False
        self.last_pressed = None

    def toggle(self, state):
        if not state and self.pressed:
            self.pressed = state
            if self.last_pressed is not None and abs(time.time() - self.last_pressed) < 1.0:
                return True
            self.last_pressed = time.time()
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
