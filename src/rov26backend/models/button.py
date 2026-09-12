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


# class PressButtonTarget:
#     def __init__(selTargetf):
#         self.pressed = False
#
#     def toggle(self, state):
#         if state and not self.pressed:
#             self.pressed = state
#             return True
#         else:
#             self.pressed = state
#             return False
