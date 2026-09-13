class SimulPressButton:
    def __init__(self):
        self.pressed1 = False
        self.pressed2 = False

    def toggle(self, click1, click2):
        if (
            self.pressed1
            and self.pressed2
            and (
                (click1 and not click2)
                or (not click1 and click2)
                or (not click1 and not click2)
            )
        ):
            self.pressed1 = click1
            self.pressed2 = click2
            return True
        else:
            self.pressed1 = click1
            self.pressed2 = click2
            return False
