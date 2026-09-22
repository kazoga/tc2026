"""Hold-only fault injection; wall-clock timeout restores GNSS input."""
import math


class GnssDropoutHold:
    def __init__(self, button=5, timeout_s=.5):
        if button < 0 or not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError('Invalid GNSS dropout button or timeout')
        self.button = button
        self.timeout_s = timeout_s
        self.pressed = False
        self.received = -math.inf

    def update(self, buttons, now):
        self.pressed = len(buttons) > self.button and buttons[self.button] == 1
        self.received = now

    def active(self, now):
        return self.pressed and 0 <= now-self.received < self.timeout_s
