"""Mouse controls for the colour task, drawn inside the camera window."""
import textwrap

import cv2
import numpy as np

from .color import masks

FONT = cv2.FONT_HERSHEY_SIMPLEX
WHITE = (235, 235, 235)
GREEN = (90, 220, 110)
AMBER = (0, 210, 255)


class ColorControls:
    PANEL_WIDTH = 360

    def init_controls(self):
        self.pending_color = None
        self.pending_aimed = False
        self.debug_open = False
        self.clear_confirm = False
        self.last_input = "No input yet"
        self.feedback = "Choose a colour, point at its pole, then press RB."
        self.ui_buttons = []

    def notify(self, message):
        self.feedback = message

    def controls_view(self, ctx, camera_view):
        height, width = camera_view.shape[:2]
        result = np.zeros((max(720, height), width + self.PANEL_WIDTH, 3), np.uint8)
        result[:height, :width] = camera_view
        result[:, width:] = (29, 32, 38)
        panel = result[:, width:]
        self.ui_buttons = []

        def text(line, y, color=WHITE, scale=0.48):
            cv2.putText(panel, line, (14, y), FONT, scale, color, 1, cv2.LINE_AA)

        def button(action, title, x, y, w, active=False):
            cv2.rectangle(panel, (x, y), (x+w, y+34),
                          (55, 92, 100) if active else (54, 59, 67), -1)
            cv2.rectangle(panel, (x, y), (x+w, y+34), (95, 110, 120), 1)
            cv2.putText(panel, title, (x+8, y+22), FONT, 0.45, WHITE, 1, cv2.LINE_AA)
            self.ui_buttons.append((action, (width+x, y, w, 34)))

        text(f"COLOUR  /  {self.method}", 27, AMBER, 0.65)
        counts = self.lab.samples
        ready = self.method == "HSV" or all(counts.values())
        text("READY TO RECOGNIZE" if ready else "SETUP: calibrate R, Y and B", 55,
             GREEN if ready else AMBER)
        text("   ".join(f"{c}: {len(counts[c])} {'OK' if counts[c] else 'NEEDED'}"
                        for c in "RYB") if self.method == "LAB" else "Using saved HSV thresholds", 79, scale=0.40)
        for color, x, label in [('R', 14, 'Red'), ('Y', 126, 'Yellow'), ('B', 238, 'Blue')]:
            button('cal_'+color, label, x, 97, 105, self.pending_color == color)
        text("Choose colour > Aim at pole > RB", 153, scale=0.44)
        button('recognize', 'Take sample (RB / C)' if self.pending_color else 'Recognize (RB / C)', 14, 169, 214)
        button('cancel', 'Cancel', 238, 169, 105)
        text(f"Sample box: {self.radius*2+1} x {self.radius*2+1} pixels", 232)
        button('smaller', 'Smaller', 14, 245, 158)
        button('larger', 'Larger', 184, 245, 159)
        button('clear', 'Confirm clear samples?' if self.clear_confirm else 'Clear Lab samples', 14, 291, 329)
        text("LAST INPUT: " + self.last_input, 352, AMBER, 0.43)
        for index, line in enumerate(textwrap.wrap(self.feedback, width=40)[:4]):
            text(line, 377 + 21*index, scale=0.44)
        button('debug', 'Close debug' if self.debug_open else 'Debug settings', 14, 471, 329)
        if self.debug_open:
            button('method', 'Switch to HSV' if self.method == 'LAB' else 'Switch to Lab', 14, 515, 158)
            button('masks', 'Hide HSV masks' if ctx.color.show_masks else 'Show HSV masks', 184, 515, 159)
            if ctx.color.show_masks and ctx.last_frame is not None:
                previews = masks(ctx.last_frame, ctx.color.ranges)
                for i, c in enumerate('RYB'):
                    small = cv2.resize(previews[c], (102, 85))
                    panel[586:671, 14+112*i:116+112*i] = cv2.cvtColor(small, cv2.COLOR_GRAY2BGR)
                    cv2.putText(panel, c+' (HSV)', (14+112*i, 579), FONT, 0.36, WHITE, 1)
        text("B: colour mode   Y: safety lock", 704, scale=0.43)
        return result

    def control_mouse(self, ctx, event, x, y, flags, param):
        if ctx.last_frame is None:
            return
        height, width = ctx.last_frame.shape[:2]
        if 0 <= x < width and 0 <= y < height:
            ctx.mouse.on_event(event, x, y, flags, param)
            if self.pending_color:
                self.pending_aimed = True
            if event == cv2.EVENT_LBUTTONDOWN:
                self.last_input = f"Picture click ({x}, {y})"
                if self.pending_color:
                    self.notify(f"Aiming at {self.pending_color}. Press RB to take one sample.")
                else:
                    self.notify("Target selected. Click Recognize or press RB.")
            return
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        # Sidebar motion must not replace the last target on the camera image.
        for action, (bx, by, bw, bh) in self.ui_buttons:
            if bx <= x <= bx+bw and by <= y <= by+bh:
                self.last_input = "Click: " + action.replace('_', ' ')
                if action != 'clear':
                    self.clear_confirm = False
                if action.startswith('cal_'):
                    self.pending_color = action[-1]
                    self.pending_aimed = False
                    self.answer = None
                    self.notify(f"Point at the {self.pending_color} pole, then press RB to take one sample.")
                elif action == 'recognize':
                    self.capture(ctx)
                elif action == 'cancel':
                    self.pending_color = None
                    self.notify("Selection cancelled.")
                elif action == 'clear':
                    if self.clear_confirm:
                        self.on_key('0', ctx)
                        self.clear_confirm = False
                    else:
                        self.clear_confirm = True
                        self.notify("Click Confirm clear samples to erase Lab calibration.")
                elif action == 'debug':
                    self.debug_open = not self.debug_open
                    if not self.debug_open:
                        ctx.color.show_masks = False
                else:
                    key = {'smaller': '[', 'larger': ']', 'method': 'v', 'masks': 'd'}[action]
                    self.on_key(key, ctx)
                return
