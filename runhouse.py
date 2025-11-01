from dataclasses import dataclass
import curses, time
from procthor.generation import PROCTHOR10K_ROOM_SPEC_SAMPLER, HouseGenerator

KEYMAP = {
    ord('w'): "MoveAhead",
    ord('s'): "MoveBack",
    ord('a'): "MoveLeft",
    ord('d'): "MoveRight",
    ord('q'): "RotateLeft",
    ord('e'): "RotateRight",
}

@dataclass
class ProcThorApp:
    split: str = "train"
    seed: int = 42
    fps: int = 50  # ~20ms per frame

    def __post_init__(self):
        self.controller = None

    def setup(self):
        self.hg = HouseGenerator(split=self.split, seed=self.seed,
                                    room_spec_sampler=PROCTHOR10K_ROOM_SPEC_SAMPLER)
        house, _ = self.hg.sample()
        house.validate(self.hg.controller)
        self.controller = self.hg.controller

    def handle_input(self, stdscr):
        act = None
        while True:
            ch = stdscr.getch()
            if ch == -1:
                break
            if ch in (3, 4, ord('x')):  # Ctrl-C, Ctrl-D, x
                return "EXIT"
            if ch in KEYMAP:
                act = KEYMAP[ch]
        return act or "Pass"

    def step(self, action):
        return self.controller.step(action=action)

    def run(self, stdscr):
        curses.cbreak()
        curses.noecho()
        stdscr.nodelay(True)
        stdscr.clear()
        stdscr.addstr(0, 0, "Controls: w/a/s/d move, q/e rotate, x to exit")
        stdscr.refresh()

        self.setup()
        try:
            delay = 1 / self.fps
            while True:
                act = self.handle_input(stdscr) # this should get from topic
                if act == "EXIT":
                    break
                ev = self.step(act)
                stdscr.addstr(2, 0, f"Action: {act:<12} success={ev.metadata.get('lastActionSuccess')}   ")
                stdscr.refresh()
                time.sleep(delay)
        finally:
            try:
                self.controller.stop()
            except Exception:
                pass

def main(stdscr):
    ProcThorApp().run(stdscr)

if __name__ == "__main__":
    curses.wrapper(main)