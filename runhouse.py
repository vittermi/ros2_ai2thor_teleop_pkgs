import curses, time
from procthor.generation import PROCTHOR10K_ROOM_SPEC_SAMPLER, HouseGenerator
from ai2thor.controller import Controller

KEYMAP = {
    ord('w'): "MoveAhead",
    ord('s'): "MoveBack",
    ord('a'): "MoveLeft",
    ord('d'): "MoveRight",
    ord('q'): "RotateLeft",
    ord('e'): "RotateRight",
}

def main(stdscr):
    curses.cbreak()
    curses.noecho()
    stdscr.nodelay(True)      
    stdscr.clear()
    stdscr.addstr(0, 0, "Controls: w/a/s/d move, q/e rotate, x to exit")
    stdscr.refresh()

    # controller = Controller(width=800, height=600, rotateStepDegrees=5, snapToGrid=False)

    # Build scene with ProcTHOR
    procthor_hg = HouseGenerator(split="train", seed=42,
                        room_spec_sampler=PROCTHOR10K_ROOM_SPEC_SAMPLER)
    house, _ = procthor_hg.sample()

    house.validate(procthor_hg.controller)  

    controller = procthor_hg.controller 
   

    try:
        while True:
            act = None
            # non blocking, read all keys in buf
            while True:
                ch = stdscr.getch()
                if ch == -1:
                    break
                if ch in (3, 4, ord('x')):   # Ctrl-C, Ctrl-D
                    return
                if ch in KEYMAP:
                    act = KEYMAP[ch]

            if act is None:
                act = "Pass"

            ev = controller.step(action=act)
            stdscr.addstr(2, 0,
                          f"Action: {act:<12} success={ev.metadata.get('lastActionSuccess')}   ")
            stdscr.refresh()
            time.sleep(0.02)
    finally:
        try:
            controller.stop()
        except Exception:
            pass

if __name__ == "__main__":
    curses.wrapper(main)
