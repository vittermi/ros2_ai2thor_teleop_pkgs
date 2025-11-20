from dataclasses import dataclass
import curses, time, json, redis
from procthor.generation import PROCTHOR10K_ROOM_SPEC_SAMPLER, HouseGenerator

KEYMAP = {
    "MOVE_AHEAD":   "MoveAhead",
    "MOVE_BACK":    "MoveBack",
    "STRAFE_LEFT":  "MoveLeft",
    "STRAFE_RIGHT": "MoveRight",
    "ROTATE_LEFT":  "RotateLeft",
    "ROTATE_RIGHT": "RotateRight",
    "LOOK_UP":      "LookUp",
    "LOOK_DOWN":    "LookDown",
    "STOP":         None,       # special case; we’ll treat as "Pass"
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

        event = self.controller.step(
            action="Initialize",
            gridSize=0.25,
            renderDepthImage=True,
            renderInstanceSegmentation=False,
        )

        assert event.metadata["lastActionSuccess"], event.metadata.get("errorMessage", "")

    def handle_input(self, command: str):
        action_name = KEYMAP.get(command)
        if action_name is None:
            return "Pass"  # unsupported command → do nothing

        return action_name

    def step(self, action):
        return self.controller.step(action=action)

    def run(self, stdscr):

        r = redis.Redis(host="localhost", port=6379, db=0)
        pubsub = r.pubsub()
        pubsub.subscribe("ai2thor_commands")

        delay = 0.05 

        self.setup()


        try:
            while True:
                msg = pubsub.get_message(ignore_subscribe_messages=True, timeout=0.1)

                command = None

                if msg is not None:
                    try:
                        payload = json.loads(msg["data"])
                        command = payload.get("action")
                    except Exception:
                        command = None

                if command is not None:
                    act = self.handle_input(command)
                else:
                    act = "Pass" 

              
                ev = self.step(act)

                stdscr.addstr(
                    0, 0,
                    f"Last command: {command or 'None':<12}  Action: {act:<12}  "
                    f"success={ev.metadata.get('lastActionSuccess')}   "
                )
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