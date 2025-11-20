from dataclasses import dataclass
import curses, time, json, redis, logging
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
    "STOP":         None, 
}

logger = logging.getLogger(__name__)

@dataclass
class ProcThorApp:
    split: str = "train"
    seed: int = 42
    fps: int = 50  

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

        self.redis = redis.Redis(host="localhost", port=6379, db=0)
        
        
    def handle_input(self, command: str):
        action_name = KEYMAP.get(command)
        if action_name is None:
            return "Pass" 

        return action_name
    

    def performAction(self, action):
        return self.controller.step(action=action)
    
    
    def publishData(self, event):
        if not hasattr(self, "redis") or self.redis is None:
            logger.warning("publishData called but self.pubsub is not set.")
            return

        md = event.metadata or {}
        agent = md.get("agent", {})

        payload = {
            "timestamp": time.time(),                     
            "sequenceId": md.get("sequenceId"),         
            "position": agent.get("position"),       # {x, y, z}
            "rotation": agent.get("rotation"),       # {x, y, z} (deg)
            "cameraHorizon": agent.get("cameraHorizon"),
        }

        try:
            serialized = json.dumps(payload, default=float)
            self.redis.publish("ai2thor_pose_sensors", serialized)
        except Exception as e:
            logger.error(f"Failed to publish agent state to Redis: {e}")


    def run(self, stdscr):

        self.setup()

        pubsub = self.redis.pubsub()
        pubsub.subscribe("ai2thor_commands")
        logger.info("Subscribed to ai2thor_commands")

        try:
            while True:
                msg = pubsub.get_message(ignore_subscribe_messages=True, timeout=0.1)

                command = None

                if msg is not None:
                    try:
                        payload = json.loads(msg["data"])
                        command = payload.get("action")
                    except Exception:
                        logger.exception("Failed to parse pubsub message")
                        command = None

                if command is not None:
                    act = self.handle_input(command)
                else:
                    act = "Pass" 

              
                event = self.performAction(act)

                self.publishData(event)

                stdscr.addstr(
                    0, 0,
                    f"Last command: {command or 'None':<12}  Action: {act:<12}  "
                    f"success={event.metadata.get('lastActionSuccess')}   "
                )
                stdscr.refresh()

        finally:
            try:
                self.controller.stop()
                logger.info("controller stopped cleanly")
            except Exception:
                pass

def main(stdscr):
    ProcThorApp().run(stdscr)

if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    curses.wrapper(main)