from dataclasses import dataclass
import curses, time, json, redis, logging, base64
import numpy as np
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

        self._redis = redis.Redis(host="localhost", port=6379, db=0)
        
        
    def handle_input(self, command: str):
        action_name = KEYMAP.get(command)
        if action_name is None:
            return "Pass" 

        return action_name
    

    def performAction(self, action):
        return self.controller.step(action=action)
    
    def publish_pose_odom(self, event):
        if not hasattr(self, "_redis") or self._redis is None:
            logger.warning("publish_pose_odom called but self.redis is not set.")
            return

        md = event.metadata or {}
        agent = md.get("agent", {})

        payload = {
            "timestamp": time.time(),
            "sequenceId": md.get("sequenceId"),
            "position": agent.get("position"),        # {x, y, z}
            "rotation": agent.get("rotation"),        # {x, y, z} (degrees)
            "cameraHorizon": agent.get("cameraHorizon"),
        }

        try:
            serialized = json.dumps(payload, default=float)
            self._redis.publish("ai2thor_pose_sensors", serialized)
        except Exception as e:
            logger.error(f"Failed to publish pose data to Redis: {e}")

    
    
    def publish_depth_data(self, event):
        if not hasattr(self, "_redis") or self._redis is None:
            logger.warning("publish_depth_data called but self.redis is not set.")
            return

        depth_frame = getattr(event, "depth_frame", None)
        if depth_frame is None:
            logger.warning("No depth_frame in event — skipping depth publish.")
            return
        
        try:
            depth_bytes = depth_frame.astype(np.float32).tobytes()
            depth_b64 = base64.b64encode(depth_bytes).decode("utf-8")
            payload = {
                "timestamp": time.time(),
                "height": depth_frame.shape[0],
                "width": depth_frame.shape[1],
                "encoding": "32FC1",
                "dtype": "float32",
                "data": depth_b64,
            }

            serialized = json.dumps(payload, default=float)
            self._redis.publish("ai2thor_depth_image", serialized)
        except Exception as e:
            logger.error(f"Failed to publish depth data to Redis: {e}")



    def run(self, stdscr):

        self.setup()

        pubsub = self._redis.pubsub()
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

                self.publish_pose_odom(event)
                self.publish_depth_data(event)

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