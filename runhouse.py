from dataclasses import dataclass
import curses, time, json, redis, logging, base64
import numpy as np
from procthor.generation import PROCTHOR10K_ROOM_SPEC_SAMPLER, HouseGenerator
from ai2thor.controller import Controller


MOVE_ACTIONS = {"MoveAhead", "MoveBack", "MoveLeft", "MoveRight"}
ROTATE_ACTIONS = {"RotateLeft", "RotateRight"}


WARMUP_IMAGE_FRAMES = 20          # publish images for first N iterations even if Pass
IDLE_IMAGE_PERIOD_S = 1.0         # after warmup, publish images at most once per second when Pass

logger = logging.getLogger(__name__)

@dataclass
class ProcThorApp:
    split: str = "train"
    seed: int = 42
    fps: int = 50  

    def __post_init__(self):
        self.controller = None


    def setup(self):
        self.controller = Controller(
            width=800,
            height=600,
            commit_id="391b3fae4d4cc026f1522e5acf60953560235971", 
            scene="Procedural",
            quality="Low"
        )

        self.hg = HouseGenerator(split=self.split, seed=self.seed,
                                    room_spec_sampler=PROCTHOR10K_ROOM_SPEC_SAMPLER, 
                                    controller=self.controller)
        
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

        event = self.controller.step(
            action="LookUp",
        )

        assert event.metadata["lastActionSuccess"], event.metadata.get("errorMessage", "")    

        self._redis = redis.Redis(host="localhost", port=6379, db=0)
        
    

    def performAction(self, action: str, magnitude: float | None = None):

        if magnitude is None:
            return self.controller.step(action=action)

        if action in MOVE_ACTIONS:
            return self.controller.step(action=action, moveMagnitude=float(magnitude))

        if action in ROTATE_ACTIONS:
            return self.controller.step(action=action, degrees=float(magnitude))

        raise ValueError(
            f"Unsupported action '{action}' for magnitude-based execution. "
            f"Known move={sorted(MOVE_ACTIONS)}, rotate={sorted(ROTATE_ACTIONS)}"
        )
    
    
    def publish_pose_odom(self, event, timestamp):
        if not hasattr(self, "_redis") or self._redis is None:
            logger.warning("publish_pose_odom called but self.redis is not set.")
            return

        md = event.metadata or {}
        agent = md.get("agent", {})

        payload = {
            "timestamp": timestamp,
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

    
    
    def publish_depth_data(self, event, timestamp):
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
                "timestamp": timestamp,
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


    def publish_rgb_data(self, event, timestamp):
        if not hasattr(self, "_redis") or self._redis is None:
            logger.warning("publish_rgb_data called but self.redis is not set.")
            return

        rgb_frame = getattr(event, "frame", None)
        if rgb_frame is None:
            logger.warning("No rgb_frame in event — skipping rgb publish.")
            return
        
        
        try:
            height, width, _ = rgb_frame.shape

            rgb_bytes = rgb_frame.tobytes()
            rgb_b64 = base64.b64encode(rgb_bytes).decode("utf-8")
            payload = {
                "timestamp": timestamp,
                "height": height,
                "width": width,
                "encoding": "rgb8",
                "dtype": "uint8",
                "data": rgb_b64,
            }

            serialized = json.dumps(payload, default=float)
            self._redis.publish("ai2thor_rgb_image", serialized)
        except Exception as e:
            logger.error(f"Failed to publish rgb data to Redis: {e}")



    def run(self, stdscr):

        self.setup()

        pubsub = self._redis.pubsub()
        pubsub.subscribe("ai2thor_commands")
        logger.info("Subscribed to ai2thor_commands")

        warmup_remaining = WARMUP_IMAGE_FRAMES
        last_idle_image_pub = 0.0

        try:
            # TODO implementare push a hz predefiniti, non "ogni volta che succede qualcosa"
            while True:
                msg = pubsub.get_message(ignore_subscribe_messages=True, timeout=0.1)

                command = None
                magnitude = None

                if msg is not None:
                    try:
                        payload = json.loads(msg["data"])
                        command = payload.get("action")
                        magnitude = payload.get("magnitude")
                    except Exception:
                        logger.exception("Failed to parse pubsub message")
                        command = None

                if command is not None:
                    act = command
                else:
                    act = "Pass" 

                event = self.performAction(act, magnitude)

                timestamp = time.time()

                self.publish_pose_odom(event, timestamp)


                publish_images = False
                if act != "Pass":
                    publish_images = True
                elif warmup_remaining > 0:
                    publish_images = True
                    warmup_remaining -= 1
                else:
                    if (timestamp - last_idle_image_pub) >= IDLE_IMAGE_PERIOD_S:
                        publish_images = True
                        last_idle_image_pub = timestamp

                if publish_images:
                    self.publish_depth_data(event, timestamp)
                    self.publish_rgb_data(event, timestamp)

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