from enum import Enum, unique

@unique
class ActionType(Enum):
    MOVE_AHEAD =   "MoveAhead"
    MOVE_BACK =    "MoveBack"
    STRAFE_LEFT =  "MoveLeft"
    STRAFE_RIGHT = "MoveRight"
    ROTATE_LEFT =  "RotateLeft"
    ROTATE_RIGHT = "RotateRight"
    LOOK_UP =      "LookUp"
    LOOK_DOWN =    "LookDown"
    STOP =         "Stop"
