from ..system.logger import log


class ObstacleStrategy:
    LEFT = "left"
    RIGHT = "right"

    def __init__(self, prefer=LEFT):
        self.prefer = prefer

    def decide(self, wall_left, wall_right, wall_front):
        if wall_front:
            free_left = not wall_left if not wall_left else False
            free_right = not wall_right if not wall_right else False
            if free_left and free_right:
                return self.prefer
            elif free_left:
                return self.LEFT
            elif free_right:
                return self.RIGHT
            else:
                return "reverse"
        return "forward"

    def set_preference(self, pref):
        self.prefer = pref
