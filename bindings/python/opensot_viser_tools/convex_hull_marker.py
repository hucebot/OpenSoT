import viser
import numpy as np

class convex_hull_marker:
    def __init__(self, server):
        self.convex_hull = None
        self.server = server

    def update(self, points):
        ch = np.array(points)
        # build segments
        segments = np.stack([ch, np.roll(ch, -1, axis=0)], axis=1)

        if not self.convex_hull:
            self.convex_hull = self.server.scene.add_line_segments(
                "/ch",
                points=segments,
                line_width=3,
                colors=(255, 0, 0),
            )
        else:
            self.convex_hull.points = segments
