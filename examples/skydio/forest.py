"""Seeded obstacle scene for the Skydio-style demo: a forest of trees and poles, a wall with a doorway, an arch, boulders, and (follow mission) the path
of a moving subject. Everything here is GROUND TRUTH of the world: the evaluation (clearance, collisions) and the scene builder read it, the
autonomy never does (it only sees the depth image).

    f = generate(seed, mission="goal")        # Forest: obstacles [Obstacle], start, goal, subject path
    f.clearance(p)                            # distance from point p to the nearest obstacle SURFACE (m), exact
    f.footprints()                            # (circles, boxes) in the ground plane, for the subject's path planner

Shapes are exact primitives, the same ones that become MJCF geoms (scene.py): vertical cylinders (trunks, poles, arch posts), spheres (canopies,
boulders) and yawed boxes (wall segments, the arch lintel). The ground is the MJCF floor plane z = 0.

World layout (x forward): start (0, 0) at the ground; forest between x = 4 and x = 62, |y| <= 14; goal near x = 58. `goal` mission: wall with a
2.4 m doorway at x = 28-34, arch at x = 42-46 (2.4 m wide, 2.8 m high under the lintel), trees and poles in between. `follow` mission: no wall,
the subject (a 1.75 m person-sized figure) walks a collision-free path, trees only plus the arch."""

import math
from dataclasses import dataclass, field

import numpy as np

X0, X1, YMAX = 4.0, 62.0, 14.0
MIN_TREE_GAP = 2.8  # m between trunk centres (so a gap of >= 2.2 m between trunk surfaces at the largest radius)
DOOR_W, ARCH_W, ARCH_H = 2.4, 2.4, 2.8  # clear widths and the lintel's underside


@dataclass
class Obstacle:
    kind: str  # cyl (vertical axis) | sphere | box
    name: str
    pos: tuple  # centre (x, y, z)
    size: tuple  # cyl: (radius, half-height); sphere: (radius,); box: (hx, hy, hz)
    yaw: float = 0.0  # rad about z (box)
    rgba: tuple = (0.5, 0.5, 0.5, 1.0)

    def distance(self, p):
        """Distance from the point p to the surface (negative inside)."""
        d = np.asarray(p, float) - np.asarray(self.pos)
        if self.kind == "sphere":
            return float(np.linalg.norm(d) - self.size[0])
        if self.kind == "cyl":
            r, h = self.size
            q = np.array([math.hypot(d[0], d[1]) - r, abs(d[2]) - h])
            return float(np.hypot(*np.maximum(q, 0)) + min(max(q), 0.0))
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        local = np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1], d[2]])
        q = np.abs(local) - np.array(self.size)
        return float(np.linalg.norm(np.maximum(q, 0)) + min(q.max(), 0.0))


@dataclass
class Forest:
    seed: int
    mission: str
    obstacles: list = field(default_factory=list)
    start: tuple = (0.0, 0.0)
    goal: tuple = (58.0, 0.0)
    path: list = field(default_factory=list)  # subject path [(x, y)], follow mission
    notes: dict = field(default_factory=dict)
    _index: tuple = field(default=None, repr=False)

    def clearance(self, p, cutoff=6.0):
        """Exact distance from p to the nearest obstacle surface, capped at `cutoff` (a bounding-sphere pre-filter skips the far ones)."""
        if self._index is None:
            reach = [
                math.hypot(*o.size[:2])
                if o.kind == "cyl"
                else (
                    o.size[0] if o.kind == "sphere" else float(np.linalg.norm(o.size))
                )
                for o in self.obstacles
            ]
            self._index = (np.array([o.pos for o in self.obstacles]), np.array(reach))
        c, reach = self._index
        near = np.nonzero(np.linalg.norm(c - np.asarray(p), axis=1) - reach < cutoff)[0]
        return min([cutoff] + [self.obstacles[i].distance(p) for i in near])

    def footprints(self):
        """Ground-plane view for the subject's planner: circles (x, y, r) of everything reaching below 1.75 m, boxes (x, y, hx, hy, yaw)."""
        circles, boxes = [], []
        for o in self.obstacles:
            if o.pos[2] - (o.size[-1] if o.kind != "sphere" else o.size[0]) > 1.75:
                continue  # canopies and the lintel are over the subject's head
            if o.kind == "cyl" or o.kind == "sphere":
                circles.append((o.pos[0], o.pos[1], o.size[0]))
            else:
                boxes.append((o.pos[0], o.pos[1], o.size[0], o.size[1], o.yaw))
        return circles, boxes

    def to_json(self):
        return dict(
            seed=self.seed,
            mission=self.mission,
            start=self.start,
            goal=self.goal,
            path=[list(p) for p in self.path],
            notes=self.notes,
            obstacles=[
                dict(
                    kind=o.kind,
                    name=o.name,
                    pos=list(o.pos),
                    size=list(o.size),
                    yaw=o.yaw,
                    rgba=list(o.rgba),
                )
                for o in self.obstacles
            ],
        )


BARK, LEAF, CONCRETE, STEEL, RUST, ROCK = ((0.33, 0.23, 0.15, 1.0), (0.17, 0.38, 0.16, 1.0), (0.66, 0.64, 0.60, 1.0), (0.32, 0.34, 0.37, 1.0),
                                          (0.62, 0.30, 0.16, 1.0), (0.46, 0.45, 0.42, 1.0))  # fmt: skip


def _wall(f, x, y_gap):
    """A 0.3 m thick, 5 m high wall across the path with a doorway of DOOR_W centred at y_gap; 8 m to each side of the middle."""
    half = 8.0
    for i, (a, b) in enumerate(
        [(-half, y_gap - DOOR_W / 2), (y_gap + DOOR_W / 2, half)]
    ):
        f.obstacles.append(
            Obstacle(
                "box",
                f"wall{i}",
                (x, (a + b) / 2, 2.5),
                (0.15, (b - a) / 2, 2.5),
                0.0,
                CONCRETE,
            )
        )


def _arch(f, x, y):
    for i, s in enumerate((-1, 1)):
        f.obstacles.append(
            Obstacle(
                "cyl",
                f"arch_post{i}",
                (x, y + s * (ARCH_W / 2 + 0.15), 1.7),
                (0.15, 1.7),
                0.0,
                RUST,
            )
        )
    f.obstacles.append(
        Obstacle(
            "box",
            "arch_lintel",
            (x, y, ARCH_H + 0.2),
            (0.2, ARCH_W / 2 + 0.45, 0.2),
            0.0,
            RUST,
        )
    )


def generate(seed, mission="goal") -> Forest:
    rng = np.random.default_rng(seed)
    f = Forest(seed, mission)
    f.goal = (58.0, float(rng.uniform(-4, 4)))
    if mission == "goal":
        xw, yg = float(rng.uniform(28, 34)), float(rng.uniform(-4, 4))
        _wall(f, xw, yg)
        xa, ya = float(rng.uniform(42, 46)), float(rng.uniform(-4, 4))
        _arch(f, xa, ya)
        f.notes.update(wall_x=xw, door_y=yg, arch_x=xa, arch_y=ya)
        solid = [(xw, yg, 3.5), (xa, ya, 3.0)]
    else:
        xa, ya = float(rng.uniform(44, 48)), float(rng.uniform(-3, 3))
        _arch(f, xa, ya)
        f.notes.update(arch_x=xa, arch_y=ya)
        solid = [(xa, ya, 3.0)]
    # trees by dart throwing: no two trunks within MIN_TREE_GAP, none at the start, goal or the structures
    trunks = [(o.pos[0], o.pos[1]) for o in f.obstacles if o.kind == "cyl"]
    n_tree = (
        int(rng.integers(50, 66)) if mission == "goal" else int(rng.integers(44, 58))
    )
    for _ in range(4000):
        if len([o for o in f.obstacles if o.name.startswith("tree")]) >= n_tree:
            break
        x, y = float(rng.uniform(X0, X1)), float(rng.uniform(-YMAX, YMAX))
        if (
            math.hypot(x - f.goal[0], y - f.goal[1]) < 4.0
            or math.hypot(x - 6.0, y) < 2.5
        ):
            continue
        if any(math.hypot(x - sx, y - sy) < sr for sx, sy, sr in solid):
            continue
        if mission == "goal" and abs(x - f.notes["wall_x"]) < 1.2:
            continue
        if any(math.hypot(x - tx, y - ty) < MIN_TREE_GAP for tx, ty in trunks):
            continue
        r = float(np.exp(rng.uniform(np.log(0.12), np.log(0.34))))
        h = float(rng.uniform(5.5, 8.0))
        i = len([o for o in f.obstacles if o.name.startswith("tree")])
        f.obstacles.append(
            Obstacle("cyl", f"tree{i}_trunk", (x, y, h / 2), (r, h / 2), 0.0, BARK)
        )
        rc = float(rng.uniform(1.4, 2.2))
        f.obstacles.append(
            Obstacle(
                "sphere",
                f"tree{i}_canopy",
                (x + float(rng.normal(0, 0.2)), y + float(rng.normal(0, 0.2)), h - 0.6),
                (rc,),
                0.0,
                LEAF,
            )
        )
        trunks.append((x, y))
    # poles (lamp posts / signs): thin, up to 4.5 m; boulders: half-buried spheres (tops at 0.6-1.3 m)
    for i in range(int(rng.integers(8, 13))):
        for _ in range(200):
            x, y = float(rng.uniform(X0, X1)), float(rng.uniform(-YMAX, YMAX))
            if (math.hypot(x - f.goal[0], y - f.goal[1]) < 3.0 or math.hypot(x - 6.0, y) < 2.0
                    or any(math.hypot(x - sx, y - sy) < sr for sx, sy, sr in solid)
                    or any(math.hypot(x - tx, y - ty) < 1.8 for tx, ty in trunks)):  # fmt: skip
                continue
            h, r = float(rng.uniform(3.5, 4.5)), float(rng.uniform(0.05, 0.09))
            f.obstacles.append(
                Obstacle("cyl", f"pole{i}", (x, y, h / 2), (r, h / 2), 0.0, STEEL)
            )
            trunks.append((x, y))
            break
    for i in range(int(rng.integers(6, 10))):
        for _ in range(200):
            x, y = float(rng.uniform(X0, X1)), float(rng.uniform(-YMAX, YMAX))
            if (
                any(math.hypot(x - tx, y - ty) < 2.0 for tx, ty in trunks)
                or math.hypot(x - 6.0, y) < 2.5
            ):
                continue
            r = float(rng.uniform(0.7, 1.1))
            f.obstacles.append(
                Obstacle(
                    "sphere",
                    f"boulder{i}",
                    (x, y, float(rng.uniform(0.0, 0.3))),
                    (r,),
                    0.0,
                    ROCK,
                )
            )
            trunks.append((x, y))
            break
    if mission == "follow":
        f.path = subject_path(f, rng)
        f.goal = f.path[-1]
    return f


def subject_path(f, rng, res=0.5):
    """A collision-free walk for the subject: A* on a 0.5 m grid over the ground-plane footprints inflated by a margin, from (6, 0) to the far side,
    through two random via points so that the route winds between the trees, then string-pulled (line-of-sight shortcuts) and returned as a polyline.
    The margin starts at 1.1 m and shrinks to 0.8 m when a tight tree pair or the arch's 2.4 m gap blocks the route."""
    via = [(6.0, 0.0), (float(rng.uniform(18, 26)), float(rng.uniform(-8, 8))), (float(rng.uniform(34, 42)), float(rng.uniform(-8, 8))),
           (float(f.goal[0]) - 2.0, float(f.goal[1]))]  # fmt: skip
    for margin in (1.1, 1.0, 0.9, 0.8):
        path = _walk(f, via, res, margin)
        if path:
            return path
    raise RuntimeError(f"no walkable subject path for seed {f.seed}")


def _walk(f, via, res, margin):
    circles, boxes = f.footprints()
    nx, ny = int((X1 + 4) / res), int(2 * (YMAX + 2) / res)
    ox, oy = -1.0, -(YMAX + 2)
    free = np.ones((nx, ny), bool)
    gx, gy = np.meshgrid(
        ox + res * (np.arange(nx) + 0.5),
        oy + res * (np.arange(ny) + 0.5),
        indexing="ij",
    )
    for cx, cy, r in circles:
        free &= np.hypot(gx - cx, gy - cy) > r + margin
    for bx, by, hx, hy, yaw in boxes:
        c, s = math.cos(yaw), math.sin(yaw)
        lx, ly = c * (gx - bx) + s * (gy - by), -s * (gx - bx) + c * (gy - by)
        free &= (np.abs(lx) > hx + margin) | (np.abs(ly) > hy + margin)
    free &= np.abs(gy) < YMAX
    cell = lambda p: (int((p[0] - ox) / res), int((p[1] - oy) / res))
    pts = []
    for a, b in zip(via[:-1], via[1:]):
        seg = _astar(free, _nearest_free(free, cell(a)), _nearest_free(free, cell(b)))
        if not seg:
            return None
        pts += seg
    xy = [(ox + res * (i + 0.5), oy + res * (j + 0.5)) for i, j in pts]
    # string pulling on the inflated grid
    out, i = [xy[0]], 0
    while i < len(xy) - 1:
        j = len(xy) - 1
        while j > i + 1 and not _los(free, xy[i], xy[j], ox, oy, res):
            j -= 1
        out.append(xy[j])
        i = j
    return [(round(x, 3), round(y, 3)) for x, y in out]


def _nearest_free(free, c):
    if free[c]:
        return c
    idx = np.argwhere(free)
    return tuple(idx[np.argmin(((idx - np.array(c)) ** 2).sum(1))])


def _los(free, a, b, ox, oy, res):
    n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / (res * 0.5)))
    for t in np.linspace(0, 1, n):
        i, j = (
            int((a[0] + t * (b[0] - a[0]) - ox) / res),
            int((a[1] + t * (b[1] - a[1]) - oy) / res),
        )
        if not free[i, j]:
            return False
    return True


def _astar(free, s, g):
    import heapq

    nx, ny = free.shape
    dist, prev, heap = {s: 0.0}, {}, [(0.0, s)]
    steps = [
        (1, 0, 1.0),
        (-1, 0, 1.0),
        (0, 1, 1.0),
        (0, -1, 1.0),
        (1, 1, 1.414),
        (1, -1, 1.414),
        (-1, 1, 1.414),
        (-1, -1, 1.414),
    ]
    while heap:
        _, u = heapq.heappop(heap)
        if u == g:
            path = [u]
            while path[-1] in prev:
                path.append(prev[path[-1]])
            return path[::-1]
        for dx, dy, w in steps:
            v = (u[0] + dx, u[1] + dy)
            if not (0 <= v[0] < nx and 0 <= v[1] < ny) or not free[v]:
                continue
            nd = dist[u] + w
            if nd < dist.get(v, 1e18):
                dist[v], prev[v] = nd, u
                heapq.heappush(heap, (nd + math.hypot(g[0] - v[0], g[1] - v[1]), v))
    return None
