import matplotlib.pyplot as plt
import random
import math
from collections import deque


class Node:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent
        self.cost = 0.0
    

def dist(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)


def sample(x_range, y_range):
    return Node(random.uniform(*x_range), random.uniform(*y_range))


def steer(src, tgt, step):

    d = dist(src, tgt)
    if d <= step:
        nx, ny = tgt.x, tgt.y
    else:
        theta = math.atan2(tgt.y - src.y, tgt.x - src.x)
        nx = src.x + step * math.cos(theta)
        ny = src.y + step * math.sin(theta)
    return Node(nx, ny)

def point_in_obstacle(x, y, obstacles):
    for (ox, oy, r) in obstacles:
        if math.hypot(x - ox, y - oy) <= r:
            return True
    return False


def collision(node, obstacles):
    return point_in_obstacle(node.x, node.y, obstacles)


def collision_edge(a, b, obstacles, resolution= 1.0):

    seg_len = dist(a, b)
    if seg_len == 0:
        return collision(a, obstacles)
    steps = int(seg_len / resolution)
    for i in range(steps + 1):
        t = i / steps if steps else 1.0
        x = a.x + t * (b.x - a.x)
        y = a.y + t * (b.y - a.y)
        if point_in_obstacle(x, y, obstacles):
            return True
    return False

# BFS
def propagate_costs(root, nodes):
    q = deque([root])
    while q:
        cur = q.popleft()
        for n in nodes:
            if n.parent is cur:
                old_cost = n.cost
                n.cost = cur.cost + dist(cur, n)
                if abs(n.cost - old_cost) > 1e-9:
                    q.append(n)

def backtrace(node):
    path = []
    while node:
        path.append((node.x, node.y))
        node = node.parent
    return path[::-1]


def rrt_star(start,goal,*,
             x_range=(0, 100),
             y_range=(0, 100),
             obstacles=(),
             step_size=2.0,
             max_iter=5000,
             goal_sample_rate=0.05,
             goal_threshold=2.0,
             gamma = 30.0,
             dim = 2):
  
    random.seed(0)

    start.cost = 0.0
    nodes = [start]

    for _ in range(max_iter):
        rnd = Node(goal.x, goal.y) if random.random() < goal_sample_rate else sample(x_range, y_range)

        nearest_node = min(nodes, key=lambda n: dist(n, rnd))
        new_node = steer(nearest_node, rnd, step_size)

        if collision_edge(nearest_node, new_node, obstacles):
            continue


        n = len(nodes) + 1
        radius = min(gamma * (math.log(n) / n) ** (1.0 / dim), step_size * 50)
        near_nodes = [nd for nd in nodes if dist(nd, new_node) <= radius]


        best_parent = nearest_node
        best_cost = nearest_node.cost + dist(nearest_node, new_node)
        for near in near_nodes:
            potential_cost = near.cost + dist(near, new_node)
            if potential_cost < best_cost and not collision_edge(near, new_node, obstacles):
                best_parent = near
                best_cost = potential_cost


        new_node.parent = best_parent
        new_node.cost = best_cost
        nodes.append(new_node)


        for near in near_nodes:
   
            alt_cost = new_node.cost + dist(new_node, near)
            if alt_cost < near.cost and not collision_edge(new_node, near, obstacles):
                near.parent = new_node
                near.cost = alt_cost
                propagate_costs(near, nodes)

        if dist(new_node, goal) <= goal_threshold and not collision_edge(new_node, goal, obstacles):
            goal.parent = new_node
            goal.cost = new_node.cost + dist(new_node, goal)
            nodes.append(goal)
            return backtrace(goal), nodes

    return None, nodes


if __name__ == "__main__":

    X_RANGE = (0, 100)
    Y_RANGE = (0, 100)
    OBSTACLES = [
        (40, 40, 10),
        (70, 70, 15),
        (60, 20, 8),
        (20, 80, 7),
    ]

    start = Node(5, 5)
    goal = Node(95, 95)

    path, nodes = rrt_star(start, goal,
                            x_range=X_RANGE,
                            y_range=Y_RANGE,
                            obstacles=OBSTACLES,
                            step_size=2.5,
                            max_iter=5000,
                            goal_sample_rate=0.05,
                            goal_threshold=2.0)

    fig, ax = plt.subplots()
    ax.set_aspect('equal')

    ax.scatter([n.x for n in nodes], [n.y for n in nodes], s=4, c='lightgray')


    for n in nodes:
        if n.parent is not None:
            ax.plot([n.x, n.parent.x], [n.y, n.parent.y], linewidth=0.5, color='lightgray')

    grid_spacing = 5
    for x in range(X_RANGE[0], X_RANGE[1]+1, grid_spacing):
        ax.axvline(x, color='lightgray', linewidth=0.5)
    for y in range(Y_RANGE[0], Y_RANGE[1]+1, grid_spacing):
        ax.axhline(y, color='lightgray', linewidth=0.5)


    for (ox, oy, r) in OBSTACLES:
        ax.add_patch(plt.Circle((ox, oy), r, color='black', fill=True, alpha=0.3))


    if path:
        px, py = zip(*path)
        ax.plot(px, py, linewidth=2, label=f"Path (len={len(path)})")
        title = "RRT* - path found"
    else:
        title = "RRT* - no path found"

    ax.plot(start.x, start.y, 'go', markersize=8, label='Start')
    ax.plot(goal.x, goal.y, 'ro', markersize=8, label='Goal')

    ax.set_xlim(*X_RANGE)
    ax.set_ylim(*Y_RANGE)
    ax.set_title(title)
    ax.legend()
    ax.grid(True, linestyle=':')
    plt.show()
