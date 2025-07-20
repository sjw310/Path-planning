import matplotlib.pyplot as plt
import random, math


class Node:
    def __init__(self,x,y,parent=None):
        self.x, self.y = x,y
        self.parent = parent


def dist(a,b):
    return math.hypot(a.x-b.x,a.y-b.y)

def sample(x_range,y_range):
    # random.uniform(a, b) : a 이상 b 이하의 실수를 무작위로 반환
    return Node(random.uniform(*x_range),random.uniform(*y_range))

def nearest(nodes, rnd):
    return min(nodes, key = lambda n: dist(n,rnd))


def steer(src, tgt, step):

    d = dist(src, tgt)
    if d <= step:
        return Node(tgt.x, tgt.y, src)
    theta = math.atan2(tgt.y - src.y, tgt.x - src.x)
    return Node(src.x + step * math.cos(theta),
                src.y + step * math.sin(theta),
                src)


def collision(node, obstacles):
    for (ox, oy, r) in obstacles:
        if math.hypot(node.x - ox, node.y - oy) <= r:
            return True
    return False

def backtrace(node):
    path = []
    while node:
        path.append((node.x, node.y))
        node = node.parent
    return path[::-1]

# 함수 정의에서의 * 뒤에 오는 인자는 무조건 명시적 키워드 인자여야
def rrt(start,goal,*,
        x_range=(0, 100), y_range=(0, 100),
        obstacles=(),
        step_size=2.0,
        max_iter=5000,
        goal_sample_rate=0.05,
        goal_threshold=2.0):
    
    random.seed(0)
    nodes = [start]

    # random.random(): 0 이상 ~ 1 미만의 난수 반환
    # goal_sample_rate 가 작을수록, 무작위성 증가해 탐색이 느려짐
    for _ in range(max_iter):
        rnd = Node(goal.x, goal.y) if random.random() < goal_sample_rate \
              else sample(x_range, y_range)

        near = nearest(nodes, rnd)
        new  = steer(near, rnd, step_size)

        if collision(new, obstacles):
            continue

        nodes.append(new)

        if dist(new, goal) <= goal_threshold:
            goal.parent = new
            return backtrace(goal), nodes

    return None, nodes


if __name__ == "__main__":

    X_RANGE = (0,100)
    Y_RANGE = (0,100)

    # (x,y,r)
    OBSTACLES = [
        (40,40,10),
        (70,70,15),
        (60,20,8),
        (20,80,7)
    ]

    start = Node(5, 5)
    goal  = Node(95, 95)

    path, nodes = rrt(start, goal,
                      x_range=X_RANGE,
                      y_range=Y_RANGE,
                      obstacles=OBSTACLES,
                      step_size=2.5,
                      max_iter=5000)
    
    fig,ax = plt.subplots()
    ax.set_aspect('equal')

    ax.scatter([n.x for n in nodes],
               [n.y for n in nodes],
               s=4, c='lightgray')

    grid_spacing = 5
    for x in range(X_RANGE[0], X_RANGE[1]+1, grid_spacing):
        ax.axvline(x, color='lightgray', linewidth=0.5)
    for y in range(Y_RANGE[0], Y_RANGE[1]+1, grid_spacing):
        ax.axhline(y, color='lightgray', linewidth=0.5)


    for ox, oy, r in OBSTACLES:
        ax.add_patch(plt.Circle((ox, oy), r, color='black', fill=True, alpha=0.3))

    if path:
        px, py = zip(*path)
        ax.plot(px, py, linewidth=2)
        ax.set_title(f"Path found - length={len(path)}")
    else:
        ax.set_title("No path found")

    
    ax.plot(start.x, start.y, 'go', markersize=8, label='Start')
    ax.plot(goal.x,  goal.y,  'ro', markersize=8, label='Goal')

    # *:unpacking
    ax.set_xlim(*X_RANGE)
    ax.set_ylim(*Y_RANGE)
    ax.set_title("RRT Search Space with Obstacles")
    ax.legend()
    plt.grid(True)
    plt.show()

