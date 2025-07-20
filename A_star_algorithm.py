import heapq

directions = [(-1,0),(1,0),(0,-1),(0,1)]


def heuristic(a,b):
    # Manhattan - 2차원 평면 공간에서 두 점사이의 수평 및 수직 이동거리의 합으로 정의
    return abs(a[0]-b[0])+abs(a[1]-b[1])


def a_star(grid, start, goal):
    rows, cols = len(grid), len(grid[0])
    open_set = []
    heapq.heappush(open_set, (0 + heuristic(start, goal), 0, start))
    
    came_from = {}
    g_score = {start: 0}
    visited = set()

    while open_set:
        f, g, current = heapq.heappop(open_set)

        if current == goal:

            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]

        if current in visited:
            continue
        visited.add(current)

        for dx, dy in directions:
            nx, ny = current[0] + dx, current[1] + dy
            neighbor = (nx, ny)

            if 0 <= nx < rows and 0 <= ny < cols and grid[nx][ny] == 0:
                tentative_g = g_score[current] + 1
                # .get(neighbor, float("inf")) : key가 딕셔너리에 존재하면 값을 반환하고, 없으면 default를 반환
                if tentative_g < g_score.get(neighbor, float("inf")):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, tentative_g, neighbor))

    return None

if __name__=="__main__":

    grid = [
        [0, 1, 0, 0, 0],
        [0, 1, 0, 1, 0],
        [0, 0, 0, 1, 0],
        [1, 1, 0, 0, 0],
        [0, 0, 0, 1, 0],
    ]

    start = (0, 0)
    goal = (4, 4)

    path = a_star(grid,start,goal)



    if path:
        print(path)
    else:
        print('None')
