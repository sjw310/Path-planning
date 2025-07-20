import heapq # heap 자료구조

def dijkstra(graph:dict,start):

    dist = {v: float('inf') for v in graph}
    dist[start] = 0
    prev = {v: None for v in graph}

    pq = [(0,start)]

    while pq:
        cur_dist,u = heapq.heappop(pq)

        if cur_dist > dist[u]:
            continue

        for v,w in graph[u]:
            alt = cur_dist+w
            if alt <dist[v]:
                dist[v]=alt
                prev[v]=u
                heapq.heappush(pq, (alt,v))


    return dist, prev

def build_path(prev:dict,target):

    path = []
    while target is not None:
        path.append(target)
        target = prev[target]
    return path[::-1]

if __name__ == "__main__":
    
    graph = {
        'A': [('B',5),('C',1)],
        'B': [('A',5),('C',2),('D',1)],
        'C': [('A',1),('B',2),('D',4),('E',8)],
        'D': [('B',1),('C',4),('E',3),('F',6)],
        'E': [('C',8),('D',3)],
        'F': [('D',6)]
    }

    dist,prev = dijkstra(graph,'A')
    print(build_path(prev, 'F'))