import networkx as nx
from collections import deque

# import matplotlib.pyplot as plt
# from matplotlib.pyplot import figure

tec = nx.Graph()
tec.add_nodes_from(['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J',
                    'K', 'L', 'M', 'N', 'Ñ', 'O', 'P', 'Q', 'R', 'S', 'T', 'U',
                    'V', 'W', 'X', 'Y', 'Z', 'E1', 'E2', 'E3'])
tec.add_weighted_edges_from([('A', 'B', 1), ('A', 'E1', 1), ('B', 'E1', 2), ('B', 'D', 1), ('B', 'C', 1),
                             ('E1', 'C', 1), ('E1', 'E2', 2), ('D', 'C', 1), ('D', 'E', 1), ('C', 'F', 1),
                             ('C', 'E2', 1), ('E', 'F', 1), ('F', 'E2', 1), ('E', 'G', 1), ('F', 'G', 1),
                             ('F', 'I', 1), ('E2', 'I', 1), ('E2', 'J', 1), ('E2', 'P', 1), ('G', 'H', 1),
                             ('G', 'I', 2), ('H', 'K', 1), ('H', 'L', 2), ('I', 'L', 3), ('I', 'J', 1), ('K', 'M', 1),
                             ('K', 'L', 2), ('L', 'M', 1), ('L', 'J', 2), ('J', 'P', 2), ('M', 'N', 1), ('M', 'O', 2),
                             ('N', 'U', 4), ('N', 'O', 1), ('O', 'Q', 1), ('O', 'Ñ', 1), ('P', 'Ñ', 1), ('P', 'S', 3),
                             ('P', 'E3', 3), ('Q', 'Ñ', 1), ('Q', 'R', 1), ('Ñ', 'R', 2), ('Ñ', 'S', 4), ('U', 'R', 1),
                             ('U', 'T', 2), ('U', 'W', 2), ('R', 'T', 1), ('R', 'S', 1), ('S', 'T', 1), ('S', 'V', 2),
                             ('S', 'E3', 1), ('T', 'W', 2), ('T', 'V', 2), ('V', 'W', 1), ('V', 'X', 2), ('V', 'E3', 1),
                             ('E3', 'W', 2), ('E3', 'X', 1), ('W', 'Y', 1), ('W', 'X', 1), ('Y', 'X', 2), ('Y', 'Z', 2),
                             ('X', 'Z', 4)])


def bfs_search(graph, start_node, target_node):
    visited = set()
    queue = deque([(start_node, [start_node])])

    while queue:
        node, path = queue.popleft()
        if node not in visited:
            visited.add(node)
            if node == target_node:
                return path
            for neighbor in graph.neighbors(node):
                if neighbor not in visited:
                    queue.append((neighbor, path + [neighbor]))

    return None


def dfs_search(graph, start_node, target_node):
    visited = []
    stack = [start_node]

    while stack:
        node = stack.pop()
        if node not in visited:
            visited.append(node)
            if node == target_node:
                return visited
            neighbors = list(graph.neighbors(node))
            stack.extend(neighbors[::-1])

    return []


def greedy_search(G, start, goal):
    visited = [start]
    while visited[-1] != goal:
        neighbors = list(G.neighbors(visited[-1]))
        h = {n: G[visited[-1]][n]['weight'] for n in neighbors if n not in visited}
        if not h:
            break
        next_node = min(h, key=h.get)
        visited.append(next_node)
    return visited

# Impresiones

# figure(figsize=(20,10), dpi=500)
# labels = nx.get_edge_attributes(tec,'weight')
# layout = nx.spring_layout(tec)
# nx.draw(tec, layout, with_labels=True,node_size=200)
# nx.draw_networkx_edge_labels(tec,layout,edge_labels=labels)
# plt.savefig("tec_networkx.png")

if __name__ == '__main__':
    print("Arruina tu vida por Hitler")
    print("Anchura")
    camino = bfs_search(tec, 'A', 'Z')
    print(camino)
    print("Profundidad")
    camino = dfs_search(tec, 'A', 'Z')
    print(camino)
    print("Greedy")
    camino = greedy_search(tec, 'A', 'Z')
    print(camino)
