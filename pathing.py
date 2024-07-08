import osmnx as ox
import networkx as nx
from typing import List, Tuple
import heapq
import matplotlib.pyplot as plt

# Update configuration using the settings module
ox.settings.log_console = False
ox.settings.use_cache = True

def get_nearest_node(G, lat, lon):
    return ox.nearest_nodes(G, lon, lat)

def k_shortest_paths(G, start, end, K):
    def get_path(pred, start, end):
        path = [end]
        while path[-1] != start:
            path.append(pred[path[-1]])
        path.reverse()
        return path

    queue = [(0, start, {})]
    paths = []
    visited = set()

    while queue and len(paths) < K:
        (cost, node, pred) = heapq.heappop(queue)
        
        if node not in visited:
            visited.add(node)
            
            if node == end:
                path = get_path(pred, start, node)
                paths.append((path, cost))
            
            for neighbor in G.neighbors(node):
                if neighbor not in visited:
                    edge_data = G.get_edge_data(node, neighbor)
                    edge_length = edge_data[0].get('length', 1)  # Default to 1 if length not available
                    new_pred = pred.copy()
                    new_pred[neighbor] = node
                    heapq.heappush(queue, (cost + edge_length, neighbor, new_pred))

    return paths

# Define the bounding box for Saint Lucia, Brisbane
north, south, east, west = -27.4920, -27.5050, 153.0200, 153.0000

# Download the street network
G = ox.graph_from_bbox(north, south, east, west, network_type="drive")

# Convert to undirected graph to allow bi-directional travel on streets
G = ox.utils_graph.get_undirected(G)

# Define start and end coordinates (latitude, longitude)
start_coord = (-27.4975, 153.0137)  # Approximate location of UQ Lakes bus station
end_coord = (-27.4994, 153.0144)    # Approximate location of UQ Centre

# Get nearest network nodes to points
start_node = get_nearest_node(G, *start_coord)
end_node = get_nearest_node(G, *end_coord)

# Find K shortest paths
K = 10
shortest_paths = k_shortest_paths(G, start_node, end_node, K)

# Print and plot results
fig, ax = ox.plot_graph(G, show=False, close=False)

colors = ['r', 'g', 'b', 'c', 'm', 'y', 'k', 'w', 'orange', 'purple']
for i, (path, length) in enumerate(shortest_paths, 1):
    print(f"Path {i}: {length:.2f} meters")
    # Use plot_graph_route instead of plot_route
    ox.plot_graph_route(G, path, route_color=colors[i % len(colors)], ax=ax)

plt.tight_layout()
plt.show()