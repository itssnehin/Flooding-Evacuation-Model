import heapq
def k_shortest_paths(G, start, end, K, weight='length'):
    def get_path(pred, start, end):
        path = [end]
        while path[-1] != start:
            path.append(pred[path[-1]])
        path.reverse()
        return path

    queue = [(0, 0, start, {})]  # Add a counter to break ties
    paths = []
    visited = set()
    counter = 1  # Initialize counter

    while queue and len(paths) < K:
        (cost, _, node, pred) = heapq.heappop(queue)

        if node not in visited:
            visited.add(node)

            if node == end:
                path = get_path(pred, start, node)
                if path not in [p for p, _ in paths]:
                    paths.append((path, cost))
                    visited = set()  # Reset visited to allow finding more paths

            for neighbor in G.neighbors(node):
                edge_data = G.get_edge_data(node, neighbor)
                min_cost = float('inf')
                for key, data in edge_data.items():
                    edge_cost = data.get(weight, 0)
                    if not isinstance(edge_cost, (int, float)):
                        edge_cost = 0  # Default to 0 if the weight is not numeric
                    min_cost = min(min_cost, edge_cost)
                
                if neighbor not in pred:
                    new_pred = pred.copy()
                    new_pred[neighbor] = node
                    heapq.heappush(queue, (cost + min_cost, counter, neighbor, new_pred))
                    counter += 1  # Increment counter

    return paths