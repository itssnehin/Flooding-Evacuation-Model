import heapq

def k_shortest_paths(G, start, end, K):
    """
    This function finds up to k shortest paths between two nodes in a graph using Dijkstra's algorithm.

    Parameters:
    G (NetworkX graph): The graph in which to find the paths.
    start (node): The starting node for the paths.
    end (node): The ending node for the paths.
    K (int): The maximum number of shortest paths to find.

    Returns:
    list: A list of tuples, where each tuple contains a path (list of nodes) and its corresponding cost.
    """

    def get_path(pred, start, end):
        """
        Helper function to reconstruct a path from the predecessor dictionary.

        Parameters:
        pred (dict): A dictionary where keys are nodes and values are their predecessors.
        start (node): The starting node for the path.
        end (node): The ending node for the path.

        Returns:
        list: A list of nodes representing the path from start to end.
        """
        path = [end]
        while path[-1] != start:
            path.append(pred[path[-1]])
        path.reverse()
        return path

    queue = [(0, start, {})]  # Initialize the queue with the starting node and an empty predecessor dictionary
    paths = []  # Initialize an empty list to store the found paths
    visited = set()  # Initialize a set to keep track of visited nodes

    while queue and len(paths) < K:  # Continue until the queue is empty or we have found K paths
        (cost, node, pred) = heapq.heappop(queue)  # Pop the node with the smallest cost from the queue

        if node not in visited:  # If the node has not been visited yet
            visited.add(node)  # Mark the node as visited

            if node == end:  # If we have reached the end node
                path = get_path(pred, start, node)  # Reconstruct the path
                paths.append((path, cost))  # Add the path and its cost to the list of found paths

            for neighbor in G.neighbors(node):  # Iterate over the neighbors of the current node
                if neighbor not in visited:  # If the neighbor has not been visited yet
                    edge_data = G.get_edge_data(node, neighbor)  # Get the edge data between the current node and the neighbor
                    edge_length = edge_data[0].get('length', 1)  # Get the length of the edge, default to 1 if not available
                    new_pred = pred.copy()  # Create a copy of the predecessor dictionary
                    new_pred[neighbor] = node  # Update the predecessor dictionary for the neighbor
                    heapq.heappush(queue, (cost + edge_length, neighbor, new_pred))  # Add the neighbor to the queue

    return paths  # Return all found paths, even if less than K