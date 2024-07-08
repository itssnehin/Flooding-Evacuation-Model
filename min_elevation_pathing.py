import osmnx as ox
import networkx as nx
import geopandas as gpd
from shapely.geometry import Point, LineString
import numpy as np
import matplotlib.pyplot as plt
from geopy.distance import distance
import time
import os
import heapq
from path_finding import k_shortest_paths
from rtree import index

# Get current Unix timestamp
timestamp = int(time.time())

# Create a directory for this run
output_dir = f"output/routes_{timestamp}"
os.makedirs(output_dir, exist_ok=True)

# OSMnx settings
ox.settings.log_console = False
ox.settings.use_cache = True

def get_nearest_node(G, lat, lon):
    return ox.nearest_nodes(G, lon, lat)

# Load contour data
contour_gdf = gpd.read_file('data/st_lucia_contour_data.gpkg')

# Simplify contour data if it's very detailed
contour_gdf['geometry'] = contour_gdf['geometry'].simplify(tolerance=1)

# Create spatial index for contours
spatial_index = index.Index()
for idx, geometry in enumerate(contour_gdf.geometry):
    spatial_index.insert(idx, geometry.bounds)

# Define the bounding box for Saint Lucia, Brisbane
south, north, west, east = -27.5050, -27.4920, 153.0000, 153.0200
bbox = (south, north, west, east)

# Download the street network for walking with a larger step size
G = ox.graph_from_bbox(bbox=bbox, network_type="drive", simplify=True, retain_all=False, truncate_by_edge=True)

# Convert to undirected graph to allow bi-directional travel on paths
G = ox.convert.to_undirected(G)

# Create GeoDataFrames from the graph
nodes_gdf, edges_gdf = ox.graph_to_gdfs(G)

# Function to assign elevation to geometries
def assign_elevation(geom):
    """
    Assigns the minimum elevation value from contour data to a given geometry.

    Parameters:
    geom (shapely.geometry.base.BaseGeometry): The geometry to assign elevation to.

    Returns:
    float: The minimum elevation value found within the contour data that intersects with the given geometry.
           If no intersection is found, returns NaN.

    Note:
    This function uses a spatial index to efficiently find potential matches in the contour data.
    It then filters these potential matches based on their actual intersection with the given geometry.
    Finally, it returns the minimum elevation value among the precise matches.
    """
    possible_matches_idx = list(spatial_index.intersection(geom.bounds))
    possible_matches = contour_gdf.iloc[possible_matches_idx]
    precise_matches = possible_matches[possible_matches.intersects(geom)]
    
    if not precise_matches.empty:
        return precise_matches['ELEV'].min()
    return np.nan

# Assign elevations to nodes and edges
nodes_gdf['elevation'] = nodes_gdf.geometry.apply(assign_elevation)
edges_gdf['min_elevation'] = edges_gdf.geometry.apply(assign_elevation)

# Fill NaN values with a default elevation (e.g., 0)
nodes_gdf['elevation'] = nodes_gdf['elevation'].fillna(0)
edges_gdf['min_elevation'] = edges_gdf['min_elevation'].fillna(0)

# Update graph with elevations
for node, row in nodes_gdf.iterrows():
    G.nodes[node]['elevation'] = row['elevation']

for (u, v, k), row in edges_gdf.iterrows():
    G[u][v][k]['min_elevation'] = row['min_elevation']

# Define start and end coordinates (latitude, longitude)
start_coord = (-27.4975, 153.0137)  # Approximate location of UQ Lakes bus station
end_coord = (27.4977, 152.9882)    # Approximate location of St Lucia Community Hall

# Get nearest network nodes to points
start_node = get_nearest_node(G, *start_coord)
end_node = get_nearest_node(G, *end_coord)

# Find K shortest paths
K = 6
shortest_paths = k_shortest_paths(G, start_node, end_node, K)

# Plot and analyze paths
for i, (path, length) in enumerate(shortest_paths, 1):
    node_elevations = [G.nodes[node]['elevation'] for node in path]
    edge_elevations = [G[path[j]][path[j+1]][0].get('min_elevation', 0) for j in range(len(path)-1)]
    all_elevations = node_elevations + edge_elevations
    min_elevation = min(all_elevations)
    
    print(f"Path {i}:")
    print(f"  Length: {length:.2f} meters")
    print(f"  Minimum Elevation: {min_elevation:.2f} meters")
    
    # Calculate distances using geopy
    coordinates = [(G.nodes[node]['y'], G.nodes[node]['x']) for node in path]
    distances = [0]
    for j in range(1, len(coordinates)):
        distances.append(distances[-1] + distance(coordinates[j-1], coordinates[j]).meters)
    
    # Plot elevation profile
    plt.figure(figsize=(10, 4))
    plt.plot(distances, node_elevations, 'o-', label='Node Elevations')
    plt.plot(distances[:-1] + np.diff(distances)/2, edge_elevations, 's-', label='Edge Minimum Elevations')
    plt.title(f"Elevation Profile for Path {i}")
    plt.xlabel("Distance (m)")
    plt.ylabel("Elevation (m)")
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f"elevation_profile_path_{i}.png"))
    plt.close()

    # Plot individual route
    fig, ax = ox.plot_graph(G, node_size=0, edge_linewidth=0.5, edge_color='#999999')
    route_edges = list(zip(path[:-1], path[1:]))
    route_lines = [G[u][v][0]['geometry'] for u, v in route_edges]
    for line in route_lines:
        ax.plot(*line.xy, color='r', linewidth=4, alpha=0.8)
    plt.title(f"Route {i}")
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f"route_{i}.png"))
    plt.close()

# Plot all paths on the graph
node_elevations = [G.nodes[node].get('elevation', 0) for node in G.nodes()]
fig, ax = ox.plot_graph(G, node_color=node_elevations, 
                        node_size=5, edge_linewidth=0.5, edge_color='#999999')

for i, (path, _) in enumerate(shortest_paths):
    route_edges = list(zip(path[:-1], path[1:]))
    route_lines = [G[u][v][0]['geometry'] for u, v in route_edges]
    color = plt.cm.tab10(i/10)
    for line in route_lines:
        ax.plot(*line.xy, color=color, linewidth=2, alpha=0.8)

plt.title("All Paths with Elevation-based Node Colors")
plt.tight_layout()
plt.savefig(os.path.join(output_dir, "all_paths.png"))
plt.close()

print(f"All route images have been saved in the '{output_dir}' directory.")