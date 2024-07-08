import geopandas as gpd
import matplotlib.pyplot as plt
import contextily as ctx

# Load the GeoPackage file
gdf = gpd.read_file('st_lucia_contour_data.gpkg')

# Check the CRS (Coordinate Reference System) of your data
print(gdf.crs)

# If the CRS is not EPSG:3857 (Web Mercator), reproject it
if gdf.crs != 'EPSG:3857':
    gdf = gdf.to_crs(epsg=3857)

# Create a plot
fig, ax = plt.subplots(figsize=(15, 10))

# Plot the contours
gdf.plot(ax=ax, column='ELEV', cmap='terrain', legend=True, 
         legend_kwds={'label': 'Elevation (m)', 'orientation': 'horizontal'})

# Add a basemap
ctx.add_basemap(ax, source=ctx.providers.OpenStreetMap.Mapnik)

# Add a title
plt.title('Contour Map of Saint Lucia, Brisbane')

# Remove axis labels
ax.set_axis_off()

# Show the plot
plt.tight_layout()
plt.show()

# Optionally, save the figure
# plt.savefig('contour_map.png', dpi=300, bbox_inches='tight')