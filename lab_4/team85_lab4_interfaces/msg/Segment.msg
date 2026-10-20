# One segment of a LiDAR scan (see team85_lab4.scan_segmentation.Segment).

# Indices of the points in the source LaserScan's ranges, ordered
# counterclockwise (may wrap from the end to the start of the scan).
uint32[] indices

# Points in the scan frame (x: forward, y: left), z = 0.
geometry_msgs/Point32[] points
