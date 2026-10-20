# Segmentation of one LaserScan. Scan points not in any segment are noise.

# Same stamp and frame as the source LaserScan.
std_msgs/Header header

Segment[] segments
