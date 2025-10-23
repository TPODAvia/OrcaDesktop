std_msgs/Header header     # frame_id: "map"

string map_name
string floor_name

# Pixel->meter transform (same semantics as your params.yaml)
float32 rotation           # radians
float32 scale              # meters per pixel
float32 translation_x      # meters
float32 translation_y      # meters

# Image meta (for rendering/layout)
uint32  image_width        # pixels
uint32  image_height       # pixels
string  image_encoding     # e.g. "mono8"
float32 image_resolution   # meters per pixel (== scale)

rmf_manager_msgs/Vertex2D[] vertices
rmf_manager_msgs/Edge2D[]   lanes

bool    wcs_available
float64 wcs_origin_lat      # degrees
float64 wcs_origin_lon      # degrees
float64 wcs_origin_alt      # meters (MSL). Use NaN if unknown.
float64 wcs_yaw             # radians; 0 means map +X = East, +Y = North
string  wcs_epsg            # optional CRS id (e.g., "EPSG:4326" or "EPSG:4979")