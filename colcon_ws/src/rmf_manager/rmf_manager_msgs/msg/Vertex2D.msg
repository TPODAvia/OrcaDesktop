# Stable per-vertex info (both meters + pixels)
string  name
float32 xm      # meters, frame_id="map"
float32 ym
float32 xp      # pixels, origin = image top-left, +y down
float32 yp
uint32  id      # sequential id (index)