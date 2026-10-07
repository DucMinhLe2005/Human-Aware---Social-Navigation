uint8 SOURCE_CAMERA=0
uint8 SOURCE_LIDAR_COAST=1
uint8 SOURCE_LIDAR_ONLY=2

uint8 MODE_FUSION=0
uint8 MODE_CAMERA_ONLY=1
uint8 MODE_LIDAR_ONLY=2
uint8 MODE_PREDICTION=3

uint32 id
float64 x
float64 y
float64 yaw
float64 vx
float64 vy
float64 speed
bool is_moving
uint32 hits
uint8 source
uint8 mode

# Predicted 2D position covariance in the odom frame (the frame tracks are published in).
float64 covariance_xx
float64 covariance_xy
float64 covariance_yy

# Seconds since this track's Kalman state was last updated by a real
# camera, LiDAR-coast, or LiDAR-only measurement (as opposed to the header
# stamp, which is re-stamped every publish tick regardless of whether the
# underlying track actually got a new measurement that tick).
float64 observation_age
