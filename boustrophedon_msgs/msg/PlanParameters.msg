bool repeat_boundary # default = true
bool outline_clockwise # default = true
bool skip_outlines # default = false
uint8 outline_layer_count # default = 0
float64 cut_spacing # default = 1.2
float64 intermediary_separation # default = 0.0
float64 cut_angle_radians # default = 0.0
bool enable_stripe_angle_orientation # default = false
bool travel_along_boundary # default = true
bool allow_points_outside_boundary # default = false
bool stripes_before_outlines # default = false
uint8 turn_type # default = TURN_FULL_U
uint8 points_per_turn # default = 10
float64 turn_start_offset # default = 0.0

uint8 TURN_BOUNDARY = 0
uint8 TURN_FULL_U = 1
uint8 TURN_HALF_Y = 2
