Header header
# robot
string status
float64 easting
float64 northing
float64 height
float64 orientation
string implement_out

# ab line
string b_id
float64 b_easting
float64 b_northing
float64 a_easting
float64 a_northing
float64 distance_to_b
float64 bearing_to_b
float64 heading_err
float64 distance_to_ab_line

# intermediate target
float64 target_easting
float64 target_northing
float64 target_distance
float64 target_bearing
float64 target_heading_err

# state: IDLE, NAV, TURN, ARR, NEW, WAIT

