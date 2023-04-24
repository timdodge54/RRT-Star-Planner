# This message communicates information required to create the mavsim MsgWorldMap
string uuid                    # Unique identifier
float64 city_width          # the city is of size (width)x(width)
int32 num_city_blocks       # number of blocks in city
float64 street_width_val    # actual width of the street
float64 building_max_height # maximum height of buildings
float64[] building_height   # height of each building (stored with outer loop on first index and inner loop on second)
