Header header
# header.frame_id contains id of the car sending the car state msg
float32[] u
float32[] state
# car_id contains id of the car whose state this is
int8 car_id
# the 2x2 information matrix (inverse of the covariance matrix)
float32[] inf