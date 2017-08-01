Header header
# header.frame_id contains id of the car sending the car state msg
float32[] u
float32[] state
# car_id contains id of the car whose state this is
# the 9x9 information matrix (inverse of the covariance matrix)
float32[] inf