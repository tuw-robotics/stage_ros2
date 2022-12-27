include "include/pioneer2dx.inc"

define hokuyolaser ranger
(
  sensor(       
    # laser-specific properties
    # factory settings for LMS200  
    range [ 0.0  5.0 ]
    fov 270.0
    samples 270
  )
  model
  (
    # generic model properties
    size [ 0.07 0.07 0.05 ] # dimensions from LMS200 data sheet  
    color "blue"
  )
)

define mycamera camera
(
	range [ 0.2 8.0 ]
	resolution [ 100 100 ]
	fov [ 70 40 ]
	pantilt [ 0 0 ]
	alwayson 1
)

# throw in a robot
define tuw_pioneer2dx pioneer2dx_no_sonar 
(
  # can refer to the robot by this name
  hokuyolaser(pose [ 0.15 0 -0.2 0 ])
  mycamera(pose [ 0 0 0.5 0 ])
  mount( pose [ 0 -0.15 0 0.000 ] color "gray")
  mount( pose [ 0  0.15 0 0.000 ] color "gray")
)
