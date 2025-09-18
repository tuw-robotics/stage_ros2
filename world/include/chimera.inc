define stereo camera
(
    range       [0.1 20.0 ]            # range in meters that the camera can see
    resolution  [900 560]              # max resolution allowed in Stage is 969x560
    size        [0.00 0.00 0.00]        # size of physical camera
    fov         [90 60]                 # field of view in degrees
    pantilt     [0 0 ]                  #inclination to the right/inclination downwards                                                     5.5 downwards in y-axis
    alwayson     1
)

define logi_camera camera
(
    range       [0.01 20.0 ]
    resolution  [1920 1080 ] # 720
    size        [0.00 0.00 0.00]
    fov         [60 40 ]
    pantilt     [0 75 ] # 15 downwards in y-axis
    alwayson    1
)

define Chimera position
(   drive "car"
    color "white"
    stereo( pose [0.220 0.060 0.110 0.000] color "red" pantilt [ 0 0])  # Left camera offset z 0.48 fov 54 110
    logi_camera( pose [0.230 0.000 0.300 0.000 0] color "black" pantilt [ 0 75 ])
    obstacle_return 0
    localization "gps"           # Change to "gps" to have impossibly perfect, global odometry
    velocity_bounds [-1 1 0 0 0 0 -45.0 45.0 ]          
    acceleration_bounds [-0.5 0.5 0 0 0 0 -45 45.0 ]
    gui_nose 1
    
)


define obstacle model
(
  size [ 0.420 0.300 0.300 ]
  gui_nose 0
)
