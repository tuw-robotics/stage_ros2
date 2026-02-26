define stereo camera
(
    range       [0.1 20.0 ]             # range in meters that the camera can see
    resolution  [672 376]               # lowest resolution of ZED (max resolution in Stage is 940x939)
    size        [0.00 0.00 0.00]        # size of physical camera
    fov         [90 60]                 # field of view in degrees
    pantilt     [0 0 ]                  # inclination to the right/inclination downwards
    alwayson     1
)

define logi_camera camera
(
    range       [0.01 20.0]
    resolution  [640 320]              # lowered resolution (max resolution possible in Stage is 940x939)
    size        [0.00 0.00 0.00]
    fov         [60 40]
    pantilt     [0 75] # 15 downwards in y-axis
    alwayson    1
)

define lidar ranger # This is the LIDAR
(
    sensor
    (
        range   [0.0 3.0] # meters that can detect
        fov     270.25
        samples  1081
    )
)

define Chimera position
(   
    drive "car"
    color "white"
    stereo( pose [0.220 0.060 0.110 0.000] color "red" pantilt [ 0 0])  # Left camera offset z 0.48 fov 54 110
    logi_camera( pose [0.230 0.000 0.300 0.000 0] color "black" pantilt [ 0 75 ])
    # stereo( pose [0.220 -0.060 0.110 0.000 0] color "pink" ) # Right camera is currently disabled
    lidar( pose [0.050 0.000 0.000 0.000] )
    obstacle_return 0
    localization "gps"           # Change to "gps" to have impossibly perfect, global odometry
    velocity_bounds [-1 1 0 0 0 0 -45.0 45.0]
    acceleration_bounds [-0.5 0.5 0 0 0 0 -45 45.0]
    gui_nose 1
)

define obstacle model
(
  size [0.450 0.280 0.220]
  gui_nose 0
)
