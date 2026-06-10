IBRARY_PATH /home/p/catkin_ws/devel/lib:/opt/ros/noetic/lib:/home/p/PX4-Autopilot/build/px4_sitl_default/build_gazebo
p@p-virtual-machine:~$ sai-cli calibrate /home/p/recording_20250731_114329.avi --tag_size 0.03827
Enabling visual-only calibration mode.

--- Camera calibration phase
Corner detection progress 94%
Failed detections 10/639.
Initialized camera 0 with pinhole model. You should set pinhole model for this camera!
Removed 10/639 invalid poses.

Estimating initial extrinsics.

Initialized optimization.
Iteration 20/70. Mean reprojection error: 0.23125318
Skipping IMU-camera extrinsics calibration.

Calibration result:
{
    "cameras": [
        {
            "distortionCoefficients": [
                27.512044551822996,
                -58.01859148281487,
                -0.0004558759195965709,
                -0.0015786230537342438,
                23.14988772306903,
                27.25062828920284,
                -57.926071562394746,
                23.80020160847425
            ],
            "focalLengthX": 402.5927201658822,
            "focalLengthY": 402.4039776127119,
            "imageHeight": 480,
            "imageWidth": 640,
            "imuToCamera": [
                [
                    1.0,
                    0.0,
                    0.0,
                    0.0
                ],
                [
                    0.0,
                    1.0,
                    0.0,
                    0.0
                ],
                [
                    0.0,
                    0.0,
                    1.0,
                    0.0
                ],
                [
                    0.0,
                    0.0,
                    0.0,
                    1.0
                ]
            ],
            "model": "brown-conrady",
            "principalPointX": 328.4746212860611,
            "principalPointY": 252.28843849710458
        }
    ]
}
p@p-virtual-machine:~$ 











@p-virtual-machine:~$ ^C
p@p-virtual-machine:~$ sai-cli calibrate /home/p/recording_20250731_113516.avi --tag_size 0.03827
Enabling visual-only calibration mode.

--- Camera calibration phase
Corner detection progress 95%
Failed detections 3/529.
Initialized camera 0 with pinhole model. You should set pinhole model for this camera!
Removed 3/529 invalid poses.

Estimating initial extrinsics.

Initialized optimization.
Iteration 31/70. Mean reprojection error: 0.23642761
Skipping IMU-camera extrinsics calibration.

Calibration result:
{
    "cameras": [
        {
            "distortionCoefficients": [
                3.997967814483801,
                -17.037278467546823,
                -0.00021276731881876072,
                -7.448898382474515e-05,
                12.81488667175374,
                3.892168271005746,
                -16.724395694181954,
                12.58619384023578
            ],
            "focalLengthX": 405.95982563914316,
            "focalLengthY": 405.3998059000121,
            "imageHeight": 480,
            "imageWidth": 640,
            "imuToCamera": [
                [
                    1.0,
                    0.0,
                    0.0,
                    0.0
                ],
                [
                    0.0,
                    1.0,
                    0.0,
                    0.0
                ],
                [
                    0.0,
                    0.0,
                    1.0,
                    0.0
                ],
                [
                    0.0,
                    0.0,
                    0.0,
                    1.0
                ]
            ],
            "model": "brown-conrady",
            "principalPointX": 330.2441680637749,
            "principalPointY": 255.56452448868114
        }
    ]
}
p@p-virtual-machine:~$ 











