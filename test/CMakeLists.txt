# study test
add_executable(GrabFruit study/GrabFruit.cpp)
target_link_libraries(GrabFruit robot)

# command test
#add_executable(PickApple command/PickApple.cpp)
#target_link_libraries(PickApple robot commandgenius utilgenius sensorgenius ydlidar_sdk  halgenius rt pthread vmxpi_hal_cpp)
#
#add_executable(testLidarAlongWall command/testLidarAlongWall.cpp)
#target_link_libraries(testLidarAlongWall robot commandgenius utilgenius sensorgenius ydlidar_sdk  halgenius rt pthread vmxpi_hal_cpp)
#
#add_executable(CameraCalibration command/CameraCalibration.cpp)
#target_link_libraries(CameraCalibration robot commandgenius utilgenius sensorgenius ydlidar_sdk  halgenius rt pthread vmxpi_hal_cpp)
#
#add_executable(testPickActCommand command/testPickActCommand.cpp)
#target_link_libraries(testPickActCommand robot commandgenius utilgenius sensorgenius ydlidar_sdk  halgenius rt pthread vmxpi_hal_cpp)
#
#add_executable(MotorPIDCommand command/MotorPIDCommand.cpp )
#target_link_libraries(MotorPIDCommand robot commandgenius utilgenius sensorgenius ydlidar_sdk  halgenius rt pthread vmxpi_hal_cpp)
#
#add_executable(MotorCommand command/MotorCommand.cpp )
#target_link_libraries(MotorCommand robot commandgenius utilgenius sensorgenius ydlidar_sdk  halgenius rt pthread vmxpi_hal_cpp)
#
#add_executable(testLiftMotorDistanceCommand command/testLiftMotorDistanceCommand.cpp)
#target_link_libraries(testLiftMotorDistanceCommand robot commandgenius utilgenius sensorgenius ydlidar_sdk  halgenius rt pthread vmxpi_hal_cpp)
#
#add_executable(testTrackingCommand command/testTrackingCommand.cpp)
#target_link_libraries(testTrackingCommand robot commandgenius utilgenius sensorgenius ydlidar_sdk  halgenius rt pthread vmxpi_hal_cpp)
#
#add_executable(testLidarCalib command/testLidarCalib.cpp)
#target_link_libraries(testLidarCalib robot commandgenius utilgenius sensorgenius ydlidar_sdk  halgenius rt pthread vmxpi_hal_cpp)
#
#add_executable(testOdomAccuracy command/testOdomAccuracy.cpp)
#target_link_libraries(testOdomAccuracy robot commandgenius utilgenius sensorgenius ydlidar_sdk  halgenius rt pthread vmxpi_hal_cpp)
#
#add_executable(testRotateCommand command/testRotateCommand.cpp)
#target_link_libraries(testRotateCommand robot commandgenius utilgenius sensorgenius ydlidar_sdk  halgenius rt pthread vmxpi_hal_cpp)
#
#add_executable(testPickServoCommand command/testPickServoCommand.cpp)
#target_link_libraries(testPickServoCommand robot commandgenius utilgenius sensorgenius ydlidar_sdk  halgenius rt pthread vmxpi_hal_cpp)
#
#add_executable(testTurnDistanceCommand command/testTurnDistanceCommand.cpp)
#target_link_libraries(testTurnDistanceCommand robot commandgenius utilgenius sensorgenius ydlidar_sdk  halgenius rt pthread vmxpi_hal_cpp)
#
#add_executable(testMainPick command/testMainPick.cpp)
#target_link_libraries(testMainPick robot commandgenius utilgenius sensorgenius ydlidar_sdk  halgenius rt pthread vmxpi_hal_cpp)
#
#add_executable(testIRcal command/testIRcal.cpp)
#target_link_libraries(testIRcal robot commandgenius utilgenius sensorgenius ydlidar_sdk  halgenius rt pthread vmxpi_hal_cpp)
#
#add_executable(testPickBasketCommand command/testPickBasketCommand.cpp)
#target_link_libraries(testPickBasketCommand robot commandgenius utilgenius sensorgenius ydlidar_sdk  halgenius rt pthread vmxpi_hal_cpp)
#
#add_executable(testCalCommad command/testCalCommad.cpp)
#target_link_libraries(testCalCommad robot commandgenius utilgenius sensorgenius ydlidar_sdk  halgenius rt pthread vmxpi_hal_cpp)

## sensor test
#add_executable(testButton sensor/testButton.cpp)
#target_link_libraries(testButton robot commandgenius utilgenius sensorgenius ydlidar_sdk  halgenius rt pthread vmxpi_hal_cpp)
#
#add_executable(testChassis sensor/testChassis.cpp)
#target_link_libraries(testChassis robot commandgenius utilgenius sensorgenius ydlidar_sdk  halgenius rt pthread vmxpi_hal_cpp)
#
#add_executable(testChServo sensor/testChServo.cpp)
#target_link_libraries(testChServo robot commandgenius utilgenius sensorgenius ydlidar_sdk  halgenius rt pthread vmxpi_hal_cpp)
#
#add_executable(testEstop sensor/testEstop.cpp)
#target_link_libraries(testEstop robot commandgenius utilgenius sensorgenius ydlidar_sdk  halgenius rt pthread vmxpi_hal_cpp)
#
#add_executable(testFbServo sensor/testFbServo.cpp)
#target_link_libraries(testFbServo robot commandgenius utilgenius sensorgenius ydlidar_sdk  halgenius rt pthread vmxpi_hal_cpp)
#
#add_executable(testIRsensor sensor/testIRsensor.cpp)
#target_link_libraries(testIRsensor robot commandgenius utilgenius sensorgenius ydlidar_sdk  halgenius rt pthread vmxpi_hal_cpp)
#
#add_executable(testLED sensor/testLED.cpp)
#target_link_libraries(testLED robot commandgenius utilgenius sensorgenius ydlidar_sdk  halgenius rt pthread vmxpi_hal_cpp)
#
#add_executable(testLidar sensor/testLidar.cpp)
#target_link_libraries(testLidar robot commandgenius utilgenius sensorgenius ydlidar_sdk  halgenius rt pthread vmxpi_hal_cpp)
#
#add_executable(testLift sensor/testLift.cpp)
#target_link_libraries(testLift robot commandgenius utilgenius sensorgenius ydlidar_sdk  halgenius rt pthread vmxpi_hal_cpp)
#
#add_executable(testLiftLimit sensor/testLiftLimit.cpp)
#target_link_libraries(testLiftLimit robot commandgenius utilgenius sensorgenius ydlidar_sdk  halgenius rt pthread vmxpi_hal_cpp)
#
#add_executable(testRhServo sensor/testRhServo.cpp)
#target_link_libraries(testRhServo robot commandgenius utilgenius sensorgenius ydlidar_sdk  halgenius rt pthread vmxpi_hal_cpp)
#
#add_executable(testTurnLimit sensor/testTurnLimit.cpp)
#target_link_libraries(testTurnLimit robot commandgenius utilgenius sensorgenius ydlidar_sdk  halgenius rt pthread vmxpi_hal_cpp)

## util test
#add_executable(testTurntable sensor/testTurntable.cpp)
#target_link_libraries(testTurntable robot commandgenius utilgenius sensorgenius ydlidar_sdk  halgenius rt pthread vmxpi_hal_cpp)
#
#add_executable(testUltrasonic sensor/testUltrasonic.cpp)
#target_link_libraries(testUltrasonic robot commandgenius utilgenius sensorgenius ydlidar_sdk  halgenius rt pthread vmxpi_hal_cpp)
#
#add_executable(MotorAngENC util/MotorAngENC.cpp)
#target_link_libraries(MotorAngENC robot sensorgenius ydlidar_sdk  halgenius rt pthread vmxpi_hal_cpp)
#
#add_executable(testLimit util/testLimit.cpp)
#target_link_libraries(testLimit robot commandgenius utilgenius sensorgenius ydlidar_sdk  halgenius rt pthread vmxpi_hal_cpp)
#
#add_executable(testServo util/testServo.cpp)
#target_link_libraries(testServo robot commandgenius utilgenius sensorgenius ydlidar_sdk  halgenius rt pthread vmxpi_hal_cpp)
#
#add_executable(testGyro util/testGyro.cpp)
#target_link_libraries(testGyro robot commandgenius utilgenius sensorgenius ydlidar_sdk  halgenius rt pthread vmxpi_hal_cpp)
#
#add_executable(testOdom util/testOdom.cpp)
#target_link_libraries(testOdom robot commandgenius utilgenius sensorgenius ydlidar_sdk  halgenius rt pthread vmxpi_hal_cpp)
#
#add_executable(testLidarSensor util/testLidarSensor.cpp)
#target_link_libraries(testLidarSensor robot commandgenius utilgenius sensorgenius ydlidar_sdk  halgenius rt pthread vmxpi_hal_cpp)
#
#add_executable(testShare util/testShare.cpp)
#target_link_libraries(testShare robot commandgenius utilgenius sensorgenius ydlidar_sdk  halgenius rt pthread vmxpi_hal_cpp)
#
#add_executable(testLabview util/testLabview.cpp)
#target_link_libraries(testLabview robot)
#
#add_executable(rgb2u8 util/rgb2u8.cpp)
#target_link_libraries(rgb2u8 robot commandgenius utilgenius sensorgenius ydlidar_sdk  halgenius rt pthread vmxpi_hal_cpp)
#
#add_executable(testCommand util/testCommand.cpp)
#target_link_libraries(testCommand robot commandgenius utilgenius sensorgenius ydlidar_sdk  halgenius rt pthread vmxpi_hal_cpp)
#
#add_executable(testPickServo util/testPickServo.cpp)
#target_link_libraries(testPickServo robot commandgenius utilgenius sensorgenius ydlidar_sdk  halgenius rt pthread vmxpi_hal_cpp)
#
#add_executable(testVision util/testVision.cpp)
#target_link_libraries(testVision robot commandgenius utilgenius sensorgenius ydlidar_sdk  halgenius rt pthread vmxpi_hal_cpp)