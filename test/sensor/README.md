Test_cpp
1.底盘测试：testChassis
2.升降测试：testLift	
3.转盘测试：testTurntable
4.夹手舵机测试：testChServo
5.抬手舵机测试：testRhServo
6.伸缩臂舵机测试：testFbServo
7. 读取超声波：testUltrasonic 
8. 读取红外：testIRsensor 
9. 读取升降限位：testLiftLimit 
10. 读取转盘限位：testTurnLimit 
11. 读取急停信号：testEstop 
12. 读取三个按钮信号：testButton 
13. 测试三个按钮led灯光：testLED 

测试现象：
1.红外： LeftIR:34.9033  RightIR:53.535
2.急停：按下->Estoplimit:0; 
3.LED灯光：start灯光亮->rest灯光亮->stop灯光亮
4.转盘限位：按下->Turnlimit:1