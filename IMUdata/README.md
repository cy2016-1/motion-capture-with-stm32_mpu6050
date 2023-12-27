This project uses MPU-6050 and STM32F103C8T6.

## dataGUI_6d0f_imu.py

This code comes from the following repository:

https://github.com/mattzzw/Arduino-mpu6050/tree/master



## MPU6050_uartUI.mlapp

I found this code somewhere. Can't remember where (sorry for not acknowledging the contributor).



NOTICE:

You should modify the keil5 project in the path ‘\motion capture\Demo_MPU\Demo\MDK-ARM’ when you use Matlab to observe data curve.

main.c:

```
			//python:
			printf("%.3f,%.3f,%.3f,\r\n",pitch,roll,yaw);
			
			//Matlab:
//			printf("p%.3fL",pitch);
//			printf("r%.3fL",roll);
//			printf("y%.3fL",yaw);
```

