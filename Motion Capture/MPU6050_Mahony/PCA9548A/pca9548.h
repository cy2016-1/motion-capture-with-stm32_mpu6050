#ifndef __PAC9548_H__
#define __PAC9548_H__
#include "i2c.h"

#define PCA9548A_SLAVE_ADDR         0x70
		
#define PCA9548A_WRITE_BIT          0x00
#define PCA9548A_READ_BIT           0x01
		
#define PCA9548A_CHANNEL_0          0x01
#define PCA9548A_CHANNEL_1          0x02
#define PCA9548A_CHANNEL_2          0x04
#define PCA9548A_CHANNEL_3          0x08
#define PCA9548A_CHANNEL_4          0x10
#define PCA9548A_CHANNEL_5          0x20
#define PCA9548A_CHANNEL_6          0x40
#define PCA9548A_CHANNEL_7          0x80

int selectPort(I2C_HandleTypeDef *hi2cx, uint8_t port);

#endif 
