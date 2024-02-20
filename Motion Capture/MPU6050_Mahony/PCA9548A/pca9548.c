#include "pca9548.h"

int selectPort(I2C_HandleTypeDef *hi2cx, uint8_t port){
	    uint8_t data;
    switch(port)
    {
        case 0:
            data = PCA9548A_CHANNEL_0;
            break; 
        case 1:    
            data = PCA9548A_CHANNEL_1;
            break; 
        case 2:    
            data = PCA9548A_CHANNEL_2;
            break; 
        case 3:    
            data = PCA9548A_CHANNEL_3;
            break; 
        case 4:    
            data = PCA9548A_CHANNEL_4;
            break; 
        case 5:    
            data = PCA9548A_CHANNEL_5;
            break; 
        case 6:    
            data = PCA9548A_CHANNEL_6;
            break; 
        case 7:    
            data = PCA9548A_CHANNEL_7;
            break;
        default:
            break;        
    }
	if(HAL_I2C_Master_Transmit(hi2cx, (PCA9548A_SLAVE_ADDR << 1) | PCA9548A_WRITE_BIT, &data, 1, 0xFFFF))
		return -1;
	return 0;
}
