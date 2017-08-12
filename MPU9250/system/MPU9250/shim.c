#include "inv_mpu.h"
#include "shim.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"

#define I2C_SHIM_TIMEOUT_MS 	50

extern I2C_HandleTypeDef hi2c1;

_EXTERN_ATTRIB int i2c_write(unsigned char slave_addr, unsigned char reg_addr,unsigned char length, unsigned char const *data)
{
	int result = ( ( HAL_I2C_Mem_Write(&hi2c1,slave_addr<<1,reg_addr,I2C_MEMADD_SIZE_8BIT,(uint8_t *)data,length,I2C_SHIM_TIMEOUT_MS) == HAL_OK ) ? 0 : -1);
	if (result == -1 ) {
		result--;
	}
	return result;
}

_EXTERN_ATTRIB int i2c_read(unsigned char slave_addr, unsigned char reg_addr,unsigned char length, unsigned char *data)
{
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Read(&hi2c1,slave_addr<<1,reg_addr,I2C_MEMADD_SIZE_8BIT,(uint8_t *)data,length,I2C_SHIM_TIMEOUT_MS);
	if ( status != HAL_OK ) {
		status = HAL_TIMEOUT;
	}
	return ((status == HAL_OK ) ? 0 : -1);
}

_EXTERN_ATTRIB int reg_int_cb(struct int_param_s *int_param)
{
	//TODO: attachInterrupt(int_param->pin, int_param->cb,RISING);
	return 0;
}

_EXTERN_ATTRIB void get_ms(unsigned long *count)
{
	*count =  HAL_GetTick();
}

_EXTERN_ATTRIB void delay_ms(unsigned long ms)
{
	HAL_Delay(ms);
}
