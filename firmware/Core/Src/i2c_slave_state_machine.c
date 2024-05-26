#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

#include "cmsis_os.h"
#include "main.h"
#include "i2c_slave_state_machine.h"

// emulated I2C EEPROM
extern const uint8_t eeprom[256];// = "THEX2024\x40\x00\x40\x00\x00\x00\x00\x00\xfe\xca\x34\x12\x00\x00OMNIWHL\x00\x00\x00";
// emulated registers
uint8_t regs[256];
bool regs_locked = false;

static uint8_t *dest;
static uint8_t offset; 	// index of current RAM cell
static uint8_t first=1;	// first byte --> new offset

extern osThreadId defaultTaskHandle;

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
	first = 1;
	HAL_I2C_EnableListen_IT(hi2c); // slave is ready again
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
	HAL_GPIO_WritePin( LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET );
    if ((AddrMatchCode >> 1) == 0x50) {
        dest = eeprom;
    } else {
        dest = regs;
        regs_locked = true;
    }
	if( TransferDirection==I2C_DIRECTION_TRANSMIT ) {
		if( first ) {
            HAL_I2C_Slave_Seq_Receive_IT(hi2c, &offset, 1, I2C_NEXT_FRAME);
		} else {
            if (dest == regs) {
    			HAL_I2C_Slave_Seq_Receive_IT(hi2c, &dest[offset], 1, I2C_NEXT_FRAME);
            } else {
                __HAL_I2C_GENERATE_NACK(hi2c);  // EEPROM is read only
            }
		}
	} else {
		HAL_I2C_Slave_Seq_Transmit_IT(hi2c, &dest[offset], 1, I2C_NEXT_FRAME);
	}
	HAL_GPIO_WritePin( LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET );
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	HAL_GPIO_WritePin( LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET );
	if(first) {
		first = 0;
	} else {
		offset++;
	}
    if (dest == regs) {
    	HAL_I2C_Slave_Seq_Receive_IT(hi2c, &dest[offset], 1, I2C_NEXT_FRAME);
    } else {
        __HAL_I2C_GENERATE_NACK(hi2c);  // EEPROM is read only
    }
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET );
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	HAL_GPIO_WritePin( LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET );
	offset++;
	HAL_I2C_Slave_Seq_Transmit_IT(hi2c, &dest[offset], 1, I2C_NEXT_FRAME);
	HAL_GPIO_WritePin( LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET );
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	HAL_GPIO_WritePin( LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET );
	if( HAL_I2C_GetError(hi2c)==HAL_I2C_ERROR_AF ) {
		// transaction terminated by master
		offset--;
        regs_locked = false;
        xTaskNotify(defaultTaskHandle, 0, 0);
	}
	HAL_GPIO_WritePin( LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET );
}
