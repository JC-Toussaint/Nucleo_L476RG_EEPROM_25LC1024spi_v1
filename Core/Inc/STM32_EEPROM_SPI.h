/* Copyright 2017-2019 Nikita Bulaev
 *
 */


#ifndef _STM32_EEPROM_SPI_H
#define _STM32_EEPROM_SPI_H

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
//#include "cmsis_os.h"

// 25LC1024 SPI EEPROM instructions
#define EEPROM_WREN      0b00000110  /*!< Write Enable */
#define EEPROM_WRDI      0b00000100  /*!< Write Disable */
#define EEPROM_RDSR      0b00000101  /*!< Read Status Register */
#define EEPROM_WRSR      0b00000001  /*!< Write Status Register */
#define EEPROM_READ      0b00000011  /*!< Read from Memory Array */
#define EEPROM_WRITE     0b00000010  /*!< Write to Memory Array */

#define EEPROM_WIP_FLAG  0b00000001  /*!< Write In Progress (WIP) flag */

#define EEPROM_PAGESIZE        256   /*!< Pagesize according to documentation  */
                                     /*!< 256 for 25LC1024 */
//#define EEPROM_BUFFER_SIZE     32    /*!< EEPROM Buffer size. Setup to your needs */

#define EEPROM_CS_HIGH()    HAL_GPIO_WritePin(EEPROM_CS_GPIO_Port, EEPROM_CS_Pin, GPIO_PIN_SET)
#define EEPROM_CS_LOW()     HAL_GPIO_WritePin(EEPROM_CS_GPIO_Port, EEPROM_CS_Pin, GPIO_PIN_RESET)

/**
 * @brief EEPROM Operations statuses
 */
typedef enum {
    EEPROM_STATUS_COMPLETE,
    EEPROM_STATUS_PENDING,
    EEPROM_STATUS_ERROR
} EEPROM_Status;

void EEPROM_SPI_INIT(SPI_HandleTypeDef * hspi);
EEPROM_Status EEPROM_SPI_WriteBuffer(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
EEPROM_Status EEPROM_WritePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
EEPROM_Status EEPROM_SPI_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);
uint8_t EEPROM_SPI_WaitStandbyState(void);

/* Low layer functions */
uint8_t EEPROM_SendByte(uint8_t byte);
void sEE_WriteEnable(void);
void sEE_WriteDisable(void);
void sEE_WriteStatusRegister(uint8_t regval);
uint8_t sEE_ReadStatusRegister(void);

void  EEPROM_SPI_SendInstruction(uint8_t *instruction, uint8_t size);
void  EEPROM_SPI_ReadStatusByte(SPI_HandleTypeDef SPIe, uint8_t *statusByte );

#ifdef __cplusplus
}
#endif

#endif // _STM32_EEPROM_SPI_H
