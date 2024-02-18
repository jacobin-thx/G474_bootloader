#include "bootloader.h"
#include "gpio.h"
#include "main.h"
#include "usart.h"

#define ACK 0x3B
#define NACK 0x71

#define CMDTX_START 0xAB
#define CMDTX_BADOPT 0xC6

#define CMDRX_GETID 0xA8
#define CMDRX_FLASH_ERRASE 0x27
#define CMDRX_FLASH_WRITE 0x9C
#define CMDRX_START_APP 0x5F

/**
 * A8
 * 2701000000
 * 9C
 */

#define IHEX_DATA 0x00
#define IHEX_EOF 0x01
#define IHEX_EXADD 0x04
#define IHEX_STARTADD 0x05

#define ID 0x6904 // 0x469

#define UART_BUFF_SIZE 20
#define UART_TIMEOUT 30

#define FLASH_ERRASE_PAGE_START 8

inline void vBlinkLed(void);

void vTransmitAck(void);
void vTransmitNack(void);

inline void vTransmitId(void);
inline void vTransmitStart(void);
inline uint8_t ucCheckOPB(void);
inline void vTransmitBadOPB(void);
inline void vFlashErrase(void);
inline uint8_t vFleshWrite(uint32_t *pulAddress, uint8_t *ucCountBytes, uint64_t ucData[]);

inline void vHexReceive(void);
uint8_t ucCheckCRC(uint8_t *pucRxData);

uint32_t ulFirstNotClearedAdd = 0;

void vStartBootloader(void)
{
    HAL_StatusTypeDef err;
    while (1)
    {
        uint8_t ucCommad;
        // vBlinkLed();
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        err = HAL_UART_Receive(&huart1, &ucCommad, 1, 500);
        if (err == HAL_OK)
        {
            switch (ucCommad)
            {
            case CMDRX_GETID:
                vTransmitAck();
                vTransmitId();
                break;
            case CMDRX_FLASH_ERRASE:
                if (ucCheckOPB())
                {
                    vTransmitBadOPB();
                }
                else
                {
                    vFlashErrase();
                }
                break;
            case CMDRX_FLASH_WRITE:
                vTransmitAck();
                vHexReceive();
                break;
            default:
                vTransmitNack();
                break;
            }
        }
        else
        {
            // vTransmitStart();
        }
    }
}

void vJumpToApp(void)
{

    uint32_t JumpAddress = *((volatile uint32_t *)(APP_START_ADDRESS + 4));
    void (*reset_handler)(void) = (void *)JumpAddress;

    // GPIO deinit
    HAL_GPIO_DeInit(B1_GPIO_Port, B1_Pin);
    HAL_GPIO_DeInit(LD2_GPIO_Port, LD2_Pin);

    // SysTick deinit
    SysTick->CTRL = 0;
    SysTick->VAL = 0;
    SysTick->LOAD = 0;

    HAL_DeInit();

    __disable_irq();

    __set_MSP(*((volatile uint32_t *)APP_START_ADDRESS));

    __DMB();
    SCB->VTOR = APP_START_ADDRESS;
    __DSB();

    reset_handler();
    while (1)
    {
    }
}

void vBlinkLed(void)
{
    static uint32_t ulTick = 0;
    if (HAL_GetTick() - ulTick > 500)
    {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        ulTick = HAL_GetTick();
    }
}

void vTransmitAck(void)
{
    uint8_t ucAck = ACK;
    HAL_UART_Transmit(&huart1, &ucAck, 1, HAL_MAX_DELAY);
}
void vTransmitNack(void)
{
    uint8_t ucNack = NACK;
    HAL_UART_Transmit(&huart1, &ucNack, 1, HAL_MAX_DELAY);
}

void vTransmitStart(void)
{
    uint8_t ucCMDStart = CMDTX_START;
    HAL_UART_Transmit(&huart1, &ucCMDStart, 1, HAL_MAX_DELAY);
}

void vTransmitId(void)
{
    uint16_t usDevID = (DBGMCU->IDCODE & 0xFFF);
    HAL_UART_Transmit(&huart1, (uint8_t *)&usDevID, 2, HAL_MAX_DELAY);
}
/**
 * @brief check OPB use dual bank
 *
 */
uint8_t ucCheckOPB()
{
    if ((FLASH->OPTR & FLASH_OPTR_DBANK) == OB_DBANK_64_BITS)
    {
        return 0;
    }
    return 1;
}

void vTransmitBadOPB(void)
{
    uint8_t ucCMDStart = CMDTX_BADOPT;
    HAL_UART_Transmit(&huart1, &ucCMDStart, 1, HAL_MAX_DELAY);
}

void vFlashErrase(void)
{
    uint32_t ucNbPages = 0;
    uint32_t ulPageError;
    FLASH_EraseInitTypeDef eraseInit;
    if (HAL_UART_Receive(&huart1, (uint8_t *)&ucNbPages, 4, UART_TIMEOUT) == HAL_OK)
    {
        eraseInit.Page = FLASH_ERRASE_PAGE_START; // for page 8 addres = 8*2048 0x08004000
        eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
        eraseInit.Banks = FLASH_BANK_1;
        if ((ucNbPages > (FLASH_PAGE_NB * 2 - FLASH_ERRASE_PAGE_START)) || (ucNbPages == 0))
        {
            vTransmitNack();
            return;
        }
        else if (ucNbPages <= (FLASH_PAGE_NB - FLASH_ERRASE_PAGE_START))
        {
            eraseInit.NbPages = ucNbPages;
            HAL_FLASH_Unlock();
            HAL_FLASHEx_Erase(&eraseInit, &ulPageError);
            HAL_FLASH_Lock();
        }
        else
        {
            eraseInit.NbPages = FLASH_PAGE_NB - FLASH_ERRASE_PAGE_START;
            HAL_FLASH_Unlock();
            HAL_FLASHEx_Erase(&eraseInit, &ulPageError);
            eraseInit.Banks = FLASH_BANK_2;
            eraseInit.Page = 0;
            eraseInit.NbPages = ucNbPages - (FLASH_PAGE_NB - FLASH_ERRASE_PAGE_START);
            HAL_FLASHEx_Erase(&eraseInit, &ulPageError);
            HAL_FLASH_Lock();
        }
        ulFirstNotClearedAdd = (FLASH_BASE + ((ucNbPages + FLASH_ERRASE_PAGE_START) * FLASH_PAGE_SIZE));
        // Ex. For ucNbPages = 5 -> (5+10)*2048 ulFirstNotClearedAdd 7800
        vTransmitAck();
    }
    else
    {
        vTransmitNack();
    }
}

void vHexReceive(void)
{

    uint8_t pucRxData[30];
    uint32_t ulExAddress = 0;
    uint32_t ulAddress = 0;
    uint8_t ucDataLenght = 0;
    uint8_t ucData[32];
    uint8_t ucCountBytes = 0;
    HAL_StatusTypeDef err;
    uint8_t ucBytesToDWord = 0;
    pucRxData[3] = 0;
    while (pucRxData[3] != IHEX_EOF)
    {
        err = HAL_UART_Receive(&huart1, pucRxData, 1, UART_TIMEOUT);
        if (err == HAL_OK)
        {
            err = HAL_UART_Receive(&huart1, pucRxData + 1, pucRxData[0] + 4, UART_TIMEOUT);
            if (err == HAL_OK)
            {
                if (ucCheckCRC(pucRxData))
                {
                    vTransmitNack();
                    continue;
                }
                else
                {
                    switch (pucRxData[3])
                    {
                    // Receive Extended Linear Address
                    case IHEX_EXADD:
                        ulExAddress = ((pucRxData[4] << 8) | pucRxData[5]) << 16;
                        break;
                    // Receiv Data
                    case IHEX_DATA:
                        if (ucCountBytes == 0)
                        {
                            ulAddress = ulExAddress | (((pucRxData[1] << 8) | pucRxData[2]));
                        }
                        ucDataLenght = pucRxData[0];
                        for (int i = 4; i < ucDataLenght + 4; i++)
                        {
                            ucData[ucCountBytes] = pucRxData[i];
                            ucCountBytes++;
                        }
                        // if receive double-word
                        if (ucCountBytes >= 8)
                        {
                            vFleshWrite(&ulAddress, &ucCountBytes, (uint64_t *)ucData);
                        }
                        __NOP();
                        break;
                    // Receive Start Linear Address
                    case IHEX_STARTADD:
                        __NOP();
                        break;
                    // Receive End Of File
                    case IHEX_EOF:
                        if (ucCountBytes > 0)
                        {
                            ucBytesToDWord = ucCountBytes % 8;
                            for (uint8_t i = 0; i < ucBytesToDWord; i++)
                            {
                                ucData[ucCountBytes + i] = 0xFF;
                            }
                            ucCountBytes += ucBytesToDWord;
                            vFleshWrite(&ulAddress, &ucCountBytes, (uint64_t *)ucData);
                        }
                        break;
                    default:
                        break;
                    }
                    vTransmitAck();
                }
            }
            else
            {
                vTransmitNack();
            }
        }
    }

    __NOP(); // TODO:
}

uint8_t ucCheckCRC(uint8_t *pucRxData)
{
    uint8_t ucSum = 0;
    size_t xSize = pucRxData[0] + 5;
    if (xSize > 30)
    {
        return 1;
    }
    for (size_t i = 0; i < xSize; i++)
    {
        ucSum += pucRxData[i];
    }
    return ucSum;
}

uint8_t vFleshWrite(uint32_t *pulAddress, uint8_t *ucCountBytes, uint64_t ucData[])
{
    uint8_t ucWriteBytes;
    ucWriteBytes = *ucCountBytes;
    HAL_FLASH_Unlock();
    for (uint8_t i = 0; *ucCountBytes >= 8; *ucCountBytes -= 8)
    {
        // Checking that the flash is cleared
        if (*pulAddress + 8 < ulFirstNotClearedAdd)
        {
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, *pulAddress, ucData[i]);
            *pulAddress += 0x08;
            i++;
        }
        else
        {
            break;
        }
    }
    __NOP();
    HAL_FLASH_Lock();
    ucWriteBytes -= *ucCountBytes;
    // TODO: Flash varify;
    // Shift unwrited bytes
    for (int i = 0; i < *ucCountBytes; i++)
    {
        ((uint8_t *)ucData)[i] = ((uint8_t *)ucData)[ucWriteBytes + i];
    }

    return ucWriteBytes;
}
