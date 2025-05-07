#include "max6675.h"

extern SPI_HandleTypeDef hspi1;  // Khai b�o d�ng SPI1 (ho?c s?a l?i n?u m�y d�ng SPI kh�c)

// ------------------- H�m d?c nhi?t d? -------------------
float Max6675_Read_Temp(void) {
		uint8_t buf[2];
    uint16_t rawData = 0;
    float temperature = 0;

   HAL_GPIO_WritePin(SSPORT, SSPIN, GPIO_PIN_RESET);
HAL_SPI_Receive(&hspi1, buf, 2, 100);
HAL_GPIO_WritePin(SSPORT, SSPIN, GPIO_PIN_SET);

	rawData = (buf[0] << 8) | buf[1];  // G?p MSB v� LSB d�ng chu?n

    // Ki?m tra bit D2 (l?i kh�ng c� thermocouple)
    if (rawData & 0x0004) {
        return -1.0;
    }

    // D? li?u n?m ? c�c bit 15-3 ? d?ch ph?i 3 bit
    rawData >>= 3;

    // M?i don v? tuong ?ng 0.25�C
    temperature = rawData * 0.25;

    return temperature;
}
