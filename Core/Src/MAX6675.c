#include "max6675.h"

extern SPI_HandleTypeDef hspi1;  // Khai báo dùng SPI1 (ho?c s?a l?i n?u mày dùng SPI khác)

// ------------------- Hàm d?c nhi?t d? -------------------
float Max6675_Read_Temp(void) {
		uint8_t buf[2];
    uint16_t rawData = 0;
    float temperature = 0;

   HAL_GPIO_WritePin(SSPORT, SSPIN, GPIO_PIN_RESET);
HAL_SPI_Receive(&hspi1, buf, 2, 100);
HAL_GPIO_WritePin(SSPORT, SSPIN, GPIO_PIN_SET);

	rawData = (buf[0] << 8) | buf[1];  // G?p MSB và LSB dúng chu?n

    // Ki?m tra bit D2 (l?i không có thermocouple)
    if (rawData & 0x0004) {
        return -1.0;
    }

    // D? li?u n?m ? các bit 15-3 ? d?ch ph?i 3 bit
    rawData >>= 3;

    // M?i don v? tuong ?ng 0.25°C
    temperature = rawData * 0.25;

    return temperature;
}
