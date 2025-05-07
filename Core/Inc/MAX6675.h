#ifndef INC_MAX6675_H_
#define INC_MAX6675_H_

#include "main.h"

#define SSPORT GPIOA       // GPIO Port of Chip Select
#define SSPIN  GPIO_PIN_4  // GPIO PIN of Chip Select

float Max6675_Read_Temp(void);

#endif /* INC_MAX6675_H_ */
