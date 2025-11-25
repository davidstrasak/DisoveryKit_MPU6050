#include "stdint.h"
#include "stdio.h"
#include "stm32f100xb.h"
#include "string.h"

void RCC_Configuration(void);
void GPIO_Configuration(void);
void Delay(uint32_t nCount);
void USART_Configuration(void);
void sendUSARTmessage(char message[50]);
void I2C_Configuration(void);
void I2C_GetData(char* message);

#define MPUaddress 0x68
#define DATA_SIZE 14

int main(void) {
   RCC_Configuration();
   GPIO_Configuration();
   USART_Configuration();
   I2C_Configuration();

   char message[100];

   while (1) {
      I2C_GetData(message);
      sendUSARTmessage(message);
      Delay(50);
   }
}

// GPIO configuration
void GPIO_Configuration(void) {
   // Enable clocku do portu A, B, C, D
   RCC->APB2ENR |= (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5);

   // PA9 - USART1 TX
   GPIOA->CRH &= ~(0xF << 4);
   GPIOA->CRH |= (0b11 << 4);  // output 50MHz
   GPIOA->CRH |= (0b10 << 6);  // PP - alternate function
   // PA10 - USART1 RX
   GPIOA->CRH &= ~(0xF << 8);
   GPIOA->CRH |= (0b00 << 8);   // Input mode
   GPIOA->CRH |= (0b10 << 10);  // Pull-up/pull-down
   // PB6 - SCL
   GPIOB->CRL &= ~(0xF << 24);
   GPIOB->CRL |= (0b11 << 24);  // output 50MHz
   GPIOB->CRL |= (0b11 << 26);  // alternate function - OPEN DRAIN!!
   // PB7 - SDA
   GPIOB->CRL &= ~(0xF << 28);
   GPIOB->CRL |= (0b11 << 28);  // output 50MHz
   GPIOB->CRL |= (0b11 << 30);  // alternate function - open drain
}

// Basic clock configuration
void RCC_Configuration(void) {
   RCC->CR |= 0x10000;  // HSE on - External high speed clock enabled
   while (!(RCC->CR & 0x20000)) {
   }  // HSE ready - waiting until the external clock is ready

   // //flash access setup
   // FLASH->ACR &= 0x00000038;   //mask register
   // FLASH->ACR |= 0x00000002;   //flash 2 wait state

   // FLASH->ACR &= 0xFFFFFFEF;   //mask register
   // FLASH->ACR |= 0x00000010;   //enable Prefetch Buffer

   // Select external clock for PLL
   RCC->CFGR |= 0x10000;
   // HPRE set to zero
   RCC->CFGR &= ~(0xF << 4);
   // Predividers
   RCC->CFGR &= ~(0b1 << 17);
   RCC->CFGR &= ~(0b111 << 8);   // set low speed clock to 1x
   RCC->CFGR &= ~(0b111 << 11);  // set high speed clock to 1x
   // PLL multiplier config
   RCC->CFGR &= ~(0b1111 << 18);
   RCC->CFGR |= (0b0001 << 18);  // 3x multiplier
   // => Total CPU freq = 24MHz

   // turning PLL on and waiting
   RCC->CR |= (1 << 24);
   while (!(RCC->CR & (1 << 25))) {
   }

   // Setting the PLL clock as the system clock and waiting
   RCC->CFGR &= ~(0b11);
   RCC->CFGR |= 0b10;
   while (!(RCC->CFGR & 0x00000008)) {
   }
}

/*Delay_ms smycka zpozduje zhruba o nCount 1 ms*/
void Delay(uint32_t nCount) {
   for (volatile uint32_t i = 0; nCount != 0; nCount--) {
      for (i = 2000; i != 0; i--);
   }
}

void USART_Configuration(void) {
   // Clock enable to USART1 and alternate functions
   RCC->APB2ENR |= (1 << 0);   // Alternate functions
   RCC->APB2ENR |= (1 << 14);  // USART1

   // No remap so USART1 lives at: TX = PA9, RX = PA10
   AFIO->MAPR &= ~(1 << 2);
   // // Remapping so USART1 lives at: TX = PB6, RX = PB7
   // AFIO->MAPR |= (1 << 2);

   // Baud rate setting: 24MHz/250000 = 96
   USART1->BRR = 96;

   // Enable transmitter and receiver
   USART1->CR1 |= (1 << 2);  // TX
   USART1->CR1 |= (1 << 3);  // RX

   // Set stop bits to: 1 stop bit
   USART1->CR2 &= ~(0b11 << 12);

   // Enable USART1
   USART1->CR1 |= (1 << 13);
}

void sendUSARTmessage(char message[50]) {
   uint8_t len = strlen(message);

   for (uint8_t i = 0; i < len; i++) {
      while (!(USART1->SR & (1 << 7))) {
      }  // Waiting until the transmit register is empty

      USART1->DR = message[i];  // Put message into the data register. USART1
                                // sends it automatically
   }
}

void I2C_Configuration(void) {
   // Enable clock for I2C1
   RCC->APB1ENR |= (1 << 21);

   // Set alternate remappings to normal
   AFIO->MAPR &= ~(1 << 1);  // SCL = PB6, SDA = PB7

   // Reset I2C1
   I2C1->CR1 |= (1 << 15);
   I2C1->CR1 &= ~(1 << 15);

   // Peripheral clock frequency set to 24MHz
   I2C1->CR2 |= (0b011000 << 0);

   // Enable acknowledge bit after I send an address
   I2C1->CR1 |= (1 << 10);

   // Set the clock freq
   // From datasheet: T_I2C = T_APB1 * CCR
   // for slow mode the equation for this is: 1/(100e3) = 1/(24e6) * X * 2
   // So I2C freq will be 100kHz
   I2C1->CCR |= 120;

   // Set the rise time. It is set as 1000ns/T_APB1 + 1
   // In my case: 1000e-9/(1/(24e6)) + 1 = 25
   I2C1->TRISE |= 25;

   // Enable I2C1
   I2C1->CR1 |= (1 << 0);
}

void I2C_GetData(char* message) {

   uint8_t data[DATA_SIZE];

   ///////////////////////// Part1: Waking up /////////////////////////////////

   // Starting I2C communication
   I2C1->CR1 |= (1 << 8);  // Start generation
   while (!(I2C1->SR1 & 1)) {
   }  // Wait for start condition

   I2C1->DR = (MPUaddress << 1 | 0);  // send address and write bit
   while (!(I2C1->SR1 & (1 << 1))) {
   }  //  Wait for end of address transmission

   // Clear ADDR flag
   (void)I2C1->SR1;
   (void)I2C1->SR2;

   I2C1->DR = (0x6B);  // Send address where I will write
   while (!(I2C1->SR1 & (1 << 7))) {
   }  // Wait until data register is empty

   I2C1->DR = (0x00);  // Write 0x00 which wakes up the device
   while (!(I2C1->SR1 & (1 << 2))) {
   }  // Wait until byte transfer finished

   I2C1->CR1 |= (1 << 9);  // STOP condition
   Delay(10);              // Delay to wait for MPU to wake up

   ///////////////////////// Part2: Write which data I want
   ////////////////////////////////////

   // Starting I2C communication again to read data
   I2C1->CR1 |= (1 << 8);  // START
   while (!(I2C1->SR1 & 1)) {
   }

   I2C1->DR = (MPUaddress << 1 | 0);  // WRITE mode
   while (!(I2C1->SR1 & (1 << 1))) {
   }
   (void)I2C1->SR1;
   (void)I2C1->SR2;

   I2C1->DR = 0x3B;  // Point to first register
   while (!(I2C1->SR1 & (1 << 2))) {
   }  // wait until byte transfer is finished

   I2C1->CR1 |= (1 << 9);  // STOP condition
   Delay(10);              // Delay to wait for MPU to wake up

   ///////////////////////// Part3: Reading data
   ////////////////////////////////////

   I2C1->CR1 |= (1 << 8);  // Repeated START
   while (!(I2C1->SR1 & 1)) {
   }

   I2C1->DR = (MPUaddress << 1 | 1);  // READ mode
   while (!(I2C1->SR1 & (1 << 1))) {
   }

   // Ensure ACK is enabled
   I2C1->CR1 |= (1 << 10);

   (void)I2C1->SR1;
   (void)I2C1->SR2;

   for (uint8_t i = 0; i < DATA_SIZE; i++) {
      while (!(I2C1->SR1 & (1 << 6))) {
      }  // Wait until there is something in the receive register

      // Set NACK for the next-to-last byte (index 12)
      // The ACK/NACK bit is sent while the next bit is being received
      // So setting it in the next-to-last iteration will stop the communication
      // in the last iteration
      if (i == DATA_SIZE - 2) {
         I2C1->CR1 &=
             ~(1 << 10);  // Clear ACK (Bit 10) - the MPU6050 reads the absence
                          // of ACK as NACK and stops sending data
         I2C1->CR1 |= (1 << 9);  // Set STOP (Bit 9) - Setting this together
                                 // with NACK per I2C implementation guide
      }

      data[i] = I2C1->DR;  // Read the data
   }

   I2C1->CR1 |= (1 << 10);  // Re-enable ACK for next time

   int16_t accel_x = (data[0] << 8) | data[1];
   int16_t accel_y = (data[2] << 8) | data[3];
   int16_t accel_z = (data[4] << 8) | data[5];
   int16_t temp = (data[6] << 8) | data[7];
   int16_t gyro_x = (data[8] << 8) | data[9];
   int16_t gyro_y = (data[10] << 8) | data[11];
   int16_t gyro_z = (data[12] << 8) | data[13];

   sprintf(message, "AX:%d,AY:%d,AZ:%d,T:%d,GX:%d,GY:%d,GZ:%d\n", accel_x,
           accel_y, accel_z, temp, gyro_x, gyro_y, gyro_z);
}