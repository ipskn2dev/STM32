#include "AM2320.h"
#include "stm32f4xx.h"
//**********************************************************************//
unsigned int CRC16(uint8_t *ptr, uint8_t length) 
{ 
      unsigned int crc = 0xFFFF; 
      uint8_t s = 0x00; 

      while(length--) {
        crc ^= *ptr++; 
        for(s = 0; s < 8; s++) {
          if((crc & 0x01) != 0) {
            crc >>= 1; 
            crc ^= 0xA001; 
          } else crc >>= 1; 
        } 
      } 
      return crc; 
} 
//**********************************************************************//
void start_sequence(uint8_t dir)
{
//I2C1->CR1 |= I2C_CR1_ACK;//Acknowledge enable
I2C1->CR1 |= I2C_CR1_START;// формирование сигнала старт
while (!(I2C1->SR1 & I2C_SR1_SB)){}// ждем пока формируется  "Старт"
(void) I2C1->SR1;//читаем SR1, для сброса бита SB. событие EV5
I2C1->DR = dir ==0? AM2320_ADDR_TX : AM2320_ADDR_RX ;// передаем адрес датчика I2C1->DR = AM2320_ADDR;	
while (!(I2C1->SR1 & I2C_SR1_ADDR)){}	// ожидаем окончания передачи адреса cобытие EV6
(void) I2C1->SR1; //сбрасываем бит ADDR (адрес принят) (чтением SR1 и SR2):
(void) I2C1->SR2;	// 
}
//**********************************************************************//
void AM2320_ReadComand(void)
{
  I2C1->DR = 0x03;	
  while (!(I2C1->SR1 & I2C_SR1_TXE)){}//Data register empty (transmitters)
	I2C1->DR = 0x00;	
  while (!(I2C1->SR1 & I2C_SR1_TXE)){}//Data register empty (transmitters)
	I2C1->DR = 0x04;	
  while (!(I2C1->SR1 & I2C_SR1_TXE)){}//Data register empty (transmitters)
	I2C1->CR1 |= I2C_CR1_STOP ;// формирование сигнала "Стоп" 	
}
//**********************************************************************//
uint8_t AM2320_ReadData(float *h,float *t)
{
 uint8_t i;
 uint8_t buf[8];
 //*******************************************************************//
 //============================wake sensor============================//
 //*******************************************************************//
 I2C1->CR1 |= I2C_CR1_START;// start signal formation
 while (!(I2C1->SR1 & I2C_SR1_SB)){}// wait until the "Start"
 (void) I2C1->SR1;// flag = I2C1->SR1;//читаем SR1, для сброса бита SB.	 
 I2C1->DR = AM2320_ADDR_TX;// передаем адрес датчика I2C1->DR = AM2320_ADDR;	
 HAL_Delay(10); 
 I2C1->CR1 |= I2C_CR1_STOP ;// формирование сигнала "Стоп" 
 //*******************************************************************//
 //==========================send read command========================//
 //*******************************************************************//
 //start_sequence(TX);//старт последовательность с TX адресом
	HAL_Delay(1);
	 
 I2C1->CR1 |= I2C_CR1_START;// формирование сигнала старт
 while (!(I2C1->SR1 & I2C_SR1_SB)){}// ждем пока формируется  "Старт"
 (void) I2C1->SR1;//читаем SR1, для сброса бита SB. событие EV5
 I2C1->DR = AM2320_ADDR_TX;// передаем адрес датчика I2C1->DR = AM2320_ADDR;	
 while (!(I2C1->SR1 & I2C_SR1_ADDR)){}	// ожидаем окончания передачи адреса cобытие EV6
 (void) I2C1->SR1; //сбрасываем бит ADDR (адрес принят) (чтением SR1 и SR2):
 (void) I2C1->SR2;	// 
	 
 //AM2320_ReadComand();//отправляем команду на чтение данных с датчика
	I2C1->DR = 0x03;	
  while (!(I2C1->SR1 & I2C_SR1_TXE)){}//Data register empty (transmitters)
	I2C1->DR = 0x00;	
  while (!(I2C1->SR1 & I2C_SR1_TXE)){}//Data register empty (transmitters)
	I2C1->DR = 0x04;	
  while (!(I2C1->SR1 & I2C_SR1_TXE)){}//Data register empty (transmitters)
	I2C1->CR1 |= I2C_CR1_STOP ;// формирование сигнала "Стоп" 
	HAL_Delay(10);
		
	
 //*******************************************************************//
 //=======================read data from sensor=======================//
 //*******************************************************************//
 //start_sequence(RX);//старт последовательность с RX адресом
	I2C1->CR1 |= I2C_CR1_START;// формирование сигнала старт
 while (!(I2C1->SR1 & I2C_SR1_SB)){}// ждем пока формируется  "Старт"
 (void) I2C1->SR1;//читаем SR1, для сброса бита SB. событие EV5
 I2C1->DR = AM2320_ADDR_RX;// передаем адрес датчика I2C1->DR = AM2320_ADDR;	
 while (!(I2C1->SR1 & I2C_SR1_ADDR)){}	// ожидаем окончания передачи адреса cобытие EV6
 (void) I2C1->SR1; //сбрасываем бит ADDR (адрес принят) (чтением SR1 и SR2):
 (void) I2C1->SR2;	// 
		
		
  for (i=0; i<8; i++){
	while (!(I2C1->SR1 & I2C_SR1_RXNE)){} 
		    buf[i]=I2C1->DR;//читаем 8 байт данных
	}	
	I2C1->CR1 |= I2C_CR1_STOP ;// формирование сигнала "Стоп" 
 //*******************************************************************//
 //=========================CRC check=================================//
 //*******************************************************************//
	unsigned int Rcrc = buf[7] << 8;
	Rcrc += buf[6];
	if (Rcrc == CRC16(buf, 6)) {
		unsigned int temperature = ((buf[4] & 0x7F) << 8) + buf[5];
		*t = temperature / 10.0;
		*t = (((buf[4] & 0x80) >> 7)== 1) ? ((*t) * (-1)) : *t ; // температура может быть отрицательрой
		unsigned int humidity = (buf[2] << 8) + buf[3];
		*h = humidity / 10.0;
		return 0;
	}
        return 2;
}
