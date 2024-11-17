#include <stm32f4xx_hal_gpio.h>
#include <stm32f4xx_hal_spi.h>
#include <stm32f4xx_hal_dma.h>
#include <stm32f4xx_hal_rcc.h>
#include "spim.h"

SPI_HandleTypeDef hspi;
DMA_HandleTypeDef hdma_spi1_tx;
void (*spi_end)(void);

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *spihandle) {
	if (&hspi == spihandle)
		if (spi_end)
			spi_end();
}

void spi_init(uint8_t word16bit) {
	hspi.Init.DataSize = word16bit ? SPI_DATASIZE_16BIT : SPI_DATASIZE_8BIT;
	HAL_SPI_Init(&hspi);
}


void spim_init(SPI_TypeDef *SPIx, uint8_t fastSpeed) {
	__HAL_RCC_GPIOA_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	if (SPIx == SPI1)
		__HAL_RCC_SPI1_CLK_ENABLE();
	else
		__HAL_RCC_SPI2_CLK_ENABLE();

	__HAL_SPI_DISABLE(&hspi);
	hspi.Instance = SPIx;
	hspi.Init.Direction = SPI_DIRECTION_2LINES;
	hspi.Init.Mode = SPI_MODE_MASTER;
	hspi.Init.CLKPolarity = SPI_POLARITY_HIGH;
	hspi.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi.Init.NSS = SPI_NSS_HARD_OUTPUT;
	hspi.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi.Init.BaudRatePrescaler =
			fastSpeed ? SPI_BAUDRATEPRESCALER_2 : SPI_BAUDRATEPRESCALER_64;
	spi_init(0);

	// DMA controller clock enable
	__HAL_RCC_DMA2_CLK_ENABLE();

	// SPI1 DMA Init
	// SPI1_TX Init
	hdma_spi1_tx.Instance = DMA2_Stream2;
	hdma_spi1_tx.Init.Channel = DMA_CHANNEL_2;
	hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_spi1_tx.Init.Mode = DMA_NORMAL;
	hdma_spi1_tx.Init.Priority = DMA_PRIORITY_HIGH;	// DMA_PRIORITY_LOW;
	hdma_spi1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	HAL_DMA_Init(&hdma_spi1_tx);

	hspi.hdmatx = &hdma_spi1_tx;
	hdma_spi1_tx.Parent = &hspi;

	// DMA interrupt init
	// DMA2_Stream2_IRQn interrupt configuration
	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

	__HAL_SPI_ENABLE(&hspi);
}

void SPI_send(SPI_TypeDef *SPIx, uint8_t word16bit, uint8_t *pBuff,
		uint16_t Len) {
	if (SPIx == hspi.Instance) {
		spi_init(word16bit);
		HAL_SPI_Transmit(&hspi, pBuff, Len, 100);
	}
}

void SPI_send_dma(SPI_TypeDef *SPIx, uint8_t word16bit, uint8_t *pBuff,
		uint16_t Len, void (*func)(void)) {
	if (SPIx == hspi.Instance) {
		spi_end = func;
		//spi_init(word16bit);
		HAL_SPI_Transmit_DMA(&hspi, pBuff, Len);
	}
}

void SPI_recv(SPI_TypeDef *SPIx, uint8_t word16bit, uint8_t *pBuff,
		uint16_t Len) {
	if (SPIx == hspi.Instance) {
		spi_init(word16bit);
		HAL_SPI_Receive(&hspi, pBuff, Len, 100);
	}
}

uint16_t SPI_sendrecv(SPI_TypeDef *SPIx, uint8_t word16bit, void *pData) {
	uint8_t result = 0;

	if (SPIx == hspi.Instance) {
		spi_init(word16bit);
		HAL_SPI_TransmitReceive(&hspi, pData, &result, 1, 100);
	}

	return result;
}

void SPI_SendRecv_dma(SPI_TypeDef *SPIx, uint8_t word16bit, uint8_t *pTxBuff,
		uint8_t *pRxBuff, uint16_t Len, void (*func)(void)) {
	if (SPIx != hspi.Instance)
		return;
	if (((!pTxBuff) && (!pRxBuff)) || (!Len))
		return;

	spi_end = func;
	spi_init(word16bit);
	HAL_SPI_TransmitReceive_DMA(&hspi, pTxBuff, pRxBuff, Len);
}
