#ifndef _STM32F10XREG_H_
#define _STM32F10XREG_H_

/****************************************************************************
 *
 *            Copyright (c) 2003-2008 by HCC Embedded
 *
 * This software is copyrighted by and is the sole property of
 * HCC.  All rights, title, ownership, or other interests
 * in the software remain the property of HCC.  This
 * software may only be used in accordance with the corresponding
 * license agreement.  Any unauthorized use, duplication, transmission,
 * distribution, or disclosure of this software is expressly forbidden.
 *
 * This Copyright notice may not be removed or modified without prior
 * written consent of HCC.
 *
 * HCC reserves the right to modify this software without notice.
 *
 * HCC Embedded
 * Budapest 1133
 * Vaci Ut 110
 * Hungary
 *
 * Tel:  +36 (1) 450 1302
 * Fax:  +36 (1) 450 1303
 * http: www.hcc-embedded.com
 * email: info@hcc-embedded.com
 *
 ***************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

#include "hcc_types.h"


#define BIT0 (1u<<0)
#define BIT1 (1u<<1)
#define BIT2 (1u<<2)
#define BIT3 (1u<<3)
#define BIT4 (1u<<4)
#define BIT5 (1u<<5)
#define BIT6 (1u<<6)
#define BIT7 (1u<<7)
#define BIT8 (1u<<8)
#define BIT9 (1u<<9)
#define BIT10 (1u<<10)
#define BIT11 (1u<<11)
#define BIT12 (1u<<12)
#define BIT13 (1u<<13)
#define BIT14 (1u<<14)
#define BIT15 (1u<<15)
#define BIT16 (1u<<16)
#define BIT17 (1u<<17)
#define BIT18 (1u<<18)
#define BIT19 (1u<<19)
#define BIT20 (1u<<20)
#define BIT21 (1u<<21)
#define BIT22 (1u<<22)
#define BIT23 (1u<<23)
#define BIT24 (1u<<24)
#define BIT25 (1u<<25)
#define BIT26 (1u<<26)
#define BIT27 (1u<<27)
#define BIT28 (1u<<28)
#define BIT29 (1u<<29)
#define BIT30 (1u<<30)
#define BIT31 (1u<<31)

#define PORT_DESC_WIDTH 4u

#define	PORT0 (PORT_DESC_WIDTH * 0u)
#define	PORT1 (PORT_DESC_WIDTH * 1u)
#define	PORT2 (PORT_DESC_WIDTH * 2u)
#define	PORT3 (PORT_DESC_WIDTH * 3u)
#define	PORT4 (PORT_DESC_WIDTH * 4u)
#define	PORT5 (PORT_DESC_WIDTH * 5u)
#define	PORT6 (PORT_DESC_WIDTH * 6u)
#define	PORT7 (PORT_DESC_WIDTH * 7u)
#define	PORT8 (PORT_DESC_WIDTH * 0u)
#define	PORT9 (PORT_DESC_WIDTH * 1u)
#define	PORT10 (PORT_DESC_WIDTH * 2u)
#define	PORT11 (PORT_DESC_WIDTH * 3u)
#define	PORT12 (PORT_DESC_WIDTH * 4u)
#define	PORT13 (PORT_DESC_WIDTH * 5u)
#define	PORT14 (PORT_DESC_WIDTH * 6u)
#define	PORT15 (PORT_DESC_WIDTH * 7u)


#define RCC_APB2_PERIPH_AFIO              BIT0
#define RCC_APB2_PERIPH_GPIOA             BIT2
#define RCC_APB2_PERIPH_GPIOB             BIT3
#define RCC_APB2_PERIPH_GPIOC             BIT4
#define RCC_APB2_PERIPH_GPIOD             BIT5
#define RCC_APB2_PERIPH_GPIOE             BIT6
#define RCC_APB2_PERIPH_GPIOF             BIT7
#define RCC_APB2_PERIPH_GPIOG             BIT8
#define RCC_APB2_PERIPH_ADC1              BIT9
#define RCC_APB2_PERIPH_ADC2              BIT10
#define RCC_APB2_PERIPH_TIM1              BIT11
#define RCC_APB2_PERIPH_SPI1              BIT12
#define RCC_APB2_PERIPH_USART1            BIT14
#define RCC_APB2_PERIPH_USART2            BIT17

#define RCC_AHBENR_PERIPH_DMA1EN             BIT0
#define RCC_AHBENR_PERIPH_DMA2EN             BIT1
#define RCC_AHBENR_PERIPH_FSMCEN             BIT8


#define RCC_PLLMUL_2     0x00000000
#define RCC_PLLMUL_3     0x00040000
#define RCC_PLLMUL_4     0x00080000
#define RCC_PLLMUL_5     0x000C0000
#define RCC_PLLMUL_6     0x00100000
#define RCC_PLLMUL_7     0x00140000
#define RCC_PLLMUL_8     0x00180000
#define RCC_PLLMUL_9     0x001C0000
#define RCC_PLLMUL_10    0x00200000
#define RCC_PLLMUL_11    0x00240000
#define RCC_PLLMUL_12    0x00280000
#define RCC_PLLMUL_13    0x002C0000
#define RCC_PLLMUL_14    0x00300000
#define RCC_PLLMUL_15    0x00340000
#define RCC_PLLMUL_16    0x00380000

#define RCC_PLLSRC_HSI_DIV2       0x0u
#define RCC_PLLSRC_HSE_DIV1       0x10000u
#define RCC_PLLSRC_HSE_DIV2       0x30000u

#define RCC_SYSCLKSRC_HSI         0x0u
#define RCC_SYSCLKSRC_HSE         0x1u
#define RCC_SYSCLKSRC_PLLCLK      0x2u

#define FLASH_BASE            	 0x40022000u
#define FLASH_OPTION_BYTES_BASE  0x1FFFF800u

#define FLASH_LATENCY_0     0x0u
#define FLASH_LATENCY_1     0x1u
#define FLASH_LATENCY_2     0x2u
#define FLASH_LATENCY_M		0x38u

#define FLASH_PREFETCH_BUFFER_DISABLE   0x0u
#define FLASH_PREFETCH_BUFFER_ENABLE    0x10u
#define FLASH_PREFETCH_BUFFER_M	   		0xFFFFFFEFu

#define FLASH_ACR_OFFSET	    0x0u
#define FLASH_KEYR_OFFSET	    0x4u
#define FLASH_OPTKEYR_OFFSET 	0x8u
#define FLASH_SR_OFFSET			0xCu
#define FLASH_CR_OFFSET			0x10u
#define FLASH_AR_OFFSET			0x14u
#define FLASH_RESERVED_OFFSET	0x18u
#define FLASH_OBR_OFFSET		0x1Cu
#define FLASH_WRPR_OFFSET		0x2Cu

#define FLASH_ACR		    (*(hcc_reg32 *)(FLASH_BASE + FLASH_ACR_OFFSET))
#define FLASH_KEYR			(*(hcc_reg32 *)(FLASH_BASE + FLASH_KEYR_OFFSET))
#define FLASH_OPTKEYR		(*(hcc_reg32 *)(FLASH_BASE + FLASH_OPTKEYR_OFFSET))
#define FLASH_SR			(*(hcc_reg32 *)(FLASH_BASE + FLASH_SR_OFFSET))
#define FLASH_CR			(*(hcc_reg32 *)(FLASH_BASE + FLASH_CR_OFFSET))
#define FLASH_AR			(*(hcc_reg32 *)(FLASH_BASE + FLASH_AR_OFFSET))
#define FLASH_RESERVED		(*(hcc_reg32 *)(FLASH_BASE + FLASH_RESERVED_OFFSET))
#define FLASH_OBR			(*(hcc_reg32 *)(FLASH_BASE + FLASH_OBR_OFFSET))
#define FLASH_WRPR			(*(hcc_reg32 *)(FLASH_BASE + FLASH_WRPR_OFFSET))


#define RCC_HCLK_DIV1                    0x00000000u
#define RCC_HCLK_DIV2                    0x00000400u
#define RCC_HCLK_DIV4                    0x00000500u
#define RCC_HCLK_DIV8                    0x00000600u
#define RCC_HCLK_DIV16                   0x00000700u

#define HPRE_2   BIT7
#define HPRE_4   (BIT7 | BIT4)
#define HPRE_8   (BIT7 | BIT5)
#define HPRE_16  (BIT7 | BIT5 | BIT4)
#define HPRE_64  (BIT7 | BIT6)
#define HPRE_128 (BIT7 | BIT6 | BIT4)
#define HPRE_256 (BIT7 | BIT6 | BIT5)
#define HPRE_512 (BIT7 | BIT6 | BIT5 | BIT4)

#define CR_HSEBYP_RESET           0xFFFBFFFFu
#define CR_HSEBYP_SET             0x00040000u
#define CR_HSEON_RESET            0xFFFEFFFFu
#define CR_HSEON_SET              0x00010000u
#define CR_HSITRIM                0xFFFFFF07u
#define CR_PLL_ON				  (0x1u << 24)
#define CR_PLL_READY			  (0x1u << 25)

#define CFGR_PLL_M                0xFFC0FFFFu
#define CFGR_PLLMull_M            0x003C0000u
#define CFGR_PLLSRC_M             0x00010000u
#define CFGR_PLLXTPRE_M           0x00020000u
#define CFGR_SWS_M                0x0000000Cu
#define CFGR_SW_M                 0xFFFFFFFCu
#define CFGR_HPRE_RESET_M         0xFFFFFF0Fu
#define CFGR_HPRE_SET_M           0x000000F0u
#define CFGR_PPRE1_RESET_M        0xFFFFF8FFu
#define CFGR_PPRE1_SET_M          0x00000700u
#define CFGR_PPRE2_RESET_M        0xFFFFC7FFu
#define CFGR_PPRE2_SET_M          0x00003800u
#define CFGR_ADCPRE_RESET_M       0xFFFF3FFFu
#define CFGR_ADCPRE_SET_M         0x0000C000u

#define RCC_BASE	0x40021000u

#define RCC_CR_OFFSET		0x0u
#define RCC_CFGR_OFFSET		0x4u
#define RCC_CIR_OFFSET		0x8u
#define RCC_APB2RSTR_OFFSET	0xCu
#define RCC_APB1RSTR_OFFSET	0x10u
#define RCC_AHBENR_OFFSET	0x14u
#define RCC_APB2ENR_OFFSET	0x18u
#define RCC_APB1ENR_OFFSET	0x1Cu
#define RCC_BDCR_OFFSET		0x20u
#define RCC_CSR_OFFSET		0x24u

#define RCC_CR 				 (*(hcc_reg32 *)(RCC_BASE + RCC_CR_OFFSET))
#define RCC_CFGR 			 (*(hcc_reg32 *)(RCC_BASE + RCC_CFGR_OFFSET))
#define RCC_CIR 	         (*(hcc_reg32 *)(RCC_BASE + RCC_CIR_OFFSET))
#define RCC_APB2RSTR         (*(hcc_reg32 *)(RCC_BASE + RCC_APB2RSTR_OFFSET))
#define RCC_APB1RSTR         (*(hcc_reg32 *)(RCC_BASE + RCC_APB1RSTR_OFFSET))
#define RCC_AHBENR  		 (*(hcc_reg32 *)(RCC_BASE + RCC_AHBENR_OFFSET))
#define RCC_APB2ENR          (*(hcc_reg32 *)(RCC_BASE + RCC_APB2ENR_OFFSET))
#define RCC_APB1ENR          (*(hcc_reg32 *)(RCC_BASE + RCC_APB1ENR_OFFSET))
#define RCC_BDCR             (*(hcc_reg32 *)(RCC_BASE + RCC_BDCR_OFFSET))
#define RCC_CSR              (*(hcc_reg32 *)(RCC_BASE + RCC_CSR_OFFSET))


#define RCC_CFGR_USBPRE        0x00400000
#define RCC_APB1RSTR_USB       0x00800000
#define RCC_APB1ENR_USB        0x00800000

#define GPIO_MODE_AIN          0x0u
#define GPIO_MODE_IN_FLOATING  0x1u
#define GPIO_MODE_IPUD 		   0x2u

#define GPIO_MODE_OUT_PP       0x0u
#define GPIO_MODE_OUT_OD       0x1u
#define GPIO_MODE_ALTF_PP      0x2u
#define GPIO_MODE_ALTF_OD      0x3u

#define GPIO_SPEED_10MHz       0x1u
#define GPIO_SPEED_2MHz		   0x2u
#define GPIO_SPEED_50MHz	   0x3u

#define AFIO_BASE   0x40010000u

#define AFIO_EVCR		(*(hcc_reg32 *)(AFIO_BASE + 0x0u))
#define AFIO_MAPR		(*(hcc_reg32 *)(AFIO_BASE + 0x4u))
#define AFIO_EXTICR1    (*(hcc_reg32 *)(AFIO_BASE + 0x8u))
#define AFIO_EXTICR2	(*(hcc_reg32 *)(AFIO_BASE + 0xCu))
#define AFIO_EXTICR3	(*(hcc_reg32 *)(AFIO_BASE + 0x10u))
#define AFIO_EXTICR4	(*(hcc_reg32 *)(AFIO_BASE + 0x14u))

#define USART2_REMAP BIT3
#define USART1_REMAP BIT2
#define I2C1_REMAP   BIT1
#define SPI1_REMAP   BIT0

#define GPIO_P0   BIT0
#define GPIO_P1   BIT1
#define GPIO_P2   BIT2
#define GPIO_P3   BIT3
#define GPIO_P4   BIT4
#define GPIO_P5   BIT5
#define GPIO_P6   BIT6
#define GPIO_P7   BIT7
#define GPIO_P8   BIT8
#define GPIO_P9   BIT9
#define GPIO_P10  BIT10
#define GPIO_P11  BIT11
#define GPIO_P12  BIT12
#define GPIO_P13  BIT13
#define GPIO_P14  BIT14
#define GPIO_P15  BIT15

#define GPIO_A_BASE	0x40010800u
#define GPIO_B_BASE	0x40010C00u
#define GPIO_C_BASE	0x40011000u
#define GPIO_D_BASE	0x40011400u
#define GPIO_E_BASE	0x40011800u
#define GPIO_F_BASE	0x40011C00u
#define GPIO_G_BASE	0x40012000u


#define GPIO_CRL_OFFSET		0x0u
#define GPIO_CRH_OFFSET		0x4u
#define GPIO_IDR_OFFSET		0x8u
#define GPIO_ODR_OFFSET		0xCu
#define GPIO_BSRR_OFFSET	0x10u
#define GPIO_BRR_OFFSET		0x14u
#define GPIO_LCKR_OFFSET	0x18u


#define GPIOx_CRL(base)		(*(hcc_reg32 *)(base + GPIO_CRL_OFFSET))	
#define GPIOx_CRH(base)		(*(hcc_reg32 *)(base + GPIO_CRH_OFFSET))	
#define GPIOx_IDR(base)		(*(hcc_reg32 *)(base + GPIO_IDR_OFFSET))	
#define GPIOx_ODR(base)		(*(hcc_reg32 *)(base + GPIO_ODR_OFFSET))	
#define GPIOx_BSRR(base)	(*(hcc_reg32 *)(base + GPIO_BSRR_OFFSET))	
#define GPIOx_BRR(base)		(*(hcc_reg32 *)(base + GPIO_BRR_OFFSET))	
#define GPIOx_LCKR(base)	(*(hcc_reg32 *)(base + GPIO_LCKR_OFFSET))
	
#define AFIOx_EVCR	    (*(hcc_reg32 *)(AFIO_BASE + AFIO_EVCR_OFFSET))	
#define AFIOx_MAPR		(*(hcc_reg32 *)(AFIO_BASE + AFIO_MAPR_OFFSET))	
#define AFIOx_EXTICR1	(*(hcc_reg32 *)(AFIO_BASE + AFIO_EXTICR1_OFFSET))	
#define AFIOx_EXTICR2	(*(hcc_reg32 *)(AFIO_BASE + AFIO_EXTICR2_OFFSET))	
#define AFIOx_EXTICR3	(*(hcc_reg32 *)(AFIO_BASE + AFIO_EXTICR3_OFFSET))	
#define AFIOx_EXTICR4	(*(hcc_reg32 *)(AFIO_BASE + AFIO_EXTICR4_OFFSET))	

#define DMA_1              0x40020000u
#define DMA_2              0x40020400u

#define DMA_CH_1           0x0u
#define DMA_CH_2           0x14u
#define DMA_CH_3           0x28u
#define DMA_CH_4           0x3cu

#define DMA_ISR(base)     (*(hcc_reg32 *)(base + 0x0u))
#define DMA_IFCR(base)    (*(hcc_reg32 *)(base + 0x4u))

#define DMA_CCR(base, ch)     (*(hcc_reg32 *)(base + ch + 0x8u))
#define DMA_CNDTR(base, ch)   (*(hcc_reg32 *)(base + ch + 0xcu))
#define DMA_CPAR(base, ch)    (*(hcc_reg32 *)(base + ch + 0x10u))
#define DMA_CMAR(base, ch)    (*(hcc_reg32 *)(base + ch + 0x14u))
#define DMA_CCR2(base, ch)    (*(hcc_reg32 *)(base + ch + 0x1cu))
#define DMA_CNDTR2(base, ch)  (*(hcc_reg32 *)(base + ch + 0x20u))

#define DMA_CCR_MINC           BIT7
#define DMA_CCR_PINC           BIT6
#define DMA_CCR_MEM_TO_PERIPH  BIT4
#define DMA_CCR_EN             BIT0           

#define DMA_ISR_TCIF_2          BIT5
#define DMA_ISR_TCIF_3          BIT9
#define DMA_ISR_TCIF_4          BIT13

#define SDIO_POWER         (*(hcc_reg32 *)0x40018000u)
#define MCIPOWER_PWR_OFF      (0u<<0)
#define MCIPOWER_PWR_UP       (2u<<0)
#define MCIPOWER_PWR_ON       (3u<<0)

#define SDIO_CLKCR         (*(hcc_reg32 *)0x40018004u)
#define SDIO_CLKCR_EN           BIT8
#define SDIO_CLKCR_PWRSAVE      BIT9
#define SDIO_CLKCR_BYPASS       BIT10
#define SDIO_CLKCR_WIDEBUS_4    BIT11
#define SDIO_CLKCR_NEGEDGE      BIT13
#define SDIO_CLKCR_HWFC_EN      BIT14

#define SDIO_ARG           (*(hcc_reg32 *)0x40018008u)
#define SDIO_CMD           (*(hcc_reg32 *)0x4001800cu)
#define MCICOMMAND_RESP       BIT6
#define MCICOMMAND_LONGRESP  (BIT6 | BIT7)
#define MCICOMMAND_INTERRUPT  BIT8
#define MCICOMMAND_PENDING    BIT9
#define MCICOMMAND_ENABLE     BIT10

#define SDIO_RESPCMD       (*(hcc_reg32 *)0x40018010u)
#define SDIO_RESP1         (*(hcc_reg32 *)0x40018014u)
#define SDIO_RESP2         (*(hcc_reg32 *)0x40018018u)
#define SDIO_RESP3         (*(hcc_reg32 *)0x4001801cu)
#define SDIO_RESP4         (*(hcc_reg32 *)0x40018020u)
#define SDIO_DTIMER        (*(hcc_reg32 *)0x40018024u)
#define SDIO_DLEN          (*(hcc_reg32 *)0x40018028u)
#define SDIO_DCTRL         (*(hcc_reg32 *)0x4001802cu)
#define MCIDATACTRL_ENABLE    BIT0
#define MCIDATACTRL_FROM_CARD BIT1
#define MCIDATACTRL_STREAM    BIT2
#define MCIDATACTRL_DMAEN     BIT3

#define SDIO_DCOUNT        (*(hcc_reg32 *)0x40018030u)
#define SDIO_STA           (*(hcc_reg32 *)0x40018034u)

#define MCISTATUS_CMDCRCFAIL         BIT0
#define MCISTATUS_DATACRCFAIL        BIT1
#define MCISTATUS_CMDTIMEOUT         BIT2
#define MCISTATUS_DATATIMEOUT        BIT3
#define MCISTATUS_TXUNDERRUN         BIT4
#define MCISTATUS_RXOVERRUN          BIT5
#define MCISTATUS_CMDREND            BIT6
#define MCISTATUS_CMDSENT            BIT7
#define MCISTATUS_DATAEND            BIT8
#define MCISTATUS_STARTBITERR        BIT9
#define MCISTATUS_DATABLOCKEND       BIT10
#define MCISTATUS_CMDACTIVE          BIT11
#define MCISTATUS_TXACTIVE           BIT12
#define MCISTATUS_RXACTIVE           BIT13
#define MCISTATUS_TXFIFOHALFEMPTY    BIT14
#define MCISTATUS_RXFIFOHALFFULL     BIT15
#define MCISTATUS_TXFIFOFULL         BIT16
#define MCISTATUS_RXFIFOFULL         BIT17
#define MCISTATUS_TXFIFOEMPTY        BIT18
#define MCISTATUS_RXFIFOEMPTY        BIT19
#define MCISTATUS_TXDATAAVLBL        BIT20
#define MCISTATUS_RXDATAAVLBL        BIT21

#define CEATAEND   BIT23
#define SDIOIT     BIT22
#define DBCKENDC   BIT10
#define STBITERRC  BIT9
#define DATAENDC   BIT8
#define CMDSENTC   BIT7
#define CMDRENDC   BIT6
#define RXOVERRC   BIT5
#define TXUNDERRC  BIT4
#define DTIMEOUTC  BIT3
#define CTIMEOUTC  BIT2
#define DCRCFAILC  BIT1
#define DCRCFAIL   BIT0


#define SDIO_ICR           (*(hcc_reg32 *)0x40018038u)

#define MCICLEAR_CMDCRCFAILCLR       BIT0
#define MCICLEAR_DATACRCFAILCLR      BIT1
#define MCICLEAR_CMDTIMEOUTCLR       BIT2
#define MCICLEAR_DATATIMEOUTCLR      BIT3
#define MCICLEAR_TXUNDERRUNCLR       BIT4
#define MCICLEAR_RXOVERRUNCLR        BIT5
#define MCICLEAR_CMDRESPENDCLR       BIT6
#define MCICLEAR_CMDSENTCLR          BIT7
#define MCICLEAR_DATAENDCLR          BIT8
#define MCICLEAR_STARTBITERRCLR      BIT9
#define MCICLEAR_DATABLOCKENDCLR     BIT10

#define SDIO_MASK          (*(hcc_reg32 *)0x4001803cu)
#define SDIO_FIFOCNT       (*(hcc_reg32 *)0x40018048u)
#define SDIO_FIFO          (*(hcc_reg32 *)0x40018080u)

#define SPI_1              0x40013000u

#define SPI_CR1_OFFSET     0x00
#define SPI_CR2_OFFSET     0x04
#define SPI_SR_OFFSET      0x08
#define	SPI_DR_OFFSET      0x0c
#define SPI_CRCPR_OFFSET   0x10
#define SPI_RXCRCR_OFFSET  0x14
#define SPI_TXCRCR_OFFSET  0x18
#define SPI_I2SCFGR_OFFSET 0x1c
#define	SPI_I2SPR_OFFSET   0x20

#define SPI_CR1(base)	  (*(hcc_reg16 *)(base + SPI_CR1_OFFSET))
#define SPI_CR2(base)	  (*(hcc_reg16 *)(base + SPI_CR2_OFFSET))
#define SPI_SR(base)	  (*(hcc_reg16 *)(base + SPI_SR_OFFSET))
#define	SPI_DR(base)	  (*(hcc_reg16 *)(base + SPI_DR_OFFSET))
#define SPI_CRCPR(base)  (*( hcc_reg16 *)(base + SPI_CRCPR_OFFSET))
#define SPI_RXCRCR(base)  (*(hcc_reg16 *)(base + SPI_RXCRCR_OFFSET))
#define SPI_RXCRCR(base)  (*(hcc_reg16 *)(base + SPI_RXCRCR_OFFSET))
#define SPI_I2SCFGR(base) (*(hcc_reg16 *)(base + SPI_I2SCFGR_OFFSET))
#define	SPI_I2SPR(base)	  (*(hcc_reg16 *)(base + SPI_I2SPR_OFFSET))

#define SPI_BIDIMODE  (1u << 15)
#define SPI_BIDIOE    (1u << 14)
#define SPI_CRCEN     (1u << 13)
#define SPI_CRCNEXT   (1u << 12)
#define SPI_DFF       (1u << 11)
#define SPI_RXONLY    (1u << 10)
#define SPI_SSM       (1u << 9)
#define SPI_SSI       (1u << 8)
#define SPI_LSBFIRST  (1u << 7)
#define SPI_SPE       (1u << 6)
#define SPI_BR_2      (0u << 3)
#define SPI_BR_4      (1u << 3)
#define SPI_BR_8      (2u << 3)
#define SPI_BR_16     (3u << 3)
#define SPI_BR_32     (4u << 3)
#define SPI_BR_64     (5u << 3)
#define SPI_BR_128    (6u << 3)
#define SPI_BR_256    (7u << 3)
#define SPI_MSTR      (1u << 2)
#define SPI_CPOL      (1u << 1)
#define SPI_CPHA      (1u << 0)

#define SPI_TX_DMA_EN BIT1
#define SPI_RX_DMA_EN BIT0

#define SPI_BSY 	  (1u << 7)
#define SPI_TXE	      (1u << 1)
#define SPI_RXNE	  (1u << 0)

#define USART_1     0x40013800u
#define USART_2     0x40004400u

#define USART_SR(base)       (*(hcc_reg16 *)(base + 0x0u))
#define USART_DR(base)       (*(hcc_reg16 *)(base + 0x4u))
#define USART_BRR(base)      (*(hcc_reg16 *)(base + 0x8u))
#define USART_CR1(base)      (*(hcc_reg16 *)(base + 0xCu))
#define USART_CR2(base)      (*(hcc_reg16 *)(base + 0x10u))
#define USART_CR3(base)      (*(hcc_reg16 *)(base + 0x14u))
#define USART_GTPR(base)     (*(hcc_reg16 *)(base + 0x18u))

#define USART_LINEN     (1u<<14)   
#define USART_STOP      (3u<<12)
#define USART_CLKEN     (1u<<11)
#define USART_CPOL      (1u<<10)
#define USART_CPHA      (1u<<9)   
#define USART_LBCL      (1u<<8)
#define USART_LBDIE     (1u<<6)
#define USART_LBDL      (1u<<5)
#define USART_ADD       (0xfu<<0)

#define USART_UE        BIT13
#define USART_M         BIT12
#define USART_WAKE      BIT11
#define USART_PCE       BIT10
#define USART_PS        BIT9
#define USART_PE        BIT8
#define USART_TXE       BIT7
#define USART_TC        BIT6
#define USART_RXNE      BIT5
#define USART_IDLE      BIT4
#define USART_TE        BIT3
#define USART_RE        BIT2
#define USART_RWU       BIT1
#define USART_SBK       BIT0


/*****************************************************************************/
/****** Register definitions for the USB intercafe. **************************/
#define USB_NB_BASE      0x40005C00UL
#define USB_NB_PMEM_BASE 0x40006000UL

#define USB_EPNR(n)  (*(hcc_reg16*)(USB_NB_BASE+((n)*4)))
#define USB_EPNR_CTRRX    (hcc_u16)BIT15
#define USB_EPNR_DTOGRX   (hcc_u16)BIT14
#define USB_EPNR_SETUP    (hcc_u16)BIT11
#define USB_EPNR_EPTYPE   (hcc_u16)(BIT9 | BIT10)
#define USB_EPNR_EPTYPE_EPT_BULK (0u<<9)
#define USB_EPNR_EPTYPE_EPT_CTRL (1u<<9)
#define USB_EPNR_EPTYPE_EPT_ISO  (2u<<9)
#define USB_EPNR_EPTYPE_EPT_INT  (3u<<9)
#define USB_EPNR_EPKIND   (hcc_u16)BIT8
#define USB_EPNR_CTRTX    (hcc_u16)BIT7
#define USB_EPNR_DTOGTX   (hcc_u16)BIT6

#define SET_EPNR_TX_STATE(state, ep)\
{\
  hcc_u16 epnr=USB_EPNR(ep);\
  epnr = (epnr & 0x70f) | (0x3<<4 & ((epnr & (0x3<<4)) ^ ((state)<<4))) \
         | USB_EPNR_CTRRX | USB_EPNR_CTRTX; \
  USB_EPNR(ep) = epnr; \
}

#define SET_EPNR_RX_STATE(state,ep)\
{\
  hcc_u16 epnr=USB_EPNR(ep);\
  epnr = (epnr & 0x70f) | (0x3<<12 & ((epnr & (0x3<<12)) ^ ((state)<<12))) \
         | USB_EPNR_CTRRX | USB_EPNR_CTRTX; \
  USB_EPNR(ep) = epnr; \
}

#define CLR_EPNR_CTR_RX(ep) \
  do {\
    hcc_u16 epnr=USB_EPNR(ep);\
    epnr &= (((1u<<4)-1u) << 0) | (((1u<<4)-1u) << 8);\
    epnr |= USB_EPNR_CTRTX;\
    USB_EPNR(ep) = epnr; \
  } while(0)

#define CLR_EPNR_CTR_TX(ep) \
  do {\
    hcc_u16 epnr=USB_EPNR(ep);\
    epnr &= (((1u<<4)-1u) << 0) | (((1u<<3)-1u) << 8);\
    epnr |= USB_EPNR_CTRRX;\
    USB_EPNR(ep) = epnr; \
  } while(0)

#define CLR_EPNR_CTR_RX(ep) \
  do {\
    hcc_u16 epnr=USB_EPNR(ep);\
    epnr &= (((1u<<4)-1u) << 0) | (((1u<<4)-1u) << 8);\
    epnr |= USB_EPNR_CTRTX;\
    USB_EPNR(ep) = epnr; \
  } while(0)


#define USB_CNTR     (*(hcc_reg16*)(USB_NB_BASE+0x40))
#define USB_CNTR_CTRM     ((hcc_u16)BIT15)
#define USB_CNTR_DOVRM    ((hcc_u16)BIT14)
#define USB_CNTR_ERRM     ((hcc_u16)BIT13)
#define USB_CNTR_WKUPM    ((hcc_u16)BIT12)
#define USB_CNTR_SUSPM    ((hcc_u16)BIT11)
#define USB_CNTR_RESETM   ((hcc_u16)BIT10)
#define USB_CNTR_SOFM     ((hcc_u16)BIT9)
#define USB_CNTR_ESOFM    ((hcc_u16)BIT8)

#define USB_CNTR_RESUME   ((hcc_u16)BIT4)
#define USB_CNTR_FSUSP    ((hcc_u16)BIT3)
#define USB_CNTR_LPMODE   ((hcc_u16)BIT2)
#define USB_CNTR_PDWN     ((hcc_u16)BIT1)
#define USB_CNTR_FRES     ((hcc_u16)BIT0)

#define USB_ISTR     (*(hcc_reg16*)(USB_NB_BASE+0x44))
#define USB_ISTR_CTR      USB_CNTR_CTRM
#define USB_ISTR_DOVR     USB_CNTR_DOVRM
#define USB_ISTR_ERR      USB_CNTR_ERRM
#define USB_ISTR_WKUP     USB_CNTR_WKUPM
#define USB_ISTR_SUSP     USB_CNTR_SUSPM
#define USB_ISTR_RESET    USB_CNTR_RESETM
#define USB_ISTR_SOF      USB_CNTR_SOFM
#define USB_ISTR_ESOF     USB_CNTR_ESOFM
#define USB_ISTR_SZDPRM   USB_CNTR_SZDPRM
#define USB_ISTR_DIR      (hcc_u16)BIT4
#define USB_ISTR_CLR_ALL  (hcc_u16)0u

#define USB_FNR      (*(hcc_reg16*)(USB_NB_BASE+0x48))
#define USB_FNR_RXDP      (hcc_u16)BIT15
#define USB_FNR_RXDM      (hcc_u16)BIT14
#define USB_FNR_LCK       (hcc_u16)BIT13
#define USB_FNR_LSOF      (hcc_u16)(BIT12 | BIT11)
#define USB_FNR_FN        (hcc_u16)((1u<<10)-1)

#define USB_DADDR    (*(hcc_reg16*)(USB_NB_BASE+0x4c))
#define USB_DADDR_EF      (hcc_u16) BIT7

#define USB_BTABLE   (*(hcc_reg16*)(USB_NB_BASE+0x50))

/* Buffer descriptor table entries in packet memory.*/

#define USB_WR_ADDRNTX(n,v) ((*(hcc_reg16*)(USB_NB_PMEM_BASE+((n)*16)+0x0))=v)
#define USB_WR_ADDRNRX(n,v) ((*(hcc_reg16*)(USB_NB_PMEM_BASE+((n)*16)+0x8))=v)
#define USB_RD_ADDRNTX(n)    (*(hcc_reg16*)(USB_NB_PMEM_BASE+((n)*16)+0x0))
#define USB_RD_ADDRNRX(n)    (*(hcc_reg16*)(USB_NB_PMEM_BASE+((n)*16)+0x8))

#define USB_WR_COUNTNTX(n,v)  ((*(hcc_reg16*)(USB_NB_PMEM_BASE+((n)*16)+0x4))=v)
#define USB_WR_COUNTNRX(n,v)  ((*(hcc_reg16*)(USB_NB_PMEM_BASE+((n)*16)+0xc))=v)
#define USB_RD_COUNTNTX(n)    (*(hcc_reg16*)(USB_NB_PMEM_BASE+((n)*16)+0x4))
#define USB_RD_COUNTNRX(n)    (*(hcc_reg16*)(USB_NB_PMEM_BASE+((n)*16)+0xc))


#define USB_COUNTNRX_BLSIZE32  BIT15

#define PMEM2CMEM(p)   (((p)<<1)+USB_NB_PMEM_BASE)
//#define PMEM2CMEM(p)   ((p)+USB_NB_PMEM_BASE)

#define BTABLE_SIZE    (8*8) //TODO: is this size correct?


/*****************************************************************************/


#ifdef __cplusplus
}
#endif
#endif


