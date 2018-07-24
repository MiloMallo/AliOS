

/**
	******************************************************************************
	* File Name 				 : main.c
	* Description 			 : Main program body
	******************************************************************************
	** This notice applies to any and all portions of this file
	* that are not between comment pairs USER CODE BEGIN and
	* USER CODE END. Other portions of this file, whether 
	* inserted by the user or by software development tools
	* are owned by their respective copyright owners.
	*
	* COPYRIGHT(c) 2017 STMicroelectronics
	*
	* Redistribution and use in source and binary forms, with or without modification,
	* are permitted provided that the following conditions are met:
	* 	1. Redistributions of source code must retain the above copyright notice,
	* 		 this list of conditions and the following disclaimer.
	* 	2. Redistributions in binary form must reproduce the above copyright notice,
	* 		 this list of conditions and the following disclaimer in the documentation
	* 		 and/or other materials provided with the distribution.
	* 	3. Neither the name of STMicroelectronics nor the names of its contributors
	* 		 may be used to endorse or promote products derived from this software
	* 		 without specific prior written permission.
	*
	* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
	* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
	* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
	* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
	* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
	* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
	* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
	* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
	* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
	*
	******************************************************************************
	*/

/* Includes ------------------------------------------------------------------*/
#include "soc_init.h"
#include "k_config.h"
#include "hal/soc/soc.h"
#include "string.h"


#if defined 										(__CC_ARM) && defined(__MICROLIB)
#define PUTCHAR_PROTOTYPE 			int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE 			int fgetc(FILE *f)
#elif defined(__ICCARM__) 			
#define PUTCHAR_PROTOTYPE 			int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE 			int fgetc(FILE *f)

#else

/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
	 set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE 			int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE 			int __io_getchar(void)
#endif /* defined (__CC_ARM) && defined(__MICROLIB) */

//cpu�δ�ʱ������(�����ʱ��)
void Init_SysTick(uint32_t ticksPerSecond)
{
	SysTick_Config(SYSTEM_CLK / ticksPerSecond);
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |			//�ر��ж�
	SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

//IOģ�⹦������:LCD/ADC
void AnalogIO(GPIOx_Type * GPIOx, uint32_t PinNum)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructureRun;

	GPIO_Get_InitPara(GPIOx, PinNum, &GPIO_InitStructureRun);

	if ((GPIO_InitStructureRun.Pin != PinNum) ||
		 (GPIO_InitStructureRun.PxINEN != GPIO_IN_Dis) ||
		 (GPIO_InitStructureRun.PxODEN != GPIO_OD_En) ||
		 (GPIO_InitStructureRun.PxPUEN != GPIO_PU_Dis) ||
		 (GPIO_InitStructureRun.PxFCR != GPIO_FCR_ANA))
	{
		GPIO_InitStructure.Pin = PinNum;
		GPIO_InitStructure.PxINEN = GPIO_IN_Dis;
		GPIO_InitStructure.PxODEN = GPIO_OD_En;
		GPIO_InitStructure.PxPUEN = GPIO_PU_Dis;
		GPIO_InitStructure.PxFCR = GPIO_FCR_ANA;

		GPIO_Init(GPIOx, &GPIO_InitStructure);
	}
}

//IO��������� 
//type 0 = ��ͨ 
//type 1 = ����
#define IN_NORMAL 							0
#define IN_PULLUP 							1

void InputIO(GPIOx_Type * GPIOx, uint32_t PinNum, uint8_t Type)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructureRun;

	GPIO_Get_InitPara(GPIOx, PinNum, &GPIO_InitStructureRun);

	if ((GPIO_InitStructureRun.Pin != PinNum) ||
		 (GPIO_InitStructureRun.PxINEN != GPIO_IN_En) ||
		 (GPIO_InitStructureRun.PxODEN != GPIO_OD_En) || ((Type == IN_NORMAL) &&
		 (GPIO_InitStructureRun.PxPUEN != GPIO_PU_Dis)) || ((Type == IN_PULLUP) &&
		 (GPIO_InitStructureRun.PxPUEN != GPIO_PU_En)) ||
		 (GPIO_InitStructureRun.PxFCR != GPIO_FCR_IN))
	{
		GPIO_InitStructure.Pin = PinNum;
		GPIO_InitStructure.PxINEN = GPIO_IN_En;
		GPIO_InitStructure.PxODEN = GPIO_OD_En;
		if (Type == IN_NORMAL) GPIO_InitStructure.PxPUEN = GPIO_PU_Dis;
		else GPIO_InitStructure.PxPUEN = GPIO_PU_En;
		GPIO_InitStructure.PxFCR = GPIO_FCR_IN;

		GPIO_Init(GPIOx, &GPIO_InitStructure);
	}
}

//IO��������� 
//type 0 = ��ͨ 
//type 1 = OD
#define OUT_PUSHPULL						0
#define OUT_OPENDRAIN 					1

void OutputIO(GPIOx_Type * GPIOx, uint32_t PinNum, uint8_t Type)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructureRun;

	GPIO_Get_InitPara(GPIOx, PinNum, &GPIO_InitStructureRun);

	if ((GPIO_InitStructureRun.Pin != PinNum) ||
		 (GPIO_InitStructureRun.PxINEN != GPIO_IN_Dis) || ((Type == OUT_PUSHPULL) &&
		 (GPIO_InitStructureRun.PxODEN != GPIO_OD_Dis)) || ((Type == OUT_OPENDRAIN) &&
		 (GPIO_InitStructureRun.PxODEN != GPIO_OD_En)) ||
		 (GPIO_InitStructureRun.PxPUEN != GPIO_PU_Dis) ||
		 (GPIO_InitStructureRun.PxFCR != GPIO_FCR_OUT))
	{
		GPIO_InitStructure.Pin = PinNum;
		GPIO_InitStructure.PxINEN = GPIO_IN_Dis;
		if (Type == OUT_PUSHPULL) GPIO_InitStructure.PxODEN = GPIO_OD_Dis;
		else GPIO_InitStructure.PxODEN = GPIO_OD_En;
		GPIO_InitStructure.PxPUEN = GPIO_PU_Dis;
		GPIO_InitStructure.PxFCR = GPIO_FCR_OUT;

		GPIO_Init(GPIOx, &GPIO_InitStructure);
	}
}

//IO�������⹦�ܿ� 
//type 0 = ��ͨ 
//type 1 = OD (OD���ܽ��������⹦��֧��)
//type 2 = ��ͨ+���� 
//type 3 = OD+����
#define ALTFUN_NORMAL 					0
#define ALTFUN_OPENDRAIN				1
#define ALTFUN_PULLUP 					2
#define ALTFUN_OPENDRAIN_PULLUP 3

void AltFunIO(GPIOx_Type * GPIOx, uint32_t PinNum, uint8_t Type)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructureRun;

	GPIO_Get_InitPara(GPIOx, PinNum, &GPIO_InitStructureRun);

	if ((GPIO_InitStructureRun.Pin != PinNum) ||
		 (GPIO_InitStructureRun.PxINEN != GPIO_IN_Dis) || (((Type & 0x01) == 0) &&
		 (GPIO_InitStructureRun.PxODEN != GPIO_OD_Dis)) || (((Type & 0x01) != 0) &&
		 (GPIO_InitStructureRun.PxODEN != GPIO_OD_En)) || (((Type & 0x02) == 0) &&
		 (GPIO_InitStructureRun.PxPUEN != GPIO_PU_Dis)) || (((Type & 0x02) != 0) &&
		 (GPIO_InitStructureRun.PxPUEN != GPIO_PU_En)) ||
		 (GPIO_InitStructureRun.PxFCR != GPIO_FCR_DIG))
	{
		GPIO_InitStructure.Pin = PinNum;
		GPIO_InitStructure.PxINEN = GPIO_IN_Dis;
		if ((Type & 0x01) == 0) GPIO_InitStructure.PxODEN = GPIO_OD_Dis;
		else GPIO_InitStructure.PxODEN = GPIO_OD_En;
		if ((Type & 0x02) == 0) GPIO_InitStructure.PxPUEN = GPIO_PU_Dis;
		else GPIO_InitStructure.PxPUEN = GPIO_PU_En;
		GPIO_InitStructure.PxFCR = GPIO_FCR_DIG;

		GPIO_Init(GPIOx, &GPIO_InitStructure);
	}
}

//IO�رգ�od����ߣ�
//������ʹ����Чʱ������ⲿ�źŸ��գ�Ҳ���ܵ���FM385�ܽ�©�磻
//���Խ�FCR����Ϊ01��GPIO�������ODEN����Ϊ1����α��©������ر�����ʹ�ܣ��������������Ϊ1
//ע��SWD�ӿڵ�PG8,9�������ı����ǵ����ý��޷�����
void CloseIO(GPIOx_Type * GPIOx, uint32_t PinNum)
{
	GPIO_InitTypeDef GPIO_InitStructureRun;

	GPIO_Get_InitPara(GPIOx, PinNum, &GPIO_InitStructureRun);

	if ((GPIO_InitStructureRun.PxFCR != GPIO_FCR_OUT))
	{
		GPIO_SetBits(GPIOx, PinNum);
		OutputIO(GPIOx, PinNum, OUT_OPENDRAIN);
	}
	else 
	{
		OutputIO(GPIOx, PinNum, OUT_OPENDRAIN);
		GPIO_SetBits(GPIOx, PinNum);
	}
}

//IO�ڳ�ʼ���״̬����
void Init_Pad_Io(void)
{
	GPIOx_DO_Write(GPIOA, 0x0000);										//
	GPIOx_DO_Write(GPIOB, 0x0000);										//
	GPIOx_DO_Write(GPIOC, 0x0000);										//
	GPIOx_DO_Write(GPIOD, 0x0000);										//
	GPIOx_DO_Write(GPIOE, 0x0000);										//
	GPIOx_DO_Write(GPIOF, 0x0000);										//
	GPIOx_DO_Write(GPIOG, 0x0000);										//
}

void IO_AnalogFunSet(void)
{
	/* PE4ģ�⹦��ѡ�� */
	GPIO_ANASEL_PE4ANS_Set(GPIO_ANASEL_PE4ANS_ACMP1_INP1);

	/* PE3ģ�⹦��ѡ�� */
	GPIO_ANASEL_PE3ANS_Set(GPIO_ANASEL_PE3ANS_ACMP1_INN1);

	/* PC15ģ�⹦��ѡ�� */
	GPIO_ANASEL_PC15ANS_Set(GPIO_ANASEL_PC15ANS_ACMP1_INP0);

	/* PC14ģ�⹦��ѡ�� */
	GPIO_ANASEL_PC14ANS_Set(GPIO_ANASEL_PC14ANS_ACMP1_INN0);

	/* PC13ģ�⹦��ѡ�� */
	GPIO_ANASEL_PC13ANS_Set(GPIO_ANASEL_PC13ANS_ADC_IN2);

	/* PC12ģ�⹦��ѡ�� */
	GPIO_ANASEL_PC12ANS_Set(GPIO_ANASEL_PC12ANS_ADC_IN1);
}


//Ĭ�Ͽ���󲿷�����ʱ�ӣ��û��ɰ�����رղ���Ҫ��ʱ��
//ʱ�ӿ���رնԹ���Ӱ�첻��
void Init_RCC_PERIPH_clk(void)
{
	//PERCLKCON1
	RCC_PERCLK_SetableEx(DCUCLK, ENABLE); 						//debug controʱ��ʹ�ܣ������
	RCC_PERCLK_SetableEx(EXTI2CLK, ENABLE); 					//EXTI�ⲿ�����жϲ���ʱ�ӣ�IO�����˲�ʱ��ʹ��
	RCC_PERCLK_SetableEx(EXTI1CLK, ENABLE); 					//EXTI�ⲿ�����жϲ���ʱ�ӣ�IO�����˲�ʱ��ʹ��
	RCC_PERCLK_SetableEx(EXTI0CLK, ENABLE); 					//EXTI�ⲿ�����жϲ���ʱ�ӣ�IO�����˲�ʱ��ʹ��
	RCC_PERCLK_SetableEx(PDCCLK, ENABLE); 						//IO����ʱ�ӼĴ���ʹ��
	RCC_PERCLK_SetableEx(ANACCLK, ENABLE);						//ģ���·����ʱ��ʹ��
	RCC_PERCLK_SetableEx(IWDTCLK, ENABLE);						//IWDT����ʱ��ʹ��
	RCC_PERCLK_SetableEx(SCUCLK, ENABLE); 						//system controlʱ��ʹ�ܣ������
	RCC_PERCLK_SetableEx(PMUCLK, ENABLE); 						//��Դ����ģ��ʱ��ʹ��
	RCC_PERCLK_SetableEx(RTCCLK, ENABLE); 						//RTC����ʱ��ʹ��
	RCC_PERCLK_SetableEx(LPTFCLK, ENABLE);						//LPTIM����ʱ��ʹ��
	RCC_PERCLK_SetableEx(LPTRCLK, ENABLE);						//LPTIM����ʱ��ʹ��

	//PERCLKCON2 SETTING
	RCC_PERCLK_SetableEx(ADCCLK, ENABLE); 						//ADCʱ��ʹ��
	RCC_PERCLK_SetableEx(WWDTCLK, ENABLE);						//WWDTʱ��ʹ��
	RCC_PERCLK_SetableEx(RAMBISTCLK, DISABLE);				//RAMBISTʱ��ʹ�ܣ�����ر�
	RCC_PERCLK_SetableEx(FLSEPCLK, DISABLE);					//Flash��д������ʱ��ʹ�ܣ�����͹�
	RCC_PERCLK_SetableEx(DMACLK, ENABLE); 						//DMAʱ��ʹ��
	RCC_PERCLK_SetableEx(LCDCLK, ENABLE); 						//LCDʱ��ʹ��
	RCC_PERCLK_SetableEx(AESCLK, ENABLE); 						//AESʱ��ʹ��
	RCC_PERCLK_SetableEx(TRNGCLK, ENABLE);						//TRNGʱ��ʹ��
	RCC_PERCLK_SetableEx(CRCCLK, ENABLE); 						//CRCʱ��ʹ��

	//PERCLKCON3 SETTING
	RCC_PERCLK_SetableEx(I2CCLK, ENABLE); 						//I2Cʱ��ʹ��
	RCC_PERCLK_SetableEx(U7816CLK1, ENABLE);					//78161ʱ��ʹ��
	RCC_PERCLK_SetableEx(U7816CLK0, ENABLE);					//78160ʱ��ʹ��
	RCC_PERCLK_SetableEx(UARTCOMCLK, ENABLE); 				//UART0~5����Ĵ���ʱ��ʹ��
	RCC_PERCLK_SetableEx(UART5CLK, ENABLE); 					//UART5ʱ��ʹ��
	RCC_PERCLK_SetableEx(UART4CLK, ENABLE); 					//UART4ʱ��ʹ��
	RCC_PERCLK_SetableEx(UART3CLK, ENABLE); 					//UART3ʱ��ʹ��
	RCC_PERCLK_SetableEx(UART2CLK, ENABLE); 					//UART2ʱ��ʹ��
	RCC_PERCLK_SetableEx(UART1CLK, ENABLE); 					//UART1ʱ��ʹ��
	RCC_PERCLK_SetableEx(UART0CLK, ENABLE); 					//UART0ʱ��ʹ��
	RCC_PERCLK_SetableEx(HSPICLK, ENABLE);						//HSPIʱ��ʹ��
	RCC_PERCLK_SetableEx(SPI2CLK, ENABLE);						//SPI2ʱ��ʹ��
	RCC_PERCLK_SetableEx(SPI1CLK, ENABLE);						//SPI1ʱ��ʹ��

	//PERCLKCON4 SETTING
	RCC_PERCLK_SetableEx(ET4CLK, ENABLE); 						//ET4ʱ��ʹ��
	RCC_PERCLK_SetableEx(ET3CLK, ENABLE); 						//ET3ʱ��ʹ��
	RCC_PERCLK_SetableEx(ET2CLK, ENABLE); 						//ET2ʱ��ʹ��
	RCC_PERCLK_SetableEx(ET1CLK, ENABLE); 						//ET1ʱ��ʹ��
	RCC_PERCLK_SetableEx(BT2CLK, ENABLE); 						//BT2ʱ��ʹ��
	RCC_PERCLK_SetableEx(BT1CLK, ENABLE); 						//BT1ʱ��ʹ��
}

void Init_PLL(void)
{
	RCC_PLL_InitTypeDef PLL_InitStruct;

	PLL_InitStruct.PLLDB = 499; 											//pll��Ƶ�� = PLLDB + 1
	PLL_InitStruct.PLLINSEL = RCC_PLLCON_PLLINSEL_XTLF; //PLLʱ��Դѡ��XTLF
	PLL_InitStruct.PLLOSEL = RCC_PLLCON_PLLOSEL_MUL1; //Ĭ��ѡ��1�������������PLLDB��1023ʱ����ʹ��2�����ʵ�ָ��ߵı�Ƶ
	PLL_InitStruct.PLLEN = DISABLE; 									//Ĭ�Ϲر�PLL

	RCC_PLL_Init(&PLL_InitStruct);
	RCC_PLLCON_PLLEN_Setable(DISABLE);								//�ر�PLL
}

//ϵͳʱ������
//ʹ��RCHF����ʱ��,define_all.h ��SYSCLKdef�����ϵͳʱ��Ƶ��
void Init_SysClk(void)
{
	RCC_RCHF_InitTypeDef RCHF_InitStruct;
	RCC_SYSCLK_InitTypeDef SYSCLK_InitStruct;

	RCHF_InitStruct.FSEL = SYSCLKdef; 								//define_all.h ��SYSCLKdef�����ϵͳʱ��Ƶ��
	RCHF_InitStruct.RCHFEN = ENABLE;									//��RCHF

	RCC_RCHF_Init(&RCHF_InitStruct);

	SYSCLK_InitStruct.SYSCLKSEL = RCC_SYSCLKSEL_SYSCLKSEL_RCHF; //ѡ��RCHF����ʱ��
	SYSCLK_InitStruct.AHBPRES = RCC_SYSCLKSEL_AHBPRES_DIV1; //AHB����Ƶ
	SYSCLK_InitStruct.APBPRES = RCC_SYSCLKSEL_APBPRES_DIV1; //APB����Ƶ
	SYSCLK_InitStruct.EXTICKSEL = RCC_SYSCLKSEL_EXTICKSEL_AHBCLK; //EXTI,�����˲�ʱ��ʹ��AHBʱ��
	SYSCLK_InitStruct.SLP_ENEXTI = ENABLE;						//����ģʽʹ���ⲿ�жϲ���
	SYSCLK_InitStruct.LPM_RCLP_OFF = DISABLE; 				//����ģʽ�¿���RCLP	

	RCC_SysClk_Init(&SYSCLK_InitStruct);
}

//Mode:0 ����ģʽ�����п��Ź����������ж�ʱ��
//Mode:1 ����ģʽ�رտ��Ź����������ж�ʱ��
void SCU_Init(uint8_t Mode)
{
	if (Mode == 1) //debug
	{
		SCU_MCUDBGCR_DBG_WWDT_STOP_Setable(ENABLE); 		//����ģʽ�¹ر�WWDT
		SCU_MCUDBGCR_DBG_IWDT_STOP_Setable(ENABLE); 		//����ģʽ�¹ر�IWDT
	}
	else //release
	{
		SCU_MCUDBGCR_DBG_WWDT_STOP_Setable(DISABLE);		//����ģʽ������WWDT
		SCU_MCUDBGCR_DBG_IWDT_STOP_Setable(DISABLE);		//����ģʽ������IWDT
	}

	SCU_MCUDBGCR_DBG_ET4_STOP_Setable(DISABLE); 			//����ģʽ������ET4
	SCU_MCUDBGCR_DBG_ET3_STOP_Setable(DISABLE); 			//����ģʽ������ET3
	SCU_MCUDBGCR_DBG_ET2_STOP_Setable(DISABLE); 			//����ģʽ������ET2
	SCU_MCUDBGCR_DBG_ET1_STOP_Setable(DISABLE); 			//����ģʽ������ET1
	SCU_MCUDBGCR_DBG_BT2_STOP_Setable(DISABLE); 			//����ģʽ������BT2
	SCU_MCUDBGCR_DBG_BT1_STOP_Setable(DISABLE); 			//����ģʽ������BT1
}

void Init_SysClk_Gen(void) //ʱ��ѡ�����
{

	/*ϵͳʱ�ӳ���24M����Ҫ��wait1*/
	if (RCHFCLKCFG >= 24) FLASH_FLSRDCON_WAIT_Set(FLASH_FLSRDCON_WAIT_1CYCLE);
	else FLASH_FLSRDCON_WAIT_Set(FLASH_FLSRDCON_WAIT_0CYCLE);

	/*PLL����*/
	Init_PLL(); 																			//Ĭ�Ϲر�PLL

	/*ϵͳʱ������*/
	Init_SysClk();																		//Ĭ��ʹ��RCHF����ʱ��

	/*����ʱ��ʹ������*/
	Init_RCC_PERIPH_clk();														//Ĭ�Ͽ���󲿷�����ʱ��

	/*DMA����RAM���ȼ�����*/
	RCC_MPRIL_MPRIL_Set(RCC_MPRIL_MPRIL_DMA); 				//Ĭ��AHB Master���ȼ�����DMA����


	/*�µ縴λ����*/
	//pdr��bor�����µ縴λ����Ҫ��һ��
	//����Դ��ѹ�����µ縴λ��ѹʱ��оƬ�ᱻ��λס		
	//pdr��ѹ��λ��׼���ǹ��ļ��ͣ������޷�������
	//bor��ѹ��λ׼ȷ������Ҫ����2uA����
	ANAC_PDRCON_PDREN_Setable(ENABLE);								//��PDR
	ANAC_BORCON_OFF_BOR_Setable(DISABLE); 						//��PDR

	/*������ƼĴ�������*/
#ifdef __DEBUG
	SCU_Init(1);																			//����ʱ���ж�ʱ��,�رտ��Ź�

#else

	SCU_Init(0);
#endif
}

void IWDT_Init(void)
{
	RCC_PERCLK_SetableEx(IWDTCLK, ENABLE);						//IWDT����ʱ��ʹ��
	IWDT_Clr(); 																			//��IWDT
	IWDT_IWDTCFG_IWDTOVP_Set(IWDT_IWDTCFG_IWDTOVP_2s); //����IWDT�������
	IWDT_IWDTCFG_IWDTSLP4096S_Setable(DISABLE); 			//��������ʱ�Ƿ�����4096s������
}

void Init_IO(void)
{
	OutputIO(LED1_Port, LED1_Pin, 0); 								//led1
	OutputIO(LED2_Port, LED2_Pin, 0); 								//led2
	OutputIO(LED3_Port, LED3_Pin, 0); 								//led3
	OutputIO(LED4_Port, LED4_Pin, 0); 								//led4
	AltFunIO(DBGCOMRX_Port, DBGCOMRX_Pin, 0); 				//debug uart rx
	AltFunIO(DBGCOMTX_Port, DBGCOMTX_Pin, 0); 				//debug uart tx
}

static void configDelay(void) //Լ40ms
{
	__IO uint32_t count;

	for (count = SYSTEM_CLK / 100; count > 0; count--);
}

void Init_System(void)
{
	UART_InitTypeDef uartConfig;

	/*����ϵͳ����*/
	__disable_irq();																	//�ر�ȫ���ж�ʹ��

	IWDT_Init();																			//ϵͳ���Ź�����
	IWDT_Clr(); 																			//��ϵͳ���Ź�	
	configDelay();																		//�ȴ��ȶ�
	Init_SysClk_Gen();																//ϵͳʱ������
	RCC_Init_RCHF_Trim(clkmode);											//RCHF����У׼ֵ����(оƬ��λ���Զ�����8M��У׼ֵ)

	/*�û���ʼ������*/
	memset(&uartConfig, 0, sizeof(uartConfig));
	uartConfig.RXEN = ENABLE;
	uartConfig.TXEN = ENABLE;
	uartConfig.SPBRG = SYSTEM_CLK / 9600;
	UART_Init(DBGUART, &uartConfig);
	__enable_irq(); 																	//��ȫ���ж�ʹ��
}


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
void soc_init(void)
{
	Init_System();
	Init_IO();
	Init_SysTick(RHINO_CONFIG_TICKS_PER_SECOND);			//cpu�δ�ʱ������(�����ʱ��)	
}

void SysTick_Handler(void)
{
	//	HAL_IncTick();
	krhino_intrpt_enter();
	krhino_tick_proc();
	krhino_intrpt_exit();
	IWDT_Clr();
	//HAL_SYSTICK_IRQHandler();
}


/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
	* @brief	This function is executed in case of error occurrence.
	* @param	None
	* @retval None
	*/
__attribute__((weak)) void _Error_Handler(char * file, int line)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1)
	{
	}

	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
	 * @brief Reports the name of the source file and the source line number
	 * where the assert_param error has occurred.
	 * @param file: pointer to the source file name
	 * @param line: assert_param error line source number
	 * @retval None
	 */
void assert_failed(uint8_t * file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
		ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}

#endif


/**
	* @brief	Retargets the C library printf function to the USART.
	* @param	None
	* @retval None
	*/
PUTCHAR_PROTOTYPE
{
	if (ch == '\n')
	{
		//hal_uart_send(&console_uart, (void *)"\r", 1, 30000);
		while (UART_UARTIF_RxTxIF_ChkEx(DBGUART, TxInt) == 0);

		UARTx_TXREG_Write(DBGUART, '\r');
	}

	while (UART_UARTIF_RxTxIF_ChkEx(DBGUART, TxInt) == 0);

	UARTx_TXREG_Write(DBGUART, ch);

	return ch;
}

/**
	* @brief	Retargets the C library scanf function to the USART.
	* @param	None
	* @retval None
	*/
GETCHAR_PROTOTYPE
{
	/* Place your implementation of fgetc here */
	/* e.g. readwrite a character to the USART2 and Loop until the end of transmission */
	if (UART_UARTIF_RxTxIF_ChkEx(DBGUART, RxInt) == 0)
	{
		return - 1;
	}
	else 
	{
		return UARTx_RXREG_Read(DBGUART);
	}
}


void krhino_idle_hook()
{
  IWDT_Clr();
}
uart_dev_t uart_0;

/**
	* @}
	*/

/**
	* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
