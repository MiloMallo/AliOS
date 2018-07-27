/*
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */

#ifndef HAL_UART_H
#define HAL_UART_H
#include "stdint.h"
#include "errno.h"
#include "fm33a0xx_include_all.h"
#include "soc_init.h"

// ============== Fm33A0X Uart Define Begin =============== //
/*
 * UART Name
 */
typedef enum 
{
  UART_0,
  UART_1,
  UART_2,
  UART_3,
  UART_4,
  UART_5
}hal_uart_name_t;

/*
 * UART Port/Pin
 */
#define UART0RX_Pin						  GPIO_Pin_3
#define UART0RX_Port 					  GPIOF
#define UART0TX_Pin						  GPIO_Pin_4
#define UART0TX_Port 					  GPIOF

#define UART1RX_Pin						  GPIO_Pin_0
#define UART1RX_Port 					  GPIOB
#define UART1TX_Pin						  GPIO_Pin_1
#define UART1TX_Port 					  GPIOB

//#define UART1RX_Pin						  GPIO_Pin_3
//#define UART1RX_Port 					  GPIOE
//#define UART1TX_Pin						  GPIO_Pin_4
//#define UART1TX_Port 					  GPIOE

#define UART2RX_Pin						  GPIO_Pin_2
#define UART2RX_Port 					  GPIOB
#define UART2TX_Pin						  GPIO_Pin_3
#define UART2TX_Port 					  GPIOB

#define UART3RX_Pin						  GPIO_Pin_10
#define UART3RX_Port 					  GPIOC
#define UART3TX_Pin						  GPIO_Pin_11
#define UART3TX_Port 					  GPIOC

#define UART4RX_Pin						  GPIO_Pin_0
#define UART4RX_Port 					  GPIOD
#define UART4TX_Pin						  GPIO_Pin_1
#define UART4TX_Port 					  GPIOD

//#define UART4RX_Pin						  GPIO_Pin_9
//#define UART4RX_Port 					  GPIOD
//#define UART4TX_Pin						  GPIO_Pin_10
//#define UART4TX_Port 					  GPIOD

#define UART5RX_Pin						  GPIO_Pin_4
#define UART5RX_Port 					  GPIOC
#define UART5TX_Pin						  GPIO_Pin_5
#define UART5TX_Port 					  GPIOC

// ============== Fm33A0X Uart Define End =============== //

/*
 * UART data width
 */
typedef enum {
    DATA_WIDTH_5BIT,
    DATA_WIDTH_6BIT,
    DATA_WIDTH_7BIT,
    DATA_WIDTH_8BIT,
    DATA_WIDTH_9BIT
} hal_uart_data_width_t;

/*
 * UART stop bits
 */
typedef enum {
    STOP_BITS_1,
    STOP_BITS_2
} hal_uart_stop_bits_t;

/*
 * UART flow control
 */
typedef enum {
    FLOW_CONTROL_DISABLED,
    FLOW_CONTROL_CTS,
    FLOW_CONTROL_RTS,
    FLOW_CONTROL_CTS_RTS
} hal_uart_flow_control_t;

/*
 * UART parity
 */
typedef enum {
    NO_PARITY,
    EVEN_PARITY,    
    ODD_PARITY
} hal_uart_parity_t;

/*
 * UART mode
 */
typedef enum {
    MODE_TX,
    MODE_RX,
    MODE_TX_RX
} hal_uart_mode_t;

/*
 * UART configuration
 */
typedef struct {
    uint32_t                baud_rate;
    hal_uart_data_width_t   data_width;
    hal_uart_parity_t       parity;
    hal_uart_stop_bits_t    stop_bits;
    hal_uart_flow_control_t flow_control;
    hal_uart_mode_t         mode;
} uart_config_t;

typedef struct {
    uint8_t       port;    /* uart port */
    uart_config_t config;  /* uart config */
    void         *priv;    /* priv data */
} uart_dev_t;

/**
 * Initialises a UART interface
 *
 *
 * @param[in]  uart  the interface which should be initialised
 *
 * @return  0 : on success, EIO : if an error occurred with any step
 */
int32_t hal_uart_init(uart_dev_t *uart)
{
  UART_SInitTypeDef UART_para;
  RCC_ClocksType RCC_Clocks;  
   UARTx_Type *UARTx = NULL;
  
  if (uart->port > UART_5)
  {
    return EIO;
  }
  
  if ((uart->config.data_width > DATA_WIDTH_9BIT ||
       uart->config.data_width <= DATA_WIDTH_6BIT) ||
      (uart->config.parity > EVEN_PARITY) ||
      (uart->config.stop_bits > STOP_BITS_2) ||
      (uart->config.flow_control > FLOW_CONTROL_CTS_RTS) ||
      (uart->config.mode > MODE_TX_RX))
  {
    return EIO;
  }
  
  RCC_PERCLK_SetableEx(UARTCOMCLK, ENABLE);	//UART0~5 all clk enable
  
  switch (uart->port)
  {
    case UART_0:
      RCC_PERCLK_SetableEx(UART0CLK, ENABLE);	//UARTx clk enable
			//	UART0 IO config
			AltFunIO(UART0RX_Port, UART0RX_Pin, 0);		//UART0 RX
			AltFunIO(UART0TX_Port, UART0TX_Pin, 0);		//UART0 TX
		
			/*NVIC interrupt config*/
			NVIC_DisableIRQ(UART0_IRQn);
			//NVIC_SetPriority(UART0_IRQn,2);//interrupt priority config
			//NVIC_EnableIRQ(UART0_IRQn);	
      UARTx = UART0;
      break;
    case UART_1:
      RCC_PERCLK_SetableEx(UART1CLK, ENABLE);	//UARTx clk enable	
			//	UART1 IO config
			AltFunIO(UART1RX_Port, UART1RX_Pin, 0);		//UART1 RX
			AltFunIO(UART1TX_Port, UART1TX_Pin, 0);		//UART1 TX
			
			/*NVIC interrupt config*/
			NVIC_DisableIRQ(UART1_IRQn);
			//NVIC_SetPriority(UART1_IRQn,2);//interrupt priority config
			//NVIC_EnableIRQ(UART1_IRQn);	
      UARTx = UART1;
      break;
    case UART_2:
      RCC_PERCLK_SetableEx(UART2CLK, ENABLE);	//UARTx clk enable
			//	UART2 IO config
			AltFunIO(UART2RX_Port, UART2RX_Pin, 0);		//UART2 RX
			AltFunIO(UART2TX_Port, UART2TX_Pin, 0);		//UART2 TX
			
			/*NVIC interrupt config*/
			NVIC_DisableIRQ(UART2_IRQn);
			//NVIC_SetPriority(UART2_IRQn,2);//interrupt priority config
			//NVIC_EnableIRQ(UART2_IRQn);	
      UARTx = UART2;
      break;
    case UART_3:
      RCC_PERCLK_SetableEx(UART3CLK, ENABLE);	//UARTx clk enable
			//	UART3 IO config
			AltFunIO(UART3RX_Port, UART3RX_Pin, 0);		//UART3 RX
			AltFunIO(UART3TX_Port, UART3TX_Pin, 0);		//UART3 TX
			
			/*NVIC interrupt config*/
			NVIC_DisableIRQ(UART3_IRQn);
			//NVIC_SetPriority(UART3_IRQn,2);//interrupt priority config
			//NVIC_EnableIRQ(UART3_IRQn);	
      UARTx = UART3;
      break;
    case UART_4:
      RCC_PERCLK_SetableEx(UART4CLK, ENABLE);	//UARTx clk enable	
			//	UART4 IO config
			AltFunIO(UART4RX_Port, UART4RX_Pin, 0);		//UART4 RX
			AltFunIO(UART4TX_Port, UART4TX_Pin, 0);		//UART4 TX
			
			/*NVIC interrupt config*/
			NVIC_DisableIRQ(UART4_IRQn);
			//NVIC_SetPriority(UART4_IRQn,2);//
			//NVIC_EnableIRQ(UART4_IRQn);	
      UARTx = UART4;
      break;
    case UART_5:
      RCC_PERCLK_SetableEx(UART5CLK, ENABLE);	//UARTx clk enable	
			//	UART5 IO config
			AltFunIO(UART5RX_Port, UART5RX_Pin, 0);		//UART5 RX
			AltFunIO(UART5TX_Port, UART5TX_Pin, 0);		//UART5 TX
			
			/*NVIC interrupt config*/
			NVIC_DisableIRQ(UART5_IRQn);
			//NVIC_SetPriority(UART5_IRQn,2);//interrupt priority config
			//NVIC_EnableIRQ(UART5_IRQn);	
      UARTx = UART5;
      break;
    default:
      break;
  }
  
  //UART init config
	UART_para.BaudRate = uart->config.baud_rate;			//baud rate
	UART_para.DataBit = (UART_DataBitTypeDef)(uart->config.data_width - DATA_WIDTH_7BIT);		  //data bit
	UART_para.ParityBit = (UART_ParityBitTypeDef) uart->config.parity;			//parity
	UART_para.StopBit = (UART_StopBitTypeDef) uart->config.stop_bits;			//stop bits
	
	RCC_GetClocksFreq(&RCC_Clocks);//get APBClock
  UART_SInit(UARTx, &UART_para, RCC_Clocks.APBCLK_Frequency);	//uart init
  
  if (MODE_TX == uart->config.mode)
  {
    UARTx_TXSTA_TXEN_Setable(UARTx, ENABLE);		//send enable
  }
  else if (MODE_TX == uart->config.mode)
  {
    UARTx_RXSTA_RXEN_Setable(UARTx, ENABLE);		//recv enable
  }
  else
  {
    UARTx_RXSTA_RXEN_Setable(UARTx, ENABLE);		//recv enable
	  UARTx_TXSTA_TXEN_Setable(UARTx, ENABLE);		//send enable
  }
  
  return 0;
}

/**
 * Transmit data on a UART interface
 *
 * @param[in]  uart  the UART interface
 * @param[in]  data  pointer to the start of data
 * @param[in]  size  number of bytes to transmit
 *
 * @return  0 : on success, EIO : if an error occurred with any step
 */
int32_t hal_uart_send(uart_dev_t *uart, const void *data, uint32_t size, uint32_t timeout)
{
  uint32_t send_size = 0;
  UARTx_Type *UARTx = NULL;
  uint32_t lastTick;
  uint8_t *buf = (uint8_t *)data;
  
  switch (uart->port)
  {
    case UART_0:
      UARTx = UART0;
      break;
    case UART_1:
      UARTx = UART1;
      break;
    case UART_2:
      UARTx = UART2;
      break;
    case UART_3:
      UARTx = UART3;
      break;
    case UART_4:
      UARTx = UART4;
      break;
    case UART_5:
      UARTx = UART5;
      break;
    default:
      return EIO;
  }
  
  lastTick = SysTick->VAL;
  while (send_size < size)
  {
    IWDT_Clr();             //feed dog
    UARTx_TXREG_Write(UARTx, buf[send_size]);		//write send register
    while(SET == UARTx_TXBUFSTA_TXFF_Chk(UARTx));	//wait for send over
    send_size++;
    if ((lastTick - SysTick->VAL > (timeout * ((__SYSTEM_CLOCK/1000)))) &&
        (send_size < size))
    {
      return EIO;
    }
  }
  
  return 0;
}

/**
 * Receive data on a UART interface
 *
 * @param[in]   uart         the UART interface
 * @param[out]  data         pointer to the buffer which will store incoming data
 * @param[in]   expect_size  number of bytes to receive
 * @param[in]   timeout      timeout in milisecond, set this value to HAL_WAIT_FOREVER
 *                           if you want to wait forever
 *
 * @return  0 : on success, EIO : if an error occurred with any step
 */
int32_t hal_uart_recv(uart_dev_t *uart, void *data, uint32_t expect_size, uint32_t timeout)
{
  uint32_t recv_size = 0;
  UARTx_Type *UARTx = NULL;
  uint32_t lastTick;
  uint8_t *buf = (uint8_t *)data;
  
  //param check
  if ((NULL == uart) ||
    (NULL == data))
  {
    return EIO;
  }
  
  switch (uart->port)
  {
    case UART_0:
      UARTx = UART0;
      break;
    case UART_1:
      UARTx = UART1;
      break;
    case UART_2:
      UARTx = UART2;
      break;
    case UART_3:
      UARTx = UART3;
      break;
    case UART_4:
      UARTx = UART4;
      break;
    case UART_5:
      UARTx = UART5;
      break;
    default:
      return EIO;
  }
  
  lastTick = SysTick->VAL;
  while (recv_size < expect_size)
  {
    IWDT_Clr();             //feed dog
    if(SET == UARTx_RXBUFSTA_RXFF_Chk(UARTx))		//wait for a byte
		{
			buf[recv_size] = UARTx_RXREG_Read(UARTx);			//read recv register
      recv_size++;
		}
    if ((lastTick - SysTick->VAL > (timeout * ((__SYSTEM_CLOCK/1000)))) &&
        (recv_size < expect_size))
    {
      return EIO;
    }
  }
  
  return 0;
}

/**
 * Receive data on a UART interface
 *
 * @param[in]   uart         the UART interface
 * @param[out]  data         pointer to the buffer which will store incoming data
 * @param[in]   expect_size  number of bytes to receive
 * @param[out]  recv_size    number of bytes received
 * @param[in]   timeout      timeout in milisecond, set this value to HAL_WAIT_FOREVER
 *                           if you want to wait forever
 *
 * @return  0 : on success, EIO : if an error occurred with any step
 */
int32_t hal_uart_recv_II(uart_dev_t *uart, void *data, uint32_t expect_size,
                      uint32_t *recv_size, uint32_t timeout)
{
  uint32_t recv_index = 0;
  UARTx_Type *UARTx = NULL;
  uint32_t lastTick;
  uint8_t *buf = (uint8_t *)data;
  
  //param check
  if ((NULL == uart) ||
    (NULL == data))
  {
    return EIO;
  }
  
  switch (uart->port)
  {
    case UART_0:
      UARTx = UART0;
      break;
    case UART_1:
      UARTx = UART1;
      break;
    case UART_2:
      UARTx = UART2;
      break;
    case UART_3:
      UARTx = UART3;
      break;
    case UART_4:
      UARTx = UART4;
      break;
    case UART_5:
      UARTx = UART5;
      break;
    default:
      return EIO;
  }
  
  lastTick = SysTick->VAL;
  while (recv_index < expect_size)
  {
    IWDT_Clr();             //feed dog
    if(SET == UARTx_RXBUFSTA_RXFF_Chk(UARTx))		//wait for a byte
		{
			buf[recv_index] = UARTx_RXREG_Read(UARTx);			//read register
      recv_index++;
		}
    if ((lastTick - SysTick->VAL > (timeout * ((__SYSTEM_CLOCK/1000)))) &&
        (recv_index < expect_size))
    {
      if(recv_size!=NULL){
        *recv_size = recv_index;
      }
      return EIO;
    }
  }
  
  return 0;
}

/**
 * Deinitialises a UART interface
 *
 * @param[in]  uart  the interface which should be deinitialised
 *
 * @return  0 : on success, EIO : if an error occurred with any step
 */
int32_t hal_uart_finalize(uart_dev_t *uart)
{
  UARTx_Type *UARTx = NULL;
  
  if (NULL == uart)
  {
    return EIO;
  }
  
  switch (uart->port)
  {
    case UART_0:
      UARTx = UART0;
      break;
    case UART_1:
      UARTx = UART1;
      break;
    case UART_2:
      UARTx = UART2;
      break;
    case UART_3:
      UARTx = UART3;
      break;
    case UART_4:
      UARTx = UART4;
      break;
    case UART_5:
      UARTx = UART5;
      break;
    default:
      return EIO;
  }
  
  UARTx_Deinit(UARTx);
  return 0;
}

#endif /* HAL_UART_H */

