/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fsmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "key.h"			//引用按键扫描相关函数
#include "ILI93xx.h"	//LCD头文件
#include "sram.h"			//引用外部内存读写相关函数
#include "malloc.h"		//引用内存管理相关函数
#include "stdio.h"

#include "includes.h"
#include "delay.h"
#include "sys.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/

/* USER CODE BEGIN PTD */
/*----------------------灯等引脚的定义---------------------------*/
#define LED_GREEN_H		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin,GPIO_PIN_SET)
#define LED_GREEN_L		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin,GPIO_PIN_RESET)
#define LED_GREEN_TO	HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin)

#define LED_RED_H		HAL_GPIO_WritePin(LED_RED_GPIO_Port,LED_RED_Pin,GPIO_PIN_SET)
#define LED_RED_L		HAL_GPIO_WritePin(LED_RED_GPIO_Port,LED_RED_Pin,GPIO_PIN_RESET)
#define LED_RED_TO	HAL_GPIO_TogglePin(LED_RED_GPIO_Port,LED_RED_Pin)

//LCD刷屏时使用的颜色
int lcd_discolor[14]={	WHITE, BLACK, BLUE,  BRED,      
						GRED,  GBLUE, RED,   MAGENTA,       	 
						GREEN, CYAN,  YELLOW,BROWN, 			
						BRRED, GRAY };

/*----------------------宏定义配置开始任务---------------------------*/
//任务优先级
#define START_TASK_PRIO		3
//任务堆栈大小	
#define START_STK_SIZE 		512
//任务控制块
OS_TCB StartTaskTCB;
//任务堆栈,这里声明了任务堆栈名，任务大小早已设置过
CPU_STK START_TASK_STK[START_STK_SIZE];
//任务函数
void start_task(void *p_arg);

/*----------------------宏定义配置任务1---------------------------*/
//任务优先级
#define TASK1_TASK_PRIO		4
//任务堆栈大小	
#define TASK1_STK_SIZE 		128
//任务控制块
OS_TCB TASK1_TaskTCB;
//任务堆栈	
CPU_STK TASK1_TASK_STK[TASK1_STK_SIZE];
void task1_task(void *p_arg);

/*----------------------宏定义配置任务2---------------------------*/
//任务优先级
#define TASK2_TASK_PRIO		5
//任务堆栈大小	
#define TASK2_STK_SIZE 		128
//任务控制块
OS_TCB TASK2_TaskTCB;
//任务堆栈	
CPU_STK TASK2_TASK_STK[TASK2_STK_SIZE];
//任务函数
void task2_task(void *p_arg);

/*----------------------宏定义配置任务3---------------------------*/
//任务优先级
#define TASK3_TASK_PRIO		6
//任务堆栈大小
#define TASK3_STK_SIZE		128
//任务控制块
OS_TCB	TASK3_TaskTCB;
//任务堆栈
__align(8) CPU_STK	TASK3_TASK_STK[TASK3_STK_SIZE];
//任务函数
void task3_task(void *p_arg);

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//重定向printf到串口1
#if 1
#include <stdio.h>
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *stream)
{
    /* 堵塞判断串口是否发送完成 */
    while((USART1->SR & 0X40) == 0);

    /* 串口发送完成，将该字符发送 */
    USART1->DR = (uint8_t) ch;

    return ch;
}
#endif



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	OS_ERR err;				//OS初始化的输入参数
	CPU_SR_ALLOC();
	delay_init(168);  //时钟初始化,同时也是为了UCOSIII的使用
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FSMC_Init();
  MX_USART1_UART_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
	TFTLCD_Init();		//初始化LCD
	
	my_mem_init(SRAMIN);		//初始化内部内存池
	my_mem_init(SRAMEX);		//初始化外部内存池
	my_mem_init(SRAMCCM);		//初始化CCM内存池
	
	LCD_Clear(WHITE);
	LCD_ShowString(30,10,210,24,24,"MY name is YYJed!");
	
	OSInit(&err);		//初始化UCOSIII
	OS_CRITICAL_ENTER();//进入临界区
	/*----------------创建开始任务-------------------*/
	OSTaskCreate(		(OS_TCB 	*  )	&StartTaskTCB,						//任务控制块
									(CPU_CHAR	*  )	"start task", 						//任务名字
									(OS_TASK_PTR )	start_task, 							//任务函数
									(void		* 	 )			0,										//传递给任务函数的参数
									(OS_PRIO	   )	START_TASK_PRIO,    			//任务优先级
									(CPU_STK   * )	&START_TASK_STK[0],				//任务堆栈基地址
									(CPU_STK_SIZE)	START_STK_SIZE/10,				//任务堆栈深度限位
									(CPU_STK_SIZE)	START_STK_SIZE,						//任务堆栈大小
									(OS_MSG_QTY  )			0,										//任务内部消息队列能够接收的最大消息数目,为0时禁止接收消息
									(OS_TICK	   )			0,										//当使能时间片轮转时的时间片长度，为0时为默认长度，
									(void   	*  )			0,										//用户补充的存储区
									(OS_OPT      )	OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //任务选项:允许对任务进行堆栈检查，在创建任务时清除堆栈
									(OS_ERR 	*  )		&err);									//存放该函数错误时的返回值		
									
										OS_CRITICAL_EXIT();											//退出临界区	 
										OSStart(&err);  												//开启UCOSIII
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/*--------------配置开始任务函数----------------*/
void start_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;

	CPU_Init();
#if OS_CFG_STAT_TASK_EN > 0u
   OSStatTaskCPUUsageInit(&err);  	//统计任务                
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN		//如果使能了测量中断关闭时间
    CPU_IntDisMeasMaxCurReset();	
#endif

#if	OS_CFG_SCHED_ROUND_ROBIN_EN  //当使用时间片轮转的时候
	 //使能时间片轮转调度功能,时间片长度为1个系统时钟节拍，既1*5=5ms
	OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);  
#endif		
	
	OS_CRITICAL_ENTER();	//进入临界区
	/*--------------------创建TASK1任务-----------------------*/
	OSTaskCreate(	 (OS_TCB 	* 	)	&TASK1_TaskTCB,		
								 (CPU_CHAR*	  )	"task1 task", 		
                 (OS_TASK_PTR )	task1_task, 			
                 (void		* 	)	0,					
                 (OS_PRIO	  	)	TASK1_TASK_PRIO,     
                 (CPU_STK   * )	&TASK1_TASK_STK[0],	
                 (CPU_STK_SIZE)	TASK1_STK_SIZE/10,	
                 (CPU_STK_SIZE)	TASK1_STK_SIZE,		
                 (OS_MSG_QTY  )	0,					
                 (OS_TICK	  	)	0,					
                 (void   	* 	)	0,					
                 (OS_OPT      )	OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
                 (OS_ERR 	* 	)	&err);				
				 
	/*--------------------创建TASK2任务-----------------------*/
	OSTaskCreate(	 (OS_TCB 	* 	)&TASK2_TaskTCB,		
								 (CPU_CHAR	* )"task2 task", 		
                 (OS_TASK_PTR )task2_task, 			
                 (void		* 	)0,					
                 (OS_PRIO	  	)TASK2_TASK_PRIO,     	
                 (CPU_STK   * )&TASK2_TASK_STK[0],	
                 (CPU_STK_SIZE)TASK2_STK_SIZE/10,	
                 (CPU_STK_SIZE)TASK2_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  	)0,					
                 (void   	* 	)0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* 	)&err);
				 
	/*--------------------创建浮点型任务-----------------------*/
	OSTaskCreate(	 (OS_TCB 	* 	)&TASK3_TaskTCB,		
								 (CPU_CHAR	* )"float test task", 		
                 (OS_TASK_PTR )task3_task, 			
                 (void		* 	)0,					
                 (OS_PRIO	  	)TASK3_TASK_PRIO,     	
                 (CPU_STK   * )&TASK3_TASK_STK[0],	
                 (CPU_STK_SIZE)TASK3_STK_SIZE/10,	
                 (CPU_STK_SIZE)TASK3_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  	)0,					
                 (void   	* 	)0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* 	)&err);	

	/*--------------------挂起开始任务----------------------*/			 
	OS_TaskSuspend((OS_TCB*)&StartTaskTCB,&err);			 
	OS_CRITICAL_EXIT();	//进入临界区
}

	/*--------------------配置任务1函数--------------------*/
void task1_task(void *p_arg)
{
	uint8_t task1_num=0;
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;
	
	POINT_COLOR = BLACK;
	OS_CRITICAL_ENTER();
	LCD_DrawRectangle(5,110,115,314); 	//画一个矩形	
	LCD_DrawLine(5,130,115,130);		//画线
	POINT_COLOR = BLUE;
	LCD_ShowString(6,111,110,16,16,"Task1 Run:000");
	OS_CRITICAL_EXIT();
	while(1)
	{
		task1_num++;	//任务执1行次数加1 注意task1_num1加到255的时候会清零！！
		LED_GREEN_TO;
		printf("任务1已经执行：%d次\r\n",task1_num);
		if(task1_num==5) 
		{
			OSTaskDel((OS_TCB*)&TASK1_TaskTCB,&err);	//任务1执行5此后删除掉任务2
			printf("任务1删除了任务2!\r\n");
		}
		LCD_Fill(6,131,114,313,lcd_discolor[task1_num%14]); //填充区域
		LCD_ShowxNum(86,111,task1_num,3,16,0x80);	//显示任务执行次数
		OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,&err); //延时1s
	}
}

	/*--------------------配置任务2函数--------------------*/
void task2_task(void *p_arg)
{
	uint8_t task2_num=0;
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;
	
	POINT_COLOR = BLACK;
	OS_CRITICAL_ENTER();
	LCD_DrawRectangle(125,110,234,314); //画一个矩形	
	LCD_DrawLine(125,130,234,130);		//画线
	POINT_COLOR = BLUE;
	LCD_ShowString(126,111,110,16,16,"Task2 Run:000");
	OS_CRITICAL_EXIT();
	while(1)
	{
		task2_num++;	//任务2执行次数加1 注意task1_num2加到255的时候会清零！！
		LED_RED_TO;
		printf("任务2已经执行：%d次\r\n",task2_num);
		LCD_ShowxNum(206,111,task2_num,3,16,0x80);  //显示任务执行次数
		LCD_Fill(126,131,233,313,lcd_discolor[13-task2_num%14]); //填充区域
		OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,&err); //延时1s
	}
}

	/*--------------------配置任务3函数--------------------*/
void task3_task(void *p_arg)
{
	CPU_SR_ALLOC();
	static float float_num=0.01;
	while(1)
	{
		float_num+=0.01f;
		OS_CRITICAL_ENTER();	//进入临界区
		printf("float_num的值为: %.4f\r\n",float_num);
		OS_CRITICAL_EXIT();		//退出临界区
		delay_ms(500);			//延时500ms
	}
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
