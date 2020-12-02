

/* 激光测距 V6 */
/* UC-OSII版本 */
/* 主控任务、测量生产数据任务、处理数据任务、AGC任务 校准调试任务 */
/* 采用消息队列通信 */
/* 使用TDC-GPX2时间测量芯片 */
/* 多阈值拟合校准 */
/* 自动增益控制(AGC) */
/* 大小目标检测切换 */
/* 优化排序，使用快速排序算法 */
/* date:2020.05.22 Emil: hdu_tangguodong@163.com */

/* 让阈值时间差跟随距离，不再单独对阈值时间差做滤波，保证阈值时间差和输出距离唯�
	��对应 */
/* 提供最后对结果缓冲区内数据两种处理方式，平均和中值，通过宏定义ARGV选择 */
/* 最后再来转double,之前都用time_ps来处理数据，减小精度损失 */
/* 设置调试校准任务 */
/* date:2020.05.24 Emil: hdu_tangguodong@163.com */

/* 930~940 GAIN 1V/V */
/* 135米 1.6V */
/* 多阈值拟合策略: 仅使用err_time3(Vth3 - Vth1) */
/* 0<err_time3<=1120 err = 2.71 */
/* 1120<err_time3<2000	err = -3.019+0.8541*ln(-10.9372*err_time3+11430) */
/* err_time>2000  err = 0.0004*err_time3+4.0737 */
/* todo : 上升沿跳变点。峰值2V以上，err_time3 峰值2V以下，err_time1 */
/* date:2020.05.25 Emil: hdu_tangguodong@163.com */

/* 增加侦察阈值，负责探测是否有有用信号返回，当干扰脉冲大于信号脉冲时能够提高测�
	��能力 */
/* 测试：测大目标基本稳定，测线（只测了26米以内）跳动1~2米，猜测是增益太大，干扰�
	�冲叠加 */
/* todo: 确定测线问题原因，必要时需减小前级增益，打开AGC_LOW功能（可能会让近距离大目
	标不准） */
/* date:2020.05.26 Emil: hdu_tangguodong@163.com */

/* 经测试，对于不同反射目标，阈值时间差有不同的误差，特别是近距离阈值时间差 */
/* 这个要解决有点恼火，软件上貌似无能为力，改善硬件或许是唯一的办法 */
/* date:2020.05.29 Emil: hdu_tangguodong@163.com */

/* 需要对拟合曲线不线性区域进行优化（暂时还没方法） */
/* 增加自动档位切换模式代替基于脉冲峰值的AGC模式 实测发现测得更远了 最远稳定测到
	1030米 */
/* date:2020.06.05 Emil: hdu_tangguodong@163.com */

/* 针对近距离截止失真后阈值时间差跳动到非线性区域导致修正误差过大，
在小于某一个距离时跳过阈值时间差落入非线性区域（1200~1500）的数据 */
/* date:2020.06.09 Emil: hdu_tangguodong@163.com */

/* 优化TDC-GPX2驱动程序，提供一次测量读取缓冲区所有数据 */
/* date:2020.06.10 Emil: hdu_tangguodong@163.com */

/* 优化BUG ,stop通道读取的所有数据，都应该减去start通道读取的第一次数据，后面再读start
	通道都是0 */
/* 硬件问题：stop通道输入的脉冲有负电压，超过了手册给出的-0.3V，导致有错误数据输出 
	*/
/* 解决思路：加二极管到地，对电压进行钳位 目前电路暂且定型，以后有机会再测试修改
	硬件 */
/* 最重要的问题1：由于之前的接收发射坏了换了个镜头和接收发射，发现和之前的回波特
	性不同，
		可能增益更大，导致每套设备可能都要修改程序参数 */
/* 最重要的问题2：严重怀疑干扰脉冲是硬件本身产生的，因为换了个镜头和接收发射小板
	，干扰脉冲的位置从十几米左右变到了5米
		而且干扰脉冲似乎变大了 猜测不一定对 */
/* todo: 可以改进去除干扰脉冲的方式，一次测量得到的几组数据中只有一个是正确数据， 
	*/

/* date:2020.06.15 Emil: hdu_tangguodong@163.com */

/* 加入看门狗，避免由于错误的脉冲输入导致TDC芯片的INTERUPT状态一直处于0，无法跳出读�
	��FIFO数据的循环，导致程序跑飞 */
/* date:2020.06.19 Emil: hdu_tangguodong@163.com */

/* 增加用户输入测量模式  是测线还是测墙 */
/* date:2020.07.06 Emil: hdu_tangguodong@163.com */
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "timer.h"
#include "includes.h"
#include "app.h"
#include "mto.h"
#include "spi.h"
#include "tdc_gpx2.h"
#include "laser.h"
#include "tpl0202.h"
#include "wdg.h"



int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置中断优先级分组为组2：2位抢占优先级，2位响应优先级
	delay_init();
	uart_init(9600);
	out_mode_set();
	tpl0202_init(); 								// 数控阈值
	KEY_Init();
	tlv5636_init(); 								// DAC端口初始化
	agc_init();
	adc_init(); 									// ADC与DMA初始化
	laser_init();									//激光发射初始化
	tdc_init(); 									//时间测量芯片初始化
	tdc_config();									//时间测量芯片配置完成
	mto_init(&mcb);
	mto_add(60,70,80,line,adj_line_60_70_80);
	mto_add(60,70,80,wall,adj_wall_60_70_80);
//	mto_add(80,90,100,line,adj_line_80_90_100);
//	mto_add(80,90,100,wall,adj_wall_80_90_100);
	mto_start(CIRCULAR_LIST,CIRCULAR_LIST);

	setRefValue(REF1);								//set tlv5636 ref voltage mode
	setDacValueBin(quantify);						//初始化DAC输出电压,默认增益
	OSInit();										//初始化RTOS 启动启动线程
	OSTaskCreate(start_task, (void *) 0, (OS_STK *) &START_TASK_STAK[START_STAK_SIZE - 1], START_TASK_PRIO); //创建起始任务
	OSStart();
}

void start_task(void * pdata)
{
	OS_CPU_SR		cpu_sr = 0; 					//采用方法3开关中断需要的中间变量

	/* 开机保持到峰值 */
	PEAK_CONTROL		= 1;
	delay_ms(1);
	PEAK_CONTROL		= 0;
	AGC_EN				= 1;						//开启AGC
	pdata				= pdata;					//pdata没用时防止waring
	q_msg				= OSQCreate((void * *) &Qstart[0], QSTART_SIZE); //创建消息队列

//	IWDG_Init(4, 1000); 							//设置看门狗 1.6s 

	OS_ENTER_CRITICAL();							//进入临界区，关中断  //防止其它中断打断以下代码执行，比如通信中断的时候就需要关中断

#ifdef ADJ //调试校准模式

	OSTaskCreate(gen_task, (void *) 0, (OS_STK *) &GEN_TASK_STAK[GEN_STAK_SIZE - 1], GEN_TASK_PRIO); //测距任务，从tdc芯片读取数据
	OSTaskCreate(adj_task, (void *) 0, (OS_STK *) &ADJ_TASK_STAK[ADJ_STAK_SIZE - 1], ADJ_TASK_PRIO); //校准任务，做拟合的时候用

#else

	OSTaskCreate(master_task, (void *) 0, (OS_STK *) &MASTER_TASK_STAK[MASTER_STAK_SIZE - 1], MASTER_TASK_PRIO); //把测距数据作多阈值拟合，再用串口传给上位机
	OSTaskCreate(gen_task, (void *) 0, (OS_STK *) &GEN_TASK_STAK[GEN_STAK_SIZE - 1], GEN_TASK_PRIO); //测距任务
	OSTaskCreate(handle_task, (void *) 0, (OS_STK *) &HANDLE_TASK_STAK[HANDLE_STAK_SIZE - 1], HANDLE_TASK_PRIO); //中值滤波
	OSTaskCreate(agc_task, (void *) 0, (OS_STK *) &AGC_TASK_STAK[AGC_STAK_SIZE - 1], AGC_TASK_PRIO); //自动增益控制任务，现在没用
#endif

	OSTaskSuspend(START_TASK_PRIO); 				//挂起起始任务
	OS_EXIT_CRITICAL(); 							//退出临界区，开中断
}

void master_task(void * pdata)
{
	OS_CPU_SR		cpu_sr = 0;
	TIME_DATA 		time_data;
	INT32U			time_ps = 0;
	INT32U			err_time = 0;
	INT8U			recive_flag = 0;				//有数据标记

//	int i=0;		//for
	//double d_distance = 0;
	CH_DATA 		ch_distance;

#ifdef AUTO_LEVEL_MOD

	/**************************auto level mode*********************************/
	double			org_data;

	AGC_EN				= 0;

	OSTaskSuspend(AGC_TASK_PRIO);					//挂起AGC任务
	level				= 0;						//初始化档位
	OS_ENTER_CRITICAL();
	quantify			= LEVEL[level]; 			//设置档位对应增益
	setDacValueBin(quantify);
	OS_EXIT_CRITICAL();
	delay_ms(500);									//上电等待500ms出数据

	while (1)
		{
		if (outmode == UARTbluetooth)
			{
			OS_ENTER_CRITICAL();
			printf("\n\nq:%d\n", quantify);
			OS_EXIT_CRITICAL();
			}

		/* 获得数据 */
		OS_ENTER_CRITICAL();

		if (ans_cnt != 0) //结果缓冲区里有数据
			{
#ifdef ARGV //结果求平均

			//将缓冲区里的数据求平均
			//printf("ans_buf :");
			for (master_i = 0; master_i < ans_cnt; master_i++)
				{
				d_sum				+= ans_buf[master_i].time_ps;
				e_sum				+= ans_buf[master_i].err_time;

				//print_int_data(ans_buf[master_i].i_data);
				}

			//printf("\n");
			//distance = d_sum/ans_cnt;
			time_ps 			= d_sum / ans_cnt;
			err_time			= e_sum / ans_cnt;

#else //结果求中值

			quick_sort_time_data(ans_buf, ans_cnt);

//			printf("\n******************\n");
//			for(i=0;i<=ans_cnt;i++)
//				{
//					printf("buf%d: ps:%d.et:%d.\n",i,ans_buf[i].time_ps,ans_buf[i].err_time3);
//				}
//			printf("\n******************\n");
			
			//distance = ans_buf[ans_cnt/2].i_data;
			time_ps 			= ans_buf[ans_cnt / 2].time_ps;
			err_time			= ans_buf[ans_cnt / 2].err_time3;
			time_data = ans_buf[ans_cnt / 2];
			
#endif

			if (time_ps < TIME_PS_TH && err_time > 2200 && mcb.measure_mode==wall) //数据挑选，阈值时间差，距离
				{
				recive_flag = 0;
				}
			else if(err_time > 5000)	//阈值时间差大于5000的数据抖动过大，剔除不要
				{
					if((1.5 * (double) time_ps) / 10000 <400 && mcb.measure_mode==wall)		//此处开放了测墙的极限，测线的极限如何处理？
						recive_flag = 0;
					else if((1.5 * (double) time_ps) / 10000 <100 && mcb.measure_mode==line)
						recive_flag = 0;
					else
						recive_flag = 1;
				}
			else 
				recive_flag = 1;



			
			ans_cnt 			= 0;			//缓冲区清零
			ans_ite 			= 0;

			
#ifdef ARGV
			e_sum				= 0;
			d_sum				= 0;
#endif
			}
		else //缓冲区没有数据，标记为0
			{
			
			recive_flag 		= 0;

			}

		OS_EXIT_CRITICAL();

		if (recive_flag) //缓冲区有数据
			{
			//有数据，档位固定
			if (level_stable == 0)
				{
				level_stable		= 1;
				}

			no_data_cnt 		= 0;				//一旦有数据，无数据计数清零

			/* time_ps转距离double */
			d_distance			= (1.5 * (double) time_ps) / 10000;
			org_data			= d_distance;

			//d_distance = u32_dou(distance);//转化为double
			if (outmode == UARTbluetooth)
				{
				OS_ENTER_CRITICAL();
				printf("ORG:%.2f\t", org_data);
				OS_EXIT_CRITICAL();
				}

			//通过阈值拟合校准
#ifdef MULTI_VTH

			switch (mcb.measure_mode)
				{
				case wall: //测墙模式
					if(outmode==UARTbluetooth)
						printf("wall\n");
						mto_adj(&time_data,&d_distance);
					break;

				case line: //测线模式
					if(outmode==UARTbluetooth)
						printf("line\n");
							mto_adj(&time_data,&d_distance);
					break;

				default:
					break;
				}

#endif

			if (outmode == UARTbluetooth) //串口助手调试输出格式
				{
				OS_ENTER_CRITICAL();

#ifdef MULTI_VTH

				//printf("No:%.2f\n",org_data);
				printf("MV:");

#else

				printf("No:");
#endif
	
				printf("d:%.2f \t\t et:%d\n", d_distance,err_time);

				//printf(" e:%d ",err_time);
				//printf("q:%d\n",quantify);
				OS_EXIT_CRITICAL();
				}
			else //按照照相机要求的格式输出数据
				{
				OS_ENTER_CRITICAL();
				
				ch_distance 		= dou_ch_data(&d_distance, err_time);

				while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == 0)
					;

				USART_SendData(USART1, ch_distance.h_data);

				while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == 0)
					;

				USART_SendData(USART1, ch_distance.l_data);

				while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == 0)
					;

				USART_SendData(USART1, ch_distance.f_data);
				OS_EXIT_CRITICAL();
				}
			}
		else //没有数据
			{
			if (outmode == camera)
				{
				OS_ENTER_CRITICAL();

				while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == 0)
					;

				USART_SendData(USART1, 0xFF);

				while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == 0)
					;

				USART_SendData(USART1, 0xFF);

				while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == 0)
					;

				USART_SendData(USART1, 0xFF);
				OS_EXIT_CRITICAL();
				}

			//档位切换
			if (level_stable == 0)
				{
				if (level < MAX_LEVEL - 1)
					{
					++level;

					}
				else 
					{
					level_stable		= 1;
					}
				OS_ENTER_CRITICAL();
				quantify			= LEVEL[level];
				setDacValueBin(quantify);
				OS_EXIT_CRITICAL();
				}
			else //档位已固定得情况下没有数据累计到一定次数后，重新设置档位
				{

				if(mcb.measure_mode == line && d_distance<35)
				{
					quantify = 1100;
				}
				else if (no_data_cnt < MAX_NO_DATA_CNT)
					{
					++no_data_cnt;
					}
				else 
					{
					level_stable		= 0;
					no_data_cnt 		= 0;
					level				= 0;
					OS_ENTER_CRITICAL();
					quantify			= LEVEL[level];
					setDacValueBin(quantify);
					OS_EXIT_CRITICAL();
					}

				}
			}

		delay_ms(500);
		}

	/**************************************************************************/
#else //AGC mode	

	/**************************AGC mode****************************************/
	//INT32U distance = 0;
	INT16U			old_q = 0;
	INT8U			master_cnt = 0; 				//master task循环计数 
	INT8U			res_cnt = 0;					//在远近目标检测周期中的结果计数
	INT8U			detect_cnt = 0; 				//检测阈值在远近目标检测周期中的计数

#ifdef ARGV
	INT32U			e_sum = 0;						//阈值时间差求和
	INT8U			master_i;
	long			d_sum = 0;						//距离求和

#endif

	AGC_EN				= 1;

	while (1)
		{
		if (outmode == UARTbluetooth)
			{
			OS_ENTER_CRITICAL();
			printf("q:%d\n", quantify);
			OS_EXIT_CRITICAL();
			}


		/* 远近目标切换判断 */
		/* 因为增益过大会导致近距离数据失效，因此必须检测用户是否由远目标时的高增�
			�切换到了近目标，导致一直没有数据 */
		/* 若2秒没有数据输出，抽出0.5秒切换至初始增益检查是否是由远目标切换至近目标 
			*/
		if (master_cnt == 4) //2秒测量到达,最多响应时间不超过4秒
			{
			if (res_cnt == 0 && detect_cnt == 0) //没有结果
				{
				if (AGC_EN == 1)
					{
					AGC_EN				= 0;
					OSTaskSuspend(AGC_TASK_PRIO);	//挂起AGC任务

					if (outmode == UARTbluetooth)
						{
						OS_ENTER_CRITICAL();
						printf("AGC task suspend\n");
						OS_EXIT_CRITICAL();
						}
					}

				old_q				= quantify; 	//保存之前的值，如果关闭AGC后还没数据，说明是没有探测到目标或目标太远，再恢复之前的增益值
				OS_ENTER_CRITICAL();
				quantify			= QUANTIFY_INIT;
				setDacValueBin(quantify);
				OS_EXIT_CRITICAL();
				}

			master_cnt			= 0;
			res_cnt 			= 0;
			detect_cnt			= 0;
			}
		else if (master_cnt == 1 && old_q != 0)
			{
			if (ans_cnt == 0 && detect_cnt == 0) //不是切换到近目标了
				{
				quantify			= old_q;		//恢复增益值
				setDacValueBin(quantify);
				AGC_EN				= 1;
				OSTaskResume(AGC_TASK_PRIO);		//恢复AGC任务

				if (outmode == UARTbluetooth)
					{
					OS_ENTER_CRITICAL();
					printf("AGC resume\n");
					OS_EXIT_CRITICAL();
					}
				}

			old_q				= 0;
			}

		/* 获得数据 */
		OS_ENTER_CRITICAL();

		if (ans_cnt != 0) //结果缓冲区里有数据
			{
#ifdef ARGV //结果求平均

			//将缓冲区里的数据求平均
			//printf("ans_buf :");
			for (master_i = 0; master_i < ans_cnt; master_i++)
				{
				d_sum				+= ans_buf[master_i].time_ps;
				e_sum				+= ans_buf[master_i].err_time;

				//print_int_data(ans_buf[master_i].i_data);
				}

			//printf("\n");
			//distance = d_sum/ans_cnt;
			time_ps 			= d_sum / ans_cnt;
			err_time			= e_sum / ans_cnt;

#else //结果求中值

			quick_sort_time_data(ans_buf, ans_cnt);

			//distance = ans_buf[ans_cnt/2].i_data;
			time_ps 			= ans_buf[ans_cnt / 2].time_ps;
			err_time			= ans_buf[ans_cnt / 2].err_time3;
#endif

			if (time_ps < 150000 && err_time > 2200) //当结果少，说明难测，而距离又近，当作噪声去掉（宁错杀，不放过）
				recive_flag = 0;
			else 
				recive_flag = 1;

			ans_cnt 			= 0;
			ans_ite 			= 0;

#ifdef ARGV
			e_sum				= 0;
			d_sum				= 0;
#endif
			}
		else //缓冲区没有数据，标记为0
			{
			recive_flag 		= 0;
			}

		OS_EXIT_CRITICAL();

		/* 校准后输出结果 */
		if (recive_flag)
			{
			res_cnt++;								//数据计数加1

			/* time_ps转距离double */
			d_distance			= (1.5 * (double) time_ps) / 10000;

			//d_distance = u32_dou(distance);//转化为double
			//通过阈值拟合校准
#ifdef MULTI_VTH
			d_distance			= multi_thread_adj(&d_distance, &time_ps);
#endif

			if (outmode == UARTbluetooth) //串口助手调试输出格式
				{
				OS_ENTER_CRITICAL();

#ifdef MULTI_VTH

				//printf("MV:");
#else

				//printf("No:");
#endif

				//printf("%.2f ",d_distance);
				//printf(" e:%d ",err_time);
				//printf("q:%d\n",quantify);
				OS_EXIT_CRITICAL();
				}
			else //按照照相机要求的格式输出数据
				{
				OS_ENTER_CRITICAL();
				ch_distance 		= dou_ch_data(&d_distance, err_time);

				while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == 0)
					;

				USART_SendData(USART1, ch_distance.h_data);

				while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == 0)
					;

				USART_SendData(USART1, ch_distance.l_data);

				while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == 0)
					;

				USART_SendData(USART1, ch_distance.f_data);
				OS_EXIT_CRITICAL();
				}

			/* 判断AGC是否开启 */
			if (d_distance > 21.0)
				{
				if (AGC_EN == 0)
					{
					AGC_EN				= 1;
					quantify			= QUANTIFY_INIT; //重启AGC任务前初始化增益
					OSTaskResume(AGC_TASK_PRIO);	//恢复AGC任务

					if (outmode == UARTbluetooth)
						{
						OS_ENTER_CRITICAL();
						printf("AGC task resume1\n");
						OS_EXIT_CRITICAL();
						}
					}
				}
			else 
				{
				if (AGC_EN == 1)
					{
					AGC_EN				= 0;
					OSTaskSuspend(AGC_TASK_PRIO);	//挂起AGC任务
					quantify			= QUANTIFY_INIT;
					setDacValueBin(quantify);		//重置增益

					if (outmode == UARTbluetooth)
						{
						OS_ENTER_CRITICAL();
						printf("AGC task suspend\n");
						OS_EXIT_CRITICAL();
						}

					}
				}

			}
		else //没有数据输出 //0xFF 0xFF 0xFF
			{
			if (detect_val != 0) //探测阈值探测到信号脉冲//此时AGC由于干扰脉冲大于信号脉冲，增益上不去，挂起AGC任务
				{
				detect_cnt++;

				if (AGC_EN == 1)
					{
					AGC_EN				= 0;
					OSTaskSuspend(AGC_TASK_PRIO);	//挂起AGC任务

					if (outmode == UARTbluetooth)
						{
						OS_ENTER_CRITICAL();
						printf("AGC task suspend--\n");
						OS_EXIT_CRITICAL();
						}
					}

				OS_ENTER_CRITICAL();
				detect_val			= 0;

				if (old_q != 0)
					{
					quantify			= old_q + 100;
					old_q				= quantify;
					}
				else 
					{
					quantify			+= 100; 	//手动增加100增益
					}

				setDacValueBin(quantify);			//重置增益
				OS_EXIT_CRITICAL();
				}
			else 
				{
				//没数据输出，开启AGC拉大增益
				if (AGC_EN == 0)
					{
					AGC_EN				= 1;
					quantify			= QUANTIFY_INIT; //重启AGC任务前初始化增益
					OSTaskResume(AGC_TASK_PRIO);	//恢复AGC任务

					if (outmode == UARTbluetooth)
						{
						OS_ENTER_CRITICAL();
						printf("AGC task resume2\n");
						OS_EXIT_CRITICAL();
						}
					}
				}

			if (outmode == camera)
				{
				OS_ENTER_CRITICAL();

				while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == 0)
					;

				USART_SendData(USART1, 0xFF);

				while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == 0)
					;

				USART_SendData(USART1, 0xFF);

				while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == 0)
					;

				USART_SendData(USART1, 0xFF);
				OS_EXIT_CRITICAL();
				}

			}

		/* 防止回波过少导致峰值保持不住，集中发送连续脉冲充电 */
		if (AGC_EN)
			{
			int 			i;

			PEAK_CONTROL		= 1;				//放电
			i					= 0;
			PEAK_CONTROL		= 0;				//峰值保持

			for (i = 0; i < 50; i++) //充电到峰值,快速发射多个脉冲
				{
				laser_plus();
				delay_ms(1);
				}
			}

		delay_ms(500);								//每隔0.5秒查询结果并输出
		master_cnt++;
		}

#endif

	/*************************************************************************/
}




/* 生产者任务，获取飞行时间，阈值时间差 */
void gen_task(void * pdata)
{
	OS_CPU_SR		cpu_sr = 0;

	/* TDC-GPX2 测量结果 */
	result			measure_data_arr[MEASURE_DATA_ARR_SIZE];
	INT8U			res_cnt;
	INT8U			i_gen;
	int			time_ps;						//飞行时间 ps
	int			time_ps_detect; 				//探测阈值测量数据
    INT32U          distance;
	int 			err_time1, err_time2, err_time3; //阈值时间差
	long			start_res;
	long			start_index;

	//CH_DATA org_data[DATA_GROUP_SIZE];//数据缓冲数组，当填充满时通过消息队列发送给消费者任务
	TIME_DATA		org_data[80];		//数据缓冲数组，当填充满时通过消息队列发送给消费者任务
	TIME_DATA		time_data;
//	TIME_DATA		over_dis_data[3];	//远距离数据缓冲区
//	INT8U			over_cnt=0;
	GEN_TIME_DATA	old_timedata={0};
	GEN_TIME_DATA	cur_timedata={0};

	TIME_DATA		data_to_mask;
	unsigned int	org_data_ite = 0;
	INT8U 			all_cnt=0;
	INT8U			measure_cycle=0;
	INT8U			data_vaild;
   

	detect_val			= 0xFFFFFFFF;

	//test 
	//OSTaskSuspend(GEN_TASK_PRIO);
	while (1)
		{
		res_cnt 			= tdc_measure_group(measure_data_arr);
//		IWDG_Feed();								//喂狗 防止TDC芯片跑飞
		start_res			= measure_data_arr[0].stopresult[0];
		start_index 		= measure_data_arr[0].reference_index[0];
		
		for (i_gen = 0; i_gen < res_cnt; i_gen++)
			{
			//测量阈值飞行时间计算
			time_ps = measure_data_arr[i_gen].stopresult[1] -start_res + \
                (measure_data_arr[i_gen].reference_index[1] -start_index) *refclk_divisions;

			//探测阈值飞行时间
			time_ps_detect = measure_data_arr[i_gen].stopresult[2] -start_res + \
                (measure_data_arr[i_gen].reference_index[2] -start_index) *refclk_divisions;

			//vth1--vth3阈值时间差计算
			err_time3 = measure_data_arr[i_gen].stopresult[3] -measure_data_arr[i_gen].stopresult[1] + \
                (measure_data_arr[i_gen].reference_index[3] -measure_data_arr[i_gen].reference_index[1]) *refclk_divisions;
			
//			err_time2 = measure_data_arr[i_gen].stopresult[3] -measure_data_arr[i_gen].stopresult[2] + \
//                (measure_data_arr[i_gen].reference_index[3] -measure_data_arr[i_gen].reference_index[2]) *refclk_divisions;
//			
//			err_time1 = measure_data_arr[i_gen].stopresult[2] -measure_data_arr[i_gen].stopresult[1] + \
//                (measure_data_arr[i_gen].reference_index[2] -measure_data_arr[i_gen].reference_index[1]) *refclk_divisions;

//			OS_ENTER_CRITICAL();
//			printf("res_cnt[%d]=%.4f.\r\n",i_gen,(1.5 * (double) time_ps) / 10000);
//			OS_EXIT_CRITICAL();
			


			if(0)	//可见光很强，干扰数据较多
			{
				//强光干扰较多,测5组数据进行比较
				if (level_stable == 0)
				{
					level_stable		= 1; 
				}
				
				/*初步滤除，滤除非1000米范围内,且明显错误的数据*/
				if ((quantify > GAIN_TH) && (time_ps < TIME_PS_TH)) //增益大，距离近的去掉,干掉干扰脉冲 (对空散射 11米左右数据)
				{
					data_vaild = 0;					
				}
				else if (err_time3 > MAX_ERR_TIME || err_time3 <= MIN_ERR_TIME) //阈值时间差有误的去掉
				{
					data_vaild	= 0;
				}
				else if(time_ps > TIME_1000M || time_ps <= MIN_TIME)	//非1000米内数据
				{
					data_vaild	= 0;
				}
				else 
				{
					data_vaild	= 1;
				}
//				distance=(1.5 * (double) time_ps) / 10000;
				/*采集5组数据，并放在全部放在一个数组里*/
				if(data_vaild == 1)	//将有效数据保存在一个数组里
					{
						create_time_data(&time_data, time_ps, err_time1, err_time2, err_time3);
						org_data[all_cnt++] = time_data;	//装载到数据缓冲区			
						measure_cycle++;
					}
				else
					{
						OS_ENTER_CRITICAL();
						printf("measure_cycle:%d.\r\n",measure_cycle);
						OS_EXIT_CRITICAL();
					}
				/*采集完5组数据，将数组进行排序*/
				if(measure_cycle==5 && i_gen==(res_cnt-1))
					{
						quick_sort_time_data(org_data, all_cnt); //快速排序

						
						/*将排序后的数据进行判断，连续5个数据相近，则判断为有效数据进行输出*/
						
						data_to_mask =data_filter(org_data,all_cnt,20000);
						
						OS_ENTER_CRITICAL();					//ans_buf	ans_cnt  ans_ite 为临界资源
						if (ans_cnt < ANS_BUF_SIZE)
							{
							ans_cnt++;							//更新缓冲区结果数
							}
						else
							{
								ans_cnt=0;
							}
			
						if (ans_ite == ANS_BUF_SIZE - 1) //如果缓冲区已满，覆盖最开始的数据
							{
							ans_buf[ans_ite]	= data_to_mask;
							ans_ite 			= 0;
							}
						else 
							{
							ans_buf[ans_ite++]	= data_to_mask;
							}
						printf("5data.\n");
						OS_EXIT_CRITICAL();
						all_cnt=0;
						measure_cycle=0;
					}
				
			}
			else
			{
				measure_cycle=0;
					/*	有效值筛选 */
				//数据有效筛选,排除掉对空噪值，通过增益和距离筛选
				//if((quantify>GAIN_TH)&&(distance<DISTANCE_TH))
#ifdef ADJ
				distance=(1.5 * (double) time_ps) / 10000;
				if ( distance>= STANDARD_DISTANCE)
					data_vaild = 1; //筛出start之前的噪声数据
				else 
					data_vaild = 0;

				//滤除干扰数据
				if ((quantify > GAIN_TH) && (time_ps < TIME_PS_TH) && err_time3 > 2000) //增益大，距离近的去掉,干掉干扰脉冲 (对空散射 11米左右数据)
				{
					data_vaild = 0;					 
				}
				else if (err_time3 > MAX_ERR_TIME || err_time3 < MIN_ERR_TIME) //阈值时间差有误的去掉
				{
					data_vaild	= 0;
				}
				else if(time_ps > MAX_TIME || time_ps <= MIN_TIME)
				{
					data_vaild	= 0;
				}
				else 
				{
					data_vaild			= 1;
				}
				
				if(distance > 35 && distance < 60)		//阈值拟合时，避开这段区域
				{
					data_vaild = 0;
				}

#else

#ifdef DUG
				/*滤除前数据*/
				OS_ENTER_CRITICAL();
				printf("\n*******before filter********");
				printf("this cycle:%d.\n",res_cnt);
				printf("q:%d.\n",quantify);
				printf("D:%.2f\t\tet3:%d.\n",(1.5 * (double) time_ps) / 10000,err_time3);
				OS_EXIT_CRITICAL(); 
#endif


				//滤除干扰数据
				if ((quantify > GAIN_TH) && (time_ps < TIME_PS_TH)) //增益大，距离近的去掉,干掉干扰脉冲 (对空散射 11米左右数据)
				{
					data_vaild = 0;					
				}
				else if (err_time3 > MAX_ERR_TIME || err_time3 <= MIN_ERR_TIME) //阈值时间差有误的去掉
				{
					data_vaild	= 0;
				}
				else if(time_ps > MAX_TIME || time_ps <= MIN_TIME)
				{
					data_vaild	= 0;
				}
				else 
				{
					data_vaild			= 1;
				}

#ifdef DUG
	//			输出滤除干扰后的数据
				if(data_vaild == 1)
				{
					OS_ENTER_CRITICAL();
					printf("\n*******behind filter********\n");
					printf("D:%.2f\t\tet3:%d.\n",(1.5 * (double) time_ps) / 10000,err_time3);
					OS_EXIT_CRITICAL(); 
				}
#endif

				switch (mcb.measure_mode)
					{
					case wall: //测墙模式数据筛选
						if(quantify<=GAIN_TH && (time_ps < TIME_PS_TH2) && err_time3>1600) //百米之内测墙,干扰剔除  !!!注意测量50米测线别被筛出去了
						{
							data_vaild = 0;
						}
						break;

					case line: //测线模式数据筛选
						
//						if ((err_time3 < 2200 | err_time3 > 3500)&& time_ps < 4*TIME_PS_TH) //剔除20米内阈值时间差在2000以下的部分
//							{ 
////								data_vaild = 0;
//							}
						if(err_time3<1100)	//测线拟合函数是从1100开始
							{
								data_vaild = 0;
							}
						else if((time_ps < TIME_PS_TH) && err_time3>2000)
							{
								data_vaild = 0;
							}
						else 
							{
								;
							}
						break;
					}

#endif


				if (time_ps_detect > 0 && time_ps_detect < 10000000 && quantify <= GAIN_TH)
					{
					OS_ENTER_CRITICAL();
					detect_val			= time_ps_detect;
					OS_EXIT_CRITICAL();
					}
				
				if (data_vaild == 0) //当前没有数据
					{
					continue;
					}

				distance=(1.5 * (double) time_ps) / 10000;
				create_time_data(&time_data, time_ps, err_time1, err_time2, err_time3);
				
				if(mcb.measure_mode==wall && distance >600)
					{

						cur_timedata.distance=distance;
						cur_timedata.timedata=time_data;

						if(abs(cur_timedata.distance-old_timedata.distance)<3)
							{
								OS_ENTER_CRITICAL();					//ans_buf	ans_cnt  ans_ite 为临界资源
								if (ans_cnt < ANS_BUF_SIZE)
									{
									ans_cnt++;							//更新缓冲区结果数
									}
								else
									{
										ans_cnt=0;
									}
					
								if (ans_ite == ANS_BUF_SIZE - 1) //如果缓冲区已满，覆盖最开始的数据
									{
									ans_buf[ans_ite]	= cur_timedata.timedata;
									ans_ite 			= 0;
									}
								else 
									{
									ans_buf[ans_ite++]	= cur_timedata.timedata;
									}
								printf("wallover600.\n");
		//						printf("D:",distance);
								OS_EXIT_CRITICAL();
								
								old_timedata.distance=cur_timedata.distance;
								old_timedata.timedata=cur_timedata.timedata;
							}
						else
							{
								old_timedata.distance=cur_timedata.distance;
								old_timedata.timedata=cur_timedata.timedata;
							}

				
						
					}
				else if(mcb.measure_mode==line && distance>80)
					{
					
						if(distance>170)break;

						//将当前数据暂时存下来
						cur_timedata.distance=distance;
						cur_timedata.timedata=time_data;

						//判断当前数据与上一次数据的差异，差异小则输出，差异大则更新上一次缓冲数据
						if(abs(cur_timedata.distance-old_timedata.distance)<2)
							{
								OS_ENTER_CRITICAL();					//ans_buf	ans_cnt  ans_ite 为临界资源
								if (ans_cnt < ANS_BUF_SIZE)
									{
									ans_cnt++;							//更新缓冲区结果数
									}
								else
									{
										ans_cnt=0;
									}
					
								if (ans_ite == ANS_BUF_SIZE - 1) //如果缓冲区已满，覆盖最开始的数据
									{
									ans_buf[ans_ite]	= cur_timedata.timedata;
									ans_ite 			= 0;
									}
								else 
									{
									ans_buf[ans_ite++]	= cur_timedata.timedata;
									}
								printf("lineover.\n");
		//						printf("D:",distance);
								OS_EXIT_CRITICAL();
								
								old_timedata.distance=cur_timedata.distance;
								old_timedata.timedata=cur_timedata.timedata;
							}
						else
							{
								old_timedata.distance=cur_timedata.distance;
								old_timedata.timedata=cur_timedata.timedata;
							}
						
					}
				else
					{
						
						org_data[org_data_ite++] = time_data;	//装载到数据缓冲区			
						
						if(org_data_ite == DATA_GROUP_SIZE)
						{
							org_data_ite		= 0;

							OSQPost(q_msg, org_data);			//发送消息队列
						}
					}
				

				
				}


			}
			
		delay_ms(10);
		}
} /* 消费者处理任务，中值滤波 */

/*
中值濾波
*/
void handle_task(void * pdata)
{
	OS_CPU_SR		cpu_sr = 0;

	TIME_DATA * 	res;

	INT8U			err;

	TIME_DATA		time_data[DATA_GROUP_SIZE]; 	//待处理数据缓冲区

	int 			handle_i;

	INT8U			mid_pos = DATA_GROUP_SIZE / 2;

	INT8U			ans_pos = 0;
	
// 	double           var=0;      //方差结果
 	
	while (1)
		{
		res 				= OSQPend(q_msg, 0, &err); //响应消息队列

		if (err != OS_ERR_NONE)
			{
#ifdef DEBUG
			printf("OSQPend err %d", err);
#endif
			}
		else 
			{
						
			for (handle_i = 0; handle_i < DATA_GROUP_SIZE; handle_i++)
				{
				time_data[handle_i] = res[handle_i];
				}

			quick_sort_time_data(time_data, DATA_GROUP_SIZE); //快速排序

			switch (mcb.measure_mode)
				{
				case wall:
					ans_pos = mid_pos; //结果取中值
					break;

				case line:
					ans_pos = 0; 
					break;

				default:
					break;
				} /* 将取得的中值加入ans_buf结果缓冲区 */

			OS_ENTER_CRITICAL();					//ans_buf	ans_cnt  ans_ite 为临界资源

			if (ans_cnt < ANS_BUF_SIZE)
				{
				ans_cnt++;							//更新缓冲区结果数
				}

			if (ans_ite == ANS_BUF_SIZE - 1) //如果缓冲区已满，覆盖最开始的数据
				{
				ans_buf[ans_ite]	= time_data[ans_pos];
				ans_ite 			= 0;
				}
			else 
				{
				ans_buf[ans_ite++]	= time_data[ans_pos];
				}

			OS_EXIT_CRITICAL();
			}

//		delay_ms(50);
		}
} /* AGC任务 */


void agc_task(void * pdata)
{
	OS_CPU_SR		cpu_sr = 0;

	while (1)
		{
		agc_control(cpu_sr);
		delay_ms(5);
		}
}


#ifdef ADJ

/* 调试校准任务，进行多阈值拟合曲线 */
/* 通过逐渐增大增益，获取不同阈值时间差下的距离误差 */
/* 输出打印阈值时间差和对应的距离误差 */
void adj_task(void * pdata)
{
	OS_CPU_SR		cpu_sr = 0;

	TIME_DATA * 	res;

	INT8U			err;

	TIME_DATA		time_data[DATA_GROUP_SIZE]; 	//待处理数据缓冲区

	int 			handle_i;

	INT32U			mid_pos = DATA_GROUP_SIZE / 2;

	double			distance;

	double			err_distance;

	double			standare_distance = STANDARD_DISTANCE;

	quantify			= MIN_GAIN;
	OS_ENTER_CRITICAL();
	setDacValueBin(quantify);						//初始最低增益	
	printf("standard distance set: %.2f\n", standare_distance);
	OS_EXIT_CRITICAL();

	while (1)
		{
		res 				= OSQPend(q_msg, 0, &err); //响应消息队列

		if (err != OS_ERR_NONE)
			{
#ifdef DEBUG
			OS_ENTER_CRITICAL();
			printf("OSQPend err %d", err);
			OS_EXIT_CRITICAL();
#endif
			}
		else 
			{
			for (handle_i = 0; handle_i < DATA_GROUP_SIZE; handle_i++)
				{
				time_data[handle_i] = res[handle_i];
				} /* 中值滤波 */

			quick_sort_time_data(time_data, DATA_GROUP_SIZE); //快速排序

			/* 将中值转化为距离 */
			distance			= (1.5 * (double) time_data[mid_pos].time_ps) / 10000;
			err_distance		= distance - STANDARD_DISTANCE; //距离误差
			OS_ENTER_CRITICAL();
			printf("%.2f,%d,,%d,%d,%d\n", err_distance, time_data[mid_pos].err_time1, \
				 time_data[mid_pos].err_time2, time_data[mid_pos].err_time3, quantify); //打印到串口
			OS_EXIT_CRITICAL();
			}
		
			if(ans_lineJudge == 5)	//5组数据之后增益增加10
			{
				quantify			+= INC_STEP;
				ans_lineJudge=0;
			}
			ans_lineJudge++;

		if (quantify > MAX_GAIN || time_data[mid_pos].err_time3 <950) //结束了，挂起任务
			{
			OS_ENTER_CRITICAL();
			printf("ADJ end\n");
			OS_EXIT_CRITICAL();
			OSTaskSuspend(GEN_TASK_PRIO);
			OSTaskSuspend(ADJ_TASK_PRIO);
			}
		else 
			{
			OS_ENTER_CRITICAL();
			setDacValueBin(quantify);				//扩大增益
			OS_EXIT_CRITICAL();
			}

		delay_ms(50);
		}
}


#endif



