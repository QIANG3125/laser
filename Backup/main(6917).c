

/* ������ V6 */
/* UC-OSII�汾 */
/* �������񡢲��������������񡢴�����������AGC���� У׼�������� */
/* ������Ϣ����ͨ�� */
/* ʹ��TDC-GPX2ʱ�����оƬ */
/* ����ֵ���У׼ */
/* �Զ��������(AGC) */
/* ��СĿ�����л� */
/* �Ż�����ʹ�ÿ��������㷨 */
/* date:2020.05.22 Emil: hdu_tangguodong@163.com */

/* ����ֵʱ��������룬���ٵ�������ֵʱ������˲�����֤��ֵʱ�����������Ψ�1�7
	�1�7�1�7��Ӧ */
/* �ṩ���Խ�����������������ִ���ʽ��ƽ������ֵ��ͨ���궨��ARGVѡ�� */
/* �������תdouble,֮ǰ����time_ps���������ݣ���С������ʧ */
/* ���õ���У׼���� */
/* date:2020.05.24 Emil: hdu_tangguodong@163.com */

/* 930~940 GAIN 1V/V */
/* 135�� 1.6V */
/* ����ֵ��ϲ���: ��ʹ��err_time3(Vth3 - Vth1) */
/* 0<err_time3<=1120 err = 2.71 */
/* 1120<err_time3<2000	err = -3.019+0.8541*ln(-10.9372*err_time3+11430) */
/* err_time>2000  err = 0.0004*err_time3+4.0737 */
/* todo : ����������㡣��ֵ2V���ϣ�err_time3 ��ֵ2V���£�err_time1 */
/* date:2020.05.25 Emil: hdu_tangguodong@163.com */

/* ���������ֵ������̽���Ƿ��������źŷ��أ���������������ź�����ʱ�ܹ���߲�1�7
	�1�7�1�7���� */
/* ���ԣ����Ŀ������ȶ������ߣ�ֻ����26�����ڣ�����1~2�ף��²�������̫�󣬸��ń1�7
	�1�7����� */
/* todo: ȷ����������ԭ�򣬱�Ҫʱ���Сǰ�����棬��AGC_LOW���ܣ����ܻ��ý������Ŀ
	�겻׼�� */
/* date:2020.05.26 Emil: hdu_tangguodong@163.com */

/* �����ԣ����ڲ�ͬ����Ŀ�꣬��ֵʱ����в�ͬ�����ر��ǽ�������ֵʱ��� */
/* ���Ҫ����е��ջ������ò������Ϊ��������Ӳ��������Ψһ�İ취 */
/* date:2020.05.29 Emil: hdu_tangguodong@163.com */

/* ��Ҫ��������߲�������������Ż�����ʱ��û������ */
/* �����Զ���λ�л�ģʽ������������ֵ��AGCģʽ ʵ�ⷢ�ֲ�ø�Զ�� ��Զ�ȶ��⵽
	1030�� */
/* date:2020.06.05 Emil: hdu_tangguodong@163.com */

/* ��Խ������ֹʧ�����ֵʱ�������������������������������
��С��ĳһ������ʱ������ֵʱ����������������1200~1500�������� */
/* date:2020.06.09 Emil: hdu_tangguodong@163.com */

/* �Ż�TDC-GPX2���������ṩһ�β�����ȡ�������������� */
/* date:2020.06.10 Emil: hdu_tangguodong@163.com */

/* �Ż�BUG ,stopͨ����ȡ���������ݣ���Ӧ�ü�ȥstartͨ����ȡ�ĵ�һ�����ݣ������ٶ�start
	ͨ������0 */
/* Ӳ�����⣺stopͨ������������и���ѹ���������ֲ������-0.3V�������д���������� 
	*/
/* ���˼·���Ӷ����ܵ��أ��Ե�ѹ����ǯλ Ŀǰ��·���Ҷ��ͣ��Ժ��л����ٲ����޸�
	Ӳ�� */
/* ����Ҫ������1������֮ǰ�Ľ��շ��仵�˻��˸���ͷ�ͽ��շ��䣬���ֺ�֮ǰ�Ļز���
	�Բ�ͬ��
		����������󣬵���ÿ���豸���ܶ�Ҫ�޸ĳ������ */
/* ����Ҫ������2�����ػ��ɸ���������Ӳ����������ģ���Ϊ���˸���ͷ�ͽ��շ���С��
	�����������λ�ô�ʮ�������ұ䵽��5��
		���Ҹ��������ƺ������ �²ⲻһ���� */
/* todo: ���ԸĽ�ȥ����������ķ�ʽ��һ�β����õ��ļ���������ֻ��һ������ȷ���ݣ� 
	*/

/* date:2020.06.15 Emil: hdu_tangguodong@163.com */

/* ���뿴�Ź����������ڴ�����������뵼��TDCоƬ��INTERUPT״̬һֱ����0���޷��������1�7
	�1�7�1�7FIFO���ݵ�ѭ�������³����ܷ� */
/* date:2020.06.19 Emil: hdu_tangguodong@163.com */

/* �����û��������ģʽ  �ǲ��߻��ǲ�ǽ */
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
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //�����ж����ȼ�����Ϊ��2��2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	delay_init();
	uart_init(9600);
	out_mode_set();
	tpl0202_init(); 								// ������ֵ
	KEY_Init();
	tlv5636_init(); 								// DAC�˿ڳ�ʼ��
	agc_init();
	adc_init(); 									// ADC��DMA��ʼ��
	laser_init();									//���ⷢ���ʼ��
	tdc_init(); 									//ʱ�����оƬ��ʼ��
	tdc_config();									//ʱ�����оƬ�������
	mto_init(&mcb);
	mto_add(60,70,80,line,adj_line_60_70_80);
	mto_add(60,70,80,wall,adj_wall_60_70_80);
//	mto_add(80,90,100,line,adj_line_80_90_100);
//	mto_add(80,90,100,wall,adj_wall_80_90_100);
	mto_start(CIRCULAR_LIST,CIRCULAR_LIST);

	setRefValue(REF1);								//set tlv5636 ref voltage mode
	setDacValueBin(quantify);						//��ʼ��DAC�����ѹ,Ĭ������
	OSInit();										//��ʼ��RTOS ���������߳�
	OSTaskCreate(start_task, (void *) 0, (OS_STK *) &START_TASK_STAK[START_STAK_SIZE - 1], START_TASK_PRIO); //������ʼ����
	OSStart();
}

void start_task(void * pdata)
{
	OS_CPU_SR		cpu_sr = 0; 					//���÷���3�����ж���Ҫ���м����

	/* �������ֵ���ֵ */
	PEAK_CONTROL		= 1;
	delay_ms(1);
	PEAK_CONTROL		= 0;
	AGC_EN				= 1;						//����AGC
	pdata				= pdata;					//pdataû��ʱ��ֹwaring
	q_msg				= OSQCreate((void * *) &Qstart[0], QSTART_SIZE); //������Ϣ����

//	IWDG_Init(4, 1000); 							//���ÿ��Ź� 1.6s 

	OS_ENTER_CRITICAL();							//�����ٽ��������ж�  //��ֹ�����жϴ�����´���ִ�У�����ͨ���жϵ�ʱ�����Ҫ���ж�

#ifdef ADJ //����У׼ģʽ

	OSTaskCreate(gen_task, (void *) 0, (OS_STK *) &GEN_TASK_STAK[GEN_STAK_SIZE - 1], GEN_TASK_PRIO); //������񣬴�tdcоƬ��ȡ����
	OSTaskCreate(adj_task, (void *) 0, (OS_STK *) &ADJ_TASK_STAK[ADJ_STAK_SIZE - 1], ADJ_TASK_PRIO); //У׼��������ϵ�ʱ����

#else

	OSTaskCreate(master_task, (void *) 0, (OS_STK *) &MASTER_TASK_STAK[MASTER_STAK_SIZE - 1], MASTER_TASK_PRIO); //�Ѳ������������ֵ��ϣ����ô��ڴ�����λ��
	OSTaskCreate(gen_task, (void *) 0, (OS_STK *) &GEN_TASK_STAK[GEN_STAK_SIZE - 1], GEN_TASK_PRIO); //�������
	OSTaskCreate(handle_task, (void *) 0, (OS_STK *) &HANDLE_TASK_STAK[HANDLE_STAK_SIZE - 1], HANDLE_TASK_PRIO); //��ֵ�˲�
	OSTaskCreate(agc_task, (void *) 0, (OS_STK *) &AGC_TASK_STAK[AGC_STAK_SIZE - 1], AGC_TASK_PRIO); //�Զ����������������û��
#endif

	OSTaskSuspend(START_TASK_PRIO); 				//������ʼ����
	OS_EXIT_CRITICAL(); 							//�˳��ٽ��������ж�
}

void master_task(void * pdata)
{
	OS_CPU_SR		cpu_sr = 0;
	TIME_DATA 		time_data;
	INT32U			time_ps = 0;
	INT32U			err_time = 0;
	INT8U			recive_flag = 0;				//�����ݱ��

//	int i=0;		//for
	//double d_distance = 0;
	CH_DATA 		ch_distance;

#ifdef AUTO_LEVEL_MOD

	/**************************auto level mode*********************************/
	double			org_data;

	AGC_EN				= 0;

	OSTaskSuspend(AGC_TASK_PRIO);					//����AGC����
	level				= 0;						//��ʼ����λ
	OS_ENTER_CRITICAL();
	quantify			= LEVEL[level]; 			//���õ�λ��Ӧ����
	setDacValueBin(quantify);
	OS_EXIT_CRITICAL();
	delay_ms(500);									//�ϵ�ȴ�500ms������

	while (1)
		{
		if (outmode == UARTbluetooth)
			{
			OS_ENTER_CRITICAL();
			printf("\n\nq:%d\n", quantify);
			OS_EXIT_CRITICAL();
			}

		/* ������� */
		OS_ENTER_CRITICAL();

		if (ans_cnt != 0) //�����������������
			{
#ifdef ARGV //�����ƽ��

			//�����������������ƽ��
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

#else //�������ֵ

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

			if (time_ps < TIME_PS_TH && err_time > 2200 && mcb.measure_mode==wall) //������ѡ����ֵʱ������
				{
				recive_flag = 0;
				}
			else if(err_time > 5000)	//��ֵʱ������5000�����ݶ��������޳���Ҫ
				{
					if((1.5 * (double) time_ps) / 10000 <400 && mcb.measure_mode==wall)		//�˴������˲�ǽ�ļ��ޣ����ߵļ�����δ���
						recive_flag = 0;
					else if((1.5 * (double) time_ps) / 10000 <100 && mcb.measure_mode==line)
						recive_flag = 0;
					else
						recive_flag = 1;
				}
			else 
				recive_flag = 1;



			
			ans_cnt 			= 0;			//����������
			ans_ite 			= 0;

			
#ifdef ARGV
			e_sum				= 0;
			d_sum				= 0;
#endif
			}
		else //������û�����ݣ����Ϊ0
			{
			
			recive_flag 		= 0;

			}

		OS_EXIT_CRITICAL();

		if (recive_flag) //������������
			{
			//�����ݣ���λ�̶�
			if (level_stable == 0)
				{
				level_stable		= 1;
				}

			no_data_cnt 		= 0;				//һ�������ݣ������ݼ�������

			/* time_psת����double */
			d_distance			= (1.5 * (double) time_ps) / 10000;
			org_data			= d_distance;

			//d_distance = u32_dou(distance);//ת��Ϊdouble
			if (outmode == UARTbluetooth)
				{
				OS_ENTER_CRITICAL();
				printf("ORG:%.2f\t", org_data);
				OS_EXIT_CRITICAL();
				}

			//ͨ����ֵ���У׼
#ifdef MULTI_VTH

			switch (mcb.measure_mode)
				{
				case wall: //��ǽģʽ
					if(outmode==UARTbluetooth)
						printf("wall\n");
						mto_adj(&time_data,&d_distance);
					break;

				case line: //����ģʽ
					if(outmode==UARTbluetooth)
						printf("line\n");
							mto_adj(&time_data,&d_distance);
					break;

				default:
					break;
				}

#endif

			if (outmode == UARTbluetooth) //�������ֵ��������ʽ
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
			else //���������Ҫ��ĸ�ʽ�������
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
		else //û������
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

			//��λ�л�
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
			else //��λ�ѹ̶��������û�������ۼƵ�һ���������������õ�λ
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
	INT8U			master_cnt = 0; 				//master taskѭ������ 
	INT8U			res_cnt = 0;					//��Զ��Ŀ���������еĽ������
	INT8U			detect_cnt = 0; 				//�����ֵ��Զ��Ŀ���������еļ���

#ifdef ARGV
	INT32U			e_sum = 0;						//��ֵʱ������
	INT8U			master_i;
	long			d_sum = 0;						//�������

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


		/* Զ��Ŀ���л��ж� */
		/* ��Ϊ�������ᵼ�½���������ʧЧ����˱������û��Ƿ���ԶĿ��ʱ�ĸ����1�7
			�1�7�л����˽�Ŀ�꣬����һֱû������ */
		/* ��2��û��������������0.5���л�����ʼ�������Ƿ�����ԶĿ���л�����Ŀ�� 
			*/
		if (master_cnt == 4) //2���������,�����Ӧʱ�䲻����4��
			{
			if (res_cnt == 0 && detect_cnt == 0) //û�н��
				{
				if (AGC_EN == 1)
					{
					AGC_EN				= 0;
					OSTaskSuspend(AGC_TASK_PRIO);	//����AGC����

					if (outmode == UARTbluetooth)
						{
						OS_ENTER_CRITICAL();
						printf("AGC task suspend\n");
						OS_EXIT_CRITICAL();
						}
					}

				old_q				= quantify; 	//����֮ǰ��ֵ������ر�AGC��û���ݣ�˵����û��̽�⵽Ŀ���Ŀ��̫Զ���ٻָ�֮ǰ������ֵ
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
			if (ans_cnt == 0 && detect_cnt == 0) //�����л�����Ŀ����
				{
				quantify			= old_q;		//�ָ�����ֵ
				setDacValueBin(quantify);
				AGC_EN				= 1;
				OSTaskResume(AGC_TASK_PRIO);		//�ָ�AGC����

				if (outmode == UARTbluetooth)
					{
					OS_ENTER_CRITICAL();
					printf("AGC resume\n");
					OS_EXIT_CRITICAL();
					}
				}

			old_q				= 0;
			}

		/* ������� */
		OS_ENTER_CRITICAL();

		if (ans_cnt != 0) //�����������������
			{
#ifdef ARGV //�����ƽ��

			//�����������������ƽ��
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

#else //�������ֵ

			quick_sort_time_data(ans_buf, ans_cnt);

			//distance = ans_buf[ans_cnt/2].i_data;
			time_ps 			= ans_buf[ans_cnt / 2].time_ps;
			err_time			= ans_buf[ans_cnt / 2].err_time3;
#endif

			if (time_ps < 150000 && err_time > 2200) //������٣�˵���Ѳ⣬�������ֽ�����������ȥ��������ɱ�����Ź���
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
		else //������û�����ݣ����Ϊ0
			{
			recive_flag 		= 0;
			}

		OS_EXIT_CRITICAL();

		/* У׼�������� */
		if (recive_flag)
			{
			res_cnt++;								//���ݼ�����1

			/* time_psת����double */
			d_distance			= (1.5 * (double) time_ps) / 10000;

			//d_distance = u32_dou(distance);//ת��Ϊdouble
			//ͨ����ֵ���У׼
#ifdef MULTI_VTH
			d_distance			= multi_thread_adj(&d_distance, &time_ps);
#endif

			if (outmode == UARTbluetooth) //�������ֵ��������ʽ
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
			else //���������Ҫ��ĸ�ʽ�������
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

			/* �ж�AGC�Ƿ��� */
			if (d_distance > 21.0)
				{
				if (AGC_EN == 0)
					{
					AGC_EN				= 1;
					quantify			= QUANTIFY_INIT; //����AGC����ǰ��ʼ������
					OSTaskResume(AGC_TASK_PRIO);	//�ָ�AGC����

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
					OSTaskSuspend(AGC_TASK_PRIO);	//����AGC����
					quantify			= QUANTIFY_INIT;
					setDacValueBin(quantify);		//��������

					if (outmode == UARTbluetooth)
						{
						OS_ENTER_CRITICAL();
						printf("AGC task suspend\n");
						OS_EXIT_CRITICAL();
						}

					}
				}

			}
		else //û��������� //0xFF 0xFF 0xFF
			{
			if (detect_val != 0) //̽����ֵ̽�⵽�ź�����//��ʱAGC���ڸ�����������ź����壬�����ϲ�ȥ������AGC����
				{
				detect_cnt++;

				if (AGC_EN == 1)
					{
					AGC_EN				= 0;
					OSTaskSuspend(AGC_TASK_PRIO);	//����AGC����

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
					quantify			+= 100; 	//�ֶ�����100����
					}

				setDacValueBin(quantify);			//��������
				OS_EXIT_CRITICAL();
				}
			else 
				{
				//û�������������AGC��������
				if (AGC_EN == 0)
					{
					AGC_EN				= 1;
					quantify			= QUANTIFY_INIT; //����AGC����ǰ��ʼ������
					OSTaskResume(AGC_TASK_PRIO);	//�ָ�AGC����

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

		/* ��ֹ�ز����ٵ��·�ֵ���ֲ�ס�����з������������� */
		if (AGC_EN)
			{
			int 			i;

			PEAK_CONTROL		= 1;				//�ŵ�
			i					= 0;
			PEAK_CONTROL		= 0;				//��ֵ����

			for (i = 0; i < 50; i++) //��絽��ֵ,���ٷ���������
				{
				laser_plus();
				delay_ms(1);
				}
			}

		delay_ms(500);								//ÿ��0.5���ѯ��������
		master_cnt++;
		}

#endif

	/*************************************************************************/
}




/* ���������񣬻�ȡ����ʱ�䣬��ֵʱ��� */
void gen_task(void * pdata)
{
	OS_CPU_SR		cpu_sr = 0;

	/* TDC-GPX2 ������� */
	result			measure_data_arr[MEASURE_DATA_ARR_SIZE];
	INT8U			res_cnt;
	INT8U			i_gen;
	int			time_ps;						//����ʱ�� ps
	int			time_ps_detect; 				//̽����ֵ��������
    INT32U          distance;
	int 			err_time1, err_time2, err_time3; //��ֵʱ���
	long			start_res;
	long			start_index;

	//CH_DATA org_data[DATA_GROUP_SIZE];//���ݻ������飬�������ʱͨ����Ϣ���з��͸�����������
	TIME_DATA		org_data[80];		//���ݻ������飬�������ʱͨ����Ϣ���з��͸�����������
	TIME_DATA		time_data;
//	TIME_DATA		over_dis_data[3];	//Զ�������ݻ�����
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
//		IWDG_Feed();								//ι�� ��ֹTDCоƬ�ܷ�
		start_res			= measure_data_arr[0].stopresult[0];
		start_index 		= measure_data_arr[0].reference_index[0];
		
		for (i_gen = 0; i_gen < res_cnt; i_gen++)
			{
			//������ֵ����ʱ�����
			time_ps = measure_data_arr[i_gen].stopresult[1] -start_res + \
                (measure_data_arr[i_gen].reference_index[1] -start_index) *refclk_divisions;

			//̽����ֵ����ʱ��
			time_ps_detect = measure_data_arr[i_gen].stopresult[2] -start_res + \
                (measure_data_arr[i_gen].reference_index[2] -start_index) *refclk_divisions;

			//vth1--vth3��ֵʱ������
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
			


			if(0)	//�ɼ����ǿ���������ݽ϶�
			{
				//ǿ����Ž϶�,��5�����ݽ��бȽ�
				if (level_stable == 0)
				{
					level_stable		= 1; 
				}
				
				/*�����˳����˳���1000�׷�Χ��,�����Դ��������*/
				if ((quantify > GAIN_TH) && (time_ps < TIME_PS_TH)) //����󣬾������ȥ��,�ɵ��������� (�Կ�ɢ�� 11����������)
				{
					data_vaild = 0;					
				}
				else if (err_time3 > MAX_ERR_TIME || err_time3 <= MIN_ERR_TIME) //��ֵʱ��������ȥ��
				{
					data_vaild	= 0;
				}
				else if(time_ps > TIME_1000M || time_ps <= MIN_TIME)	//��1000��������
				{
					data_vaild	= 0;
				}
				else 
				{
					data_vaild	= 1;
				}
//				distance=(1.5 * (double) time_ps) / 10000;
				/*�ɼ�5�����ݣ�������ȫ������һ��������*/
				if(data_vaild == 1)	//����Ч���ݱ�����һ��������
					{
						create_time_data(&time_data, time_ps, err_time1, err_time2, err_time3);
						org_data[all_cnt++] = time_data;	//װ�ص����ݻ�����			
						measure_cycle++;
					}
				else
					{
						OS_ENTER_CRITICAL();
						printf("measure_cycle:%d.\r\n",measure_cycle);
						OS_EXIT_CRITICAL();
					}
				/*�ɼ���5�����ݣ��������������*/
				if(measure_cycle==5 && i_gen==(res_cnt-1))
					{
						quick_sort_time_data(org_data, all_cnt); //��������

						
						/*�����������ݽ����жϣ�����5��������������ж�Ϊ��Ч���ݽ������*/
						
						data_to_mask =data_filter(org_data,all_cnt,20000);
						
						OS_ENTER_CRITICAL();					//ans_buf	ans_cnt  ans_ite Ϊ�ٽ���Դ
						if (ans_cnt < ANS_BUF_SIZE)
							{
							ans_cnt++;							//���»����������
							}
						else
							{
								ans_cnt=0;
							}
			
						if (ans_ite == ANS_BUF_SIZE - 1) //��������������������ʼ������
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
					/*	��Чֵɸѡ */
				//������Чɸѡ,�ų����Կ���ֵ��ͨ������;���ɸѡ
				//if((quantify>GAIN_TH)&&(distance<DISTANCE_TH))
#ifdef ADJ
				distance=(1.5 * (double) time_ps) / 10000;
				if ( distance>= STANDARD_DISTANCE)
					data_vaild = 1; //ɸ��start֮ǰ����������
				else 
					data_vaild = 0;

				//�˳���������
				if ((quantify > GAIN_TH) && (time_ps < TIME_PS_TH) && err_time3 > 2000) //����󣬾������ȥ��,�ɵ��������� (�Կ�ɢ�� 11����������)
				{
					data_vaild = 0;					 
				}
				else if (err_time3 > MAX_ERR_TIME || err_time3 < MIN_ERR_TIME) //��ֵʱ��������ȥ��
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
				
				if(distance > 35 && distance < 60)		//��ֵ���ʱ���ܿ��������
				{
					data_vaild = 0;
				}

#else

#ifdef DUG
				/*�˳�ǰ����*/
				OS_ENTER_CRITICAL();
				printf("\n*******before filter********");
				printf("this cycle:%d.\n",res_cnt);
				printf("q:%d.\n",quantify);
				printf("D:%.2f\t\tet3:%d.\n",(1.5 * (double) time_ps) / 10000,err_time3);
				OS_EXIT_CRITICAL(); 
#endif


				//�˳���������
				if ((quantify > GAIN_TH) && (time_ps < TIME_PS_TH)) //����󣬾������ȥ��,�ɵ��������� (�Կ�ɢ�� 11����������)
				{
					data_vaild = 0;					
				}
				else if (err_time3 > MAX_ERR_TIME || err_time3 <= MIN_ERR_TIME) //��ֵʱ��������ȥ��
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
	//			����˳����ź������
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
					case wall: //��ǽģʽ����ɸѡ
						if(quantify<=GAIN_TH && (time_ps < TIME_PS_TH2) && err_time3>1600) //����֮�ڲ�ǽ,�����޳�  !!!ע�����50�ײ��߱�ɸ��ȥ��
						{
							data_vaild = 0;
						}
						break;

					case line: //����ģʽ����ɸѡ
						
//						if ((err_time3 < 2200 | err_time3 > 3500)&& time_ps < 4*TIME_PS_TH) //�޳�20������ֵʱ�����2000���µĲ���
//							{ 
////								data_vaild = 0;
//							}
						if(err_time3<1100)	//������Ϻ����Ǵ�1100��ʼ
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
				
				if (data_vaild == 0) //��ǰû������
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
								OS_ENTER_CRITICAL();					//ans_buf	ans_cnt  ans_ite Ϊ�ٽ���Դ
								if (ans_cnt < ANS_BUF_SIZE)
									{
									ans_cnt++;							//���»����������
									}
								else
									{
										ans_cnt=0;
									}
					
								if (ans_ite == ANS_BUF_SIZE - 1) //��������������������ʼ������
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

						//����ǰ������ʱ������
						cur_timedata.distance=distance;
						cur_timedata.timedata=time_data;

						//�жϵ�ǰ��������һ�����ݵĲ��죬����С�������������������һ�λ�������
						if(abs(cur_timedata.distance-old_timedata.distance)<2)
							{
								OS_ENTER_CRITICAL();					//ans_buf	ans_cnt  ans_ite Ϊ�ٽ���Դ
								if (ans_cnt < ANS_BUF_SIZE)
									{
									ans_cnt++;							//���»����������
									}
								else
									{
										ans_cnt=0;
									}
					
								if (ans_ite == ANS_BUF_SIZE - 1) //��������������������ʼ������
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
						
						org_data[org_data_ite++] = time_data;	//װ�ص����ݻ�����			
						
						if(org_data_ite == DATA_GROUP_SIZE)
						{
							org_data_ite		= 0;

							OSQPost(q_msg, org_data);			//������Ϣ����
						}
					}
				

				
				}


			}
			
		delay_ms(10);
		}
} /* �����ߴ���������ֵ�˲� */

/*
��ֵ�V��
*/
void handle_task(void * pdata)
{
	OS_CPU_SR		cpu_sr = 0;

	TIME_DATA * 	res;

	INT8U			err;

	TIME_DATA		time_data[DATA_GROUP_SIZE]; 	//���������ݻ�����

	int 			handle_i;

	INT8U			mid_pos = DATA_GROUP_SIZE / 2;

	INT8U			ans_pos = 0;
	
// 	double           var=0;      //������
 	
	while (1)
		{
		res 				= OSQPend(q_msg, 0, &err); //��Ӧ��Ϣ����

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

			quick_sort_time_data(time_data, DATA_GROUP_SIZE); //��������

			switch (mcb.measure_mode)
				{
				case wall:
					ans_pos = mid_pos; //���ȡ��ֵ
					break;

				case line:
					ans_pos = 0; 
					break;

				default:
					break;
				} /* ��ȡ�õ���ֵ����ans_buf��������� */

			OS_ENTER_CRITICAL();					//ans_buf	ans_cnt  ans_ite Ϊ�ٽ���Դ

			if (ans_cnt < ANS_BUF_SIZE)
				{
				ans_cnt++;							//���»����������
				}

			if (ans_ite == ANS_BUF_SIZE - 1) //��������������������ʼ������
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
} /* AGC���� */


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

/* ����У׼���񣬽��ж���ֵ������� */
/* ͨ�����������棬��ȡ��ͬ��ֵʱ����µľ������ */
/* �����ӡ��ֵʱ���Ͷ�Ӧ�ľ������ */
void adj_task(void * pdata)
{
	OS_CPU_SR		cpu_sr = 0;

	TIME_DATA * 	res;

	INT8U			err;

	TIME_DATA		time_data[DATA_GROUP_SIZE]; 	//���������ݻ�����

	int 			handle_i;

	INT32U			mid_pos = DATA_GROUP_SIZE / 2;

	double			distance;

	double			err_distance;

	double			standare_distance = STANDARD_DISTANCE;

	quantify			= MIN_GAIN;
	OS_ENTER_CRITICAL();
	setDacValueBin(quantify);						//��ʼ�������	
	printf("standard distance set: %.2f\n", standare_distance);
	OS_EXIT_CRITICAL();

	while (1)
		{
		res 				= OSQPend(q_msg, 0, &err); //��Ӧ��Ϣ����

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
				} /* ��ֵ�˲� */

			quick_sort_time_data(time_data, DATA_GROUP_SIZE); //��������

			/* ����ֵת��Ϊ���� */
			distance			= (1.5 * (double) time_data[mid_pos].time_ps) / 10000;
			err_distance		= distance - STANDARD_DISTANCE; //�������
			OS_ENTER_CRITICAL();
			printf("%.2f,%d,,%d,%d,%d\n", err_distance, time_data[mid_pos].err_time1, \
				 time_data[mid_pos].err_time2, time_data[mid_pos].err_time3, quantify); //��ӡ������
			OS_EXIT_CRITICAL();
			}
		
			if(ans_lineJudge == 5)	//5������֮����������10
			{
				quantify			+= INC_STEP;
				ans_lineJudge=0;
			}
			ans_lineJudge++;

		if (quantify > MAX_GAIN || time_data[mid_pos].err_time3 <950) //�����ˣ���������
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
			setDacValueBin(quantify);				//��������
			OS_EXIT_CRITICAL();
			}

		delay_ms(50);
		}
}


#endif



