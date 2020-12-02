

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
#include "includes.h"

#include "spi.h"
#include "tdc_gpx2.h"
#include "laser.h"
#include "tpl0202.h"
#include "tlv5636.h"
#include "agc.h"
#include "key.h"
#include "wdg.h"

//#define DEBUG

/* У׼����ģʽ�������� */
#define STANDARD_DISTANCE		0 //����Ŀ���׼����
#define INC_STEP				10		   //�������Ӳ���
#define MIN_GAIN				920 	  //��С���� quantify DAC����ֵ
#define MAX_GAIN				1350	   //�������

/**********************************************/
/* �궨��ģʽ���� */
/* ͨ��ע�ͻ����ѡ��ģʽ */
//#define ADJ //У׼����ģʽ ,��������������У׼ģʽ
#define MULTI_VTH //����������ѽ�����ж���ֵ���У׼

//#define AGC_LOW	//�������AGC_LOW,AGC���ƻὫ����CONTROL_MAX_VOLTAGE��ʱ��������棬����ֻ����������
#define AUTO_LEVEL_MOD //�Զ���λ�л�ģʽ ��������ΪAGCģʽ

//#define ARGV	//�Ƿ񽫻����������ƽ������Ȼȡ��ֵ

/**********************************************/

/* ��������ת��Ϊ3���ֽڱ�ʾ */
/* ��ֵʱ��� */
typedef struct CHANGE_3CHAR_DATA
{
	/* h_data : �������ֽ� */
	/* l_data ���������ֽ� */
	/* f_data : С��λ */
	INT8U			h_data;
	INT8U			l_data;
	INT8U			f_data; 						//С��λΪ С��*256��ʾ ����0.01*256 = 2 
	INT32U			err_time;
} CH_DATA;

/* INT32U��ʾ������ */
/* ��ֵʱ��� */
typedef struct CHANGE_INT_DATA
{
	INT32U			i_data; 						//���� ��8λС���������ʾ����
	INT32U			err_time;
} INT_DATA;


/* ����ʱ�����ֵʱ��� */
typedef struct TIME_PS_DATA
{
	INT32U			time_ps;
	INT32U			err_time1;
	INT32U			err_time2;
	INT32U			err_time3;
} TIME_DATA;


//#define MAX_LEVEL 8
///* ���浵λ�� */
//const INT16U LEVEL[MAX_LEVEL] = 
//{
//	1100,1200,1300,1400,1500,1600,1700,1800
//};
#define MAX_LEVEL				14

/* ���浵λ�� */
const INT16U	LEVEL[MAX_LEVEL] =
{
	1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000, 2200, 2300, 2500
};


enum 
{
	wall,											//��ǽģʽ
	line											//����ģʽ
};


//#define MAX_LEVEL 1
///* ���浵λ�� */
//const INT16U LEVEL[MAX_LEVEL] = 
//{
//	1300
//};
INT8U			level = 0; //��λ

//UCOS ��������
//���ÿ�ʼ�������ȼ�
#define START_TASK_PRIO 		10//һ����10�����ȼ�����ʼ��������ȼ����Ϊ10
#define START_STAK_SIZE 		64//���ÿ�ʼ�����ջ��С
OS_STK			START_TASK_STAK[START_STAK_SIZE]; //��ʼ�����ջ

//���������߳����ȼ����
#define MASTER_TASK_PRIO		5
#define MASTER_STAK_SIZE		512
__align(8)
OS_STK			MASTER_TASK_STAK[MASTER_STAK_SIZE];

//���ü���������������
#define GEN_TASK_PRIO			6
#define GEN_STAK_SIZE			512
OS_STK			GEN_TASK_STAK[GEN_STAK_SIZE];

//�������ݴ�������������
#define HANDLE_TASK_PRIO		7
#define HANDLE_STAK_SIZE		128
__align(8)
OS_STK			HANDLE_TASK_STAK[HANDLE_STAK_SIZE]; //ʹ��printf��ӡ�����������ڣ���Ҫ�����ջ8�ֽڶ���

//����AGC��������
#define AGC_TASK_PRIO			8
#define AGC_STAK_SIZE			64
OS_STK			AGC_TASK_STAK[AGC_STAK_SIZE];

#ifdef ADJ

//���õ���У׼���� 
#define ADJ_TASK_PRIO			4
#define ADJ_STAK_SIZE			64
__align(8)
OS_STK			ADJ_TASK_STAK[ADJ_STAK_SIZE];

#endif

void start_task(void * pdata);
void master_task(void * pdata);
void gen_task(void * pdata);
void handle_task(void * pdata);
void agc_task(void * pdata);
void adj_task(void * pdata);

#define DATA_GROUP_SIZE 		5 //ÿһ��������ݳ���
#define QSTART_SIZE 			256    //��Ϣ����ָ�����黺������С
void *			Qstart[QSTART_SIZE]; //��Ϣ����ָ������
OS_EVENT *		q_msg; //��Ϣ����

#define ANS_BUF_SIZE			10	 //��������������С

//INT_DATA ans_buf[ANS_BUF_SIZE];//������������
TIME_DATA		ans_buf[ANS_BUF_SIZE]; //������������

//INT32U err_buf[ANS_BUF_SIZE];//�����ֵʱ������
INT8U			ans_ite = 0; //��ǰд��λ��
INT8U			ans_cnt = 0; //�������н������

/* λ������ */
#define setbit(x, y)			(x)|=(1<<(y))
#define clrbit(x, y)			(x)&=~(1<<(y))
#define reversebit(x, y)		(x)^=(1<<(y))
#define getbit(x, y)			((x)>>(y)&1)

/* ��ʼ������ֵ���� */
#define QUANTIFY_INIT			1200  //��ʼ������ֵ DAC
#define START_THREAD			80			
#define STOP_THREAD1			60
#define STOP_THREAD2			80
#define STOP_THREAD3			80

/* ����ɸѡУ׼���� */
#define GAIN_TH 				1200	  //������ֵ
#define DISTANCE_TH 			21	  //��������
#define TIME_PS_TH				100000 //15�׵�ʱ�� 
#define MAX_TIME				10000000 //������ʱ�� 1500�׵�ʱ�� ps
#define MIN_TIME				0		 //��С����ʱ��  
#define DETECT_MIN_TIME 		300000 //̽����ֵ��С����ʱ�� 45m
#define MAX_ERR_TIME			10000 //�����ֵʱ���
#define MIN_ERR_TIME			0	 //��С��ֵʱ���

/* AGC ���� */
unsigned char	AGC_EN;
unsigned short	quantify = QUANTIFY_INIT; //DAC ����ֵ
INT32U			detect_val = 0; //�����ֵ��⵽��ֵ time_ps

/* AGC Control parameter	 */
#define CONTROL_MAX_VOLTAGE 	3
#define FLOW_RANGE				0.2
#define CONTROL_DIFF_TIME		2000
#define TIME_FLOW_RANGE 		200

#define MAX_NO_DATA_CNT 		4

const int		refclk_divisions = 125000; //8M����  ,TDC�ο�ʱ��

unsigned char	outmode = 0; //�������ģʽ 0:�����������ĸ�ʽ  1:��ӡ�����ڵ����ַ���ʽ
unsigned char	measure_mode = wall; //����ģʽ������ģʽ����ǽģʽ

void out_mode_set(void);
double ch_data_dou(CH_DATA * pch_data);
CH_DATA dou_ch_data(double * d_data, INT32U err_time);
INT32U ch_data_u32(CH_DATA * pch_data);
double u32_dou(INT32U data);
INT32U dou_u32(double * d_data);
void quick_sort_int_data(INT_DATA * arr, INT32U size); //��������
void quick_sort_time_data(TIME_DATA * arr, INT32U size);
void print_ch_data(CH_DATA * ch_data);
void print_int_data(INT32U int_data);
double my_log(double a);
void agc_control(OS_CPU_SR cpu_sr);
void create_time_data(TIME_DATA * ptime_data, INT32U time_ps, INT32U err_time1, INT32U err_time2, INT32U err_time3);
double multi_thread_adj(double * org, unsigned int * time);

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

	// ������10202 ���õ�ѹ��ֵ
	write_wa(STOP_THREAD1, &TPL1_CS, &TPL1_CLK, &TPL1_D); //set voltage thread of stop1 pulse
	write_wb(START_THREAD, &TPL1_CS, &TPL1_CLK, &TPL1_D); //set voltage thread of start pulse
	write_wa(STOP_THREAD2, &TPL2_CS, &TPL2_CLK, &TPL2_D); //set voltage thread of stop2 pulse
	write_wb(STOP_THREAD3, &TPL2_CS, &TPL2_CLK, &TPL2_D); //set voltage thread of stop3 pulse

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

	IWDG_Init(4, 1000); 							//���ÿ��Ź� 1.6s 

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


/* ������ */
#define coe1					2.4
#define coe2					2.2
double			d_distance = 0;
INT8U			level_stable = 0;
INT8U			no_data_cnt = 0;


void master_task(void * pdata)
{
	OS_CPU_SR		cpu_sr = 0;
	INT32U			time_ps = 0;
	INT32U			err_time = 0;
	INT8U			recive_flag = 0;				//�����ݱ��

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
		if (outmode == 1)
			{
			OS_ENTER_CRITICAL();
			printf("q:%d\n", quantify);
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

			//distance = ans_buf[ans_cnt/2].i_data;
			time_ps 			= ans_buf[ans_cnt / 2].time_ps;
			err_time			= ans_buf[ans_cnt / 2].err_time3;
#endif

			if (time_ps < TIME_PS_TH && err_time > 2200) //������ѡ����ֵʱ������
				{
				recive_flag = 0;
				}
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
			//printf("de:%.2f\n",(1.5*(double)detect_val)/10000);
			//if(detect_val>100000)
			recive_flag 		= 0;

			//else
			//{
			//	recive_flag = 1;
			//time_ps = detect_val;
			//detect_val = 0xFFFFFFFF;
			//printf("de:%.2f\n",(1.5*(double)time_ps)/10000);
			//}
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
			if (outmode == 1)
				{
				OS_ENTER_CRITICAL();
				printf("\n\nORG:%.2f\n", org_data);
				OS_EXIT_CRITICAL();
				}

			//ͨ����ֵ���У׼
#ifdef MULTI_VTH

			switch (measure_mode)
				{
				case wall: //��ǽģʽ
					if (org_data < 50)
						{
						d_distance			= d_distance - coe1;
						}
					else 
						{
						d_distance			= multi_thread_adj(&org_data, &err_time);
						}

					break;

				case line: //����ģʽ
					d_distance = multi_thread_adj(&org_data, &err_time) - 0.3;
					break;

				default:
					break;
				}

#endif

			if (outmode == 1) //�������ֵ��������ʽ
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
			if (outmode == 0)
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
				if (no_data_cnt < MAX_NO_DATA_CNT)
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
		if (outmode == 1)
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

					if (outmode == 1)
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

				if (outmode == 1)
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

			if (outmode == 1) //�������ֵ��������ʽ
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

					if (outmode == 1)
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

					if (outmode == 1)
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

					if (outmode == 1)
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

					if (outmode == 1)
						{
						OS_ENTER_CRITICAL();
						printf("AGC task resume2\n");
						OS_EXIT_CRITICAL();
						}
					}
				}

			if (outmode == 0)
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


#define MEASURE_DATA_ARR_SIZE	30

/* ���������񣬻�ȡ����ʱ�䣬��ֵʱ��� */
void gen_task(void * pdata)
{
	OS_CPU_SR		cpu_sr = 0;

	/* TDC-GPX2 ������� */
	result			measure_data_arr[MEASURE_DATA_ARR_SIZE];
	INT8U			res_cnt;
	INT8U			i_gen;
	INT32U			time_ps;						//����ʱ�� ps
	INT32U			time_ps_detect; 				//̽����ֵ��������
	int 			err_time1, err_time2, err_time3; //��ֵʱ���
	long			start_res;
	long			start_index;

	//double distance;//δ�������
	//CH_DATA org_data[DATA_GROUP_SIZE];//���ݻ������飬�������ʱͨ����Ϣ���з��͸�����������
	TIME_DATA		org_data[DATA_GROUP_SIZE];		//���ݻ������飬�������ʱͨ����Ϣ���з��͸�����������

	//CH_DATA ch_data;
	TIME_DATA		time_data;
	unsigned int	org_data_ite = 0;
	INT8U			data_vaild;

	detect_val			= 0xFFFFFFFF;

	//test 
	//OSTaskSuspend(GEN_TASK_PRIO);
	while (1)
		{
		res_cnt 			= tdc_measure_group(measure_data_arr);
		IWDG_Feed();								//ι�� ��ֹTDCоƬ�ܷ�
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

			//OS_ENTER_CRITICAL();
			//printf("i_gen:%d,d:%.2f,err_time:%d\n",i_gen,(1.5*(double)time_ps)/10000,err_time3);
			//OS_EXIT_CRITICAL();

			/*	��Чֵɸѡ */
			//������Чɸѡ,�ų����Կ���ֵ��ͨ������;���ɸѡ
			//if((quantify>GAIN_TH)&&(distance<DISTANCE_TH))
#ifdef ADJ

			if ((1.5 * (double) time_ps) / 10000 >= STANDARD_DISTANCE)
				data_vaild = 1; //ɸ��start֮ǰ����������
			else 
				data_vaild = 0;

#else

			switch (measure_mode)
				{
				case wall: //��ǽģʽ����ɸѡ
					if ((quantify > GAIN_TH) && (time_ps < TIME_PS_TH)) //����󣬾������ȥ��,�ɵ���������
						{
						data_vaild = 0;
						}
					else if (err_time3 > MAX_ERR_TIME || err_time3 < MIN_ERR_TIME) //��ֵʱ��������ȥ��
						{
						//if(time_ps<200000&&err_time3>2000)//�ж��ǲ�ǽģʽ�½����������  30�ף�ʱ���2000
						//{
						//	measure_mode = line;//�л�������ģʽ
						//}
						data_vaild	= 0;
						}
					else 
						{
#ifdef AUTO_LEVEL_MOD

						if (time_ps < 400000&&data_vaild==1) //��ǽģʽ������Ч<60�ף��ر������л�
							{
							OS_ENTER_CRITICAL();
							level_stable		= 1;
							level				= 0;
							quantify			= LEVEL[level];
							setDacValueBin(quantify);
							OS_EXIT_CRITICAL();

							}

#endif

						data_vaild			= 1;
						}

					break;

				case line: //����ģʽ����ɸѡ
					if ((quantify > GAIN_TH) && (time_ps < TIME_PS_TH) && err_time3 > 3000) //����󣬾�����ģ���ֵʱ�����ȥ��
						{
						data_vaild			= 0;
						}
					else if (err_time3 > MAX_ERR_TIME || err_time3 < 2000) //��ֵʱ��������ȥ��������ģʽֻȡ������û���͵Ļز�
						{ //if(err_time3<1100)//�ж����ڲ���ģʽ�²���ǽ

						//{
						//	measure_mode = wall;//�л�����ǽģʽ
						//}
#ifdef AUTO_LEVEL_MOD
						OS_ENTER_CRITICAL();
						level_stable		= 1;	//����ģʽ����ǿ�ź�ʱ���̶�����

						//level = 0;
						//quantify = LEVEL[level];
						//setDacValueBin(quantify);
						no_data_cnt 		= 0;	//����ģʽ����ǿ�ź�ʱ�����û���ݼ���������master�����������浵λ�л�
						OS_EXIT_CRITICAL();
#endif

						data_vaild			= 0;
						}
					else 
						{
#ifdef AUTO_LEVEL_MOD

						if (time_ps < 200000 && quantify < 1300) //����ģʽ��<30M �ر������л�
							{
							OS_ENTER_CRITICAL();
							level_stable		= 1;
							level				= 0;
							quantify			= LEVEL[level];
							setDacValueBin(quantify);
							OS_EXIT_CRITICAL();
							}

#endif

						data_vaild			= 1;
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

			if (time_ps > MAX_TIME || time_ps <= MIN_TIME || data_vaild == 0) //��ǰû������
				{
				continue;
				}

			create_time_data(&time_data, time_ps, err_time1, err_time2, err_time3);
			org_data[org_data_ite++] = time_data;	//װ�ص����ݻ�����			

			if (org_data_ite == DATA_GROUP_SIZE) //���ݻ������������
				{
				org_data_ite		= 0;
				OSQPost(q_msg, org_data);			//������Ϣ����
				}
			}

		delay_ms(10);
		}
} /* �����ߴ���������ֵ�˲� */

/*
��ָ�V��
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

			switch (measure_mode)
				{
				case wall:
					ans_pos = mid_pos; //���ȡ��ֵ
					break;

				case line:
					ans_pos = 0; //���ȡ
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

		delay_ms(50);
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
			printf("%.2f,%d,,%d,%d,%d\n", err_distance, time_data[mid_pos].err_time1, 
				\ time_data[mid_pos].err_time2, time_data[mid_pos].err_time3, quantify); //��ӡ������
			OS_EXIT_CRITICAL();
			}

		quantify			+= INC_STEP;

		if (quantify > MAX_GAIN) //�����ˣ���������
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

/* �������������ʽ */
/* outmodeΪ1ʱ����ַ�����ʽ */
/* outmodeΪ0ʱ����ֽ�����ʽ */
/* MODEΪ0ʱ��ǽģʽ */
/* MODEΪ1ʱ����ģʽ */
void out_mode_set()
{ /* ���ģʽ�趨 */
	outmode 			= 1;

	if (MODE == 1)
		{
		measure_mode		= line;
		}
	else 
		{
		measure_mode		= wall;
		}
} /* CH_DATAתdouble */


double ch_data_dou(CH_DATA * pch_data)
{
	double			ans;

	INT16U			i_temp = 0;

	ans 				= (double)
	pch_data->f_data / 256;
	i_temp				= ((INT16U) (pch_data->h_data) << 8) +pch_data->l_data;
	return ans + (double)
	i_temp;
} /* doubleתCH_DATA */


CH_DATA dou_ch_data(double * d_data, INT32U err_time)
{
	CH_DATA 		ch_data;

	INT32U			i_temp;

	i_temp				= *d_data;					//��ȡ����λ
	ch_data.h_data		= (i_temp >> 8) & 0xFF;
	ch_data.l_data		= i_temp & 0xFF;
	ch_data.f_data		= (INT8U) ((*d_data - i_temp) * 256);
	ch_data.err_time	= err_time;
	return ch_data;
} /* CH_DATAתu32 */


INT32U ch_data_u32(CH_DATA * pch_data)
{
	INT32U			ans = 0;

	ans 				+= pch_data->h_data;
	ans 				= ans << 8;
	ans 				+= pch_data->l_data;
	ans 				= ans << 8;
	ans 				+= pch_data->f_data;
	return ans;
} /* u32תdouble */


double u32_dou(INT32U data)
{
	double			ans;

	ans 				= (double) (data & 0xFF) / 256; //ȡ��С��
	ans 				= ans + (data >> 8);		//��������λ
	return ans;
} /* doubleתu32 */


INT32U dou_u32(double * d_data)
{
	INT32U			ans;

	CH_DATA 		temp;

	temp				= dou_ch_data(d_data, 0);
	ans 				= ch_data_u32(&temp);
	return ans;



} /* ��CH_DATA��ʽ�����ݴ�ӡ�ɸ�������ʾ */
/* ��ʾС�������λ */


void print_ch_data(CH_DATA * ch_data)
{
	printf("%d.%02d ", 
		((INT16U) ch_data->h_data << 8) +ch_data->l_data, ch_data->f_data * 100 / 256);




} /* ��U32��ʽ�����ݴ�ӡ�ɸ�������ʾ(3���ֽںϲ�) */
/* ��ʾС�������λ */


void print_int_data(INT32U int_data)
{
	double			res;

	res 				= u32_dou(int_data);

	//printf("%d.%.2f ",int_data>>8,((double)(int_data&0xFF))/256);
	printf("%.2f ", res);
} /* ���������㷨 */


int partiton_int_data(INT_DATA * arr, int low, int high)
{
	int 			pivotkey;

	INT_DATA		d_temp;

	pivotkey			= arr[low].i_data;

	while (low < high)
		{
		while (low < high && arr[high].i_data >= pivotkey)
			high--;

		d_temp				= arr[low];
		arr[low]			= arr[high];
		arr[high]			= d_temp;				//swap(arr,low,high);

		while (low < high && arr[low].i_data <= pivotkey)
			low++;

		d_temp				= arr[low];
		arr[low]			= arr[high];
		arr[high]			= d_temp;				//swap(arr,low,high);
		}

	return low;
}


void q_sort_int_data(INT_DATA * arr, int low, int high)
{
	int 			pivot;

	if (low < high)
		{
		pivot				= partiton_int_data(arr, low, high);
		q_sort_int_data(arr, low, pivot - 1);
		q_sort_int_data(arr, pivot + 1, high);
		}
}


void quick_sort_int_data(INT_DATA * arr, INT32U size)
{
	q_sort_int_data(arr, 0, size - 1);
} /* ���������㷨 */


int partiton_time_data(TIME_DATA * arr, int low, int high)
{
	int 			pivotkey;

	TIME_DATA		d_temp;

	pivotkey			= arr[low].time_ps;

	while (low < high)
		{
		while (low < high && arr[high].time_ps >= pivotkey)
			high--;

		d_temp				= arr[low];
		arr[low]			= arr[high];
		arr[high]			= d_temp;				//swap(arr,low,high);

		while (low < high && arr[low].time_ps <= pivotkey)
			low++;

		d_temp				= arr[low];
		arr[low]			= arr[high];
		arr[high]			= d_temp;				//swap(arr,low,high);
		}

	return low;
}


void q_sort_time_data(TIME_DATA * arr, int low, int high)
{
	int 			pivot;

	if (low < high)
		{
		pivot				= partiton_time_data(arr, low, high);
		q_sort_time_data(arr, low, pivot - 1);
		q_sort_time_data(arr, pivot + 1, high);
		}
}


void quick_sort_time_data(TIME_DATA * arr, INT32U size)
{
	q_sort_time_data(arr, 0, size - 1);
} /*********��������Ln(x)**********/


double my_log(double a)
{
	int 			N	= 100000;

	long			k, nk;

	double			x, xx, y;

	if (a < 0)
		{
		a					= -a;
		}

	x					= (a - 1) / (a + 1);
	xx					= x * x;
	nk					= 2 * N + 1;
	y					= 1.0 / nk;

	for (k = N; k > 0; k--)
		{
		nk					= nk - 2;
		y					= 1.0 / nk + xx * y;
		}

	return 2.0 * x * y;



} /*******************************/
/*********AGC ���ƺ���**********/
/*********���ڷ�ֵ��ѹ**********/


void agc_control(OS_CPU_SR cpu_sr)
{
	static double	ADC_Value;						//ADC value

	static double	err;							//������Ʋ�ֵ

	/* Get ADC_Value */
	ADC_Value			= (double)
	ADC_ConvertedValue / 4096 * 3.3;

	/* AGC Control Code */

	/* �������� */
	if (ADC_Value < CONTROL_MAX_VOLTAGE)
		{
		err 				= CONTROL_MAX_VOLTAGE - ADC_Value;
		}
	else if (ADC_Value >= CONTROL_MAX_VOLTAGE)
		{
		err 				= ADC_Value - CONTROL_MAX_VOLTAGE;
		}

	if (ADC_Value < CONTROL_MAX_VOLTAGE - FLOW_RANGE)
		{
		if (quantify > 2000)
			quantify = 2000;
		else if (err > 1.0)
			{
			quantify			+= 10;
			}
		else if (err > 0.8)
			{
			quantify			+= 8;
			}
		else if (err > 0.6)
			{
			quantify			+= 4;
			}
		else if (err > 0.4)
			{
			quantify			+= 2;
			}
		else 
			{
			quantify			+= 1;
			}

		OS_ENTER_CRITICAL();
		setDacValueBin(quantify);
		OS_EXIT_CRITICAL();
		}

#ifdef AGC_LOW
	else if (ADC_Value > CONTROL_MAX_VOLTAGE + FLOW_RANGE)
		{
		if (quantify <= 10)
			quantify = 10;
		else if (err > 1.0)
			quantify -= 10;
		else if (err > 0.8)
			quantify -= 8;
		else if (err > 0.6)
			quantify -= 4;
		else if (err > 0.4)
			quantify -= 2;
		else 
			quantify -= 1;

		OS_ENTER_CRITICAL();
		setDacValueBin(quantify);
		OS_EXIT_CRITICAL();
		}

#endif
} /* ����TIME_DATA */


void create_time_data(TIME_DATA * ptime_data, INT32U time_ps, INT32U err_time1, INT32U err_time2, 
	INT32U err_time3)
{
	ptime_data->time_ps = time_ps;

	ptime_data->err_time1 = err_time1;
	ptime_data->err_time2 = err_time2;
	ptime_data->err_time3 = err_time3;
} /* ����ֵ������� */


double multi_thread_adj(double * org, unsigned int * time)
{
	double			ans;

	if (*time > 1100 && *time < 2000)
		{
		ans 				= *org - 0.8541 * my_log(-10.9372 * (*time) + 11430) + 3.019 - coe1 + coe2;
		}
	else if ((*time) >= 2000)
		{
		ans 				= *org - 0.0004 * (*time) - 4.0737 - coe1 + coe2;
		}
	else 
		{
		ans 				= *org - coe1;
		}

	return ans;
}


