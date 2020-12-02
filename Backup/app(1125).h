#ifndef app_h
#define app_h


#include "os_cpu.h"
#include "includes.h"
#include "tlv5636.h"
#include "usart.h"
#include "key.h"
#include "agc.h"
#include <math.h>


/*********************************************************************************************/
/* ����������У׼���� */
#define coe						0.2
#define coe1					2.4
#define coe2					2.2
extern double			d_distance;
extern float			batch_err;			//��ͬ���β�ǽ����
extern float			caps;				//��������߲���
extern INT8U			level_stable;
extern INT8U			no_data_cnt;

//������ģʽ����
enum 
{
	wall,											//��ǽģʽ
	line											//����ģʽ
};

//���������ģʽ����
enum
{
	camera,
	UARTbluetooth
};
#define OutModeSet UARTbluetooth		//���ģʽ����
extern unsigned char outmode;

/*********************************************************************************************/
////��̬����
#define MAX_LEVEL				13
/* ���浵λ�� */
extern const INT16U	LEVEL[MAX_LEVEL];
//�̶�����
//#define MAX_LEVEL 1
/////* ���浵λ�� */
//extern const INT16U LEVEL[MAX_LEVEL];
extern INT8U			level;


//#define DEBUG
//#define DUG		 //����ԭʼ�������˲����� 

/* У׼����ģʽ�������� */
#define STANDARD_DISTANCE		0 //����Ŀ���׼����
#define INC_STEP				10		   //�������Ӳ���
#define MIN_GAIN				1000 	  //��С���� quantify DAC����ֵ
#define MAX_GAIN				1500	   //�������

/**********************************************/
/* �궨��ģʽ���� */
/* ͨ��ע�ͻ����ѡ��ģʽ */
//#define ADJ //У׼����ģʽ ,��������������У׼ģʽ
#define MULTI_VTH //����������ѽ�����ж���ֵ���У׼

//#define AGC_LOW	//�������AGC_LOW,AGC���ƻὫ����CONTROL_MAX_VOLTAGE��ʱ��������棬����ֻ����������
#define AUTO_LEVEL_MOD //�Զ���λ�л�ģʽ ��������ΪAGCģʽ

//#define ARGV	//�Ƿ񽫻����������ƽ������Ȼȡ��ֵ

/*********************************************************************************************/
/****************�ṹ�嶨��**********************/
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

typedef struct GEN_TIME_DATA
{
	TIME_DATA		timedata;
	INT32U			distance;
}GEN_TIME_DATA;

/*********************************************************************************************/
//UCOS ��������
//���ÿ�ʼ�������ȼ�
#define START_TASK_PRIO 		10//һ����10�����ȼ�����ʼ��������ȼ����Ϊ10
#define START_STAK_SIZE 		64//���ÿ�ʼ�����ջ��С
extern OS_STK			START_TASK_STAK[START_STAK_SIZE]; //��ʼ�����ջ

//���������߳����ȼ����
#define MASTER_TASK_PRIO		5
#define MASTER_STAK_SIZE		512
extern OS_STK			MASTER_TASK_STAK[MASTER_STAK_SIZE];

//���ü���������������
#define GEN_TASK_PRIO			6
#define GEN_STAK_SIZE			512*3
#define MEASURE_DATA_ARR_SIZE	18
extern OS_STK			GEN_TASK_STAK[GEN_STAK_SIZE];

//�������ݴ�������������
#define HANDLE_TASK_PRIO		7
#define HANDLE_STAK_SIZE		128
extern OS_STK			HANDLE_TASK_STAK[HANDLE_STAK_SIZE]; //ʹ��printf��ӡ�����������ڣ���Ҫ�����ջ8�ֽڶ���

//����AGC��������
#define AGC_TASK_PRIO			8
#define AGC_STAK_SIZE			64
extern OS_STK			AGC_TASK_STAK[AGC_STAK_SIZE];

#ifdef ADJ

//���õ���У׼���� 
#define ADJ_TASK_PRIO			4
#define ADJ_STAK_SIZE			64
extern OS_STK			ADJ_TASK_STAK[ADJ_STAK_SIZE];

#endif

void start_task(void * pdata);
void master_task(void * pdata);
void gen_task(void * pdata);
void handle_task(void * pdata);
void agc_task(void * pdata);
void adj_task(void * pdata);

/*********************************************************************************************/
//���ܺ����������
#define DATA_GROUP_SIZE 		5 //ÿһ��������ݳ���
#define QSTART_SIZE 			256    //��Ϣ����ָ�����黺������С
extern void *			Qstart[QSTART_SIZE]; //��Ϣ����ָ������
extern OS_EVENT *		q_msg; //��Ϣ����

#define ANS_BUF_SIZE			8	 //��������������С

//INT_DATA ans_buf[ANS_BUF_SIZE];//������������
extern TIME_DATA		ans_buf[ANS_BUF_SIZE]; //������������

//INT32U err_buf[ANS_BUF_SIZE];//�����ֵʱ������
extern INT8U			ans_ite; //��ǰд��λ��
extern INT8U			ans_cnt; //�������н������
//extern INT8U 			ans_lineJudge;		//���������жϳɲ��߸���

/* λ������ */
#define setbit(x, y)			(x)|=(1<<(y))
#define clrbit(x, y)			(x)&=~(1<<(y))
#define reversebit(x, y)		(x)^=(1<<(y))
#define getbit(x, y)			((x)>>(y)&1)

/* ��ʼ������ֵ���� */
#define QUANTIFY_INIT			1100  //��ʼ������ֵ DAC
#define START_THREAD			80			
#define STOP_THREAD1			60		//ԭ60
#define STOP_THREAD2			80		
#define STOP_THREAD3			80		//ԭ80

/* ����ɸѡУ׼���� */
#define GAIN_TH 				1200	  //������ֵ
#define DISTANCE_TH 			21	  //��������
#define TIME_PS_TH				100000 //15�׵�ʱ�� 
#define TIME_PS_TH2				650000 //97.5��ʱ��
#define MAX_TIME				10000000 //������ʱ�� 1500�׵�ʱ�� ps
#define TIME_1000M				6666667		//1000�׷���ʱ��
#define MIN_TIME				0		 //��С����ʱ��  
#define DETECT_MIN_TIME 		300000 //̽����ֵ��С����ʱ�� 45m
#define MAX_ERR_TIME			10000 //�����ֵʱ���
#define MIN_ERR_TIME			0	 //��С��ֵʱ���

/* AGC ���� */
extern unsigned char	AGC_EN;
extern unsigned short	quantify; //DAC ����ֵ
extern INT32U			detect_val; //�����ֵ��⵽��ֵ time_ps

/* AGC Control parameter	 */
#define CONTROL_MAX_VOLTAGE 	3
#define FLOW_RANGE				0.2
#define CONTROL_DIFF_TIME		2000
#define TIME_FLOW_RANGE 		200

#define MAX_NO_DATA_CNT 		5

extern const int		refclk_divisions; //8M����  ,TDC�ο�ʱ��


void out_mode_set(void);
unsigned char Mode_Judge(void);
double ch_data_dou(CH_DATA * pch_data);
CH_DATA dou_ch_data(double * d_data, INT32U err_time);
INT32U ch_data_u32(CH_DATA * pch_data);
double u32_dou(INT32U data);
INT32U dou_u32(double * d_data);
void quick_sort_int_data(INT_DATA * arr, INT32U size); //��������
void quick_sort_time_data(TIME_DATA * arr, INT32U size);
void print_ch_data(CH_DATA * ch_data);
void print_int_data(INT32U int_data);
double getvar_orgdata(TIME_DATA* array,INT8U length);
double my_log(double a);
void agc_control(OS_CPU_SR cpu_sr);
void create_time_data(TIME_DATA * ptime_data, INT32U time_ps, INT32U err_time1, INT32U err_time2, INT32U err_time3);
double multi_thread_adj(double * org, unsigned int * time);
TIME_DATA data_filter(TIME_DATA* arr, unsigned char len, unsigned int diff_time);

#include "mto.h"
#endif

