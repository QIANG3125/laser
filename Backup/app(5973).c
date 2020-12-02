#include "app.h"

//���������ز���
double			d_distance = 0;
float			batch_err = 0.28;			//��ͬ���β�ǽ����
float			caps	  = 0;
INT8U			level_stable = 0;
INT8U			no_data_cnt = 0;

//�������ģʽ
unsigned char outmode= OutModeSet; //�������ģʽ camera:�����������ĸ�ʽ  UARTbluetooth:��ӡ�����ڵ����ַ���ʽ

/*********************************************************************************************/
//��̬����
/* ���浵λ�� */

const INT16U	LEVEL[MAX_LEVEL] =
{
	1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000, 2200, 2300,2500
};
//�̶�����
/* ���浵λ�� */
//const INT16U LEVEL[MAX_LEVEL] = 
//{
//	1200
//};
INT8U			level = 0; //��λ

/*********************************************************************************************/
//UCOS II ������ر�������
OS_STK			START_TASK_STAK[START_STAK_SIZE]; //��ʼ�����ջ
__align(8) OS_STK			MASTER_TASK_STAK[MASTER_STAK_SIZE];
OS_STK			GEN_TASK_STAK[GEN_STAK_SIZE];
__align(8) OS_STK			HANDLE_TASK_STAK[HANDLE_STAK_SIZE]; //ʹ��printf��ӡ�����������ڣ���Ҫ�����ջ8�ֽڶ���
OS_STK			AGC_TASK_STAK[AGC_STAK_SIZE];
#ifdef ADJ
__align(8) OS_STK			ADJ_TASK_STAK[ADJ_STAK_SIZE];
#endif

/*********************************************************************************************/
//���ܺ�����ر�������
void *			Qstart[QSTART_SIZE]; //��Ϣ����ָ������
OS_EVENT *		q_msg; //��Ϣ����

//INT_DATA ans_buf[ANS_BUF_SIZE];//������������
TIME_DATA		ans_buf[ANS_BUF_SIZE]; //������������

//INT32U err_buf[ANS_BUF_SIZE];//�����ֵʱ������
INT8U			ans_ite = 0; //��ǰд��λ��
INT8U			ans_cnt = 0; //�������н������
//INT8U 			ans_lineJudge = 0;		//���������жϳɲ��߸���

/* AGC ���� */
unsigned char	AGC_EN;
unsigned short	quantify = QUANTIFY_INIT; //DAC ����ֵ
INT32U			detect_val = 0; //�����ֵ��⵽��ֵ time_ps

const int		refclk_divisions = 125000; //8M����  ,TDC�ο�ʱ��


/* �������������ʽ */
/* outmodeΪUARTbluetoothʱ����ַ�����ʽ */
/* outmodeΪcameraʱ����ֽ�����ʽ */
/* MODEΪ0ʱ��ǽģʽ */
/* MODEΪ1ʱ����ģʽ */
void out_mode_set()
{ /* ���ģʽ�趨 */

	if(outmode==UARTbluetooth)		//�����������
		{
		//		Ӳ���жϲ������ǽģʽ
			if (MODE == 1)
				{
				mcb.measure_mode=line;
				}
			else 
				{
				mcb.measure_mode=wall;
				}
		}
	else 	//��λ���������
	{
	//��λ��ѡ����߲�ǽģʽ
		while(Mode_Judge());
	}
} /* CH_DATAתdouble */

unsigned char Mode_Judge()
{
	INT16U pos=0;

	if(USART_RX_STA>2)	//Э��ָ���Ϊ3,ָ����������Ŵ���
	{
		if(USART_RX_BUF[pos]==0xCD)	// ���ܵ�֡ͷ
			{
				switch (USART_RX_BUF[pos+1])
				{
					case 0X4F:
						if(USART_RX_BUF[pos+2] == 0X11)
							mcb.measure_mode=line;
						else if(USART_RX_BUF[pos+2] == 0x33)
							mcb.measure_mode=wall;
						break;
				//Э�����չ
						default:break;
				}
				//���ܵ�ָ��
				return 0;
			}
		else
			{
				pos++;
			}
	}

    //δ���ܵ�ָ��
    return 1;
	
}


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
} 

double getvar_orgdata(TIME_DATA* array,INT8U length)
{
	double result=0;
	double distance[DATA_GROUP_SIZE]={0};
	INT8U i=0;
	INT32S sum=0,average=0;
	
	for(i=0;i<length;i++)
		{
			distance[i]=(1.5 * (double) array[i].time_ps) / 10000;
			sum+=distance[i];
		}
	average= sum/length;
	
	for(i=0;i<length;i++)
	{
		result+= pow(distance[i]-average,2)/length;
	}
	
	return result;
}


/*********��������Ln(x)**********/
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
} 
/*******************************/
/*********AGC ���ƺ���**********/
/*********���ڷ�ֵ��ѹ**********/

void agc_control(OS_CPU_SR cpu_sr)
{
	static double	ADC_Value;						//ADC value

	static double	err;							//������Ʋ�ֵ

	/* Get ADC_Value */
	ADC_Value			= (double)ADC_ConvertedValue / 4096 * 3.3;

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
		ans 				= *org - 0.8541 * my_log(-10.9372 * (*time) + 11430) + 3.019 - coe;
		}
	else if ((*time) >= 2000)
		{
		ans 				= *org - 0.0004 * (*time) - 4.0737 - coe;
		}
	else 
		{
		ans 				= *org - coe1;
		}

	return ans;
}



/* ���ݷ���ɸѡ */
/* arr: �������� */
/* len: ���鳤�� */
/* diff_time: ���෶Χps */
TIME_DATA data_filter(TIME_DATA* arr, unsigned char len, unsigned int diff_time)
{
	unsigned char cur_cnt;
	unsigned char max_ele=0,max_pos = 0;
	int i = 0;
	unsigned int cur_category;

	/* �������ݷ��� */
	cur_category = arr[0].time_ps;
	cur_cnt = 1;
	for (i = 1; i < len; i++)
	{
		if (arr[i].time_ps - cur_category < diff_time)//���ڵ�ǰ����
		{
			cur_cnt++;//����ֵ������һ
		}
		else//�µķ���
		{
			if (cur_cnt > max_ele)//����������ֵ
			{
				max_ele = cur_cnt;//������
				max_pos = i - max_ele;//������ֵ��ԭʼ�����е�λ��
				
			}				
			/* ���·������� */
			cur_category = arr[i].time_ps;
			cur_cnt = 1;
		}
	}
	//����߽����
	if(cur_cnt > max_ele)
	{
		max_ele = cur_cnt;//������
		max_pos = i - max_ele;//������ֵ��ԭʼ�����е�λ��

	}
	// ȡ���������������е���ֵ
	//�������
//	return (1.5 * (double)arr[((max_pos << 1) + max_ele) / 2].time_ps) / 10000;	
	return arr[((max_pos << 1) + max_ele) / 2];
}

