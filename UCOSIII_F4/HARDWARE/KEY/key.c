#include "key.h"

//����������
//���ذ���ֵ
//mode:0,��֧��������;1,֧��������;
//0��û���κΰ�������
//1��KEY0����
//2��KEY1����
//3��KEY2���� 
//4��WKUP���� WK_UP
//ע��˺�������Ӧ���ȼ�,KEY0>KEY1>KEY2>WK_UP!!
uint8_t KEY_Scan(uint8_t mode)
{
	static uint8_t key_up=1;//�������ɿ���־
	if(mode)key_up=1;  			//֧������		  
	if(key_up&&(KEY_0==0||KEY_1==0||KEY_2==0||WK_UP==1))
	{
		delay_ms(10);//ȥ���� 
		key_up=0;
		if(KEY_0==0)return 1;
		else if(KEY_1==0)return 2;
		else if(KEY_2==0)return 3;
		else if(WK_UP==1)return 4;
	}else if(KEY_0==1&&KEY_1==1&&KEY_2==1&&WK_UP==0)key_up=1; 	    
 	return 0;// �ް�������
}
