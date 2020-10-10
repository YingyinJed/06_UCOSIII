#include "key.h"

//按键处理函数
//返回按键值
//mode:0,不支持连续按;1,支持连续按;
//0，没有任何按键按下
//1，KEY0按下
//2，KEY1按下
//3，KEY2按下 
//4，WKUP按下 WK_UP
//注意此函数有响应优先级,KEY0>KEY1>KEY2>WK_UP!!
uint8_t KEY_Scan(uint8_t mode)
{
	static uint8_t key_up=1;//按键按松开标志
	if(mode)key_up=1;  			//支持连按		  
	if(key_up&&(KEY_0==0||KEY_1==0||KEY_2==0||WK_UP==1))
	{
		delay_ms(10);//去抖动 
		key_up=0;
		if(KEY_0==0)return 1;
		else if(KEY_1==0)return 2;
		else if(KEY_2==0)return 3;
		else if(WK_UP==1)return 4;
	}else if(KEY_0==1&&KEY_1==1&&KEY_2==1&&WK_UP==0)key_up=1; 	    
 	return 0;// 无按键按下
}
