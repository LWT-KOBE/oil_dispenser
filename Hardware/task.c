#include "task.h"
#include "system.h"
#include "Key.h"
#include "math.h"
#include "MyCAN.h"
MOTOR motor;
Encoding_Wheel EW;
int flag = 0;
int i = 0;

/**
  * 函    数：TIM2中断函数
  * 参    数：无
  * 返 回 值：无
  * 注意事项：此函数为中断函数，无需调用，中断触发后自动执行
  *           函数名为预留的指定名称，可以从启动文件复制
  *           请确保函数名正确，不能有任何差异，否则中断函数将不能进入
  */
void TIM4_IRQHandler(void)
{
	//10ms进一次中断
	
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)		//判断是否是TIM2的更新事件触发的中断
	{												
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);			//清除TIM2更新事件的中断标志位
															//中断标志位必须清除
		
		
		
		//获取编码器计数
		EW.Encoder_pr = Read_Encoder(2);
		//累加
		EW.mileage += EW.Encoder_pr;
		//计算里程
		EW.Current_Mileage = EW.mileage / 200.0f / 4 * 30.0f;
		//计算速度
		EW.speed = EW.Encoder_pr /200.0f /4 * 30.0f * 100.0f;
		
		
		//如果实际里程大于等于目标里程，则停止喷油
		if(EW.Current_Mileage >= EW.target_mileage){
			EW.finish = 1;
		}else{
			EW.finish = 0;
		}
		
		//功能按键按下
		if(EW.first_num){
			if(PBin(6) == 0){
				PBout(11) = 0;
			
				//判断速度是否大于30cm/s
				if(fabs(EW.speed) > 30){
				EW.time++;
				//持续时间大于1.5秒则开始抽油
				if(EW.time >= 150){
					EW.time = 150;
					//抽油计数
					EW.time_period++;
					PBout(12) = 0;
					//抽油时间为100ms
					if(EW.time_period<15){
						AX_MOTOR_B_SetSpeed(190);
					}else if(EW.time_period > 15){
						AX_MOTOR_B_SetSpeed(0);
					}
					//两秒后重新循环抽油
					if(EW.time_period > 220){
						EW.time_period = 0;
					}
					
				}
				
				}else{
					//关闭抽油,状态清零
					EW.time = 0;
					PBout(12) = 1;
					EW.time_period = 0;
					AX_MOTOR_B_SetSpeed(0);
				}

			
		}else{
			EW.time = 0;
			EW.time_period = 0;
			//AX_MOTOR_B_SetSpeed(0);
			EW.Current_Mileage = 0;
			EW.mileage = 0;
			
			//关闭显示灯
			PBout(11) = 1;
			PBout(12) = 1;
			
			if(PBin(7) == 0 && EW.first_num){
				AX_MOTOR_B_SetSpeed(500);
				PBout(13) = 0;
			}else{
				PBout(13) = 1;
				AX_MOTOR_B_SetSpeed(0);
				}
			}
		}else{
			//PBout(11) = 0;
			PBout(12) = 1;
			i++;
			if(i < 50){
				PBout(11) = 0;
				
			}else if(i > 50 && i < 100){
				PBout(11) = 1;
				
			}else if(i > 100){
				i = 0;
			}
			
		}
		
		
		
		Vofa_sendData(EW.Current_Mileage,EW.speed,can1_rx.recv[6]);
		
		
			
		}
		
		
		
		
					
															//否则中断将连续不断地触发，导致主程序卡死
}
