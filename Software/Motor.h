#ifndef __MOTOR_H
#define __MOTOR_H

typedef struct{
	float Kp;	   		    // 参数: 比例环路增益
	float Ki;	   		    // 参数: 积分增益	
	float Kd;           	// 参数: 微分系数
	float Ref;	  		  	// 输入: 参考设定点
	float Fbk;	 		    // 输入: 反馈值
	float Out;	   	    	// 输出: 控制器输出
	float Err;  		    // 数据; 误差
	float ErrLast;      	// 数据: 上次误差
  	float Ui;	          	// 数据: 积分项
	float OutMax; 		  	// 参数: 上限饱和限制
	float OutMin;		    // 参数: 下限饱和限制
	float ErrMax;     		// 参数: 误差上限
	float ErrMin;     		// 参数: 误差下限	
	float ErrLimltFlag;		// 参数: 误差限幅标志
	float D_Filter;		    // 参数: 微分项滤波系数 (new)
	float LastDerivative;   // 数据: 上次微分项值 (new)
}PID_STRUCT;




#endif // MOTOR_H
