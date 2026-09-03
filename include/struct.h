#ifndef STRUCT_H_
#define STRUCT_H_

#pragma once
#include <stdint.h>
#include "imm.h"
#pragma pack(4)

struct debugger_track{

	float d_dan;		//弹径	m
	float m_dan;		//弹体质量	kg
	float p_air;		//空气密度	kg/m3

	float angle_hang;	//航向角		rad

	float cx;			//x阻力系数
	float cy;			//y阻力系数
	float cz;			//z阻力系数
	float wx;			//纵风自东向西		m/s
	float wy;			//横风自北向南		m/s
	float wz;			//上风		m/s

    float Vmin;          // 目标速度下限 (m/s)，低于此速度不转入TAS跟踪
    float Vmax;          // 目标速度上限 (m/s)，高于此速度认为不是炮弹

	float time_down;	//关联时间下限ms
	float time_up;		//关联时间上线ms
	float beam_time;	//cpi时间s		（0.0056）
//	int beam1;			//波束分布
//	int beam2;			//波束分布
	float azi_down;		//雷达盲区方位角边界下限	rad
	float azi_up;		//雷达盲区方位角边界上限	rad
	float ele_down;		//雷达盲区俯仰角边界下限	rad
	float ele_up;		//雷达盲区俯仰角边界上限	rad
	float range_down;	//雷达盲区距离边界下限		m
	float range_up;		//雷达盲区距离边界上限		m
	float azi_phase_comp;	//方位角相位补偿		°度
	float ele_phase_comp;	//俯仰角相位补偿		°度
	float corr_distance;	//距离补偿值关 LSB=1m
	float corr_sidelobe;	//压副瓣补偿值 LSB=1dB
	//float filter_range_low[5];   // 每组过滤的起始距离
	//float filter_range_high[5];  // 每组过滤的结束距离
	//int   filter_range_num;      // 过滤组数
};

struct TARGETPIONT
{

long frameSn;             		//cpi序号，从1开始累加
uint32_t targetNum;         	//目标数量[1,127]
uint32_t tgtnum;         	//目标号
uint32_t workMode;          	//跟踪搜索；0：搜索，1：跟踪
uint32_t beamNo;            	//波位号
uint32_t en_azi_ele;            //方位角/俯仰角使能    0：计算方位角	1：计算俯仰角

uint32_t  Year;             	//时间，年
uint32_t  Month;            	//时间，月
uint32_t  Day;              	//时间，日
uint32_t  Hour;             	//时间，时
uint32_t  Minute;           	//时间，分
uint32_t  Second;           	//时间，秒
float  mSecond;          		//时间，毫秒

int en_num;

int  range_gate[64];         	//距离门号
int  velocity_gate[64];      	//速度门号
int	blur_en;					//距离模糊使能	0不工作	1工作

float beamangle;				//中心角度		°度

float range;         			//距离门大小		1m
float velocity;					//速度门大小		20us->波长/(2*20us*64),19us->波长/(2*19us*64)		m/s
float Amp_Sum_I[64];         	//和幅度 I
float Amp_Sum_Q[64];         	//和幅度 Q
float Amp_diff_I[64];        	//差幅度 I
float Amp_diff_Q[64];        	//差幅度 Q

float Amp_Pro_I[64];         	//保护通道幅度 I
float Amp_Pro_Q[64];         	//保护通道幅度 Q
};

struct TARGETPIONT_OUT
{

long frameSn;             		//cpi序号，从1开始累加

uint32_t targetNum;         	//目标数量[1,127]
uint32_t tgtnum;         	//目标号
//uint32_t workMode;          	//跟踪搜索；0：搜索，1：跟踪
uint32_t beamNo;            	//波位号

uint32_t  Year;             	//时间，年
uint32_t  Month;            	//时间，月
uint32_t  Day;              	//时间，日
uint32_t  Hour;             	//时间，时
uint32_t  Minute;           	//时间，分
uint32_t  Second;           	//时间，秒
float  mSecond;          	//时间，毫秒

float beamangle_azi;			//方位中心角度	°度
float beamangle_ele;			//俯仰中心角度	°度

float velocity[10];				//速度大小		m/s
float range[10];         		//距离			m
float azi[10];         			//方位角			rad弧度
float ele[10];        			//俯仰角			rad弧度

float azi_angle[10];			//方位角偏移量	°度
float ele_angle[10];			//俯仰角偏移量	°度
float azi_ratio[10];			//方位角差和幅度比
float ele_ratio[10];			//俯仰角差和幅度比

float Amp_Sum_I_azi[10];         	//方位和幅度 I
float Amp_Sum_Q_azi[10];         	//方位和幅度 Q
float Amp_diff_I_azi[10];        	//方位差幅度 I
float Amp_diff_Q_azi[10];        	//方位差幅度 Q
float range_azi[10];         		//方位距离
int  range_gate_azi[10];         	//方位距离门号
int  velocity_gate_azi[10];      	//方位速度门号

float Amp_Sum_I_ele[10];         	//俯仰和幅度 I
float Amp_Sum_Q_ele[10];         	//俯仰和幅度 Q
float Amp_diff_I_ele[10];        	//俯仰差幅度 I
float Amp_diff_Q_ele[10];        	//俯仰差幅度 Q
float range_ele[10];         		//俯仰距离
int  range_gate_ele[10];         	//俯仰距离门号
int  velocity_gate_ele[10];      	//俯仰速度门号

int	num_azi[10];				//奇数帧数据号
int	num_ele[10];				//偶数帧数据号

int	Use_Flag[10];				//有效标志位
};

struct TARGETPIONT_azi
{

long frameSn;             		//cpi序号，从1开始累加

uint32_t targetNum;         	//目标数量[1,127]
uint32_t tgtnum;         	//目标号
uint32_t workMode;          	//跟踪搜索；0：搜索，1：跟踪
uint32_t beamNo;            	//波位号

uint32_t  Year;             	//时间，年
uint32_t  Month;            	//时间，月
uint32_t  Day;              	//时间，日
uint32_t  Hour;             	//时间，时
uint32_t  Minute;           	//时间，分
uint32_t  Second;           	//时间，秒
float  mSecond;          	//时间，毫秒

float max[64];

float beamangle;				//中心角度		°度
float velocity[64];				//速度大小		m/s
float range[64];         		//距离			m
float azi[64];         			//方位角			°度
float azi_angle[64];			//方位角偏移量	°度
float azi_ratio[64];			//方位角差和幅度比

float Amp_Sum_I_azi[64];         	//方位和幅度 I
float Amp_Sum_Q_azi[64];         	//方位和幅度 Q
float Amp_diff_I_azi[64];        	//方位差幅度 I
float Amp_diff_Q_azi[64];        	//方位差幅度 Q
int  range_gate_azi[64];         	//方位距离门号
int  velocity_gate_azi[64];      	//方位速度门号

int	blur_en;					//距离模糊使能	0不工作	1工作
int Use_flag_azi[64];			//有效标志位
int	num[64];					//数据号
};

struct TARGETPIONT_ele
{

long frameSn;             		//cpi序号，从1开始累加

uint32_t targetNum;         	//目标数量[1,127]
uint32_t tgtnum;         	//目标号
uint32_t workMode;          	//跟踪搜索；0：搜索，1：跟踪
uint32_t beamNo;            	//波位号

uint32_t  Year;             	//时间，年
uint32_t  Month;            	//时间，月
uint32_t  Day;              	//时间，日
uint32_t  Hour;             	//时间，时
uint32_t  Minute;           	//时间，分
uint32_t  Second;           	//时间，秒
float  mSecond;          	//时间，毫秒

float max[64];

float beamangle;				//中心角度		°度
float velocity[64];				//速度大小		m/s
float range[64];         		//距离			m
float ele[64];        			//俯仰角			°度
float ele_angle[64];			//俯仰角偏移量	°度
float ele_ratio[64];			//俯仰角差和幅度比

float Amp_Sum_I_ele[64];         	//俯仰和幅度 I
float Amp_Sum_Q_ele[64];         	//俯仰和幅度 Q
float Amp_diff_I_ele[64];        	//俯仰差幅度 I
float Amp_diff_Q_ele[64];        	//俯仰差幅度 Q
int  range_gate_ele[64];         	//俯仰距离门号
int  velocity_gate_ele[64];      	//俯仰速度门号

int	blur_en;					//距离模糊使能  	0 不执行距离模糊   1 执行距离模糊
int Use_flag_ele[64];				//有效标志位
int	num[64];						//数据号
};



struct MISSIONPARA
{
uint32_t MissionID;			//任务ID，不用
uint32_t CarrierType;			//平台类型，1—轰炸机，2—歼击机
uint32_t LineAngle;			//飞机航线航向角（相对真北），LSB：0.1度；
uint32_t LineAlt;				//飞机高度，LSB：1米（什么时候的高度？）

uint32_t BombModel; 			//炸弹类型：1—100kg航爆弹，2—250kg航爆弹，
							//3—500kg航爆弹，4—激光500炸弹，
							//5—90-1火箭弹，6—122-1火箭弹；
uint32_t BombNum;				//炸弹数量，[1,4]，可同时出现；

float latTarget;  			//靶心坐标纬度，单位：度
float lonTarget;  			//靶心坐标经度，单位：度
float altTarget;  			//靶心坐标高度，单位：米
float latRadar;				//雷达坐标纬度，单位：度
float lonRadar; 			//雷达坐标经度，单位：度
float altRadar; 			//雷达坐标高度，单位：米
float initAz;  				//伺服初始控守（地理系）方位，单位°，LSB°
float initEL;				//阵面俯仰角（地理系），单位°（相较于水平面）
float initRo;				//横滚角，单位°
};

struct TARGETPIONT_1
{

long frameSn;
uint32_t targetNum;
uint32_t tgtnum;
uint32_t workMode;
uint32_t beamNo;

uint32_t  Year;
uint32_t  Month;
uint32_t  Day;
uint32_t  Hour;
uint32_t  Minute;
uint32_t  Second;
float  mSecond;			//ms

float  	azi[60];		//rad弧度
float  	ele[60];		//rad弧度
float 	range[60];		//m
float 	velocity[60];	//m/s

int	Use_Flag_1[60];		//标志位	1有效	0无效

};


//struct TARGETTRACE
//{
//uint32_t traceType;
//uint32_t batchNum;
//uint32_t  Year;
//uint32_t  Month;
//uint32_t  Day;
//uint32_t  Hour;
//uint32_t  Minute;
//uint32_t  Second;
//float  mSecond;
//
//float range;
//float azi;
//float ele;
//float lat;
//float lon;
//float alt;
//float X;
//float Y;
//float Z;
//float Vel;
//float Vx;
//float Vy;
//float Vz;
//};

struct TARGETTRACE_RENEW
{
	uint32_t num;
	uint32_t tgtnum;         	//目标号
	uint32_t traceType;
	uint32_t batchNum;
	uint32_t  Year;
	uint32_t  Month;
	uint32_t  Day;
	uint32_t  Hour;
	uint32_t  Minute;
	uint32_t  Second;
	float  mSecond;
	float range;                
	float azi;        			//rad弧度
	float ele;                  //rad弧度
	float lat;					//纬度		°度
	float lon;					//经度		°度
	float alt;                  //高度		m
	float X;					//m
	float Y;					//m
	float Z;					//m
	float Vel;                  //	m/s
	float Vx;                   //	m/s
	float Vy;                   //	m/s
	float Vz;                   //	m/s

    int     num_P;

};

struct TARGETTRACE_END
{
	uint32_t num;
	uint32_t tgtnum;         	//目标号
	uint32_t traceType;
	uint32_t batchNum;
	uint32_t  Year;
	uint32_t  Month;
	uint32_t  Day;
	uint32_t  Hour;
	uint32_t  Minute;
	uint32_t  Second;
	float  mSecond;
	float range;
	float azi;
	float ele;
//	float lat;
//	float lon;
//	float alt;
	float X0;
	float Y0;
	float Z0;
	float Vx0;
	float Vy0;
	float Vz0;
	float X1;
	float Y1;
	float Z1;
	float Vx1;
	float Vy1;
	float Vz1;
	float Vel;

    int     num_P;

};

struct SHOTPOINT
{
	uint16_t MissionID;
	uint16_t BombNo;
	uint16_t tgtnum;	//落地目标号
	uint32_t  Year;		// 落地时刻，年
	uint32_t  Month;	// 落地时刻，月
	uint32_t  Day;		// 落地时刻，日
	uint32_t  Hour;		// 落地时刻，时
	uint32_t  Minute;	// 落地时刻，分
	uint32_t  Second;	// 落地时刻，秒
	float  mSecon;		// 落地时刻，毫秒
	float lat;			//弹着点，纬度，单位：度
	float lon;			//弹着点，经度，单位：度
	float Vel;			// 落速，矢量和，单位：米/秒
	float Angle;		// 落角，俯仰角，单位：度
	float X;			//弹着点，ECEF坐标系X，米
	float Y;			//弹着点，ECEF坐标系Y，米
	float Z;			//弹着点，ECEF坐标系Z，米

	int k;				//k = 1 预测超时

    int     num_P;
};

struct  target_track
{
	long frameSn;
	uint32_t tgtnum;         	//目标号
	uint32_t beamNo;            	//波束号
    uint32_t  Year;
	uint32_t  Month;
	uint32_t  Day;
	uint32_t  Hour;
	uint32_t  Minute;
	uint32_t  Second;
	float  mSecond;

    float	x;                      
    float 	y;                     	
    float 	z;

    float   range;
    float   azi;
    float   ele;
    float 	velocity;
    int		Use_Flag_1;

};

struct temp_track
{
	uint32_t tgtnum;         	//目标号
	uint32_t beamNo;            	//波束号
	uint32_t  Year;
	uint32_t  Month;
	uint32_t  Day;
	uint32_t  Hour;
	uint32_t  Minute;
	uint32_t  Second;
	float  mSecond;

    float  X[6];		
    float  P[6][6];		

    int asso_flag;		


};


typedef struct reliable
{
	uint32_t tgtnum;         	//目标号
    uint32_t  Year;
	uint32_t  Month;
	uint32_t  Day;
	uint32_t  Hour;
	uint32_t  Minute;
	uint32_t  Second;
	float  mSecond;

    float  X0[6];
    float  P0[6][6];
    float  X1[6];
    float  P1[6][6];

    IMM_STATE imm;

    uint32_t init_beamNo;        //初始化波束号
    int 	predict_flag;	//连续外推次数
    int		track_update_flag;		//1表示已经过滤波测量更新
    int     num_P;//航迹批号，-1表示消亡
//    int     f;

}RELIABLE_TRACK;


typedef struct MANAGEPARA
{
    long frameSn;
  	float threshold;
  	float velocity;
  	float scanAz;		//	°度
    float scanEl;		//	°度
    float range;		//	m
    float servoAz;
    int num_P;
}MANAGEPARA;

//struct  tas
//{
//
//    float   range;		//m
//    float   azi;		//
//    float   ele;
//
//};

//struct datamn{
//
//    float dcal1;
//    float dcal1max;
//    int n;
//};
#endif /* STRUCT_H_ */
