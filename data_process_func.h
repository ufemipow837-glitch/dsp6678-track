#ifndef DATA_PROCESS_FUNC_H_
#define DATA_PROCESS_FUNC_H_

#include "struct.h"

//目标原始信息
//------------------------------------------------------------------------
typedef struct
{
	int 			vp;				//speed point location of mtd matrix
	int				rp;				//range point location of mtd matrix
	float			asw[4];			//最大值前后各两个点，用于加权求距离，相当于内插
	float			as;				//sum beam amplitude
	float			ad;   			//用于传参
	float			ad_azi;				//dif beam amplitude
	float  			ad_pitch;
	float			ns;				//noise
	unsigned int 	pbn;			//pitch beam num
	unsigned int	pri_n;
} TARGET_ORG;
//------------------------------------------------------------------------
// 目标转换信息
//------------------------------------------------------------------------
typedef struct
{
	float			v;				//speed
	float			r;				//range
	float			az;				//azimuth
	float			p;				//pitch
	float           pitch_n;
	float			snr;			//信噪比
	float			asv;			//不同俯仰幅度配平系数，为脉宽T*带宽B*脉冲数N乘积的倒数*和幅度
	int				flag;			//标志，0未融合，1被融合
	int				w_n_op;			//宽窄脉冲标志  0-宽脉冲  1-窄脉冲
	unsigned int	time;			//时间ms
	float			rcs;			//雷达散射截面积
} TARGET_CONV;
//------------------------------------------------------------------------
// 一次目标信息
//------------------------------------------------------------------------
typedef struct
{
	TARGET_ORG		org;			//original target information
	TARGET_CONV		conv;
} TARGET_PRI_INFO;
//------------------------------------------------------------------------
// 500个目标目标包
//------------------------------------------------------------------------
typedef struct
{
	unsigned short 		narw_mtd_noise;		//mtd噪声，db，例如100.00db，传10000
	unsigned short 		wide_mtd_noise;		//mtd噪声，db，例如100.00db，传10000
	unsigned short 		bpn;				//波位号  0-16
	unsigned short 		n;					//target num
	int					cpi_cnt;			//CPI帧计数
	unsigned int 		deal_n;				//从核处理次数，处理一次一个波列cpi
	unsigned int		twstas_op;			//0TWS 1TAS
	unsigned int		pihao;				//批号
	TARGET_PRI_INFO		info[500];
} TARGET_500_PACK;
/* 航迹输出：指向内部可靠航迹池 */
typedef struct {
    int                   reliable_track_num;
    struct TARGETTRACE_RENEW  *tracks;
} TRACK_OUTPUT;

/** 波束输出：指向内部波束参数数组 * */
typedef struct {
    int          reliable_track_num;
    MANAGEPARA  *beams;               /* 指向 par 数组 */
} TAS_BEAM_CFG;

int DataProcess2DspFunc(const TARGET_500_PACK *targetPack,
                        TRACK_OUTPUT *trackOutput,
                        TAS_BEAM_CFG *tasCfg);

/* ========================================================================
 * 上位机参数配置接口
 * 上位机通过通信口（网口/串口/SRIO）发来参数后，直接填充 TRACK_CONFIG_PARAM
 * 结构体，调用 TrackSetParam() 即可实时更新算法参数，无需重启
 * ======================================================================== */
typedef struct
{
    /* ---- 硬件/CPI参数 ---- */
    float       beam_time;          /* CPI驻留时间（秒），例：0.010f = 10ms */

    /* ---- 弹种气动参数 ---- */
    float       d_dan;              /* 弹径（米） */
    float       m_dan;              /* 弹质量（kg） */
    float       cx;                 /* x方向阻力系数 */
    float       cy;                 /* y方向阻力系数 */
    float       cz;                 /* z方向阻力系数 */

    /* ---- 风速补偿（m/s） ---- */
    float       wx;
    float       wy;
    float       wz;

    /* ---- 速度门限（m/s） ---- */
    float       Vmin;
    float       Vmax;

    /* ---- 关联时间窗（ms） ---- */
    float       time_down;
    float       time_up;

    /* ---- 观测空域（单位：度！接口统一用度，函数内部转弧度） ---- */
    float       azi_down;           /* 方位下限（度） */
    float       azi_up;             /* 方位上限（度） */
    float       ele_down;           /* 俯仰下限（度） */
    float       ele_up;             /* 俯仰上限（度） */
    float       range_down;         /* 距离下限（米） */
    float       range_up;           /* 距离上限（米） */

    /* ---- 雷达阵面安装角（度） ---- */
    float       initAz;             /* 阵面方位偏角 */
    float       initEL;             /* 阵面俯仰倾角 */
    float       initRo;             /* 阵面横滚角 */

    /* ---- 雷达/靶心高度（米） ---- */
    float       altTarget;          /* 靶心高度 */
    float       altRadar;           /* 雷达架设高度 */

    /* ---- 算法门限 ---- */
    float       asso_th;            /* 马氏距离关联门限 */
} TRACK_CONFIG_PARAM;

/**
 * @brief  设置跟踪算法参数（上位机配置用）
 * @param  pParam  参数结构体指针，填好后传入；NULL则直接应用默认炮弹参数
 * @retval 0=成功
 * @note   可随时调用，实时生效；上电不调用则自动使用默认炮弹参数
 */
int TrackSetParam(const TRACK_CONFIG_PARAM *pParam);

/**
 * @brief  读取当前算法参数（上位机查询用）
 * @param  pParam  输出结构体，当前参数会拷贝到这里
 */
void TrackGetParam(TRACK_CONFIG_PARAM *pParam);

#endif
