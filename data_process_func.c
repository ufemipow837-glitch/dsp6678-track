#include <stdio.h>
#include <string.h>
#include <math.h>
#include "data_process_func.h"
#include "track.h"
#include "matrix.h"
#include "matrix_mul.h"
#include "global_variable.h"

/* ========================================================================
* 外部全局变量声明（桥接原系统常驻内存的底层大池子）
 * 注：所有全局变量已在 global_variable.h 中声明、global_variable.c 中定义，
 *     无需再重复 extern 或定义
 * ======================================================================== */

#define PI  3.141592653589793f

static void ApplyDefaultParam(void);  /* 前置声明 */

int DataProcess2DspFunc(const TARGET_500_PACK *targetPack,
                        TRACK_OUTPUT *trackOutput,
                        TAS_BEAM_CFG *tasCfg)
{
    int i;
    int safe_target_num;
    static int cache_inited = 0;

    if (!cache_inited) {
        volatile unsigned int *mar;
        int mar_idx;
        *(volatile unsigned int *)0x01840020 = 0x7;
        *(volatile unsigned int *)0x01840040 = 0x7;
        *(volatile unsigned int *)0x01840000 = 0x3;
        for(mar_idx = 128; mar_idx <= 135; mar_idx++){
            mar = (volatile unsigned int *)(0x01848000 + mar_idx * 4);
            *mar = 0x1;
        }
        cache_inited = 1;
    }

    if (targetPack == NULL || trackOutput == NULL || tasCfg == NULL) {
        return 0;
    }

    // 0.1 安全初始化：仅当关键参数未设置时应用默认值（不覆盖外部已配置参数）
    if (track_debugger.beam_time <= 0.0f) {
        ApplyDefaultParam();
    }

    /* ============================================
   1. 将 FPGA 输入点迹包转换为内部点迹池格式
     * ============================================ */
    TARGETPIONT.frameSn   = targetPack->cpi_cnt;
    TARGETPIONT.Year      = 0;
    TARGETPIONT.Month     = targetPack->twstas_op;
    TARGETPIONT.Day       = 0;
    TARGETPIONT.Hour      = 0;
    TARGETPIONT.Minute    = 0;
    TARGETPIONT.Second    = 0;

    // 拦截点迹上限，防止输入 targetPack->n 异常引发内部数组越界
    safe_target_num = targetPack->n;
    if (safe_target_num > 60) {
        safe_target_num = 60;
    }
    if (safe_target_num < 0) {
        safe_target_num = 0;
    }
    TARGETPIONT.targetNum = safe_target_num;

    TARGETPIONT.workMode  = targetPack->twstas_op;   // 0=TWS, 1=TAS
    TARGETPIONT.tgtnum    = targetPack->pihao;

    // 帧级时间戳：始终从包中获取（即使n=0，FPGA通常也在info[0]填充帧时间）
    TARGETPIONT.mSecond = (float)targetPack->info[0].conv.time;

    // 转换并装载有效点迹
    for (i = 0; i < safe_target_num; i++) {
    	// 角度：度 -> 弧度
        TARGETPIONT.azi[i]        = targetPack->info[i].conv.az * PI / 180.0f;
        TARGETPIONT.ele[i]        = targetPack->info[i].conv.p  * PI / 180.0f;
        TARGETPIONT.range[i]      = targetPack->info[i].conv.r;
        // 径向速度取绝对值：算法内部用位置差分算三维速度模长，
        // 这里只做点迹凝聚比较用速度大小，避免多普勒测速符号（靠近/远离）影响凝聚
        TARGETPIONT.velocity[i]   = fabsf(targetPack->info[i].conv.v);
        TARGETPIONT.Use_Flag_1[i] = 1;

        TARGETPIONT.Year      = 0;
        TARGETPIONT.Day       = 0;
        TARGETPIONT.Hour      = 0;
        TARGETPIONT.Minute    = 0;
        TARGETPIONT.Second    = 0;
    }

    // 【死循环漏洞修复】：清空多余的过检通道槽位，上限死锁在 60
    for (i = safe_target_num; i < 60; i++) {
        TARGETPIONT.azi[i]        = 0.0f;
        TARGETPIONT.ele[i]        = 0.0f;
        TARGETPIONT.range[i]      = 0.0f;
        TARGETPIONT.velocity[i]   = 0.0f;
        TARGETPIONT.Use_Flag_1[i] = 0;
    }

    /* ============================================
     * 2. 调用核心数据处理
     * ============================================ */
    track(&track_debugger,
          &radar,
          &TARGETPIONT,
          &temp_track[0][0],
          &track_asso_data[0][0],
          &reliable_track[0],
          &track_renew[0],
          &track_end[0],
          &par[0],
          &reliable_track_num,
          &shotpoint[0],
          &open_USEFLAG);

    /* ============================================
    * 3. 落地零拷贝赋权：将内部全局大池子地址直接丢给输出接口
     * ============================================ */

    // 3.1  航迹输出绑定
    trackOutput->reliable_track_num = reliable_track_num;
    if (reliable_track_num > 0) {

        trackOutput->tracks = &track_renew[0];
    } else {
        trackOutput->tracks = NULL;
    }

    // 3.2 波束引导输出绑定
    tasCfg->reliable_track_num = reliable_track_num;
    if (reliable_track_num > 0 && par[0].num_P != 0) {

        tasCfg->beams = (MANAGEPARA *)&par[0];
    } else {
        tasCfg->beams = NULL;
    }

    // 返回当前活跃的可靠航迹数量作为执行状态反馈
    return reliable_track_num;
}

/* ========================================================================
 * 上位机参数配置接口实现
 * ======================================================================== */
static void ApplyDefaultParam(void)
{
    track_debugger.beam_time   = 0.164f;
    track_debugger.cx          = 0.35f;
    track_debugger.cy          = 0.35f;
    track_debugger.cz          = 0.35f;
    track_debugger.d_dan       = 0.155f;
    track_debugger.m_dan       = 45.0f;
    track_debugger.p_air       = 1.225f;
    track_debugger.wx          = 0.0f;
    track_debugger.wy          = 0.0f;
    track_debugger.wz          = 0.0f;
    track_debugger.Vmin        = 5.0f;
    track_debugger.Vmax        = 1500.0f;
    track_debugger.time_down   = 0.0f;
    track_debugger.time_up     = 4500.0f;
    track_debugger.azi_down    = -60.0f * PI / 180.0f;
    track_debugger.azi_up      =  60.0f * PI / 180.0f;
    track_debugger.ele_down    = -5.0f * PI / 180.0f;
    track_debugger.ele_up      =  60.0f * PI / 180.0f;
    track_debugger.range_down  = 100.0f;
    track_debugger.range_up    = 50000.0f;
    track_debugger.angle_hang  = 0.0f;
    TRACK_ASSO_TH = 15.0f;
    radar.initAz = 0.0f;
    radar.initEL = 0.0f;
    radar.initRo = 0.0f;
    radar.altTarget = 0.0f;
    radar.altRadar = 0.0f;
}

int TrackSetParam(const TRACK_CONFIG_PARAM *pParam)
{
    if (pParam == NULL) {
        ApplyDefaultParam();
        return 0;
    }

    if (pParam->beam_time > 0.0f) {
        track_debugger.beam_time = pParam->beam_time;
    }
    if (pParam->d_dan > 0.0f) {
        track_debugger.d_dan = pParam->d_dan;
    }
    if (pParam->m_dan > 0.0f) {
        track_debugger.m_dan = pParam->m_dan;
    }
    if (pParam->cx > 0.0f) track_debugger.cx = pParam->cx;
    if (pParam->cy > 0.0f) track_debugger.cy = pParam->cy;
    if (pParam->cz > 0.0f) track_debugger.cz = pParam->cz;

    track_debugger.wx = pParam->wx;
    track_debugger.wy = pParam->wy;
    track_debugger.wz = pParam->wz;

    if (pParam->Vmin >= 0.0f) track_debugger.Vmin = pParam->Vmin;
    if (pParam->Vmax > 0.0f)  track_debugger.Vmax = pParam->Vmax;

    if (pParam->time_down >= 0.0f) track_debugger.time_down = pParam->time_down;
    if (pParam->time_up > 0.0f)    track_debugger.time_up   = pParam->time_up;

    /* 空域范围：接口传度，内部存弧度，做有效性检查 */
    if(pParam->azi_down > -180.0f && pParam->azi_down < 180.0f) track_debugger.azi_down = pParam->azi_down * PI / 180.0f;
    if(pParam->azi_up > -180.0f && pParam->azi_up < 180.0f && pParam->azi_up > pParam->azi_down) track_debugger.azi_up = pParam->azi_up * PI / 180.0f;
    if(pParam->ele_down > -90.0f && pParam->ele_down < 90.0f) track_debugger.ele_down = pParam->ele_down * PI / 180.0f;
    if(pParam->ele_up > -90.0f && pParam->ele_up < 90.0f && pParam->ele_up > pParam->ele_down) track_debugger.ele_up = pParam->ele_up * PI / 180.0f;
    if(pParam->range_down >= 0.0f && pParam->range_down < 100000.0f) track_debugger.range_down = pParam->range_down;
    if(pParam->range_up > 0.0f && pParam->range_up > track_debugger.range_down) track_debugger.range_up = pParam->range_up;

    /* 雷达安装角（度） */
    radar.initAz = pParam->initAz;
    radar.initEL = pParam->initEL;
    radar.initRo = pParam->initRo;

    /* 雷达/靶心高度（米） */
    radar.altTarget = pParam->altTarget;
    radar.altRadar = pParam->altRadar;

    /* 关联门限 */
    if (pParam->asso_th > 0.0f) {
        TRACK_ASSO_TH = pParam->asso_th;
    }

    return 0;
}

void TrackGetParam(TRACK_CONFIG_PARAM *pParam)
{
    if (pParam == NULL) return;

    pParam->beam_time  = track_debugger.beam_time;
    pParam->d_dan      = track_debugger.d_dan;
    pParam->m_dan      = track_debugger.m_dan;
    pParam->cx         = track_debugger.cx;
    pParam->cy         = track_debugger.cy;
    pParam->cz         = track_debugger.cz;
    pParam->wx         = track_debugger.wx;
    pParam->wy         = track_debugger.wy;
    pParam->wz         = track_debugger.wz;
    pParam->Vmin       = track_debugger.Vmin;
    pParam->Vmax       = track_debugger.Vmax;
    pParam->time_down  = track_debugger.time_down;
    pParam->time_up    = track_debugger.time_up;
    /* 弧度转度返回 */
    pParam->azi_down   = track_debugger.azi_down * 180.0f / PI;
    pParam->azi_up     = track_debugger.azi_up   * 180.0f / PI;
    pParam->ele_down   = track_debugger.ele_down * 180.0f / PI;
    pParam->ele_up     = track_debugger.ele_up   * 180.0f / PI;
    pParam->range_down = track_debugger.range_down;
    pParam->range_up   = track_debugger.range_up;
    pParam->initAz     = radar.initAz;
    pParam->initEL     = radar.initEL;
    pParam->initRo     = radar.initRo;
    pParam->altTarget  = radar.altTarget;
    pParam->altRadar   = radar.altRadar;
    pParam->asso_th    = TRACK_ASSO_TH;
}
