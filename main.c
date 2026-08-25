#include <stdio.h>
#include <c6x.h>
#include <math.h>
#include "track.h"
#include "struct.h"
#include "global_variable.h"
#include "data_entry.h"
#include "data_process_func.h"
#include "matrix.h"
#include "matrix_mul.h"
#include <stdint.h>
#include <float.h>
#include <string.h>

#pragma pack(4)

#define TOTAL_FRAMES 100
//1. 硬件耗时打点相关寄存器/变量定义
extern void CSL_tscEnable(void);
unsigned int StartTime = 0;
unsigned int EndTime = 0;
unsigned int Total_cycle_ticks = 0;
extern cregister volatile unsigned int TSCL;

uint32_t utilReadTime32()
{
    uint32_t timeVal;
    timeVal = TSCL;
    return timeVal;
}


// 全局变量已统一定义在 global_variable.c，此处无需重复定义

//线下测试专用：将 data_entry 产生的结构体逆向适配为接口输入的工具函数
static void ConvertToPack(const struct TARGETPIONT_1 *src, TARGET_500_PACK *dst)
{
    int k;
    dst->cpi_cnt    = src->frameSn;
    dst->n          = src->targetNum;
    dst->twstas_op  = src->workMode;
    dst->pihao      = 1;
    // 确保info[0]始终有时间戳(即使n=0空帧)
    dst->info[0].conv.time = (unsigned int)src->mSecond;
    for (k = 0; k < src->targetNum && k < 500; k++) {
        dst->info[k].conv.az = src->azi[k] * 180.0f / pi;
        dst->info[k].conv.p  = src->ele[k] * 180.0f / pi;
        dst->info[k].conv.r  = src->range[k];
        dst->info[k].conv.v  = src->velocity[k];
        dst->info[k].conv.time = (unsigned int)src->mSecond;
    }
}



int main(void)
{
    int i;

    // Cache配置已在data_process_func.c的DataProcess2DspFunc中统一处理
    printf("\n");

     data_entry(&target_data[0]);

  // --------------------------------------------------------------------
    // 无人机实测数据参数配置
    track_debugger.beam_time   = 0.164f;   // 波位间隔164ms
    track_debugger.cx          = 0.35f;
    track_debugger.cy          = 0.35f;
    track_debugger.cz          = 0.35f;
    track_debugger.d_dan       = 0.155f;
    track_debugger.m_dan       = 45.0f;
    track_debugger.p_air       = 1.225f;
    track_debugger.time_down   = 0.0f;
    track_debugger.time_up     = 4500.0f;  // 时间窗口4.5s(容忍1.6圈扫描)

    track_debugger.wx          = 0.0f;
    track_debugger.wy          = 0.0f;
    track_debugger.wz          = 0.0f;
    track_debugger.Vmin        = 5.0f;     // 最小速度5m/s(覆盖低速无人机)
    track_debugger.Vmax        = 1500.0f;

    track_debugger.azi_down    = -90.0f * pi / 180.0f;  // 方位门限±90°(测试用,根据实际阵面安装调整)
    track_debugger.azi_up      =  90.0f * pi / 180.0f;
    track_debugger.ele_down    = -5.0f * pi / 180.0f;   // 俯仰-5°~60°
    track_debugger.ele_up      =  60.0f * pi / 180.0f;
    track_debugger.range_down  = 100.0f;
    track_debugger.range_up    = 50000.0f;  // 距离门限50km

    radar.initAz = 0;
    radar.initEL = 0;
    radar.initRo = 0;
    // 实例化封装函数对外的标准出入口结构体
       // 更改后：分配在全局静态存储区，彻底免疫栈溢出
    static TARGET_500_PACK  test_input_pack;
    static TRACK_OUTPUT     test_track_output;
    static TAS_BEAM_CFG     test_tas_cfg;

    int time_p[13][8] = {0};

    printf("====================  ====================\n");

     // --------------------------------------------------------------------
    for(i = 0; i < TOTAL_FRAMES; i++)
    {
         memset(&test_input_pack, 0, sizeof(TARGET_500_PACK));
        ConvertToPack(&target_data[i], &test_input_pack);

          CSL_tscEnable();
        StartTime = utilReadTime32();

        DataProcess2DspFunc(&test_input_pack, &test_track_output, &test_tas_cfg);


        EndTime = utilReadTime32();
        Total_cycle_ticks = EndTime - StartTime;


        printf("Frame %03d | t=%.0fms | n=%d | Cycle: %.3f ms\n",
               i, target_data[i].mSecond, target_data[i].targetNum, Total_cycle_ticks / 1000000.0f);
        printf("  -> Reliable Tracks: %d\n", test_track_output.reliable_track_num);

        if (test_track_output.reliable_track_num > 0 && test_track_output.tracks != NULL)
        {
            int k;
            for(k = 0; k < test_track_output.reliable_track_num && k < 5; k++){
                printf("  -> Track[%d]: Az=%.2fdeg El=%.2fdeg R=%.1fm V=%.1fm/s\n",
                       k, test_track_output.tracks[k].azi,
                       test_track_output.tracks[k].ele,
                       test_track_output.tracks[k].range,
                       test_track_output.tracks[k].Vel);
            }
        }
        printf("------------------------------------------------------------------------\n");


        time_p[(i)/8][(i)%8] = Total_cycle_ticks;
    }
    (void)time_p;

    printf("========================================\n");
    return 0;
}
