#include <stdio.h>
#include <c6x.h>
#include <math.h>
#include <string.h>
#include "track.h"
#include "struct.h"
#include "matrix.h"
#include "global_variable.h"
#include <stdint.h>
#include <float.h>

#pragma pack(4)
/*
 * dot_coh.c  点迹预处理：杂波过滤 + 输出有效点
 *  - 滤除近距盲区、远距杂波、俯仰异常点
 *  - 保留所有通过过滤的点（不再强制双帧匹配，机扫模式兼容）
 *  - 帧间关联交给后续 track_initial/track_asso 的马氏距离门限处理
 */
void dot_coh(struct target_track (*dot_data),
        struct target_track (*target)){
    int i,j,k3 = 0;

    // 使用配置参数做杂波过滤
    float rng_low  = (track_debugger.range_down >= 0.0f) ? track_debugger.range_down : 100.0f;
    float rng_high = (track_debugger.range_up > rng_low)    ? track_debugger.range_up   : 50000.0f;
    float ele_low  = (track_debugger.ele_down > -1.57f && track_debugger.ele_down < track_debugger.ele_up) ? track_debugger.ele_down : -0.1745f;
    float ele_high = (track_debugger.ele_up > ele_low && track_debugger.ele_up < 1.57f) ? track_debugger.ele_up : 1.047f;
    float azi_low  = (track_debugger.azi_down > -3.1416f && track_debugger.azi_down < track_debugger.azi_up) ? track_debugger.azi_down : -1.047f;
    float azi_high = (track_debugger.azi_up > azi_low && track_debugger.azi_up < 3.1416f) ? track_debugger.azi_up : 1.047f;

    // 清空输出
    for(j = 0; j < cpi_num; j++){
        target[j].Use_Flag_1 = 0;
    }

    // 第一步：对两帧数据做杂波过滤
    for(i = 0; i < 2; i++){
        for(j = 0; j < cpi_num; j++){
            float r   = dot_data[i*cpi_num+j].range;
            float ele = dot_data[i*cpi_num+j].ele;
            float azi = dot_data[i*cpi_num+j].azi;

            if(r < rng_low || r > rng_high){
                dot_data[i*cpi_num+j].Use_Flag_1 = 0;
                continue;
            }
            if(ele < ele_low || ele > ele_high){
                dot_data[i*cpi_num+j].Use_Flag_1 = 0;
                continue;
            }
            if(azi < azi_low || azi > azi_high){
                dot_data[i*cpi_num+j].Use_Flag_1 = 0;
                continue;
            }
        }
    }

    // 第二步：优先输出当前帧（i=1）所有有效点
    for(j = 0; j < cpi_num && k3 < cpi_num; j++){
        if(dot_data[1*cpi_num+j].Use_Flag_1 != 0){
            target[k3] = dot_data[1*cpi_num+j];
            target[k3].Use_Flag_1 = 1;
            k3++;
        }
    }

    // 如果当前帧没有点（第一帧启动/本帧全被滤掉），输出上一帧（i=0）的点
    if(k3 == 0){
        for(j = 0; j < cpi_num && k3 < cpi_num; j++){
            if(dot_data[0*cpi_num+j].Use_Flag_1 != 0){
                target[k3] = dot_data[0*cpi_num+j];
                target[k3].Use_Flag_1 = 1;
                k3++;
            }
        }
    }

    // 剩余输出槽位清零
    for(; k3 < cpi_num; k3++){
        target[k3].Use_Flag_1 = 0;
    }

    if(k3 == 0){
        target[0].mSecond = dot_data[1*cpi_num+0].mSecond;
        target[0].frameSn = dot_data[1*cpi_num+0].frameSn;
    }
}
