#include <stdio.h>
#include <math.h>
#include <string.h>

// 模拟C代码的关键逻辑来诊断无人机初始化失败原因

#define PI 3.14159265f
#define TRACK_INIT_TH 15.0f
#define TRACK_QUALITY_THRESHOLD 0.8f
#define VELOCITY_CONSISTENCY_THRESHOLD 0.5f
#define ANGLE_CHANGE_THRESHOLD 160.0f
#define MAX_NEW_TRACKS_PER_FRAME 20
#define MAX_MISSING_FRAMES_SEARCH 160
#define CONFIDENCE_THRESHOLD 0.65f

// 无人机实测数据（前几帧的关键数据点）
typedef struct {
    int frame;
    int beamNo;
    float range;      // 距离(m)
    float ele;        // 俯仰角(度)
    float azi;        // 方位角(度, 雷达方位角: 正北=0°)
    float velocity;   // 径向速度(m/s, 负=靠近)
    float mSecond;    // 时间戳(ms)
} DronePoint;

// 从无人机实测数据中提取的无人机点
DronePoint drone_points[] = {
    // Frame 4, beam 2
    {4,  2, 1094.84, 5.16777f, 16.8225f, -11.7406f, 898796},
    // Frame 5, beam 3
    {5,  3, 1095.84, 4.78606f, 17.3911f, -11.7406f, 898960},
    // Frame 21, beam 2 (下一圈扫描)
    {21, 2, 1124.10, 4.62256f, 16.5781f, -11.7406f, 901584},
    // Frame 22, beam 3
    {22, 3, 1124.83, 4.60890f, 17.2821f, -11.7406f, 901748},
    // Frame 38, beam 2
    {38, 2, 1153.14, 4.76548f, 17.3900f, -11.1945f, 904372},
    // Frame 39, beam 3
    {39, 3, 1154.56, 3.79858f, 18.3887f, -11.1945f, 904536},
    // Frame 55, beam 2 (根据数据推算)
    {55, 2, 1180.00, 4.50000f, 17.0000f, -11.2000f, 907160},
};

int num_drone_points = sizeof(drone_points) / sizeof(drone_points[0]);

// 将雷达方位角(度)转换为数学方位角(弧度)
float radar_azi_to_math_azi_rad(float radar_azi_deg) {
    return (PI / 2.0f) - radar_azi_deg * PI / 180.0f;
}

// 球坐标转直角坐标（模拟track.c）
void spherical_to_cartesian(float range, float ele_rad, float azi_rad, 
                           float *x, float *y, float *z) {
    *x = range * cosf(ele_rad) * cosf(azi_rad);
    *y = range * cosf(ele_rad) * sinf(azi_rad);
    *z = range * sinf(ele_rad);
}

// 数学方位角(弧度)转雷达方位角(度)
float math_azi_rad_to_radar_deg(float math_azi_rad) {
    return (PI / 2.0f - math_azi_rad) * 180.0f / PI;
}

// 计算两帧间的时间差(秒)
float calc_T(DronePoint *p0, DronePoint *p1) {
    return (p1->mSecond - p0->mSecond) / 1000.0f;
}

// 计算位置差分速度
void calc_pos_diff_velocity(DronePoint *p0, DronePoint *p1, float T,
                             float *vx, float *vy, float *vz) {
    float x0, y0, z0, x1, y1, z1;
    float ele0_rad, azi0_rad, ele1_rad, azi1_rad;
    
    ele0_rad = p0->ele * PI / 180.0f;
    azi0_rad = radar_azi_to_math_azi_rad(p0->azi);
    
    ele1_rad = p1->ele * PI / 180.0f;
    azi1_rad = radar_azi_to_math_azi_rad(p1->azi);
    
    spherical_to_cartesian(p0->range, ele0_rad, azi0_rad, &x0, &y0, &z0);
    spherical_to_cartesian(p1->range, ele1_rad, azi1_rad, &x1, &y1, &z1);
    
    *vx = (x1 - x0) / T;
    *vy = (y1 - y0) / T;
    *vz = (z1 - z0) / T;
}

// 计算径向单位向量
void calc_radial_unit_vec(DronePoint *p, float *er_x, float *er_y, float *er_z) {
    float r_el = p->ele * PI / 180.0f;
    float r_azi = radar_azi_to_math_azi_rad(p->azi);
    
    *er_x = cosf(r_el) * cosf(r_azi);
    *er_y = cosf(r_el) * sinf(r_azi);
    *er_z = sinf(r_el);
}

// 模拟get_track_asso1的速度修正逻辑
void simulate_velocity_correction(DronePoint *p0, DronePoint *p1, float T,
                                    float *vx_corr, float *vy_corr, float *vz_corr) {
    float vx_old, vy_old, vz_old;
    float er_x, er_y, er_z;
    float v_radial_old, vx_tan, vy_tan, vz_tan;
    float r_vr;
    float tan_damp = 0.08f;
    
    // Step 1: kalman_filter_init - 位置差分速度
    calc_pos_diff_velocity(p0, p1, T, &vx_old, &vy_old, &vz_old);
    
    // Step 2: 雷达Vr修正
    r_vr = p1->velocity;  // 雷达实测径向速度(带符号)
    calc_radial_unit_vec(p1, &er_x, &er_y, &er_z);
    
    v_radial_old = vx_old * er_x + vy_old * er_y + vz_old * er_z;
    
    vx_tan = vx_old - v_radial_old * er_x;
    vy_tan = vy_old - v_radial_old * er_y;
    vz_tan = vz_old - v_radial_old * er_z;
    
    *vx_corr = r_vr * er_x + vx_tan * tan_damp;
    *vy_corr = r_vr * er_y + vy_tan * tan_damp;
    *vz_corr = r_vr * er_z + vz_tan * tan_damp;
}

// 计算速度向量夹角
float calc_angle_change(float v1x, float v1y, float v1z,
                        float v2x, float v2y, float v2z) {
    float dot = v1x*v2x + v1y*v2y + v1z*v2z;
    float n1 = sqrtf(v1x*v1x + v1y*v1y + v1z*v1z);
    float n2 = sqrtf(v2x*v2x + v2y*v2y + v2z*v2z);
    float cos_angle;
    
    if(n1 < 0.1f || n2 < 0.1f) return 180.0f;
    
    cos_angle = dot / (n1 * n2);
    if(cos_angle > 1.0f) cos_angle = 1.0f;
    if(cos_angle < -1.0f) cos_angle = -1.0f;
    
    return acosf(cos_angle) * 180.0f / PI;
}

// 计算位置变化夹角
float calc_position_angle_change(DronePoint *p0, DronePoint *p1, DronePoint *p2) {
    float x0, y0, z0, x1, y1, z1, x2, y2, z2;
    float ele_rad, azi_rad;
    float dx01, dy01, dz01, dx12, dy12, dz12;
    float dot, n01, n12, cos_angle;
    
    // 转换为直角坐标
    ele_rad = p0->ele * PI / 180.0f;
    azi_rad = radar_azi_to_math_azi_rad(p0->azi);
    spherical_to_cartesian(p0->range, ele_rad, azi_rad, &x0, &y0, &z0);
    
    ele_rad = p1->ele * PI / 180.0f;
    azi_rad = radar_azi_to_math_azi_rad(p1->azi);
    spherical_to_cartesian(p1->range, ele_rad, azi_rad, &x1, &y1, &z1);
    
    ele_rad = p2->ele * PI / 180.0f;
    azi_rad = radar_azi_to_math_azi_rad(p2->azi);
    spherical_to_cartesian(p2->range, ele_rad, azi_rad, &x2, &y2, &z2);
    
    dx01 = x1 - x0; dy01 = y1 - y0; dz01 = z1 - z0;
    dx12 = x2 - x1; dy12 = y2 - y1; dz12 = z2 - z1;
    
    dot = dx01*dx12 + dy01*dy12 + dz01*dz12;
    n01 = sqrtf(dx01*dx01 + dy01*dy01 + dz01*dz01);
    n12 = sqrtf(dx12*dx12 + dy12*dy12 + dz12*dz12);
    
    if(n01 < 0.1f || n12 < 0.1f) return 180.0f;
    
    cos_angle = dot / (n01 * n12);
    if(cos_angle > 1.0f) cos_angle = 1.0f;
    if(cos_angle < -1.0f) cos_angle = -1.0f;
    
    return acosf(cos_angle) * 180.0f / PI;
}

// 计算距离
float calc_dist_3d(float x1, float y1, float z1, float x2, float y2, float z2) {
    float dx = x2 - x1, dy = y2 - y1, dz = z2 - z1;
    return sqrtf(dx*dx + dy*dy + dz*dz);
}

// 模拟calculate_vr_quality
float simulate_calculate_vr_quality(float vr0, float vr1, float vr2) {
    int sign0, sign1, sign2, n_valid;
    float vr_avg_abs, vr_min_abs, vr_max_abs, vr_ratio;
    
    sign0 = (fabsf(vr0) > 0.01f) ? ((vr0 > 0.0f) ? 1 : -1) : 0;
    sign1 = (fabsf(vr1) > 0.01f) ? ((vr1 > 0.0f) ? 1 : -1) : 0;
    sign2 = (fabsf(vr2) > 0.01f) ? ((vr2 > 0.0f) ? 1 : -1) : 0;
    
    n_valid = (sign0 != 0) + (sign1 != 0) + (sign2 != 0);
    
    if(n_valid >= 2) {
        if(sign0 != 0 && sign1 != 0 && sign0 != sign1) return 0.0f;
        if(sign0 != 0 && sign2 != 0 && sign0 != sign2) return 0.0f;
        if(sign1 != 0 && sign2 != 0 && sign1 != sign2) return 0.0f;
    } else {
        return 0.0f;
    }
    
    vr_avg_abs = (fabsf(vr0) + fabsf(vr1) + fabsf(vr2)) / (float)n_valid;
    
    vr_min_abs = 1e6f;
    if(sign0 != 0 && fabsf(vr0) < vr_min_abs) vr_min_abs = fabsf(vr0);
    if(sign1 != 0 && fabsf(vr1) < vr_min_abs) vr_min_abs = fabsf(vr1);
    if(sign2 != 0 && fabsf(vr2) < vr_min_abs) vr_min_abs = fabsf(vr2);
    
    vr_max_abs = 0.0f;
    if(sign0 != 0 && fabsf(vr0) > vr_max_abs) vr_max_abs = fabsf(vr0);
    if(sign1 != 0 && fabsf(vr1) > vr_max_abs) vr_max_abs = fabsf(vr1);
    if(sign2 != 0 && fabsf(vr2) > vr_max_abs) vr_max_abs = fabsf(vr2);
    
    if(vr_max_abs > 0.01f && vr_min_abs > 0.01f) {
        vr_ratio = vr_min_abs / vr_max_abs;
    } else {
        vr_ratio = 0.0f;
    }
    
    if(vr_avg_abs < 2.0f) {
        return vr_ratio * 0.5f;
    } else if(vr_avg_abs < 30.0f) {
        return vr_ratio;
    } else if(vr_avg_abs < 100.0f) {
        return vr_ratio * 0.8f;
    } else {
        return vr_ratio * 0.3f;
    }
}

// 模拟calculate_confidence_for_track
float simulate_calculate_confidence_for_track(
    float vr0, float vr1, float vr2,
    float v1x, float v1y, float v1z,
    float v2x, float v2y, float v2z,
    float Vmin, float Vmax) {
    
    float vr_avg_abs, vr_min_abs, vr_max_abs;
    int is_low_speed;
    float v1_norm, v2_norm, dot_prod, cos_theta;
    float angle_consistency, min_norm, max_norm, speed_ratio;
    float vr_consistency, confidence;
    float v1_valid;
    
    vr0 = fabsf(vr0);
    vr1 = fabsf(vr1);
    vr2 = fabsf(vr2);
    
    vr_avg_abs = (vr0 + vr1 + vr2) / 3.0f;
    vr_min_abs = vr0; if(vr1 < vr_min_abs) vr_min_abs = vr1; if(vr2 < vr_min_abs) vr_min_abs = vr2;
    vr_max_abs = vr0; if(vr1 > vr_max_abs) vr_max_abs = vr1; if(vr2 > vr_max_abs) vr_max_abs = vr2;
    
    is_low_speed = (vr_avg_abs >= 1.0f && vr_avg_abs <= 30.0f) ? 1 : 0;
    
    v1_norm = sqrtf(v1x*v1x + v1y*v1y + v1z*v1z);
    v2_norm = sqrtf(v2x*v2x + v2y*v2y + v2z*v2z);
    
    if(v1_norm < 0.5f || v2_norm < 0.5f) return 0.0f;
    
    dot_prod = v1x*v2x + v1y*v2y + v1z*v2z;
    cos_theta = dot_prod / (v1_norm * v2_norm + 1e-10f);
    if(cos_theta > 1.0f) cos_theta = 1.0f;
    if(cos_theta < -1.0f) cos_theta = -1.0f;
    
    angle_consistency = (1.0f + cos_theta) / 2.0f;
    min_norm = (v1_norm < v2_norm) ? v1_norm : v2_norm;
    max_norm = (v1_norm > v2_norm) ? v1_norm : v2_norm;
    speed_ratio = min_norm / (max_norm + 1e-10f);
    
    v1_valid = (vr_avg_abs >= Vmin && vr_avg_abs <= Vmax) ? 1.0f : 0.3f;
    
    if(is_low_speed) {
        if(vr_max_abs > 1e-10f && vr_min_abs > 1e-10f) {
            vr_consistency = vr_min_abs / vr_max_abs;
        } else {
            vr_consistency = 0.5f;
        }
        if(vr_max_abs - vr_min_abs < 15.0f) {
            vr_consistency = 0.7f + 0.3f * vr_consistency;
        }
        confidence = vr_consistency * 0.6f
                   + angle_consistency * 0.2f
                   + speed_ratio * 0.1f
                   + v1_valid * 0.1f;
    } else {
        confidence = (angle_consistency + speed_ratio + v1_valid + v1_valid) / 4.0f;
    }
    
    if(vr_avg_abs < Vmin || vr_avg_abs > Vmax) {
        confidence *= 0.5f;
    }
    
    return confidence;
}

#ifdef DIAGNOSE_DRONE_INIT
int main() {
    int i, j;
    float T, vx_old, vy_old, vz_old;
    float vx_corr_1, vy_corr_1, vz_corr_1;  // 2点初始化后的速度
    float vx_corr_2, vy_corr_2, vz_corr_2;  // 3点初始化后的速度
    float pos_angle_change, vel_angle_change;
    float track_quality;
    float confidence;
    float x0, y0, z0, x1, y1, z1, x2, y2, z2;
    float ele_rad, azi_rad;
    float dist_sq, gate_sq, gate;
    float v1x, v1y, v1z, v2x, v2y, v2z;
    float v1_norm, v2_norm;
    
    printf("=== 无人机3点初始化诊断 ===\n\n");
    
    printf("1. 无人机数据汇总:\n");
    printf("%-6s %-6s %-8s %-8s %-10s %-10s %-12s\n", 
           "Frame", "Beam", "Range(m)", "El(deg)", "Az(deg)", "Vr(m/s)", "Time(ms)");
    for(i = 0; i < num_drone_points; i++) {
        printf("%-6d %-6d %-8.1f %-8.2f %-10.4f %-10.4f %-12.0f\n",
               drone_points[i].frame, drone_points[i].beamNo,
               drone_points[i].range, drone_points[i].ele,
               drone_points[i].azi, drone_points[i].velocity,
               drone_points[i].mSecond);
    }
    
    // 分析关键组合: Frame 4 + Frame 5 + Frame 21
    printf("\n\n2. 分析关键3点组合 (Frame 4 + 5 + 21):\n");
    {
        DronePoint *p0 = &drone_points[0];  // Frame 4, beam 2
        DronePoint *p1 = &drone_points[1];  // Frame 5, beam 3
        DronePoint *p2 = &drone_points[2];  // Frame 21, beam 2
        
        float T_01, T_12;
        
        // 2点初始化: Frame 4 -> Frame 5
        T_01 = calc_T(p0, p1);
        printf("\n--- 2点初始化 (Frame %d + %d) ---\n", p0->frame, p1->frame);
        printf("时间间隔 T = %.3f s\n", T_01);
        
        calc_pos_diff_velocity(p0, p1, T_01, &vx_old, &vy_old, &vz_old);
        printf("位置差分速度: vx=%.2f, vy=%.2f, vz=%.2f m/s\n", vx_old, vy_old, vz_old);
        printf("位置差分速度模: |v|=%.2f m/s\n", sqrtf(vx_old*vx_old + vy_old*vy_old + vz_old*vz_old));
        
        simulate_velocity_correction(p0, p1, T_01, &vx_corr_1, &vy_corr_1, &vz_corr_1);
        printf("Vr修正后速度: vx=%.2f, vy=%.2f, vz=%.2f m/s\n", vx_corr_1, vy_corr_1, vz_corr_1);
        v1_norm = sqrtf(vx_corr_1*vx_corr_1 + vy_corr_1*vy_corr_1 + vz_corr_1*vz_corr_1);
        printf("修正后速度模: |v|=%.2f m/s\n", v1_norm);
        
        // 距离门检查 (1->2)
        gate = 1500.0f * T_01 + 150.0f;
        gate_sq = gate * gate;
        
        ele_rad = p0->ele * PI / 180.0f;
        azi_rad = radar_azi_to_math_azi_rad(p0->azi);
        spherical_to_cartesian(p0->range, ele_rad, azi_rad, &x0, &y0, &z0);
        
        ele_rad = p1->ele * PI / 180.0f;
        azi_rad = radar_azi_to_math_azi_rad(p1->azi);
        spherical_to_cartesian(p1->range, ele_rad, azi_rad, &x1, &y1, &z1);
        
        dist_sq = (x1-x0)*(x1-x0) + (y1-y0)*(y1-y0) + (z1-z0)*(z1-z0);
        printf("\n距离门检查: gate=%.0fm, dist=%.1fm, %s\n", 
               gate, sqrtf(dist_sq), 
               dist_sq < gate_sq ? "PASS" : "FAIL!");
        
        // 速度一致性检查
        printf("\n速度一致性检查 (calculate_vr_quality):\n");
        printf("  vr0=%.4f, vr1=%.4f\n", p0->velocity, p1->velocity);
        track_quality = simulate_calculate_vr_quality(p0->velocity, p1->velocity, 0.0f);
        printf("  质量=%.3f (阈值=%.1f): %s\n", 
               track_quality, VELOCITY_CONSISTENCY_THRESHOLD,
               track_quality >= VELOCITY_CONSISTENCY_THRESHOLD ? "PASS" : "FAIL!");
        
        // 3点初始化: Frame 5 -> Frame 21
        T_12 = calc_T(p1, p2);
        printf("\n--- 3点初始化 (Frame %d + %d + %d) ---\n", p0->frame, p1->frame, p2->frame);
        printf("时间间隔 T_12 = %.3f s\n", T_12);
        
        // 计算位置
        ele_rad = p2->ele * PI / 180.0f;
        azi_rad = radar_azi_to_math_azi_rad(p2->azi);
        spherical_to_cartesian(p2->range, ele_rad, azi_rad, &x2, &y2, &z2);
        
        // 计算位置夹角
        pos_angle_change = calc_position_angle_change(p0, p1, p2);
        printf("位置变化夹角: %.2f° (阈值=%.0f°): %s\n", 
               pos_angle_change, ANGLE_CHANGE_THRESHOLD,
               pos_angle_change <= ANGLE_CHANGE_THRESHOLD ? "PASS" : "FAIL!");
        
        // 3点速度修正
        simulate_velocity_correction(p1, p2, T_12, &vx_corr_2, &vy_corr_2, &vz_corr_2);
        printf("3点Vr修正后速度: vx=%.2f, vy=%.2f, vz=%.2f m/s\n", vx_corr_2, vy_corr_2, vz_corr_2);
        v2_norm = sqrtf(vx_corr_2*vx_corr_2 + vy_corr_2*vy_corr_2 + vz_corr_2*vz_corr_2);
        printf("3点修正后速度模: |v|=%.2f m/s\n", v2_norm);
        
        // 计算速度夹角
        vel_angle_change = calc_angle_change(vx_corr_1, vy_corr_1, vz_corr_1,
                                             vx_corr_2, vy_corr_2, vz_corr_2);
        printf("速度变化夹角: %.2f° (阈值=%.0f°): %s\n", 
               vel_angle_change, ANGLE_CHANGE_THRESHOLD,
               vel_angle_change <= ANGLE_CHANGE_THRESHOLD ? "PASS" : "FAIL!");
        
        // 质量检查
        printf("\n3点速度质量检查 (calculate_vr_quality):\n");
        printf("  vr0=%.4f, vr1=%.4f, vr2=%.4f\n", 
               p0->velocity, p1->velocity, p2->velocity);
        track_quality = simulate_calculate_vr_quality(
            p0->velocity, p1->velocity, p2->velocity);
        printf("  质量=%.3f (阈值=%.1f): %s\n", 
               track_quality, TRACK_QUALITY_THRESHOLD,
               track_quality >= TRACK_QUALITY_THRESHOLD ? "PASS" : "FAIL!");
        
        // 几何门限检查
        printf("\n几何门限检查:\n");
        {
            float avg_azi, avg_el, avg_r;
            float new_azi, new_el, new_r;
            float dazi, del, dr;
            
            avg_azi = (p0->azi + p1->azi) / 2.0f;
            avg_el = (p0->ele + p1->ele) / 2.0f;
            avg_r = (p0->range + p1->range) / 2.0f;
            
            new_azi = p2->azi;
            new_el = p2->ele;
            new_r = p2->range;
            
            dazi = fabsf(new_azi - avg_azi);
            del = fabsf(new_el - avg_el);
            dr = fabsf(new_r - avg_r);
            
            printf("  方位差: %.4f° (阈值5°): %s\n", dazi, dazi < 5.0f ? "PASS" : "FAIL!");
            printf("  俯仰差: %.4f° (阈值5°): %s\n", del, del < 5.0f ? "PASS" : "FAIL!");
            printf("  距离差: %.1fm (阈值500m): %s\n", dr, dr < 500.0f ? "PASS" : "FAIL!");
        }
        
        // 置信度计算
        printf("\n置信度计算:\n");
        confidence = simulate_calculate_confidence_for_track(
            p0->velocity, p1->velocity, p2->velocity,
            vx_corr_1, vy_corr_1, vz_corr_1,
            vx_corr_2, vy_corr_2, vz_corr_2,
            2.0f, 1500.0f);  // Vmin=2, Vmax=1500
        
        printf("  置信度=%.3f (阈值=%.2f): %s\n", 
               confidence, CONFIDENCE_THRESHOLD,
               confidence >= CONFIDENCE_THRESHOLD ? "PASS" : "FAIL!");
        
        // 综合检查
        printf("\n=== 综合判断 ===\n");
        {
            int all_pass = 1;
            float T_span;
            
            T_span = (p2->mSecond - p0->mSecond) / 1000.0f;
            printf("时间跨度: %.3fs (阈值<25s): %s\n", T_span, 
                   T_span < 25.0f ? "PASS" : "FAIL!");
            if(T_span >= 25.0f) all_pass = 0;
            
            printf("位置夹角: %s\n", pos_angle_change <= ANGLE_CHANGE_THRESHOLD ? "PASS" : "FAIL!");
            if(pos_angle_change > ANGLE_CHANGE_THRESHOLD) all_pass = 0;
            
            printf("速度夹角: %s\n", vel_angle_change <= ANGLE_CHANGE_THRESHOLD ? "PASS" : "FAIL!");
            if(vel_angle_change > ANGLE_CHANGE_THRESHOLD) all_pass = 0;
            
            printf("VR质量: %s\n", track_quality >= TRACK_QUALITY_THRESHOLD ? "PASS" : "FAIL!");
            if(track_quality < TRACK_QUALITY_THRESHOLD) all_pass = 0;
            
            printf("置信度: %s\n", confidence >= CONFIDENCE_THRESHOLD ? "PASS" : "FAIL!");
            if(confidence < CONFIDENCE_THRESHOLD) all_pass = 0;
            
            // 低速检查
            printf("低速检查 (1-30m/s): ");
            if(fabsf(p0->velocity) >= 1.0f && fabsf(p0->velocity) <= 30.0f &&
               fabsf(p1->velocity) >= 1.0f && fabsf(p1->velocity) <= 30.0f &&
               fabsf(p2->velocity) >= 1.0f && fabsf(p2->velocity) <= 30.0f) {
                printf("PASS (low_speed_pass=1)\n");
            } else {
                printf("FAIL\n");
                all_pass = 0;
            }
            
            printf("\n>>> 3点初始化结果: %s\n", all_pass ? "成功!" : "失败!");
        }
    }
    
    // 分析Frame 4+5+38组合
    printf("\n\n3. 分析备选3点组合 (Frame 4 + 5 + 38):\n");
    {
        DronePoint *p0 = &drone_points[0];  // Frame 4
        DronePoint *p1 = &drone_points[1];  // Frame 5
        DronePoint *p2 = &drone_points[4];  // Frame 38
        
        float T_12;
        float vx_corr, vy_corr, vz_corr;
        float pos_angle, vel_angle;
        float v1x, v1y, v1z, v2x, v2y, v2z;
        float track_q, conf;
        
        T_12 = calc_T(p1, p2);
        printf("时间间隔 T_12 = %.3f s\n", T_12);
        
        // 位置夹角
        pos_angle = calc_position_angle_change(p0, p1, p2);
        printf("位置变化夹角: %.2f° (阈值=%.0f°): %s\n", 
               pos_angle, ANGLE_CHANGE_THRESHOLD,
               pos_angle <= ANGLE_CHANGE_THRESHOLD ? "PASS" : "FAIL!");
        
        // 3点速度修正
        simulate_velocity_correction(p1, p2, T_12, &vx_corr, &vy_corr, &vz_corr);
        v2x = vx_corr; v2y = vy_corr; v2z = vz_corr;
        
        // 获取2点速度
        {
            float vx2, vy2, vz2, T01;
            T01 = calc_T(p0, p1);
            simulate_velocity_correction(p0, p1, T01, &vx2, &vy2, &vz2);
            v1x = vx2; v1y = vy2; v1z = vz2;
        }
        
        vel_angle = calc_angle_change(v1x, v1y, v1z, v2x, v2y, v2z);
        printf("速度变化夹角: %.2f° (阈值=%.0f°): %s\n", 
               vel_angle, ANGLE_CHANGE_THRESHOLD,
               vel_angle <= ANGLE_CHANGE_THRESHOLD ? "PASS" : "FAIL!");
        
        track_q = simulate_calculate_vr_quality(
            p0->velocity, p1->velocity, p2->velocity);
        printf("VR质量: %.3f (阈值=%.1f): %s\n", 
               track_q, TRACK_QUALITY_THRESHOLD,
               track_q >= TRACK_QUALITY_THRESHOLD ? "PASS" : "FAIL!");
        
        conf = simulate_calculate_confidence_for_track(
            p0->velocity, p1->velocity, p2->velocity,
            v1x, v1y, v1z, v2x, v2y, v2z,
            2.0f, 1500.0f);
        printf("置信度: %.3f (阈值=%.2f): %s\n", 
               conf, CONFIDENCE_THRESHOLD,
               conf >= CONFIDENCE_THRESHOLD ? "PASS" : "FAIL!");
    }
    
    return 0;
}
#endif /* DIAGNOSE_DRONE_INIT */
