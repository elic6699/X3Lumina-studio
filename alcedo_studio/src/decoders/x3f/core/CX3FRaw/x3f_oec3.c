/* ====================================================================
 * OECVer3 逐位复刻（2026-10-27）—— SPP dylib 完全破译。
 *
 * 对质状态：oec_rep.c（本文件核心的验证底稿，commit e39e2a5）vs dylib
 * 真值全图 16,215,552 floats 0 diff；g4 合成探针亦逐位。机制详见
 * 。
 *
 * 机制要点（ + 探针实证）：
 *  1) hlp 预处理（前 453 行）：m=max(R*a0,G*a1,B*a2)；t=clip((m-44000)/12000,0,1)；
 *     trans_ch = fmaf(hlp_ch, 1-t, m*t/a_ch)。
 *  2) 主图 2944x1836 原地逐行光栅：t=(max-11000)/3000，t<=0 跳过（输出=输入）；
 *     t 不钳位、统一作用于三通道混合；逐通道 t_ch 只进 A/B 系数。
 *  3) 8x8 hlp 格窗口（行 [y/4-4, y/4+4) 底边钳 453；列 start=0 若 x<49
 *     否则 ((x-64)<0 ? x-49 : x-64)>>4，end=184 若 x>2895 否则 (x+64)>>4）。
 *     逐格惩罚 = max_ch( (trans_ch-main_ch)^2 * coef_ch )（正域），coef
 *     上/下非对称：上 A_ch=t_ch*1e-10+(1-t_ch)*1e-6，下 B_ch=t_ch*0.01+(1-t_ch)*1e-6；
 * 负域取 min（fcmgt+bif=取小）后 floor -25.328436，
 *     再加空间项 -0.0015*d2（d2 主图像素距离，int32），w=expf(arg)。
 *     up = sum(w*trans)/max(sum w, 9.24e-22)；空窗口 up=0 但 winner 块仍执行。
 *  4) min9（flat 偏移，OOB=0；连续缓冲回绕语义）：{i±W±1, i-W, i+W, i-1, i+1, i}，
 *     其中 i-1 = 当前行左邻【已改写输出】。门控：up>=min9 任一通道 或
 *     max(up) >= 0.5*max(min9) → 用窗口平均；否则 winner 搜索 5 组
 *     {G1=(i-W-1) G2=(i-W) G3=(i-W+1) G4=(i-1) G5=中心}，得分=最大通道值，
 *     严格大于才换（tie 取最早）；G5 胜 → path B 灰化回注：
 *     t2=clip((mx-11000)/3000,0,1)，up_ch=fmaf(1-t2, main_ch, mx*t2/a_ch)；
 *     其余 up=winner 像素原值。G1-3/G4 是已改写输出 → 火核恒定值传播链。
 * 5) out = clip(fmaf(t, up, main*(1-t)), 0, 65535)。
 *
 * 编译注意：#pragma clang fp contract(off)（文件级）——实证无此 pragma 时
 * sr += w*tvR 等会被收缩成 fma 破坏逐位一致性；加了 pragma 与
 * -ffp-contract=off 编译结果逐位相同（已验证）。除法/maf 顺序照抄底稿。
 *
 * 在我们域上的对应物（与 2026-10-26 版一致，待任务②标定）：
 *  - main16 = clip(2x2 均值(lin)*K_c, 0, 14000)，K_c=SCALE*14000/clip_c；
 *  - thumb T = est*K 的 16x4 块均值（不裁剪），前 453 行做 hlp 预处理；
 *  - 折回全分辨率：lin_new = s9b*Rb_c/K_c + (1-s9b)*lin_c（half 场双线性）。
 *
 * 门控：X3F_OEC3=1 启用（默认关，零回归）。env 调参：
 * X3F_OEC3_SCALE / _CLIP / _TGAIN / _TLUT / _WB / _DUMP / _TDUMP / _LINDUMP。
 * ==================================================================== */
#pragma clang fp contract(off)

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "x3f_oec3.h"

static float oec3_bit(uint32_t b)
{
    float f;
    memcpy(&f, &b, 4);
    return f;
}

#define OEC3_FW 2944
#define OEC3_FH 1836
#define OEC3_AX0 108
#define OEC3_AY0 14
#define OEC3_AW 2712
#define OEC3_AH 1808

static inline float oec3_getmod(const float *buf, long i, long n)
{
    return (i >= 0 && i < n) ? buf[i] : 0.0f;
}

static inline float oec3_max3(float a, float b, float c)
{
    float m = a > b ? a : b;
    return m > c ? m : c;
}

/* ---- X3F_OEC3_TCURVE：逐通道值依赖曲线（2501 抓包 T->hlp 40 分位拟合） ----
 * SPP ApplyLUTColorShadingGaintoHLPs 行为实证：T/hlp 比值空间平坦但随幅度
 * 强变化（G 单调降 1.04->0.71，R/B 先降后升）——每通道一条曲线。
 * 拟合 R2 0.47~0.69；端点外平延伸。 */
static float oec3_tcurve(float v, const float *xs, const float *ys, int n)
{
    if (v <= xs[0]) return ys[0];
    if (v >= xs[n - 1]) return ys[n - 1];
    int lo = 0, hi = n - 1;
    while (hi - lo > 1) {
        int mid = (lo + hi) >> 1;
        if (xs[mid] <= v) lo = mid; else hi = mid;
    }
    float w = (v - xs[lo]) / (xs[hi] - xs[lo]);
    return ys[lo] + w * (ys[hi] - ys[lo]);
}

static const float oec3_tc_Rx[40] = {
    854.2f, 946.5f, 1057.0f, 1201.3f, 1385.5f, 1584.1f, 1790.7f, 1986.9f,
    2184.7f, 2358.8f, 2529.0f, 2683.7f, 2823.3f, 2962.7f, 3108.5f, 3238.2f,
    3369.7f, 3501.3f, 3627.9f, 3759.7f, 3878.6f, 4010.6f, 4145.2f, 4268.3f,
    4406.8f, 4562.5f, 4727.8f, 4921.3f, 5127.1f, 5412.9f, 5705.6f, 6050.9f,
    6687.4f, 7576.2f, 8840.7f, 10736.6f, 12294.8f, 13483.3f, 19948.7f, 30981.8f
};
static const float oec3_tc_Ry[40] = {
    1150.4f, 1230.8f, 1336.5f, 1552.9f, 1910.8f, 2207.2f, 2421.2f, 2900.2f,
    3134.6f, 3512.9f, 3677.5f, 3712.8f, 3934.7f, 4233.3f, 4355.5f, 4498.4f,
    4564.5f, 4680.8f, 5091.6f, 5091.6f, 5233.2f, 5290.8f, 5536.7f, 5622.2f,
    5694.5f, 5790.3f, 6053.3f, 6336.4f, 6491.3f, 6491.3f, 6599.3f, 7113.6f,
    7228.8f, 7264.2f, 7695.5f, 7963.9f, 13396.0f, 14198.1f, 19543.7f, 48215.7f
};
static const float oec3_tc_Gx[40] = {
    875.2f, 998.1f, 1136.1f, 1285.6f, 1466.3f, 1707.5f, 2016.1f, 2405.0f,
    2853.9f, 3321.5f, 3778.0f, 4172.3f, 4556.4f, 4910.8f, 5267.0f, 5573.7f,
    5860.9f, 6120.4f, 6418.8f, 6675.2f, 6920.9f, 7151.5f, 7389.8f, 7624.8f,
    7864.2f, 8130.5f, 8388.9f, 8647.7f, 8959.0f, 9320.2f, 9698.2f, 10146.1f,
    10702.8f, 11446.2f, 12545.9f, 14086.0f, 14759.4f, 19369.9f, 28985.9f, 44483.5f
};
static const float oec3_tc_Gy[40] = {
    995.3f, 1030.0f, 1138.3f, 1276.2f, 1392.4f, 1678.9f, 1999.9f, 2382.4f,
    2773.4f, 3244.6f, 3581.2f, 3871.9f, 3964.6f, 4210.2f, 4747.4f, 4866.0f,
    5121.0f, 5121.0f, 5331.5f, 5461.5f, 5951.3f, 5951.3f, 6114.8f, 6285.9f,
    6514.6f, 6589.2f, 6793.3f, 6963.4f, 7198.6f, 7437.6f, 7501.8f, 7791.1f,
    8081.7f, 8416.3f, 8416.3f, 8820.6f, 12162.7f, 12162.7f, 16714.6f, 36706.4f
};
static const float oec3_tc_Bx[40] = {
    865.3f, 1028.4f, 1244.2f, 1479.5f, 1748.4f, 1949.3f, 2149.8f, 2384.1f,
    2571.1f, 2768.5f, 2951.3f, 3125.9f, 3310.1f, 3492.6f, 3673.1f, 3833.2f,
    3996.6f, 4143.9f, 4303.0f, 4450.9f, 4598.1f, 4761.4f, 4892.8f, 5041.2f,
    5168.9f, 5311.5f, 5474.7f, 5655.6f, 5849.6f, 6045.0f, 6247.5f, 6499.2f,
    6775.3f, 7086.4f, 7542.4f, 8269.8f, 9647.0f, 12474.5f, 17889.8f, 24529.8f
};
static const float oec3_tc_By[40] = {
    1391.2f, 1694.7f, 2103.9f, 2501.3f, 2889.8f, 2926.4f, 3535.4f, 3680.0f,
    4244.3f, 4244.3f, 4244.3f, 4244.3f, 4386.0f, 4559.4f, 4681.9f, 4774.1f,
    4966.6f, 4966.6f, 5104.7f, 5209.2f, 5309.1f, 5606.7f, 5606.7f, 5606.7f,
    5606.7f, 5834.7f, 5959.4f, 6045.7f, 6278.2f, 6404.3f, 6534.6f, 6680.6f,
    6831.1f, 7070.5f, 7205.0f, 7615.2f, 8677.3f, 8677.3f, 14431.6f, 23517.9f
};

/* ---- X3F_OEC3_TLUT：稠密值域 LUT（128 节点/通道，2501 抓包 T_g1->hlp_in_R1
 * 全帧 40 分位 + 焰区 20 倍加权拟合，2026-10-27）。替代标量 TGAIN：帧级受控实验
 * （SPP main 固定，oec_rep 逐位核心）焰核 MAE 8047->7195、焰区 1437->1420，
 * 焰核 B/R 0.706->0.58（真值 0.576）。中间调轻微劣化（main 内容误差主导）。
 * 128 节点=焰区精度与表体积折中（96/192 差<2%）。 */
static const float oec3_tl0_x[127] = {
-18.2321f,6.96919f,10.8044f,13.5387f,16.0962f,18.82f,21.9315f,25.2447f,28.698f,32.0497f,35.3737f,38.4366f,41.1924f,44.0122f,47.024f,50.0071f,53.0825f,56.0987f,59.5181f,62.4191f,65.4496f,68.3602f,71.7792f,75.5631f,79.0018f,82.1524f,85.4952f,88.1293f,90.9101f,93.8703f,96.6818f,99.7444f,103.19f,106.644f,109.681f,112.741f,116.21f,119.519f,122.708f,126.166f,129.377f,133.006f,136.492f,140.239f,144.4f,147.725f,151.242f,155.982f,160.201f,164.251f,168.335f,171.887f,175.856f,179.41f,183.505f,186.825f,190.952f,195.703f,200.479f,206.255f,212.078f,217.0f,223.51f,228.088f,232.881f,239.411f,244.866f,250.797f,257.795f,265.554f,272.932f,282.494f,290.411f,299.339f,309.57f,320.156f,332.269f,346.285f,358.728f,373.239f,388.209f,406.654f,426.271f,443.718f,462.375f,483.27f,504.296f,533.123f,581.147f,639.126f,719.285f,813.777f,924.833f,1101.12f,1394.88f,1695.87f,1996.62f,2303.48f,2597.32f,2856.5f,3082.36f,3281.38f,3458.15f,3684.12f,3869.43f,4106.74f,4325.33f,4601.96f,4920.68f,5321.13f,5817.42f,6555.3f,7737.55f,9182.62f,10828.3f,11930.8f,12294.8f,12295.4f,12317.2f,12816.2f,14795.0f,18234.5f,22916.8f,27336.3f,31315.3f,32140.4f,32212.4f
};
static const float oec3_tl0_y[127] = {
-5937.07f,-72.0096f,-45.0665f,-34.0504f,-20.9208f,-13.9016f,-3.5229f,0.0f,6.99547f,14.2964f,20.628f,24.4311f,31.6534f,35.4205f,38.9705f,42.7089f,46.2797f,53.4812f,57.3585f,60.7976f,64.1572f,67.8765f,74.9954f,78.3298f,81.3777f,84.6568f,91.2058f,94.5821f,97.0789f,100.456f,103.646f,107.038f,113.502f,116.592f,120.031f,123.252f,126.801f,131.513f,135.801f,139.145f,141.922f,145.143f,151.597f,154.921f,158.437f,161.509f,164.592f,167.93f,174.791f,178.31f,181.557f,184.965f,189.328f,195.898f,199.41f,203.628f,207.748f,214.278f,218.438f,222.588f,227.056f,232.689f,238.159f,243.018f,247.407f,255.096f,260.673f,268.526f,275.496f,280.704f,286.266f,295.223f,302.115f,308.022f,318.866f,325.773f,336.313f,345.163f,357.051f,368.41f,381.522f,394.314f,407.483f,423.278f,440.323f,456.032f,474.9f,496.954f,520.359f,546.946f,576.176f,612.911f,651.505f,696.141f,771.957f,869.418f,996.374f,1157.98f,1397.38f,1670.45f,2046.87f,2465.4f,2930.86f,3373.56f,3830.28f,4249.89f,4662.31f,5057.22f,5418.84f,5851.45f,6199.79f,6650.79f,7282.87f,7985.16f,8885.18f,10222.7f,12069.7f,18156.8f,21024.4f,23131.6f,26312.9f,29655.9f,32831.2f,38860.2f,45701.0f,51522.2f,51808.5f
};

static const float oec3_tl1_x[129] = {
15.1407f,24.7901f,30.5628f,39.6271f,45.1743f,48.9854f,51.8462f,54.992f,58.3633f,61.4313f,64.5545f,67.3189f,69.9803f,72.7239f,75.5095f,78.2011f,81.1882f,84.0372f,86.9121f,89.7959f,92.8504f,96.2791f,99.7048f,103.08f,106.486f,109.719f,113.698f,116.893f,119.566f,121.617f,124.484f,127.227f,130.107f,132.788f,135.031f,138.263f,140.866f,143.713f,146.713f,150.098f,153.422f,158.06f,162.076f,165.781f,169.849f,173.3f,177.832f,182.557f,186.255f,190.358f,194.284f,198.307f,202.414f,206.532f,210.333f,214.235f,218.977f,224.003f,229.343f,233.439f,238.908f,243.731f,249.121f,255.543f,261.587f,267.99f,274.244f,280.236f,287.587f,293.941f,301.754f,309.533f,317.883f,328.078f,339.237f,353.278f,367.403f,384.667f,402.205f,418.411f,437.513f,454.852f,473.858f,494.988f,516.844f,541.881f,574.336f,619.418f,673.643f,739.913f,811.277f,914.071f,1042.54f,1215.86f,1455.7f,1736.64f,1980.65f,2425.21f,2983.38f,3579.4f,4267.71f,4898.39f,5569.43f,6031.06f,6499.89f,6921.05f,7289.84f,7713.94f,8148.43f,8574.84f,9108.73f,9743.68f,10547.1f,11786.8f,12900.9f,14425.4f,14606.4f,14606.4f,14608.8f,14706.6f,15200.2f,17278.9f,21140.1f,27051.5f,33658.5f,39946.6f,44928.1f,45832.1f,45895.8f
};
static const float oec3_tl1_y[129] = {
-4642.71f,-69.9776f,-45.1579f,-34.3549f,-22.6718f,-15.3623f,-7.46712f,3.73763f,7.575f,15.3648f,19.3707f,23.5749f,33.4857f,37.1757f,39.4432f,43.2288f,46.8059f,50.3256f,57.321f,60.6068f,63.7199f,66.4475f,69.6064f,72.6776f,79.0932f,82.4287f,85.4539f,88.5868f,91.0214f,97.1857f,100.901f,103.75f,106.261f,109.357f,112.781f,115.85f,121.579f,125.166f,128.292f,131.4f,133.078f,136.185f,139.257f,143.671f,148.085f,150.891f,153.735f,156.211f,161.446f,167.078f,169.937f,172.99f,175.96f,179.679f,184.019f,190.338f,194.11f,196.991f,200.765f,204.716f,211.951f,215.616f,219.502f,223.959f,232.087f,237.164f,241.042f,245.416f,251.605f,256.246f,260.442f,264.833f,271.053f,281.715f,288.995f,299.403f,305.968f,314.886f,325.494f,336.073f,349.694f,363.708f,377.034f,392.648f,410.886f,430.446f,448.339f,467.828f,491.689f,520.886f,554.443f,588.84f,633.553f,679.967f,747.62f,822.193f,943.256f,1085.6f,1274.65f,1584.12f,1890.05f,2279.32f,2785.37f,3277.64f,3782.48f,4235.65f,4610.36f,5082.36f,5503.38f,5903.16f,6337.18f,6795.59f,7239.26f,7758.79f,8461.4f,9180.44f,10155.2f,11221.2f,12443.4f,13971.6f,15667.6f,17334.0f,19414.7f,22003.7f,24512.9f,28869.3f,34100.5f,40424.5f,56475.8f
};

static const float oec3_tl2_x[129] = {
-16.4678f,-8.07001f,-5.19273f,-2.90369f,-0.788634f,0.977868f,2.66053f,4.19325f,5.61512f,7.04879f,8.42348f,9.69228f,10.9345f,12.1606f,13.3948f,14.4809f,15.6252f,16.8728f,18.0633f,19.178f,20.3113f,21.3769f,22.3805f,23.411f,24.5206f,25.4371f,26.64f,27.7513f,28.8331f,29.9091f,30.9596f,31.9902f,33.0866f,34.1838f,35.3308f,36.3194f,37.5613f,38.7814f,39.7954f,41.0348f,41.9481f,42.9003f,43.9876f,45.1771f,46.3395f,47.5684f,48.9762f,50.6387f,52.2226f,53.8953f,56.0284f,57.7733f,59.9607f,62.188f,64.3335f,66.3118f,68.1464f,70.4132f,72.8206f,74.9915f,77.2744f,79.5029f,82.2363f,84.7957f,87.6383f,90.847f,94.2305f,97.7243f,100.988f,104.917f,109.163f,112.772f,117.016f,121.867f,126.729f,131.671f,137.253f,142.032f,148.846f,156.001f,165.311f,174.032f,184.951f,195.147f,206.765f,219.889f,232.145f,249.22f,265.258f,286.62f,309.984f,341.991f,374.818f,419.192f,484.117f,559.248f,654.148f,787.542f,1026.94f,1432.02f,1867.35f,2203.88f,2539.32f,2892.67f,3267.55f,3609.9f,3968.34f,4211.54f,4492.41f,4801.47f,4973.82f,5057.92f,5145.29f,5278.94f,5461.8f,5728.68f,6031.59f,6406.12f,6875.76f,7456.93f,8547.25f,9703.59f,11622.7f,14532.0f,18012.8f,21476.5f,24241.6f,24814.9f,24862.6f
};
static const float oec3_tl2_y[129] = {
-49.7628f,0.0f,0.0f,7.68569f,10.1872f,10.975f,12.2789f,15.2078f,16.8941f,19.0376f,21.213f,23.0137f,25.3848f,27.0582f,28.9108f,30.992f,33.2444f,35.2212f,37.5082f,39.6281f,42.145f,44.5533f,46.5444f,49.021f,51.2396f,53.5313f,55.406f,57.7663f,59.8697f,62.0242f,64.5016f,66.3316f,68.9642f,71.2456f,73.3575f,75.7908f,77.7031f,79.9373f,81.8074f,84.3904f,86.5871f,88.8334f,91.3457f,93.2228f,95.7859f,98.1669f,100.163f,102.956f,105.735f,108.095f,110.598f,113.086f,115.627f,118.448f,121.28f,124.166f,127.346f,130.465f,133.821f,136.419f,139.734f,143.116f,146.614f,149.868f,152.719f,156.453f,159.856f,163.547f,167.568f,171.665f,176.53f,181.884f,187.79f,192.617f,198.929f,206.259f,212.157f,219.083f,227.086f,236.201f,245.233f,254.059f,264.432f,275.905f,287.604f,300.6f,314.922f,333.031f,353.418f,376.443f,401.388f,427.965f,458.666f,499.098f,543.742f,613.007f,691.034f,794.451f,928.338f,1103.04f,1309.51f,1615.33f,1889.55f,2248.13f,2565.42f,2942.69f,3290.29f,3661.5f,4076.68f,4471.23f,4862.76f,5201.47f,5577.88f,5953.04f,6283.96f,6643.59f,7109.63f,7570.91f,8131.57f,8867.52f,9554.42f,10666.2f,11808.2f,13161.5f,14492.3f,17439.9f,20089.7f,24204.9f,69089.5f
};


float *x3f_oec3_envelope(const float *lin, const float *est, int w, int h)
{
    /* ---- 参数（env 可覆盖） ---- */
    double clipc[3] = { 2.0099, 1.4148, 2.8789 };   /* own 白电平平台 */
    double scale = 1.34;                             /* Blob IoU 标定 */
    /* thumb 深度增益（逐通道）。默认=2501 抓包 hlp_in_R1 一阶标定
     * （2026-10-27：T vs SPP hlp 中位比值 R0.753/G1.080/B0.744 的倒数，
     * G 再经焰内 MAE 坐标搜索回调）。2501 实测：焰内 |d|MAE 8.105→7.296
     * （v8b 9.201）、全图 3.468（v8b 3.489）；0337 低焰图无回归。
     * 属一阶域校正——结构性修复（任务② hlp 重建标定）落地后应重标或移除。 */
    double tgain[3] = { 1.3275, 0.9261, 1.20 };
    double wbd[3] = { 1.0, 1.0, 1.0 };               /* wb 中性化（=1 逐位域） */
    /* 半分辨率 develop 链 —— 色度域（帧域 3x3：我方 lin 空间 -> SPP main 空间）。
     * 由来：以 2501 抓包的 SPP OECVer3 输入缓冲 main_in_R1（active 配准
     * offset (14,108)）为真值，对 box2x2(lin) 逐通道最小二乘标定，
     * 样本剔除饱和像素（Y<13900 且 diag(K)·X<13900）以免 14000 硬裁引入偏置。
     * 9 自由 / 4.9M 样本，5% 子集间逐元素相对离散度 7e-4 —— 视为确定性变换。
     * 白平衡无关性：WB 是对角阵且两侧同序作用会被约掉，
     *   故 M = CCM_spp · CCM_ours^{-1}，可跨 WB 设置复用。
     * 默认=标定矩阵（2026-10-28 收口：端到端 2501 全图 7.274->7.097、
     *   亮部 11.86->10.67，0337 逐位相同=零回归）；X3F_OEC3_CCM 行优先覆盖。
     * 2501 实测：帧域 MAE 256.7 -> 13.1；过 Stage2 后 1920.6 -> 150.1
     *   （SPP main 自洽参照 186.3）。 */
    double ccmd[9] = { 6231.79, 3342.92, 1465.81,
                       5439.15, 7583.60, 4829.19,
                       -353.75,  857.67, 2758.14 };
    int ccm_given = 0;
    const char *e;
    if ((e = getenv("X3F_OEC3_SCALE")) != NULL && *e != '\0') scale = atof(e);
    if ((e = getenv("X3F_OEC3_TGAIN")) != NULL && *e != '\0') {
        double g1 = 1.0, g2 = 0, g3 = 0;
        int nf = sscanf(e, "%lf,%lf,%lf", &g1, &g2, &g3);
        if (nf == 1) { tgain[0] = tgain[1] = tgain[2] = g1; }
        else if (nf == 3) { tgain[0] = g1; tgain[1] = g2; tgain[2] = g3; }
    }
    if ((e = getenv("X3F_OEC3_CLIP")) != NULL && *e != '\0') {
        if (sscanf(e, "%lf,%lf,%lf", &clipc[0], &clipc[1], &clipc[2]) != 3)
            return NULL;
    }
    if ((e = getenv("X3F_OEC3_WB")) != NULL && *e != '\0') {
        if (sscanf(e, "%lf,%lf,%lf", &wbd[0], &wbd[1], &wbd[2]) != 3)
            return NULL;
    }
    if ((e = getenv("X3F_OEC3_CCM")) != NULL && *e != '\0') {
        if (sscanf(e, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                   &ccmd[0], &ccmd[1], &ccmd[2], &ccmd[3], &ccmd[4],
                   &ccmd[5], &ccmd[6], &ccmd[7], &ccmd[8]) != 9)
            return NULL;
        ccm_given = 1;
    }
    float K[3], wb[3];
    for (int c = 0; c < 3; c++) {
        K[c] = (float)(scale * 14000.0 / clipc[c]);
        wb[c] = (float)wbd[c];
    }
    /* K 仍用于 thumb（est·K 块均值）；main16 已默认走标定 CCM（diag(K) 旧路废弃） */
    /* 算法常量（与 disasm 位精确） */
    const float C_FLOOR = oec3_bit(0xC1CAA090u);   /* -25.328436（=ln 1e-11） */
    const float C_SP    = oec3_bit(0xBAC49BA6u);   /* -0.0015 */
    const float C_TINY  = oec3_bit(0x1C971DA0u);   /* ~9.24e-22（sum w 下限） */
    const float C_1E6   = oec3_bit(0x358637BDu);
    const float C_1E10  = oec3_bit(0x2EDBE6FFu);
    const float C_001   = 0.01f;
    const float C_FMAX  = oec3_bit(0x477FFF00u);   /* 65535 */

    if (w != 2 * OEC3_AW || h != 2 * OEC3_AH) {
        fprintf(stderr, "[oec3] geometry %dx%d unsupported, skip\n", w, h);
        return NULL;
    }
    const size_t fnn = (size_t)OEC3_FW * OEC3_FH;
    float *main16 = (float *)malloc(fnn * 3 * sizeof(float));
    float *T = (float *)malloc((size_t)459 * 184 * 3 * sizeof(float));
    float *s9f = (float *)calloc((size_t)OEC3_AW * OEC3_AH, sizeof(float));
    float *Rf = (float *)calloc((size_t)OEC3_AW * OEC3_AH * 3, sizeof(float));
    float *outb = (float *)malloc((size_t)w * h * 3 * sizeof(float));
    if (!main16 || !T || !s9f || !Rf || !outb) {
        free(main16); free(T); free(s9f); free(Rf); free(outb);
        return NULL;
    }

    /* ---- 1. 主图 half16（帧坐标，active 外边缘复制，clip 0..14000） ---- */
    for (int fy = 0; fy < OEC3_FH; fy++) {
        int ay = fy - OEC3_AY0;
        if (ay < 0) ay = 0; else if (ay >= OEC3_AH) ay = OEC3_AH - 1;
        const float *l0 = lin + (size_t)(2 * ay) * w * 3;
        const float *l1 = l0 + (size_t)w * 3;
        for (int fx = 0; fx < OEC3_FW; fx++) {
            int ax = fx - OEC3_AX0;
            if (ax < 0) ax = 0; else if (ax >= OEC3_AW) ax = OEC3_AW - 1;
            const float *p00 = l0 + (size_t)(2 * ax) * 3;
            const float *p01 = p00 + 3;
            const float *p10 = l1 + (size_t)(2 * ax) * 3;
            const float *p11 = p10 + 3;
            float *d = main16 + ((size_t)fy * OEC3_FW + fx) * 3;
            const float m0 = (p00[0] + p01[0] + p10[0] + p11[0]) * 0.25f;
            const float m1 = (p00[1] + p01[1] + p10[1] + p11[1]) * 0.25f;
            const float m2 = (p00[2] + p01[2] + p10[2] + p11[2]) * 0.25f;
            for (int c = 0; c < 3; c++) {
                float v = (float)(m0 * ccmd[c] + m1 * ccmd[3 + c]
                                + m2 * ccmd[6 + c]);
                d[c] = v < 0.0f ? 0.0f : (v > 14000.0f ? 14000.0f : v);
            }
        }
    }

    /* ---- 2. thumb：est*K 的 16x4 块均值（不裁剪；帧坐标 pad 同上） ---- */
    for (int ty = 0; ty < 459; ty++) {
        for (int tx = 0; tx < 184; tx++) {
            double acc[3] = { 0.0, 0.0, 0.0 };
            for (int dy = 0; dy < 4; dy++) {
                int fy = ty * 4 + dy;
                int ay = fy - OEC3_AY0;
                if (ay < 0) ay = 0; else if (ay >= OEC3_AH) ay = OEC3_AH - 1;
                for (int dx = 0; dx < 16; dx++) {
                    int fx = tx * 16 + dx;
                    int ax = fx - OEC3_AX0;
                    if (ax < 0) ax = 0; else if (ax >= OEC3_AW) ax = OEC3_AW - 1;
                    const float *p = est + ((size_t)(2 * ay) * w + 2 * ax) * 3;
                    const float *q = p + (size_t)w * 3;
                    for (int c = 0; c < 3; c++)
                        acc[c] += ((double)p[c] + q[c] + p[3 + c] + q[3 + c])
                                  * 0.25 * K[c];
                }
            }
            float *d = T + ((size_t)ty * 184 + tx) * 3;
            for (int c = 0; c < 3; c++) d[c] = (float)(acc[c] / 64.0 * tgain[c]);
        }
    }
    {   /* X3F_OEC3_TCURVE=1：逐通道值依赖曲线（拟合自 TGAIN=1 域，须配
         * X3F_OEC3_TGAIN=1 使用，否则双重校正） */
        const char *ec = getenv("X3F_OEC3_TCURVE");
        if (ec != NULL && *ec == '1') {
            for (int i = 0; i < 459 * 184; i++) {
                float *cell = T + (size_t)i * 3;
                cell[0] = oec3_tcurve(cell[0], oec3_tc_Rx, oec3_tc_Ry, 40);
                cell[1] = oec3_tcurve(cell[1], oec3_tc_Gx, oec3_tc_Gy, 40);
                cell[2] = oec3_tcurve(cell[2], oec3_tc_Bx, oec3_tc_By, 40);
            }
        }
    }
    {   /* X3F_OEC3_TLUT=1：稠密值域 LUT（替代标量 TGAIN，强制 tgain=1） */
        const char *elu = getenv("X3F_OEC3_TLUT");
        if (elu != NULL && *elu == '1') {
            tgain[0] = tgain[1] = tgain[2] = 1.0;
            for (int i = 0; i < 459 * 184; i++) {
                float *cell = T + (size_t)i * 3;
                cell[0] = oec3_tcurve(cell[0], oec3_tl0_x, oec3_tl0_y, 127);
                cell[1] = oec3_tcurve(cell[1], oec3_tl1_x, oec3_tl1_y, 129);
                cell[2] = oec3_tcurve(cell[2], oec3_tl2_x, oec3_tl2_y, 129);
            }
        }
    }
    {   /* [TEMP-DEBUG] thumb dump */
        const char *et = getenv("X3F_OEC3_TDUMP");
        if (et != NULL && *et != '\0') {
            FILE *ft = fopen(et, "wb");
            if (ft != NULL) {
                fwrite(T, sizeof(float), (size_t)459 * 184 * 3, ft);
                fclose(ft);
                fprintf(stderr, "[oec3] dump thumb -> %s\n", et);
            }
        }
    }
    {   /* [TEMP-DEBUG] input lin dump（量纲/坐标系对质用） */
        const char *el = getenv("X3F_OEC3_LINDUMP");
        if (el != NULL && *el != '\0') {
            FILE *fl = fopen(el, "wb");
            if (fl != NULL) {
                fwrite(lin, sizeof(float), (size_t)w * h * 3, fl);
                fclose(fl);
                fprintf(stderr, "[oec3] dump lin -> %s\n", el);
            }
        }
    }

    /* ---- 3. prologue：T 前 453 行 hlp 预处理（= oec_rep step 1，wb=1 逐位） ---- */
    for (int i = 0; i < 453 * 184; i++) {
        float *cell = T + (size_t)i * 3;
        float mr = cell[0] * wb[0];
        float mg = cell[1] * wb[1];
        float mb = cell[2] * wb[2];
        float m = mr > mg ? mr : mg; m = m > mb ? m : mb;
        float t = (m - 44000.0f) / 12000.0f;
        if (t > 1.0f) t = 1.0f;
        if (t < 0.0f) t = 0.0f;
        float mt = m * t, omt = 1.0f - t;
        for (int c = 0; c < 3; c++)
            cell[c] = fmaf(cell[c], omt, mt / wb[c]);
    }

    /* ---- 4. OECVer3 逐位核心（全帧原地逐行光栅，oec_rep.c 移植） ---- */
    const long NP = (long)OEC3_FW * OEC3_FH * 3;
    for (int y = 0; y < OEC3_FH; y++) {
        for (int x = 0; x < OEC3_FW; x++) {
            size_t i = (size_t)y * OEC3_FW + x;
            float *px = main16 + i * 3;
            float m0 = px[0], m1 = px[1], m2 = px[2];
            float mx = m0 > m1 ? m0 : m1; mx = mx > m2 ? mx : m2;
            float t = (mx - 11000.0f) / 3000.0f;
            if (t <= 0.0f) continue;               /* 输出=输入（原地） */

            float t0 = (m0 - 11000.0f) / 3000.0f;
            float t1 = (m1 - 11000.0f) / 3000.0f;
            float t2 = (m2 - 11000.0f) / 3000.0f;
            if (t0 > 1.0f) t0 = 1.0f; if (t0 < 0.0f) t0 = 0.0f;
            if (t1 > 1.0f) t1 = 1.0f; if (t1 < 0.0f) t1 = 0.0f;
            if (t2 > 1.0f) t2 = 1.0f; if (t2 < 0.0f) t2 = 0.0f;
            float A0 = fmaf(1.0f - t0, C_1E6, t0 * C_1E10);
            float B0 = fmaf(1.0f - t0, C_1E6, t0 * C_001);
            float A1 = fmaf(1.0f - t1, C_1E6, t1 * C_1E10);
            float B1 = fmaf(1.0f - t1, C_1E6, t1 * C_001);
            float A2 = fmaf(1.0f - t2, C_1E6, t2 * C_1E10);
            float B2 = fmaf(1.0f - t2, C_1E6, t2 * C_001);

            /*窗口（ 位精确；行底边钳 453）*/
            int r0 = y / 4 - 4; if (r0 < 0) r0 = 0;
            int r1 = (y < 1800) ? y / 4 + 4 : 453;
            int c0;
            if (x < 49) c0 = 0;
            else { int v = (x - 64 < 0) ? (x - 49) : (x - 64); c0 = v >> 4; }
            int c1 = (x > 2895) ? 184 : ((x + 64) >> 4);

            float sw = 0.0f, sr = 0.0f, sg = 0.0f, sb = 0.0f;
            for (int cr = r0; cr < r1; cr++) {
                int dy = 4 * cr - y, dy2 = dy * dy;
                const float *trow = &T[(size_t)cr * 184 * 3];
                for (int cc = c0; cc < c1; cc++) {
                    int dx = 16 * cc - x;
                    int32_t d2i = dy2 + dx * dx;
                    const float *tc = &trow[(size_t)cc * 3];
                    float tvR = tc[0], tvG = tc[1], tvB = tc[2];
                    float d0 = tvR - m0, d1 = tvG - m1, d3 = tvB - m2;
                    /* 负域惩罚取 min（= 正域 max）；floor 后加空间项 */
                    float p0 = -(d0 * d0) * (d0 > 0 ? A0 : B0);
                    float p1 = -(d1 * d1) * (d1 > 0 ? A1 : B1);
                    float p2 = -(d3 * d3) * (d3 > 0 ? A2 : B2);
                    float pen = p0 < p1 ? p0 : p1; pen = pen < p2 ? pen : p2;
                    if (pen < C_FLOOR) pen = C_FLOOR;
                    float arg = pen + C_SP * (float)d2i;
                    float wv = expf(arg);
                    sw += wv;
                    sr += wv * tvR;
                    sg += wv * tvG;
                    sb += wv * tvB;
                }
            }
            float den = sw < C_TINY ? C_TINY : sw;
            float upR = sr / den, upG = sg / den, upB = sb / den;

            /* min9（flat 偏移，OOB=0；i-1 = 当前行左邻【已改写输出】） */
            long ia = (long)i - OEC3_FW - 1, ib = (long)i - OEC3_FW,
                 ic = (long)i - OEC3_FW + 1;
            long id = (long)i - 1, ie = (long)i + 1;
            long ifa = (long)i + OEC3_FW - 1, ig = (long)i + OEC3_FW,
                 ihh = (long)i + OEC3_FW + 1;
            float minR = oec3_getmod(main16, ia * 3, NP);
            float v;
            v = oec3_getmod(main16, ib * 3, NP);     if (v < minR) minR = v;
            v = oec3_getmod(main16, ic * 3, NP);     if (v < minR) minR = v;
            v = oec3_getmod(main16, id * 3, NP);     if (v < minR) minR = v;
            v = oec3_getmod(main16, ie * 3, NP);     if (v < minR) minR = v;
            v = m0;                                   if (v < minR) minR = v;
            v = oec3_getmod(main16, ifa * 3, NP);    if (v < minR) minR = v;
            v = oec3_getmod(main16, ig * 3, NP);     if (v < minR) minR = v;
            v = oec3_getmod(main16, ihh * 3, NP);    if (v < minR) minR = v;
            float minG = oec3_getmod(main16, ia * 3 + 1, NP);
            v = oec3_getmod(main16, ib * 3 + 1, NP); if (v < minG) minG = v;
            v = oec3_getmod(main16, ic * 3 + 1, NP); if (v < minG) minG = v;
            v = oec3_getmod(main16, id * 3 + 1, NP); if (v < minG) minG = v;
            v = oec3_getmod(main16, ie * 3 + 1, NP); if (v < minG) minG = v;
            v = m1;                                   if (v < minG) minG = v;
            v = oec3_getmod(main16, ifa * 3 + 1, NP); if (v < minG) minG = v;
            v = oec3_getmod(main16, ig * 3 + 1, NP); if (v < minG) minG = v;
            v = oec3_getmod(main16, ihh * 3 + 1, NP); if (v < minG) minG = v;
            float minB = oec3_getmod(main16, ia * 3 + 2, NP);
            v = oec3_getmod(main16, ib * 3 + 2, NP); if (v < minB) minB = v;
            v = oec3_getmod(main16, ic * 3 + 2, NP); if (v < minB) minB = v;
            v = oec3_getmod(main16, id * 3 + 2, NP); if (v < minB) minB = v;
            v = oec3_getmod(main16, ie * 3 + 2, NP); if (v < minB) minB = v;
            v = m2;                                   if (v < minB) minB = v;
            v = oec3_getmod(main16, ifa * 3 + 2, NP); if (v < minB) minB = v;
            v = oec3_getmod(main16, ig * 3 + 2, NP); if (v < minB) minB = v;
            v = oec3_getmod(main16, ihh * 3 + 2, NP); if (v < minB) minB = v;

            float amaxmin = oec3_max3(minR, minG, minB);
            float amaxup  = oec3_max3(upR, upG, upB);

            int skip = (upR >= minR) || (upG >= minG) || (upB >= m2) ||
                       (upG >= m1) || (upR >= m0) || (upB >= minB) ||
                       (amaxup >= 0.5f * amaxmin);

            float fupR, fupG, fupB;
            if (skip) {
                fupR = upR; fupG = upG; fupB = upB;
            } else {
                /* winner 5 组：G1(row-1,x-1) G2(row-1,x) G3(row-1,x+1)
                 * G4(当前行,x-1 已改写) G5(中心)；得分=最大通道，严格大于才换 */
                float s1 = oec3_max3(oec3_getmod(main16, ia * 3, NP),
                                     oec3_getmod(main16, ia * 3 + 1, NP),
                                     oec3_getmod(main16, ia * 3 + 2, NP));
                float s2 = oec3_max3(oec3_getmod(main16, ib * 3, NP),
                                     oec3_getmod(main16, ib * 3 + 1, NP),
                                     oec3_getmod(main16, ib * 3 + 2, NP));
                float s3 = oec3_max3(oec3_getmod(main16, ic * 3, NP),
                                     oec3_getmod(main16, ic * 3 + 1, NP),
                                     oec3_getmod(main16, ic * 3 + 2, NP));
                float s4 = oec3_max3(oec3_getmod(main16, id * 3, NP),
                                     oec3_getmod(main16, id * 3 + 1, NP),
                                     oec3_getmod(main16, id * 3 + 2, NP));
                float s5 = oec3_max3(m0, m1, m2);
                float best = s1; long widx = ia;
                if (s2 > best) { best = s2; widx = ib; }
                if (s3 > best) { best = s3; widx = ic; }
                if (s4 > best) { best = s4; widx = id; }
                if (s5 > best) {
                    /* path B：灰化回注 */
                    float tt = (s5 - 11000.0f) / 3000.0f;
                    if (tt > 1.0f) tt = 1.0f;
                    if (tt < 0.0f) tt = 0.0f;
                    float st = s5 * tt, omt = 1.0f - tt;
                    fupR = fmaf(omt, m0, st / wb[0]);
                    fupG = fmaf(omt, m1, st / wb[1]);
                    fupB = fmaf(omt, m2, st / wb[2]);
                } else {
                    fupR = oec3_getmod(main16, widx * 3, NP);
                    fupG = oec3_getmod(main16, widx * 3 + 1, NP);
                    fupB = oec3_getmod(main16, widx * 3 + 2, NP);
                }
            }

            /* 记录折回场（active 区内） */
            {
                const int ay = y - OEC3_AY0, ax = x - OEC3_AX0;
                if (ay >= 0 && ay < OEC3_AH && ax >= 0 && ax < OEC3_AW) {
                    s9f[(size_t)ay * OEC3_AW + ax] = t;
                    Rf[((size_t)ay * OEC3_AW + ax) * 3 + 0] = fupR;
                    Rf[((size_t)ay * OEC3_AW + ax) * 3 + 1] = fupG;
                    Rf[((size_t)ay * OEC3_AW + ax) * 3 + 2] = fupB;
                }
            }

            /* 输出（原地写回，供后续像素 min9/winner 读取） */
            float omt = 1.0f - t;
            float oR = fmaf(t, fupR, m0 * omt);
            float oG = fmaf(t, fupG, m1 * omt);
            float oB = fmaf(t, fupB, m2 * omt);
            if (oR > C_FMAX) oR = C_FMAX; if (oR < 0.0f) oR = 0.0f;
            if (oG > C_FMAX) oG = C_FMAX; if (oG < 0.0f) oG = 0.0f;
            if (oB > C_FMAX) oB = C_FMAX; if (oB < 0.0f) oB = 0.0f;
            px[0] = oR; px[1] = oG; px[2] = oB;
        }
    }

    /* ---- 4b. [TEMP-DEBUG] X3F_OEC3_FBIN：外部帧覆盖（边界实验用）。
     * 把抓包真帧（如 s2out.bin，2944x1836 f32x3）灌进折回，测"完美帧 +
     * 我们下游"的渲染上限。s9 场仍取自本 run（空间近似）。 */
    int fbin_mode = 0;
    {
        const char *eb = getenv("X3F_OEC3_FBIN");
        if (eb != NULL && *eb != '\0') {
            FILE *ffb = fopen(eb, "rb");
            if (ffb != NULL) {
                if (fread(main16, sizeof(float), fnn * 3, ffb) == fnn * 3) {
                    fbin_mode = 1;
                    fprintf(stderr, "[oec3] frame override <- %s\n", eb);
                } else {
                    fprintf(stderr, "[oec3] FBIN short read, ignored\n");
                }
                fclose(ffb);
            }
        }
    }

    /* ---- 5. 折回全分辨率：lin_new = s9b*Rb_c/K_c + (1-s9b)*lin_c ---- */
    {   /* X3F_OEC3_FBK="R,G,B"：折回除数覆盖（默认=K 逐通道裁剪域）。
         * 2026-10-27 lldb 抓包定标：SPP 无折回——Stage2 直吃 OEC 65k 帧
         * （s2out/frame 逐带比中位 1.0000），折回除数纯为我们链路的自由
         * 参数。2501 对 SPP 参考扫描：fbk_B 6516→7000 三指标全改善
         * （全图 7.296→~7.27、亮部 MAE 11.91→~11.86、死白 7.21%→~6.96%）；
         * 色相口径（亮部 B/R）在 fbk_B≈8000 达 参考真值 0.801 但全图劣化
         * ——剩余为 T 内容空间结构差，标量旋钮到顶。R/G 维持 K。 */
        double fbk[3] = { K[0], K[1], K[2] * 1.0743 };  /* B: 6516→7000 */
        const char *ef = getenv("X3F_OEC3_FBK");
        if (ef != NULL && *ef != '\0') {
            double f1, f2, f3;
            if (sscanf(ef, "%lf,%lf,%lf", &f1, &f2, &f3) == 3) {
                fbk[0] = f1; fbk[1] = f2; fbk[2] = f3;
            }
        }
    for (int y = 0; y < h; y++) {
        const float gy = (float)y * 0.5f + 0.25f;
        int iy = (int)gy;
        float fy = gy - (float)iy;
        if (iy < 0) { iy = 0; fy = 0.0f; }
        if (iy >= OEC3_AH - 1) { iy = OEC3_AH - 2; fy = 1.0f; }
        for (int x = 0; x < w; x++) {
            const float gx = (float)x * 0.5f + 0.25f;
            int ix = (int)gx;
            float fx = gx - (float)ix;
            if (ix < 0) { ix = 0; fx = 0.0f; }
            if (ix >= OEC3_AW - 1) { ix = OEC3_AW - 2; fx = 1.0f; }
            const size_t i00 = (size_t)iy * OEC3_AW + ix;
            const size_t i01 = i00 + 1;
            const size_t i10 = i00 + OEC3_AW;
            const size_t i11 = i10 + 1;
            const float w00 = (1.0f - fy) * (1.0f - fx);
            const float w01 = (1.0f - fy) * fx;
            const float w10 = fy * (1.0f - fx);
            const float w11 = fy * fx;
            const float s9b = w00 * s9f[i00] + w01 * s9f[i01]
                            + w10 * s9f[i10] + w11 * s9f[i11];
            float *d = outb + ((size_t)y * w + x) * 3;
            const float *srcp = lin + ((size_t)y * w + x) * 3;
            if (s9b <= 0.0f) {
                d[0] = srcp[0]; d[1] = srcp[1]; d[2] = srcp[2];
                continue;
            }
            for (int c = 0; c < 3; c++) {
                float rb;
                if (fbin_mode) {
                    /* 帧与折回格 1:1（active (ix,iy) = frame (108+ix, 14+iy)） */
                    rb = main16[((size_t)(OEC3_AY0 + iy) * OEC3_FW
                                 + (OEC3_AX0 + ix)) * 3 + c];
                } else {
                    rb = w00 * Rf[i00 * 3 + c] + w01 * Rf[i01 * 3 + c]
                       + w10 * Rf[i10 * 3 + c] + w11 * Rf[i11 * 3 + c];
                }
                d[c] = s9b * rb / (float)fbk[c] + (1.0f - s9b) * srcp[c];
            }
        }
    }
    }   /* FBK scope */

    /* ---- 诊断 dump ---- */
    if ((e = getenv("X3F_OEC3_DUMP")) != NULL && *e != '\0') {
        FILE *fd = fopen(e, "wb");
        if (fd != NULL) {
            fwrite(main16, sizeof(float), fnn * 3, fd);
            fclose(fd);
            fprintf(stderr, "[oec3] dump frame -> %s\n", e);
        }
    }
    free(main16); free(T); free(s9f); free(Rf);
    return outb;
}
