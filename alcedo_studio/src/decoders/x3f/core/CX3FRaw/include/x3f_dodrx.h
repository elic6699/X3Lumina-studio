// x3f_dodrx.h — SPP Stage3（ProcessToOutput48RowDRX / DoDRX）的纯 C 忠实移植
//
//来源： 符号
//   ProcessToOutputRowTemplate<FPixmapTemplate<FPixelClass48RGB>>::DoDRX
//（wrapper，DoDRX 本体，__TEXT vmaddr=0）
//2026-10-22 从转写。
//
// ★ 最终导出语义定论（2026-10-22 晚，cap_0001↔on.tif 全帧 100.0000% 逐位）：
//   SPP 最终导出 tif = flut5[ lut1[ clamp(src, 31744) ] ]
//   —— 一次 clamp + 两张逐通道 LUT，无窗口、无增益、无 3×3 矩阵混合。
//   此前「剩余 2.32% 差异 = 真实 matrix1(CCM) 未捕获」的推断不成立：
//   差异像素满足 on = flut5[lut1[src]] 零误差，等效矩阵就是恒等；
//   cap_0001 断点抓到的 DoDRX 调用（带 sat 混合）与产生 on.tif 的调用
//   不是同一次（同一导出触发多次调用，混合仅在部分路径生效）。
//   产品复现 SPP 导出只需合成曲线 combined = flut5∘lut1
//   （参数化节点见 outputs/tools/sppref/fit_combined_curve.py / .json）。
//   本模块保留完整 DoDRX（含 sat 混合）供需要该路径时使用。
//
// 算法（非 mono、bool0=bool1=0 的主路径）：
//   1) 窗口：v = clamp((src - lo) * gain, 0, 31744)；gain = 16384/(hi-lo)，
//      仅当 0<=lo<=63488 && 0<=hi<=63488 && hi>lo，否则 gain=1（即不做窗口）
//   2) FColorFloat 三通道增益（double），饱和到 [0,65535]
//   3) 两级 LUT（FCLUT 布局：ch[0]→G 曲线、ch[1]→R 曲线、ch[2]→B 曲线，
//      查表前先 min(index, numEntries)）：
//         R1=lut1.ch[1][Rv]  G1=lut1.ch[0][Gv]  B1=lut1.ch[2][Bv]
//         R2=lut2.ch[0][R1]  G2=lut2.ch[1][G1]  B2=lut2.ch[2][B1]
//   4) 饱和度权（float32 域，Rec.601）：
//         Cb = -0.169·Rn - 0.331·Gn + 0.5·Bn
//         Cr =  0.5·Rn  - 0.419·Gn - 0.081·Bn      (Rn=G2/32767f 等)
//         sat = min(sqrt(Cb²+(2.125·Cr)²)/0.707107f, 1)
//         w = sat<=0.85 ? 1 : (1-sstep((sat-0.85)/0.15))·0.15+0.85
//           （sstep(t)=3t²-2t³，按 t(1-t)²+(1-t)(1-t²)=1-sstep 原式转写）
//   5) 系数矩阵（全部 Rec.601 亮度权重）：
//         gR=w·0.701+0.299  gG=w·0.419875+0.587  gB=w·0.886+0.114
//         hR=(1-w)·0.299    hG=(1-w)·0.587       hB=(1-w)·0.114
//         P = M2 · [kR kG kB]ᵀ（kR=(gR,hR,hR) kG=(hG,gG,hG) kB=(hB,hB,gB)）
//         C = P · M1ᵀ
//   6) 12.4 定点应用：fx=trunc(C·4096+0.5)；
//         out = (Σ fx_ij·v_j + 0x800) >> 12，饱和 [0,65535]
//
//未移植（抓包实证为退化路径，见 第五套探针）：
//   - mono 平面合并（bool 路径）：cap_0001 的 flut_int=常数 256 → ratio 恒走
//     快速通道，mono 实际不参与
//   - bool0 亮度微加（Δ=round((B-32768)/1000·bell(R))）、bool1 尾段色度块、
//     直方图统计（vector<int>，只影响诊断计数）
// 验证（dodrx_probe.c 单行 16/16 逐位 + dodrx_port_test.c 全帧）：
//纯 C 移植 vs oracle 全帧 99.9995% 逐位一致
//   （274/58,839,552 像素差，max|d|=64≈0.25 灰阶，sat 边界舍入）
// 管线接入状态：模块已编译进 CX3FRaw。产品复现 SPP 导出不需要本模块的
//   矩阵段（见顶部定论），接入 = 我方 Stage2 域数据 → clamp 31744 →
//   combined 曲线（fit_combined_curve.py 参数化，1024 节点 max 误差
//   19.5/65535 ≈ 0.08 个 8bit 灰阶）。待办仅剩我方 Stage2 输出域与
//对齐标定（此前 source-swap 实验已证 MAE 1.72 的兼容度）。

#ifndef X3F_DODRX_H
#define X3F_DODRX_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 与 SPP FCLUTTemplate<uint16_t,FColorUInt16> 内存布局对齐的前 3 指针 +
// numEntries（查表上限）。
typedef struct {
    const uint16_t *ch[3];   // ch[0]→R 曲线  ch[1]→G 曲线  ch[2]→B 曲线
    uint32_t numEntries;     // 查表前 min(index, numEntries)
} x3f_dodrx_fclut;

// 输出级查表（FLUTTemplate<uint16_t,FColorUInt16>：data+mask）。
// DoDRX 尾段 out = data[(out & mask)]——cap_0001 实抓表是一条真色调曲线
// （暗部斜率 13 → 中间 0.7 → 高光回升），不是恒等表。
typedef struct {
    const uint16_t *data;
    uint32_t mask;
} x3f_dodrx_ushortlut;

// 处理一行 n 个 48RGB 像素（src/dst 均为 u16×3 交错）。
//   matrix1/matrix2: 9 个 double 的行主序 3×3（传 NULL = 单位阵）
//   gains[3]:        FColorFloat 前三通道增益（传 NULL = 1,1,1）
//   win_lo/win_hi:   窗口（lo==hi 或越界 = 不做窗口）
//   out_lut:         输出级查表（data 为 NULL 时跳过）
//   src_x0:          本行起始列（仅用于语义对齐，纯 C 版直接由调用方偏移 src）
void x3f_dodrx_row(const uint16_t *src, int n,
                   const x3f_dodrx_fclut *lut1, const x3f_dodrx_fclut *lut2,
                   const double *matrix1, const double *matrix2,
                   const double *gains, double win_lo, double win_hi,
                   const x3f_dodrx_ushortlut *out_lut,
                   int src_x0, uint16_t *dst);

#ifdef __cplusplus
}
#endif

#endif // X3F_DODRX_H
