/* X3F_RENDER.H — X3F 调整（渲染）接口
 *
 * 分工：
 *   1) x3f_render_load()  把 X3F 解成**线性 sRGB**（float，0..1，高光可 >1）
 *      —— 白平衡、色彩矩阵、ISO 缩放都按相机标定算好，但不做伽马
 *   2) x3f_render_apply() 在预览/导出分辨率上套用调整参数，输出 8 位 sRGB
 *
 * 之所以停在"线性"这一层，是因为调整要在线性空间做才正确
 * （曝光是乘法、Fill Light 是提阴影），最后一步才编码到显示空间。
 *
 * 基于 x3f_tools（BSD-3-Clause）。
 */

#ifndef X3F_RENDER_H
#define X3F_RENDER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    X3FRENDER_OK         =  0,
    X3FRENDER_ERR_OPEN   = -1,
    X3FRENDER_ERR_PARSE  = -2,
    X3FRENDER_ERR_NORAW  = -3,
    X3FRENDER_ERR_DECODE = -4,
    X3FRENDER_ERR_NOMEM  = -5,
    X3FRENDER_ERR_PARAM  = -6,
    /* 两段式 SPP：请求的状态与当前句柄不符（例如换了降噪档），
     * 调用方需要重新 prepare。 */
    X3FRENDER_ERR_STATE  = -7
};

/* ---- 色彩来源（只有一条）------------------------------------------
 *
 * 线性数据用 x3fuse 的 scene-linear：写 DNG 时那一套色彩变换
 * （相机标定 + AsShotNeutral + 相对数字 ISO 增益，全部照相机元数据算）。
 * 所以 JPEG / HEIC / 预览 的色彩与同一个文件导出的 DNG 一致。
 *
 * 以前这里是个开关：关掉就退回"自己拼矩阵 + kMerrillFix 拟合补丁"的旧链。
 * 那条链少了相机标定这一步，DP Merrill 整幅偏绿、黄红类高饱和色发闷；
 * 既然色彩已经对齐 DNG，旧链连同三块补丁（kMerrillFix / 逐通道曲线 /
 * 色相 x 色度重映射）一起删掉了。
 *
 * 它不改变影调：显示用的"相机观感"基础曲线与各项调整照旧。 */
/* ---- 解码后端开关 ------------------------------------------------
 *
 * 二进制里同时有两套解码实现，运行时可切：
 *   0 = x3fuse（Rust）   —— 移植版，保留作对照/后备
 *   1 = 旧 x3f_tools C 版 —— **默认**，更快
 *
 * 两套是同一份 C API 的两个实现（x3fuse 是它的 Rust 重写），所以对上层
 * 来说只是"换了个解码器"，渲染、参数、导出全都不变。
 *
 * 实测（DP1 Merrill / DP2 Quattro，min-of-7）：
 *   - RAW 解码本身**逐位相同**（-unprocessed 零差异）
 *   - Merrill 只差预处理的黑电平：旧版把 Right 暗参考列算进黑电平，
 *     x3fuse 按固件 bug 跳过，差约 21 个 raw 级
 *   - 关掉渲染层的 Merrill 修正能看出这点差别（相机中性像素上的偏离
 *     6.8% -> 5.6%，DP1M1896），但**修正一开就落在同一个值上**，
 *     所以默认设置下两版的出图不可分辨
 *   - 速度：Rust 版慢 1.5~2.3x —— 这是把默认值定成旧 C 版的唯一理由
 *
 * 切换是全局的、立即生效；已经解出来的图像不受影响（要重解才变）。
 * 进程启动时可用环境变量 X3F_DECODER 预设：legacy / c 走旧版，
 * 其它值（含 x3fuse）走 Rust 版。 */
void        x3f_render_set_decoder(int which);   /* 0 / 1，见上 */
int         x3f_render_decoder(void);
const char *x3f_render_decoder_name(void);       /* "x3fuse" / "legacy-c" */

/* 调整参数。取值范围都归一到 -1..+1 或 0..1，界面直接绑滑块。
 *
 * 「填充光」（fill_light）2026-10 按结果重写成了 这套算子
 * （局部 ln 比值 + tanh 增益表 + 2x2 金字塔），不再是启发式，见
 * 。
 * 「辉度杂讯 / 色度杂讯」仍然没有：那一族是噪声模型驱动的小波加权滤波
 * （σ²=a+b·v+c·v²），复刻成本高，降噪目前走 x3fuse 的 NLM。
 * 「锐度」是通用做法（显示域小半径 unsharp）；「细节」按结果实现成
 * Detail 滑块语义（残差比值图 × 强度），见下面那个字段的说明。
 *
 * 这里**没有**色度降噪：自制的那套「彩噪抑制」（显示域色度双边低通）已删除，
 * 降噪改走 x3fuse 的 NLM —— 它本来就是只压色度、效果还强得多，但只该在导出时
 * 开（见 x3f_render_load_ex3 的 nlm 参数）。 */
/*---- 「色彩模式」（Color Mode）--------------------------------
 *
 * SPP 面板上那个下拉 + 它下面的强度滑块，对应两组东西：
 *
 *   1) 下拉选哪一档 → 文件 CAMF 里的三张表（逐模式各一份）：
 *        CMCM_<模式>           3x3 色彩矩阵（Standard = 单位阵）
 *        CMCC_<模式>           对比度补偿标量（-0.3 … +0.3）
 *        MultiAxisTable_<模式> 21 个色相格上的（色相偏移, 饱和度倍率）
 *   2) 强度滑块 → 一个**全局**权重 w（与选哪一档无关，Standard 也吃）：
 * 滑块内部值 v ∈ [-5, +5]，默认 0；w 由 `AdjustColorModeEffectParam`
 * 0xa0960 的表给出（ 起，逐模式 11 个数 × [-5,-4,…,5] 线性插值）。
 *        实测（`tools/sppref/modesweep`）绝大多数档就是 w = 1 + 0.2·v；
 *        **mode 5/8/11/12 那四档正半段是 1 + 0.1·v（1.5 封顶）**，
 *        mode 13（fp 时代那一档）是 1 + 0.15·v —— 这三张形状都按 mode 索引查表。
 * 界面上按 说明（zh_CN 资源 12016「通过滑块操作，可以调节
 *        色彩效果的强弱」）把它当 0..100 显示，50 = 默认档。
 *
 * 权重落在三处（ + 与真身对拍，见 ）：
 * · 色彩矩阵 M = w·CMCM_<模式> + (1−w)·N （N 是 rodata 里的常量）
 *   · 多轴表     色相偏移 ×w、饱和度倍率按 w 向 1 靠
 *   · 色调曲线   在 SPP 那套 ToneParams 的两组之间插值 —— 我们不做公式，
 * 直接把 相机算法（ `GenerateF20ToneCurve`）跑出来的
 *                LUT 存成增益表（include/x3f_colormode_tone.h，生成文件）
 *
 * 我们这条管线的基线是对着**相机直出**标定的（基础曲线 + 手工拟合的 kColorMatch），
 * 而标定用的那批 SPP 导出是 **mode 7 = 风景 Landscape**（ 里那组默认
 * ToneParams，见 §18.15，`sppref 7` 那两条 LUT）。
 * 所以"相对基线"的口径就是相对风景档：
 *   · 矩阵：CMCM_Landscape 不是单位阵，照旧直接用 M；
 *   · 多轴表：取 MultiAxisTable_<模式> / MultiAxisTable_Standard 的比值与差值；
 *   · 色调曲线：表里只有 5 档与基线不同（标准/鲜艳/中性/肖像/FOV 黄色经典），
 *     其余 6 档（风景/电影院/夕阳红/森林绿/FOV 蓝/黑白）增益恰好 1.0；
 *   · CMCC：只读出来做诊断（我们的曲线不是 SPP 那条，暂时不接）。
 * 于是**风景档 + 强度 50** 整条链恒等 —— 实测出图与加这个功能之前逐字节相同
 * （`cmrender … Landscape 50` 与 `… notone` 的 md5 一致）。
 *
 * 色调曲线那一层的口径说明（两条近似，都量过）：
 *   · 增益按**输入**（线性值 / 1.4）取，与我们既有的「过曝矫正」色调表同一个
 * 做法（那个已知在高光段偏保守约 6 成）；把 色调段整段换成它的真 LUT
 *     才能分毫不差，那是另一件事；
 *   · 表里 7 个 w 列之间在 log 域几何插值，留出校验（v = ±2、±4）最大偏差
 *     2.2%（Vivid），多数档 ≤0.5%。
 */
#define X3F_COLOR_MODE_AXIS 21

typedef struct {
    double m[9];        /* 线性 sRGB 域的 3x3（行主序），已按强度混合。
                         *
                         * **不做色彩空间共轭**：这几张表的行和恒为 1（= 作用在
                         * "中性 = 等通道"的域），SPP 也是把它乘在白平衡结果之后。
                         * 拿 cam_to_rgb 去共轭会让 Cinéma 那档的行和变成 1.44，
                         * 中性色直接染色 —— 试过，是错的。 */
    int    unit;        /* 1 = m 是单位阵，渲染时整段跳过 */
    double hue_shift[X3F_COLOR_MODE_AXIS];  /* 每个色相格的偏移（度），相对 Standard，
                                             * 已按强度缩放 */
    double sat_gain[X3F_COLOR_MODE_AXIS];   /* 每个色相格的饱和度倍率，相对 Standard */
    int    axis;        /* 0 = 没有多轴表 / 相对 Standard 无差别 */
    /*【每档的色调曲线】模式表不只换色彩矩阵，还换一条色调曲线
     * （ `SetUpToneParamsF20` 那张逐模式 ToneParams：TCFSteepness /
     * TCFSteepness2 / TCFBreak 各不相同）。我们不去仿它的公式，而是让 SPP
     * 真身把每条曲线算出来（`sppref/modesweep.c`），存成"相对基线的增益"
     * —— 表在 include/x3f_colormode_tone.h（**生成文件，不入库**：本机装了
     * SPP 就跑 outputs/tools/sppref/gen_mode_tone.py 生出来。没有这份表时
     * 这一项整体不参与，tone_row 恒为 -1）。
     *
     * tone_row = 家族下标（kCMToneRows）；-1 = 这一档与基线完全相同（不参与）。
     * tone_w   = 滑块换算出来的权重 w（0..2），决定在表里 7 列之间插值。
     *
     * 基线是谁：mode 7 = **风景 Landscape**（= 里那组默认 ToneParams），
     * 也正是当初标定基础曲线时用的口径（见 §18.15
     * 里 `sppref 7` 那两条 LUT）。所以**风景档保持逐字节不变**，
     * 标准/鲜艳/中性/肖像/FOV 黄 五档各带自己的小 S 形（中调 ±2~7%）。 */
    int    tone_row;
    double tone_w;
    double cmcc;        /* 该模式的 CMCC（原始值，仅诊断） */
    double cmcc_std;    /* Standard 的 CMCC（同上） */
} x3f_color_mode_t;

typedef struct {
    double exposure;     /* 曝光      SPP 滑杆值 1:1（SPP 全程 ±2 EV）。
                          * 实现 = 2^ex 增益折进 tone LUT 曲线自变量（基础曲线
                          * 之前），与 相机算法一致：ToneLUT 域长 N=psat·2^(−ex)·
                          * tp[0x48]（GenerateF20ToneCurve 实证），
                          * 增益改曲线自变量而非乘在输出上 → 高光沿曲线肩部
                          * 同步滚降、不易死白。2026-10-22 闭环确证。*/
    double contrast;     /* 对比度    SPP 滑杆值 1:1（app 界面 -1..+1，SPP 全程
                          * ±2）。实现 = 相机算法：基础曲线 logit 域陡度缩放
                          * f = σ(2^(co/2)·logit(f₀))，枢轴 = 输出 0.5 固定
                          * （CombineToneCurve + CreateBasicCurve
                          * ，陡度 = TCFSteepness·2^(co/2) 实证）。
                          * 2026-10-22 闭环，替换旧 smoothstep 插值。*/
    double shadow;       /* 阴影      SPP 滑杆值 1:1（app 界面 -1..+1，SPP 全程
                          * ±2）。正值提亮暗部。只改显示曲线 0…25% 段，端点
                          * 固定；实现同 DoAdditinalAdjustment（
                          * 已读通+探针实证，局部幂曲线，不是全局加法）。 */
    double highlight;    /* 高光      SPP 滑杆值 1:1。正值压暗高光（恢复）。
                          * 只改显示曲线 75%…100% 段，端点固定；压暗方向带
                          * ×0.25 强度压缩（探针实证）。*/
    double saturation;   /* 饱和度：SPP 滑杆值 1:1（SPP 全程 -2..+2，app 界面
                          * 只露 -1..+1）。实现 = Rec.601 饱和度矩阵
                          * M(u)，u = Sat/2 + 1（ §4.4，完整）。*/
    double detail;       /*细节（ Detail 滑块）0 .. 1，每 1/6 一格共 7 档。
                          *
                          * 档位 = (int)(v·6 + 0.1)；档位查表得残留强度 R
                          * （0/.1/.2/.3/1/1.25/1.5），再得增益
                          * g = GMAX·(R/1.5)，GMAX 默认 1.1（按 SPP 参考图定标）。
                          *
                          * 实现 = SPP 那条链见
                          * ）：
                          *   残差 r = (本像素 + eps) / (多尺度低通 + eps)  ← 比值图，中性 1
                          *   r > 1 → m = 1 + (r−1)·g ；r ≤ 1 → m = 1/((1−g) + g/r)
                          *   R,G,B *= m
                          * g = 1 时 m ≡ r（残差原样）；g = 0 时 m ≡ 1（不贴）。
                          *
                          * **档 0 = 整段关**（SPP 那边是常开）。我们基线没有这一步，
                          * 所以默认 0 时出图与加这个参数之前逐字节相同。
                          * 低通目前是多尺度盒式模糊 —— 顶替 保边双边金字塔，
                          * 那部分是建模的，等读了 BilateralFilterMonoF23 再换。 */
    double sharpness;    /* 锐度：SPP 滑杆值 1:1（SPP 全程 -2..+2，app 界面只露
                          * 0..+1；负值=平滑，核函数支持负号，滑杆还没放开）。
                          * 实现 = 乘性残差锐化（ §4.5，完整）：
                          *   detail = clamp(0.25·D·Δ, ±1000/32768) → clamp(±0.35·S)
                          *   out_c = c·(1 + detail/S)（色相不变，防光晕）
                         * D = Y − blur(Y) 顶替 SMono 细节层，幅度差用
                         * X3F_SHARP_GAIN 定标（默认 3.4 = 2026-10-24 SPP
                         * 真身实测：DP2Q1884 两档导出差值法，双方非零
                         * 纹理区逐像素比中位数；此前 19.2 为与旧 unsharp
                         * 等强度的内部口径，实测是 5 倍）。*/
    double sharp_res;    /* 锐化量的**分辨率缩放**（SPP ComputeSharpenAmount 的
                          * sqrt((sx+sy)/2)，sx = 输出宽/全宽；TonalAdjustments
                          * §4.5(b)）。全尺寸导出/1:1 显示 = 1.0；
                          * 预览在低分辨率上跑同一条链，必须按 渲染长边/全尺寸
                          * 长边 收 Δ —— 否则盒式高通的相对带宽宽 1/scale 倍、
                          * 过冲还是全尺寸的幅度，全图浏览就是满图锯齿/光晕
                          * （100% 看原生分辨率反而正常）。0 或负 = 按 1.0
                          * （memset 式老调用方）；X3F_SHARP_RES 可覆盖（lab）。 */
    double wb_temp;      /* 白平衡：色温  -1 .. +1（正=更暖） */
    double wb_tint;      /* 白平衡：色调  -1 .. +1（正=偏品红） */
    double hl_recover;   /* 高光恢复    0 .. +1（正=把过曝的高光层次摊出来） */
    double fill_light;   /* 填充光      -2.0 … +2.0，步进 0.1（正=提亮暗部），
                          * 与 SIGMA Photo Pro 的 Fill Light 同量程同步进。
                          *
                          * 实现 = SPP 那套：局部 ln 比值经
                          * exp(1.33F·tanh(·)) 增益表映射后乘回线性域。
                          * 亮点是"提亮暗部但局部对比不掉"；0 时严格 no-op。
                          *
                          * 2026-10-21 已加入 SPP 6.9.0 的原始 Retinex map
                          * 复刻：int16 log/方差平面、PyramidFilter、k0+512、
                          * C=32·sqrt(var+64)、有符号 int16 map→FLUT。
                          * 通过 X3F_FL_69=1 启用。默认仍用兼容标定路径：
                          * SPP 吃 Stage1 传感器域 RGB，本渲染器目前只有
                          * scene-linear sRGB，两者之间的输入换算尚未闭合。 */
    double hl_keep;      /* 高光保留 0..1 —— 默认 0.5。
                          *
                          * 基础曲线的"相机观感"是对着相机内嵌 JPEG 拟合的，它
                          * 高光端是一段**硬肩**（最后 4 档 EV 只有 0.987→0.999
                          * 的跨度，落到 8bit 就是 252~255），于是 RAW 里那 1~2 档
                          * 余量全挤在几个码值里 —— "切得少了但看不出层次"。
                          * 这个参数把那段硬肩拉成有梯度的斜坡：
                          *   0   = 原样（最接近相机直出，高光平）
                          *   1   = 斜坡 0.90→1.00（余量摊开，亮部整体略暗）
                          *   0.5 = 中间，默认
                          * 实测（DP2Q2055，相机切平处）：0 时标准差 0.52、
                          * ≥250 占 99.5%；1 时标准差 1.87、≥250 占 10.9%
                          * （余量真的摊开了）。这是"更像相机"和"高光有层次"
                          * 之间的连续取舍，交给用户。
                          * 注意它只动显示用的那条曲线，不动 RAW 解码，
                          * 所以和 DNG 的颜色口径仍然一致。 */
    /* ---- Fill Light 的调校旋钮（2026-10-20 高光回拉研究） ----
     * fl_tune = 0 时这三个字段全部忽略，行为与加进来之前逐字节相同；
     * App 面板会写 1。命令行仍可用 X3F_FL_HLZRED / X3F_FL_LMEANR /
     * X3F_FL_MSAT 覆盖（环境变量优先，方便 A/B）。语义与实测见
     * outputs/FillLight_高光回拉_第二轮_2026-10-20.md。 */
    int    fl_tune;      /* 0 = 用内置默认；1 = 用下面三个值 */
    double fl_hlzred;    /* 高光回拉 0..1：亮部（A > k0）把局部项收回多少。
                          * 0 = 不收回（皮肤更亮），1 = 全收回（皮肤发灰）。
                          * 内置默认 0.7。 */
    double fl_lmean_r;   /* 局部均值半径（map 像素，0 = 原来的多尺度金字塔）。
                          * 实测 16 / 32 的取舍见上面那份笔记。 */
    double fl_msat;      /* map 软饱和 m' = S·tanh(m/S)，0 = 关。
                          * S 越小越压两端；800–1500 能压住暗端过提，
                          * 同时把橙木偏高的 G 收下来。 */
    double fl_mapgain;   /* 全局项系数（默认 0.75）。真实 map 拟合值 0.415。 */
    double fl_k0off;     /* 锚点偏移（A 的单位，默认 0）。拟合值约 +280。 */
    double fl_zgain;     /* 局部项整体增益（默认 1.5）。1.0 = 旧行为。 */
    double nr_banding;   /* 条纹杂讯 0..1 —— 默认 0（关）。
                          *
                          * SPP 面板上的「条纹杂讯」，**只有 Merrill 机型有**
                          * （证实：Quattro 那个 build 里这条链是死代码）。
                          * 0..1 会被量化成 5 档（floor(v·4)），与 SPP 一致。
                          *
                          * 实现（SPP：ProcessTiles → MatchColOffsets →
                          * DirectionalBilateral）：按**逐通道二次噪声模型**
                          * σ²=a+b·v+c·v² 归一的方向性双边，只在一个方向上
                          * 取邻居；窗口半宽 {15,21,27,33,39}、强度
                          * {0.625..0.125}（都以 ISO 缩放）。系数来自相机
                          * 自己的 CAMF 标定（ColorNoiseModel 等），
                          * 详见 。*/
    double nr_luma;      /* 辉度杂讯 0..1 —— 默认 0（关）。
                          *
                          * 逐通道的**噪声归一化二维双边**：权重
                          * exp(−0.5·Σ Δ_c²/σ²_c)，σ² 是相机自己的二次噪声模型
                          * （σ²=a+b·v+c·v²，来自 CAMF ColorNoiseModel）。
                          * 档位（floor(v·4)）决定 σ 缩放与窗半径：
                          *   σ 缩放 = {0.25,0.5,1,2,4}（DenoisingSlidersStep 列 0）
                          *   半径   = {2,3,4,5,6}
                          * 所以滑块越大 → σ 越大 → "多少算噪声"越宽 → 压得越狠，
                          * 而边缘（远超 σ）始终不动。 */
    double nr_chroma;    /* 色度杂讯 0..1 —— 默认 0（关）。
                          *
                          * 同一套噪声归一化双边，但**只在 Cb/Cr 上做**（Y 不动）。
                          * 窗更大：半径 {3,4,6,8,10}。σ 缩放 = SPP 实测
                          * m_fBFNoiseScaling 曲线（ 抓 ，档位
                          * less/mid/more = 0.06/0.5/1.5，强非线性，
                          * 锚定档 4 = σ6）：
                          *   σ = {0.24, 0.69, 2.0, 3.46, 6.0}
                          * （2026-10-22 重标；X3F_NR_CHROMA_LEGACY=1 回
                          * 2026-10-14 视觉标定旧表 {2,3,4,5,6}。）
                          * 人眼对色度分辨率不敏感、对色噪敏感，
                          * 所以色度可以下重手而不损清晰度 —— 与 SPP 一致。 */

    /*---- 「黑白」(B&W) 模式 ------------------------------------
     *
     * 修正调色板有两个页签：**彩色 / 黑白**（E_STR_ADJ_TAB_COLOR /
     * _TAB_BW）。切到黑白，面板跟着换一批项：饱和度不可用、「色度噪讯」
     * 不适用，多出「色彩混合器」与「色调」。下面这几个字段就是那条链。
     *
     * 依据逐行可查）：
     * IPManager2::priSetup（ 起是黑白分支、0x521ac 起是彩色分支）
     *     tag+0x15c/0x15e/0x160 → IPM2+0x580/0x588/0x590（色彩混合器 R/G/B）
     *     tag+0x188/0x18a/0x18c → IPM2+0x598/0x5a0/0x5a8（色调 R/G/B）
     *   字段号来自 Common_MAUI.dll 的 E_FID_usColorMixingRed=50 /
     * E_FID_usMonochromeToneRed=59（见 ）。
     *   彩色分支把 0x580/0x588/0x590 清零（= 不混合），所以黑白是**加法**。
     *
     * 换算（XDoColorMixing 与 AdjustGrayBalanceGainsStage4QD）：
     *   1) 三个混合比先归一化：和为 0 → 各 1/3（默认），和为 1 → 原样，
     *      否则各自除以和；
     *   2) 灰度 = wR·R + wG·G + wB·B（16 位 48RGB 域，即**色调曲线之后**）；
     *   3) 色调 = 保亮度的三通道增益：
     *        L = 0.3·tR + 0.59·tG + 0.11·tB   （SPP 那三个权重就是 0.3/0.59/0.11）
     *        增益_c = t_c / L
     *      （SPP 走的是"用图像均值反解 gray balance 增益"那条，
     *        黑白下图像已经是中性灰 ⇒ 均值三通道相同 ⇒ 化简成上面这个式子。）
     * t 全为 0 或全相等 → 增益恒为 1（等价"不调整"，与
     *      「黑＆白」预置 = 不调整一致）。
     *
     * 黑白模式下：饱和度不参与（SPP 手册 §9-4 明说不可用）、相机色彩匹配
     * 不参与（灰度图没有色度可匹配）、色度噪讯按 黑白分支置 0。
     * 高光去饱和/高光控制照旧 —— 它们只改亮度，对已经中性的像素是 no-op。 */
    int    mono;         /* 0 = 彩色（默认）, 1 = 黑白 */
    double mono_mix_r;   /* 色彩混合器：R 混合比（相对量，三个一起归一化） */
    double mono_mix_g;   /* 色彩混合器：G */
    double mono_mix_b;   /* 色彩混合器：B */
    double mono_tone_r;  /* 色调：R 目标值（相对量，按 0.3/0.59/0.11 保亮度） */
    double mono_tone_g;  /* 色调：G */
    double mono_tone_b;  /* 色调：B */

    /*---- 黑白「胶片颗粒」（FilmGrain）----------------------------
     * SPP FilmGrainFactory_F23：位置确定的模板（逐像素双种子 xorshift 哈希
     * → r%10000 查 cdf → 幅度桶），高斯整形后叠到 48RGB 上。四张静态表
     * （counts/cdf/amp/T10）逐值取自 lib95（见 x3f_mono_grain_tables.h）。
     *
     * mono_grain   启动开关（0 = 关，关时一个字节都不动）
     * mono_grain_size     尺寸大小 0..1：档位 = clamp(floor(v*10-1),0,9)，
     *              选 cdf/amp 那一行（43→5 桶，档位越高颗粒越粗）
     * mono_grain_roughness 粗糙度 0..1：同样 10 档，选高斯半径
     *              （档位越高 σ 越小、纹理越糙）
     * 位置确定性：t38/t40 两个种子偏移按 0 走（SPP 从流水线带值，
     * 逐帧不变；0 只是另一个固定相位）。 */
    int    mono_grain;              /* 0 = 关（默认）, 1 = 开 */
    double mono_grain_size;         /* 0..1 */
    double mono_grain_roughness;    /* 0..1 */

    /*---- 「色彩模式」--------------------------------------------
     * 这一坨**不是**用户直接填的：宿主先用 x3f_color_mode_resolve() 把
     * 「模式名 + 强度」换算成它，再原样搬进来（apply 那一步拿不到文件路径，
     * 而这三张表在 CAMF 里）。
     *
     * x3f_edit_defaults() 填的是**单位阵 + 无多轴表** —— 与加这个功能之前
     * 逐字节相同。黑白下不参与（SPP 也是：黑白页签里没有色彩模式）。 */
    x3f_color_mode_t color;
    /*X3I(SFD) 专用显示曲线：1 = 走 X3I 默认档那条实测表
     * （kX3FSFDTone：白点推到 lin≈1.4 以后，lin=1.0 只到 236/255），
     * 0 = 普通基础曲线 + hl_keep。表里已经含了 高光肩，所以 SFD 模式下
     * hl_keep 不参与。由来见 x3f_render.c 里那张表上面的注释。 */
    int sfd_tone;
    /* ---- 色彩分离（参照 Lightroom 的 Split Toning / Color Grading）----
     *
     * 暗/亮两个分区各自一个色相 + 强度，加平衡与混合。作用在**显示域链的
     * 最后**（色彩匹配/后校正之后、输出之前）：拿 601 亮度做分区权重，
     * 注入「保亮度色差」 out_c = d_c + (T_c − T_601)·w —— 601 亮度严格
     * 不变，只往暗部/亮部里加色相。
     *
     * 零滑杆（两路强度都为 0）时整段不参与，输出与没有这个功能逐位相同。
     * 黑白下照常可用（给黑白照片上色调正是它的经典用途）；
     * SPP 没有这一项，所以没有"对位 SPP"一说，数值口径照 Lightroom：
     *   hue 0..360 度、sat 0..1、balance -1..1（正=分界往高光挪）、
     *   blending 0..1（越大过渡越宽）。 */
    double split_sh_hue;   /* 暗部色相（度，0..360） */
    double split_sh_sat;   /* 暗部强度（0..1，0 = 关） */
    double split_hl_hue;   /* 亮部色相（度） */
    double split_hl_sat;   /* 亮部强度（0..1） */
    double split_balance;  /* 平衡（-1..1，0 = 分界在中间亮度） */
    double split_blending; /* 混合（0..1，0 = 硬分界，1 = 全程渐变） */

    /* 过曝矫正**关闭**时的截断高光处理：
     *   1 = 在 93% 亮度以上快速撤掉 Detail/锐度叠加的结构，纯白处只留低通；
     *   0 = 管线保持原样（过曝矫正开启时走 JPEG 融合，不需要也不允许改 RAW 链）。
     * 默认 0（x3f_edit_defaults 清零），宿主按 jpegHlFusion 是否开启来置位。 */
    int hl_soften;

    /* ---- SPP「色调曲线」四滑杆（阴影/暗部/亮度/高光）+ 三档标尺 ----
     * GenerateToneCurveEditorLut 真身数学复刻（x3f_tonecurve.c，
     * 15 组探针 ±1 LSB 对质），显示域 u16→u16 LUT，作用在色调链
     * （基础曲线/高光恢复/对比度/色彩模式曲线）之后的显示值上 —— 与
     * SPP 曲线编辑器预览语义一致。四滑杆全 0 = 恒等（不施加）。
     * tc[0..3] = 阴影/暗部/亮度/高光滑杆（0=中性，满格 ±2.0 —— SPP 滑杆
     * 量纲 −2.0…+2.0、0.1 步进，值即本结构的原始 val，不做二次缩放）；
     * tc[4..6] = 标尺 r2/r3/r4（0..255，默认 64/128/192）。 */
    double tc[7];

    /* ---- X3Lumina「辉光」(glow)：高光辉光 Bloom ----
     * SPP 没有这一项，X3Lumina 独有显示域效果，存 sidecar、不写 SECs 段。
     * 作用在主渲染路径最终显示像素之上（apply_ex_common 末尾后处理 pass），
     * glow == 0 时整段跳过，输出逐位不变（零滑杆 md5 铁律）。
     *   glow          强度 0..1（0 = 关）
     *   glow_radius   模糊尺度（相对，默认 0.5）
     *   glow_threshold 高光起点 0..1（默认 0.7） */
    double glow;
    double glow_radius;
    double glow_threshold;

    /* ---- 白平衡「RGB 三通道强度」(Mac RawEditParams.wbRed/Green/Blue) ----
     * 在白平衡（预设 + 色温/色调）之上再微调各通道增益的**线性域乘子**：
     * 1.0 = 100%（不动），范围 0..2（0%..200%）。作用在 wb_gain_calc 的
     * 增益上（temp/tint 折完再乘），与 Mac 侧 applyChannelGain 同一语义
     * （解码后、色调映射前）。三通道全 1 时 wb_gain_calc 的调用门槛不变，
     * 输出与加这个字段之前逐位相同（零滑杆 md5 铁律）。
     * 注意：**1.0 = 中性**（不是 0）—— x3f_edit_defaults() 把三者填 1.0，
     * Swift 零初始化会得到 0，宿主 toC() 必须显式填 RawEditParams 的值。 */
    double wb_red;
    double wb_green;
    double wb_blue;

    /* ---- 亮部保色（高光段色度保真，2026-10-31）----------------------
     *
     * 病灶（DP2Q2443 麻园站黄墙，屋檐下明暗交界）：**不是红加多了，是蓝压
     * 不下去**。逐通道 tone LUT 把 R/G/B 一起推向 1.0，越接近白，色度掉得
     * 越快：饱和黄被稀释成"低饱和暖白"，肉眼判成粉。三方同块实测（显示域
     * 均值）我方 R213 G202 B157（R−B 56）| 机内 JPEG 228/213/147（80）|
     * SPP 217/189/118（99）—— 色相 hue 与机内 JPEG 几乎一致（49.5 vs 49.3），
     * 差的是饱和度。所以全局提饱和度是错药（暗部会过），要在**亮部按通道
     * 保色度**。
     *
     * 做法：显示域按亮度门控，把 (d − Y) 放大 k 倍，Y 用 601 亮度严格不变
     * （不动明度）：  d_c = Y + (d_c − Y)·k,  k = 1 + (hl_chroma − 1)·w(Y)·w(hue)
     *   · w(Y)  = smoothstep(win_lo, win_hi, Y)  —— 默认 0.78..0.93（显示域
     *     199..237），膝点比 SPP 早约 30 级的地方开始补。
     *   · w(hue) = 色相闸（软边 10°）。实测缺色只发生在黄/橙家族（屋檐带
     *     195–225 的 sat 比 SPP 低 0.20），蓝天、火核属于"不该被提色"的，
     *     开闸后天空 B−G 只动 0.05 级（实测 −2.41 → −2.46，SPP −2.38）。
     *
     * 零回归：hl_chroma = 1.0 时 k 恒为 1，d 逐位不变（整块 if 都不进）。
     * 只在**基础曲线显示链**上生效 —— mode8 放行时旁路整条显示链，不参与。
     * 环境变量优先，方便命令行 A/B：X3F_HL_CHROMA / X3F_HL_CHROMA_WIN /
     * X3F_HL_CHROMA_HUE（见 x3f_render.c 里 apply_common）。
     *
     *   hl_chroma         k：1.0 = 关（默认），1.8 ≈ 机内 JPEG，2.4 上限
     *   hl_chroma_win_*   作用窗（亮度 0..1），默认 0.78 / 0.93
     *   hl_chroma_hue_*   色相闸（度）；hi − lo ≥ 359.5 = 不限色相（默认 0/360） */
    double hl_chroma;
    double hl_chroma_win_lo;
    double hl_chroma_win_hi;
    double hl_chroma_hue_lo;
    double hl_chroma_hue_hi;

    /*---- Quattro 基线亮度降噪（2026-11-01，**追加在结构体末尾**）------
     *
     * ⚠️ 这两个字段必须在整个结构体最后 —— renderlab 的 X3F_RL_PARAMS 是
     * 把 App dump 的 `x3f_edit_params_t` 按**字节前缀**覆盖到它的 P 上
     *（见 renderlab.c 那段注释："结构体只许在后面加字段"）。插在中间会让
     * 旧 dump 之后的所有字段整体错位，静默变成"参数没生效"。
     *
     * 起因：用户报「楼顶每根亮竖框正上方都留一条 1~2 级、5~10px 的淡竖纹」。
     * 根因不是列 FPN，而是 **nr_luma 默认 0 时整条链一级亮度降噪都不跑**：
     * Quattro 的 CAMF 里没有噪声模型（§10.6），g_nr 走的是烘进去的
     * nr_quattro_model；SPP 自己默认就在跑那条双边，我们的默认是「什么都不做」。
     *
     * 所以给 Quattro 一个**基线档**：实际亮度降噪 = max(nr_luma, nr_q_base)。
     * 用户把「辉度杂讯」拉过基线时按用户的档走（不叠加）；基线本身可以关。
     * Merrill **不受影响**（它有自己的 CAMF 噪声模型，默认值一个字不动）。
     *
     * ⚠️ 2026-11-04：这条基线**默认关**。细竖纹真因已查明是显示域解码侧
     * unsharp 的贴边暗下冲（不是噪声），基线降噪方向不对且加大损画质。
     * 字段/能力保留，需要时把 nr_q_baseline 置 1 或设 X3F_NR_Q_BASELINE=1。
     *
     * 实测（DP3Q 09 伦敦金融城，强逆光蓝天 + 深色玻璃尖顶楼）：
     *   竖纹列剖面 p95 2.45 → 1.32（机身内嵌 JPEG 1.29）
     *   窗格立面细节 13.56 → 13.41（−1.0%）、小黄瓜格栅 15.87 → 15.68（−1.2%）
     * 即「抹掉低于感知阈值的结构」，不是「糊细节」。 */
    int    nr_q_baseline; /* 0 = 关（**默认**），1 = 开。关掉 = 回到 2026-11-01
                           * 之前的行为（nr_luma 是多少就跑多少）。
                           * 环境变量 X3F_NR_Q_BASELINE=1 可开 / =0 强制关
                           * （CLI / 标定用，优先级高于本字段）。 */
    double nr_q_base;     /* 基线档 0..1（默认 0.8 = 第 3 档）。<=0 视同关闭。
                           * 会被 floor(v·4) 量化成 5 档，与 nr_luma 同一套口径。
                           * 环境变量 X3F_NR_Q_BASE 可用小数覆盖。 */

    /*---- 基线降噪的**结构门控**（2026-11-01，仍然必须追加在末尾）----
     *
     * 起因：基线 0.8 把竖纹压到 1.32 的同时，树冠 −28%、窗格 −22%、全图
     * 1–2px 能量 −30% —— 用户一句"细节好像也少了很多"是准的。基线是**我们
     * 自己加的**（用户没要求），加的量应该只用在"确实没结构的地方"。
     *
     * 做法（不是按梯度门控，是按**局部信噪比**门控）：双边跑完之后看残差
     * r = 原始 − 滤波，取 3×3 邻域均值能量 E = mean(Σ_c r_c²/3)，用相机
     * 自己的噪声模型折成"噪声应有的能量" σ²(v)（v = 原始像素×fullscale，
     * 系数就是 g_nr.cn_iso，与 EC 模式同源）：
     *     x    = E / σ²                （1 = 噪声量级，>1 才有结构）
     *     s    = clamp((x−lo)/(hi−lo), 0, 1)   lo/hi 默认 1.0 / 4.0
     *     keep = keep_max · s
     *     out  = 滤波 + keep·(原始 − 滤波)
     * 平坦区 E≈σ² → s≈0 → 一个像素都不还（竖纹原样被压掉，因为它的残差
     * 也在噪声量级）；真纹理的残差是噪声的几倍 → 还回去。
     *
     * 只作用在**基线**那一档（用户自己把「辉度杂讯」拉过基线时不动 —— 那是
     * 明确要求，不该被偷偷打折）。字段默认开，但基线默认关，故平时不生效。 */
    int    nr_q_gate;     /* 0 = 关（回到纯基线），1 = 开（默认 1）。
                           * 环境变量 X3F_NR_Q_GATE=0 覆盖。 */
    double nr_q_gate_k;   /* keep_max：纹理区最多把多少比例的原始残差还回去
                           * （默认 0.75）。0 = 与关掉门控等价，1 = 纹理区
                           * 完全不降噪。环境变量 X3F_NR_Q_GATE_K 覆盖。
                           * 阈值本身（x 的起效/全保点 1.1 / 2.6）不出面板 ——
                           * 它们是标定常数，X3F_NR_Q_GATE_LO/HI 可覆盖。 */

    /*---- 输出色彩空间（2026-11-04，**仍然必须追加在末尾**）------------
     *
     * 取值见下面的 enum。**全链路广色域**：六个档位走的是同一条管线、同一条
     * 相机观感曲线，只是最后一步的容器不同。
     *
     * 关键在顺序（2026-11-04 重做）：
     *
     *   相机线性（sRGB 原色）→ 逐通道观感曲线（sRGB 域，它标定的地方）
     *        → 显示域一系列算子（对比度 / oe_tone / 色彩模式小 S / 色调曲线 /
     *          黑白 / 饱和度 / 高光保色 / 分离色调）
     *        → decode 回线性（**可负、可 >1**）
     *        → 3×3 转域矩阵 → 目标域线性
     *        → 目标域传递函数编码 → 8/16bit 存储
     *
     * 为什么必须这个顺序：观感曲线是按 sRGB 线性值逐通道标定的，把它搬到
     * 目标域上直接查表，同一块颜色走的是另一组工作点 —— 实测把一块黄压成
     * 灰绿（DP2Q2661 差 12 级）。反过来，把曲线整个跳过（旧"母版语义"）则
     * 观感与预览差 25 级（中间调压低、发灰）。两种都不对，因为问题不在
     * 曲线也不在色域，而在**顺序**：曲线是线性域逐通道函数，转域是线性矩阵，
     * 矩阵放在曲线之后就两者都不牺牲。
     *
     * 负值 / >1 的处理：相机矩阵按 sRGB 原色分解，sRGB 之外的真实颜色就落在
     * 负值上。全链路用**奇对称外推** f(-x) = -f(x)（srgb_encode_s / out_encode_s），
     * 符号一路带过转域矩阵，到目标域变成正常正值。夹 0 就等于在门口把
     * sRGB 之外的内容删掉 —— 那正是"宽容器、窄内容"的来历。
     *
     * 已知代价：
     *   · 线性高光余量仍会被曲线的肩部压掉（曲线设计如此，要余量只能不走
     *     观感链 —— 那是另一个开关，不是这里的事）。
     *   · 显示域算子的数值口径是 sRGB 显示值；目标域码值换了 encode 曲线
     *     （ProPhoto γ1.8 / BT.2020 OETF 等），所以阴影高光/锐度的亮度判据
     *     按目标域原色的 Y 行系数换算（见 apply_spatial 的 yw）。
     *   · sRGB（0）逐位零回归：所有新分支都以 `out_space != X3F_OUT_SRGB`
     *     为条件，sRGB 一个字节都不变（回归基线 app-fuse 7f927ca4）。
     */
    int    out_space;

    /* 观感延后（最小验证，2026-11-05）：默认 0 = 现行为，逐位零回归。
     *
     * 打开后，渲染链**停在场景线性**：apply_row 做完 WB 增益 / 填充光 /
     * 色彩模式矩阵就写像素，曲线、对比度、阴影高光、饱和度、色调曲线、
     * 分离色调、高光保色、以及全部显示域空间处理（色度中值、空间增益、
     * 解码 unsharp、辉光）一律不施加；apply_ex_common 也不做后处理。
     *
     * 输出仍是 16 位通道交错，但码值语义变了：**值是线性 sRGB，不是
     * 显示值**（v = 线性，0..1，越界夹掉）。调用方据此决定下一步 ——
     * 现有唯一用途是把这份 scene-linear 喂给胶片引擎，让引擎看到场景光
     * 而不是"已经调好色的照片"，跑完再回 C 施加观感（配合
     * x3f_render_set_base_curve(0) 只施加用户滑杆，不重复基础曲线）。
     *
     * 为什么用参数而不是环境变量：同一次导出里这条链要跑两遍（前段线性、
     * 后段观感），env 是进程级的，分不开。
     *
     * 只影响 16 位路径的语义；8 位路径也响应这个开关（输出线性 8 位，
     * 精度不够，仅调试可用）。 */
    int    look_defer;

    /* 第二段「不重放相机对齐表」（2026-11-11）：默认 0 = 现行为，逐位零回归。
     *
     * `fold_align_look`（`align_look_on` 那张逐通道码值重映射）是按 Quattro
     * 机内 JPEG 标定的**相机侧塑形**：暗部 B+4 / G−2 这一级。它属于"把 RAW
     * 做成相机直出"这件事，只在**单段显示链**里成立。
     *
     * 两段式（`look_defer` + 引擎）里第二段的输入已经是引擎显影好的成品，
     * 再叠一遍这张表就是二次塑形：2026-11-11 用 DP0Q + Portra 400 对拍官方
     * SpektraLab，中性/天空区实测再压 −1.0 个 b*，正是"偏青蓝"的一部分。
     * 置 1 时第二段跳过这张表（第一段 look_defer 本来就不用它，无影响）。
     *
     * 与色彩模式同一处：`p->color` 传单位阵（`x3f_color_mode_default`）即可
     * 让第二段不重放相机色彩矩阵。两者都由 App 的第二段显式设置。 */
    int    align_look_off;
} x3f_edit_params_t;

/* 输出色彩空间（x3f_edit_params_t.out_space）
 *
 * 全部走同一条管线，只在**最后一步**分叉：把 sRGB 显示值解回线性，用 3×3
 * 矩阵转到目标空间的线性域，再用目标空间自己的传递函数编码。矩阵都是
 * 「原色坐标 → XYZ → 目标原色」合并出来的常数，满足 (v,v,v)→(v,v,v)。
 *
 * 六个档位都是完整观感链（见上面 out_space 字段的说明）：
 *   · 曲线、亮度相关的算子全部在 sRGB 域做完，转域是最后一步线性变换；
 *   · 负值（sRGB 外颜色）按奇对称外推保留，不夹 0；
 *   · 只有 8/16bit 存储这一步才夹 [0,1]，那是容器的物理上限。
 *
 * 早期（2026-11-04 之前）广色域走"母版语义"（跳过观感曲线），结果是导出
 * 与预览差 25 级、发灰；再早一版改成"sRGB 域渲染 + 整幅换域"，观感对了但
 * 换域发生在整数码值上，负值已被夹掉，容器宽内容窄。现在的做法把换域放进
 * C 层的 double 链末尾，两个问题一起解决。
 */
enum {
    X3F_OUT_SRGB     = 0,   /* 默认，逐位零回归；完整观感链 */
    X3F_OUT_PROPHOTO = 1,   /* ProPhoto RGB，γ1.8，D50（含 Bradford 适配） */
    X3F_OUT_ADOBE    = 2,   /* Adobe RGB (1998)，γ563/256，D65 */
    X3F_OUT_P3       = 3,   /* Display P3，P3 原色 D65 + sRGB 曲线（sRGB 的真超集） */
    X3F_OUT_2020     = 4,   /* Rec.ITU-R BT.2020，BT.2020 原色 D65 + BT.2020 OETF */
    X3F_OUT_DCI_P3   = 5    /* DCI-P3（SMPTE RP 431），P3 原色 + γ2.6（系统 ICC 按 D65 解释） */
};

/* 明度层 finish 的胶片颗粒（App 用）：与 mono_grain 同一套 SPP FilmGrain
 * 模板/表/高斯/幅度，落到 [0,255] 的 float Y 平面（增量 = delta/257）。
 * size/rough 均 0..1，档位映射与 mono_grain_* 相同（clamp(floor(v*10-1),0,9)）；
 * originX/originY 为瓦片全局坐标偏移（整幅传 0,0）。 */
void x3f_film_grain_yplane(float *y, int w, int h,
                           int originX, int originY,
                           double size, double rough);

void x3f_edit_defaults(x3f_edit_params_t *p);

int  x3f_render_base_curve_enabled(void);

/* 设置基础曲线开关（0/1）。默认开。 */
void x3f_render_set_base_curve(int enable);

/* 高光去饱和 —— 默认开启（1.0）。
 *
 * 相机在接近白点处会把颜色往中性收。蓝天+薄云这类场景里，RAW 各通道到顶
 * 的时刻不同，直接渲染会在接近纯白处留下品红/青色斑块；相机 JPEG 看不到，
 * 正是因为它在高光处做了去饱和。本管线补上这一步，让天空干净、自然。
 *
 * 只在**最低通道**超过内部 knee（0.80，即三通道都接近白）之后才按 smoothstep
 * 渐入，亮度保持不变（只掉饱和度）：中低亮度和纯色（如亮黄色）完全不受影响。
 * strength 取 0..1，0 关闭。 */
void   x3f_render_set_hl_desat(double strength);
double x3f_render_hl_desat(void);

/* SDQ / SDQH 高光归一收色 —— **默认 0（关）**，App 上是「SDQ 高光」滑杆。
 *
 * SDQH 的 CAMF `DigitalISOGain=(4,4,2)`（DP-Q 系是 1,1,1），解码器按上游契约
 * 把它折进相机矩阵，于是反解出的通道饱和比 sat 是「相对本机通道白点」而不是
 * 「相对统一传感器白电平」：SDQH0184 水花光点 sat 最高才 0.544，够不到
 * clip_score 的 0.90 门槛 → 高光收色**一次都没触发**，蓝通道被矩阵算成负数
 * 夹到 0，纯白的光点显示成纯黄（镜像情形 = 粉点）。
 *
 * strength > 0 时：① clip 评分前把 sat 乘回 DigitalISOGain；② 该强度与
 * x3f_render_set_hl_desat 相乘叠加驱动高光收色。dig==(1,1,1) 的文件乘 1.0
 * 精确无变化 ⇒ DP-Q / Merrill 零回归；strength==0 时整条支路不启用。
 *
 * 必须在 x3f_render_load_ex2 **之前**调用（dig 在加载时读取）。
 * A/B 也可用环境变量 X3F_SDQ_HL=0..1。 */
void   x3f_render_set_sdq_hl(double strength);
/* 场景判据开关（2026-11-14）：只给「引擎前置」两段式的第二段打开。
 * 打开后 SDQ 收色额外读解码段的 RAW 饱和分（clip），cs≥2.4（三通道全饱和）
 * 直接满权重 —— 判据从「引擎滚降后的显示亮度」换成「RAW 说这是不是白」，
 * 与不挂胶片时同一口径。默认关（无胶片路径逐位零回归）。 */
void   x3f_render_set_sdq_scene_judge(int on);
double x3f_render_sdq_hl(void);

/* 这个文件的 DigitalISOGain 是否 ≠ (1,1,1)（= SDQ/SDQH 那代，归一唯一生效的
 * 机型）。纯元数据探测：面板用它决定要不要摆出「SDQ 高光」滑杆。 */
int    x3f_render_probe_sdq_hl(const char *path);

/* 相机色彩匹配 —— 默认开启。
 *
 * 我们的 RAW 渲染+色调曲线出来的颜色总体比相机内嵌 JPEG 偏淡（肤色最明显，
 * R 低约 8%、B 高约 20%），影调是准的、差的是色彩浓度。这里在显示空间按
 * 【色相】把色度做一个小 2x2 变换（等效"按色相缩放饱和度 + 少量色相旋转"），
 * 亮度不动。系数是对 6 张不同题材样片拟合的，留一图验证色度误差降 20%。
 * 传 0 关闭（关了就退回原来的"纯 RAW 渲染"配色）。 */
void x3f_render_set_color_match(int enable);
int  x3f_render_color_match_enabled(void);

/* 预览 apply 的 GPU 链（Metal/Vulkan，2026-10-23）——默认**开**。
 *
 * apply_ex_common 8 位路里的 lutloop / 色度中值 / down8 三段上 GPU，
 * 与 CPU double 链差 ±1 LSB 级（float 精度，非逐位）。Mac/Metal 下默认开且校验过。
 * 2026-10-31 定位 Windows/Vulkan（Intel UHD）"整图错图"根因：Vulkan 后端对
 * storage/uniform 缓冲的 **f32 类型读取整体返回 0**（nlm3 全 u32 就没事）——
 * apply 链三个 shader 全部改成 array<u32> + bitcast<f32> 还原后修复，gpu-cmp
 * 实测 max=2、0.4% 像素 ±1~2（float 运算顺序经中值链放大，不可见）。
 * 需要位级一致回归可设 X3F_GPU_APPLY=0 强制回 CPU。未显式设置时看
 * X3F_GPU_APPLY 环境变量。覆盖范围之外（填充光/黑白/hl_y_only/HL_WARM/COLOR_FIT
 * 实验项）自动回 CPU。对拍校验见 x3f_render_set_gpu_apply_cmp /
 * x3f_render_set_gpu_cmp_report。 */
void x3f_render_set_gpu_apply(int enable);
int  x3f_render_gpu_apply_enabled(void);
void x3f_render_set_gpu_apply_cmp(int enable);
void x3f_render_set_gpu_cmp_report(void (*cb)(const char *));
/* 色度中值段单独门控（默认关=CPU；X3FDEC_GPU_CHRMED=1 / setter 打开）——
 * Intel UHD 上 sel 二分 31 趟 kernel 反而慢，2026-10-31 起默认回 CPU。 */
void x3f_render_set_gpu_chrmed(int enable);
int  x3f_render_gpu_chrmed_enabled(void);

/* 彩色阴影 / 暗角校正（spatial gain）——默认开启，仅对非 Quattro 机型生效
 * （与上游 x3f_tools 的 `-sgain` 默认行为一致：Merrill 做，Quattro 不做）。
 * 关掉后 Merrill 的偏色会随画面位置变化（角部与中心不一致）。 */
void x3f_render_set_spatial_gain(int enable);
int  x3f_render_spatial_gain_enabled(void);

/* SPP「过曝矫正」（Overexposure Correction）——默认**关**。
 *
 * 复刻自 的 DP23HLPsRemoval/F23HPRemoval（推导见
 * §9–§16）：
 * 传感器上有一批"专用高光像素"（HLP），在中间层是每 2 列 / 每 8 行一个
 * （= 图像坐标每 4 列 / 每 16 行；CAMF HighlightPixelsInfo=[0,0,16,4]）。
 * 这批像素在**接近饱和处**读数和周围不一样（实测：过曝区的网格点比 3×3 邻域
 * 最大值低约 6~8% 的饱和电平），SPP 就用 3×3 min/max 包络 + 噪声模型阈值
 * （系数直接来自 的 rodata）把它们修回去。
 *
 * 打开后：只改"网格 ∧ 饱和"的点（饱和判据 = 逐通道饱和比 ≥ 14300/16383），
 * 其余像素逐位不变。也可用环境变量 X3F_HLP_FIX=1 打开，
 * X3F_HLP_DRYRUN=1 只统计不改、X3F_HLP_DEBUG=1 打计数。 */
void x3f_render_set_hlp_fix(int enable);
int  x3f_render_hlp_fix_enabled(void);

/* 「高光孔洞修正」（hole-fix，实现见 x3f_hole_fix.h/.c）：解码级 lin 域 G/B
 * 增益，修冷色高光的粉色斑块。默认开；App UI / CLI 在每次解码 load 之前
 * 用 set_enabled 显式交给 C（铁律：不用 setenv）。这里只做转发声明，
 * 供 Swift 侧（CX3FRaw module）调用。 */
void x3f_hole_fix_set_enabled(int en);
int  x3f_hole_fix_enabled(void);

/* m8od（X3F_HL9_ODM8 对应项，2026-10-30 接 App UI）：mode8 基线 + 输出域 Δ
 * 修正场（od Δ 挂 mode8，只进过曝 mask，±30/255 级封顶；表 = 2501 焰核单场景
 * 标定，跨图泛化未验证）。apply 级开关——在 toC（每次 apply 前）交给 C，
 * 切换不需要重解码。显式 setter 优先于环境变量 X3F_HL9_ODM8（铁律：不用
 * setenv；且 C 侧 env 判据只看非空，=0 也是开）。 */
void x3f_render_set_hl9_odm8(int enable);

/* 「过曝矫正」：作用在 RAW 解码（scene-linear 域）的高光修复，三档语义：
 *
 *   - mode 4（产品「关」档）= SPP Stage2 高光中性化（HighlightNeutralize，
 *     WB 增益域，apply_oe_neutralize_wb，as-shot WB 从 CAMF 读）：把裁剪
 *     高光里无意义的通道比中和成中性白，对齐相机直出观感（消 own 解码
 *     裁剪区的品红偏色）。
 *   - mode 7（产品「开」档）= 裁剪感知高光去饱和（apply_oe_restore）：只有
 *     真裁剪的像素参与，宽渐变 smoothstep(0.75→2.2) 向 luma 白滚移，
 *     mask 软门控 + 权重场空间平滑（消白核边沿轮廓线）；焰身橙黄层次
 * 保留，观感向 SPP「过曝矫正开」靠拢（完全重建需半层原始
 *     数据，融合后不可得，此为无新伪影前提下的最优近似）。
 *   - mode 8（实验档，2026-10-23）= SPP Stage2 直通重放（oe_spec_row，
 *     在 apply_ex_common 里做，不在 load 步）：M·v+bias → T_c →
 *     combined + u 域残差 3D LUT，整条显示链旁路（RAW 调整滑杆不参与），
 *     输出 ≈ SPP 直出。**只对 Quattro 有表**（x3f_oe_spec.h），其它机型
 *     空表优雅退化 = 原样输出。
 *   - mode 9（实验档，2026-10-25）= mode8 + p 层高光重建 v1（§13.13）：
 *     先对 lin 裁剪区做 B 驱动结构展开（est = C+A·(B_μ7/16384)^g，
 *     仅替换 max≥0.98·白点 的裁剪区），再走 mode8 同一条链。与
 *     mode7/mode8 并列对比用；跨帧泛化未验证，参数为 2501 in-sample。
 *
 * mode 含义同环境变量 X3F_OE_NEUT（0=关，4/7/8/9=产品档）；**传 ≥0 会覆盖
 * 环境变量**，传 -1 恢复跟随环境变量（lab 对拍用）。
 *
 * ★ 机型门（2026-11-05）：mode 4 的动作是 Quattro 引擎 Stage2 的
 *   HighlightNeutralization（判据来自 own 解码的裁剪 mask）。**非 Quattro
 *   机型（SD14/SD1/Merrill）没有这一段**，实现侧统一按 0（真关）处理 ——
 *   否则会把正常曝光的高光（WB 域 alpha 0.49 起）当裁剪区抹成中性白，
 *   人脸高光失色。显式实验档 1/2/3/5/6/7 不受机型门限制。作用在解码那一步
 * （load_common，scene-linear 域；mode 8 在 apply 步），所以 App 必须在
 * 每次 load 前置位、且切换档位要重解码（解码缓存键带上这个值）。 */
void x3f_render_set_oe_neutralize(int mode);

/* 读回当前生效的档位（含 X3F_OE_NEUT / FORCE 覆盖后的结果）。
 * 给「引擎前置」两段式用：第二段要临时关掉 mode8 的 SPP 直通重放，
 * 跑完必须原样恢复，否则同一进程里后续的预览帧会串档。 */
int x3f_render_get_oe_neutralize(void);

/* 「过曝矫正」HLP / OECVer3 阶段（环境变量 X3F_OE_HLP）：
 *
 *   - mode 0 = 关（默认）
 *   - mode 1 = 内部 guide 混合（apply_oe_hlp_blend）
 *
 * 与 x3f_render_set_oe_neutralize 平行：App 在每次 load 之前显式置位（覆盖环境变量），
 * -1 = 恢复跟随环境变量。该阶段作用在解码（scene-linear 域）那一步，所以切换档位要重解码。
 * 注意：这是与「过曝矫正」Stage2 高光中性化（oe_neutralize）**并行**的另一阶段。 */
void x3f_render_set_oe_hlp(int mode);

/* SPP「过曝矫正」勾选框的**另一半**：勾选框还会换掉整条色调曲线。
 *
 * 见 §18）：
 *   FIPEngine2::priProcessGenerateToneLUT
 *     priMakeToneLUT 把勾选框取反写进 request+0x101
 *     request+0x101 == 0（= 勾选）时 → SetUpToneParamsF20 出来的
 *     ToneParams.m_fTCFSteepness2 += (−0.7)
 * 该字段在 GenerateF20ToneCurve 里变成 sigmoid 第二段（拐点 TCFBreak 之上）
 * 的陡度 → 勾选态 1.4、未勾选态 0.7，整条曲线跟着变
 * （未勾选：暗部/中间调略亮，高光被压下来 ~10%）。
 *
 * 我们的基线（基础曲线标定）本来就是**勾选态**，所以这里只需在
 * "未勾选"时把 差乘回去。系数表直接来自 SPP 自己生成的 LUT：
 * 用 的 GenerateF20ToneCurve（ 调用真身）跑出两条 16K LUT，
 * 取其比值，再换算到"基础曲线输出值"这个自变量上（见 §18.11 的说明）。
 *
 * 0 = 勾选态（默认，与本功能加入前逐字节一致）
 * 1 = 未勾选态（乘上 差）
 * 也可用环境变量 X3F_OE_TONE=1 打开。 */
void x3f_render_set_oe_tone_off(int off);
int  x3f_render_oe_tone_off(void);

/* oe_tone 未勾选差值曲线的边缘限幅（opt-in，默认关 = 逐位现行为）。
 * 开启后：强边缘 ±1px 带内把「未勾选 vs 勾选」两条 tone LUT 的差按边缘掩膜
 * 收回（= 勾选态处理 ≈ SPP 对同样像素的行为），平坦区不动。用于压掉高对比
 * 1px 边缘（建筑鳍片/天线）被差值曲线再推亮的细线，不改变用户拍板的平坦区
 * 观感。App 走显式 setter；env X3F_OE_EDGE_CLIP=1 只给 lab A/B 用。
 * 1 = 开，0 = 关。 */
void x3f_render_set_oe_edge_clip(int enable);

/*两份「从 相机算法跑出来的」生成表在不在（bit0 = 色彩模式的每档色调曲线，
 * bit1 = 过曝矫正的勾选/未勾选差）。生成文件（x3f_colormode_tone.h /
 * x3f_oe_tone_gain.h）**不进仓库**，见 ：
 * 本机跑过生成脚本就有；没有表时那两项回落成"不参与"（增益恒 1）。
 * 宿主拿它决定界面上相关说明怎么写，别把话写成"一定有表"。 */
int  x3f_render_has_spp_tables(void);

/* 这个 X3F 是不是 Merrill（DP1M/DP2M/DP3M）——只看文件头，不解码。
 * 现在渲染层不再需要它（Merrill 专属的修正层已删），保留是因为它本身
 * 仍是个有用的判断，将来真要做 Merrill 专属处理还用得上。 */
int x3f_render_probe_is_merrill(const char *path);

/* 探测这张文件是否 Quattro 世代（X3F_IMAGE_RAW_QUATTRO 布局，或 X3I(SFD) 容器 ——
 * SFD 是 Quattro 才有的连拍模式）。宿主用它区分「Quattro（own 解码）」和
 * 「经典 Foveon（SD9/SD10/SD14/SD15/DP1 系等，必须走 x3fuse）」—— own 解码器
 * 对经典 Foveon 直接报错不回落，发错了就是渲染失败。 */
int x3f_render_probe_is_quattro(const char *path);

/* Quattro **世代**探针（过曝矫正门控口径）：在 x3f_render_probe_is_quattro 的
 * 判据上再纳入 SD Quattro / SD Quattro H 的独立 type_format
 * （X3F_IMAGE_RAW_SDQ / X3F_IMAGE_RAW_SDQH）。绿增强不用它（只在 DP-Q
 * 上标定过，SDQ 保持窄口径），宿主 OE 门控与面板置灰走这个。 */
int x3f_render_probe_is_quattro_gen(const char *path);

/* 这张文件是否属于 Classic-Foveon auto-tone 代（SD14 / SD1 等）。
 * 判据与 load_common 设置 g_classic_active 一致（CAMMODEL=="SIGMA SD1" 或
 * CAMF 含 DBG_AutoContrast）。供 Swift 侧预览降噪门槛绕过用。只看文件头/CAMF，
 * 不解码整帧。 */
int x3f_render_probe_is_classic_tone(const char *path);

/* ── Merrill「SPP 一致模式」旁路 ────────────────────────────────────────
 *
 * Merrill 原生链（float 前端 → SPP NR → tone → 色彩矩阵）在本宿主里
 * 只以**成品**形式存在（x3fuse 的 spp::render 出 16 位 sRGB，不产
 * scene-linear f32），所以它不能直接接管渲染链 —— 那会让自家滑杆 / WB /
 * 色调曲线全废。这两个入口走**旁路**：单独导出 SPP 那张成品图。
 *
 * 门控 = 环境变量 X3F_MERRILL_SPP=1 **或**程序化开关
 * x3f_render_spp_set_enabled(1)，**默认关** → 既有路径逐字节不变。
 *
 * 能不能走由 SPP 侧自己的门控定：必须 Merrill + HI 分辨率档 + 非 Cineon +
 * denoise==10 + CPU 后端 + SkipStepFlags bit23=0 + FPNSuppression=0。
 * 不满足时这两个入口返回 0 / 失败码，调用方应回退既有路径。
 *
 * 成品是**传感器方向、未旋转**的 16 位 sRGB（每通道 u16，0..65535），
 * 和 x3f_render_load_* 返回的 lin 同一朝向，上层那套 orientation 照常生效。 */

/* 程序化门控（产品前端用）。
 *
 * on = 1 强制开 SPP 一致模式，on = 0 强制关，on = -1 回到「只听环境变量
 * X3F_MERRILL_SPP」（默认状态）。
 *
 * 一旦被显式调用就以它的值为准，**优先于环境变量**。产品用它把门控绑到
 * 用户的前端选项上（用户选了「SPP 原生链」才开），而不必去 setenv
 * —— setenv 是进程级全局副作用，且多线程里改环境不安全。
 *
 * 注意：门控只决定「SPP 这条路**允许**走」，走不走由调用方自己的逻辑决定。
 * 打开门控不会让既有路径的输出变一字节。 */
void x3f_render_spp_set_enabled(int on);

/* 程序化门控（产品前端用）：SFD 七帧合成的帧对齐开关。
 *
 * on = 1 强制开帧对齐，on = 0 强制关，on = -1 回到「只听环境变量 X3F_SFD_ALIGN」（默认状态）。
 *
 * 一旦被显式调用就以它的值为准，**优先于环境变量**。产品用它把门控绑到用户的前端选项上
 * （用户在面板上勾「SFD 帧对齐」才开），而不必去 setenv —— setenv 是进程级全局副作用，
 * 且多线程里改环境不安全。
 *
 * 见 x3f_sfd.c 里 `sfd_align_on` 的实现。门控只决定「帧对齐**允许**做」，做不做由
 * x3f_render_load_sfd_ex 内部逻辑决定（默认关时测量与 warp 全不执行）。
 * 打开门控不会让既有路径的输出变一字节。 */
void x3f_render_set_sfd_align(int on);

/* 这个文件能不能走 SPP 一致模式？能 → 返回 1 并把尺寸写进 w / h；
 * 不能 → 返回 0。 */
int x3f_render_spp_capable(const char *path, int *w, int *h);

/* 导出一张 SPP 原生链的 16 位 sRGB 成品。
 * out 的 stride 单位是**字节**（= w*3*2），与 x3f_render_apply_ex16 一致；
 * stride 更大时逐行拷。返回 0 成功；非 0 失败。 */
int x3f_render_spp_export(const char *path, uint16_t *out, int stride,
                           int *w, int *h);

/* 同上，但把宿主的创意滑杆直接落在 **SPP 自己的 tone 阶段**。
 *
 * tone 曲线是机身标定在 camera-space 码值域上的，所以这几个参数
 * 是**原生生效**的，不需要任何跨域换算：
 *   exposure   曝光 EV（正=变亮）
 *   contrast   对比，叠加在 CAMF 标定值之上
 *   shadows    阴影，正=提亮（与 SPP 同号）
 *   highlights 高光，**正=提亮**（SPP 语义；产品「正=压暗」，调用方取负）
 *   saturation 饱和度，**SPP 滑杆值**（-2..+2，0 = 原生）。SPP 定义
 *               u = Sat/2 + 1，在 tone 之后用 Rec.601 饱和度矩阵
 *               （out_c = Y601 + (c − Y601)·u，Y601 = 0.299R+0.587G+0.114B）；
 *               产品 fuse 链用的是**同一套定义**（x3f_render.c 的 apply_common），
 *               所以这个值原样直传即可，两条解码链的手感一致
 *   nr_level   SPP 降噪档 0..4（CAMF DenoisingSlidersStep 的行），-1 = 原生中档
 *
 * 传 NaN = 该项不覆盖，走原生值；此时与 x3f_render_spp_export 逐字节一致。 */
int x3f_render_spp_export_tone(const char *path, uint16_t *out, int stride,
                               int *w, int *h,
                               double exposure, double contrast,
                               double shadows, double highlights,
                               double saturation, int nr_level);

/* ------------------------------------------------------------------ *
 * 两段式 SPP：prepare 一次，tone 段反复跑。
 * ------------------------------------------------------------------ *
 *
 * x3f_render_spp_export_tone 每调一次都把整条 SPP 链重跑一遍（24MP 上
 * ~4.1 s）。两段式把链拆开：
 *
 *   x3f_render_spp_prepare     解码 + 浮点前端 + 降噪，~3.9 s，做一次；
 *   x3f_render_spp_apply_tone  只重算 tone 曲线 + 输出变换 + 逐像素，~0.03 s。
 *
 * 于是拖曝光/对比/阴影/高光/饱和度这五根滑杆时，每次只付 30 ms 而不是 4 s。
 * 降噪档（nr_level）是例外：降噪本身就跑在 prepare 段里，换档必须重新
 * prepare，此时 apply_tone 返回 X3FRENDER_ERR_STATE。
 *
 * 两段式的输出与一站式**逐位一致**（已实测：中性档与创意档各 44,255,232
 * 个采样值 0 差异）。
 *
 * handle 由 prepare 返回、apply_tone 消费，用 x3f_render_spp_prepared_free
 * 释放。 */

/* 准备好的 SPP 缓冲区（不透明句柄）。前向声明，完整 typedef 在
 * x3f_spp.h 里 —— 两者指的是同一个 struct。 */
struct x3f_spp_prepared;

/* 阶段一：解码 + 降噪。
 * nr_level 是 SPP 降噪档 0..4，-1 = 原生中档；只有它在这一段生效。
 * 成功时 *out 拿到句柄，w / h 拿到输出尺寸（与 x3f_render_spp_export 同几何）。 */
int x3f_render_spp_prepare(const char *path, int nr_level,
                            struct x3f_spp_prepared **out, int *w, int *h);

/* 阶段二：套 tone 曲线。
 * 参数含义与 x3f_render_spp_export_tone 完全一致（NaN = 不覆盖）。
 * out 的 stride 单位是**字节**（= w*3*2）。
 * 返回 0 成功；X3FRENDER_ERR_STATE = 降噪档与 prepare 时不符，要重新 prepare。 */
int x3f_render_spp_apply_tone(struct x3f_spp_prepared *handle, uint16_t *out, int stride,
                              int *w, int *h,
                              double exposure, double contrast,
                              double shadows, double highlights,
                              double saturation, int nr_level);

/* 释放 prepare 拿到的句柄。 */
void x3f_render_spp_prepared_free(struct x3f_spp_prepared *handle);

/* 只读文件头里记录的**色彩模式**（Standard / Vivid / … / Monochrome）。
 *
 * 注意：这个字段是 X3F 2.3 才有的，**Quattro（v4 头）里没有** ——
 * 那批文件的色彩模式写在内嵌 JPEG 的 EXIF MakerNote 里（见
 * §6）。所以：
 *   返回 1 = 头里有（Merrill / SD1 这类老文件），out 填好；
 *   返回 0 = 头里没有（Quattro），out 置空串，宿主得去问内嵌 JPEG。
 * out 至少 cap 字节，取到的是 NUL 结尾的 ASCII。 */
int x3f_render_probe_header_color_mode(const char *path, char *out, int cap);

/* ---- 「色彩模式」的取数口（见上面 x3f_color_mode_t 的说明）----------
 *
 * 模式名表 = SPP 菜单里那 11 档的 **CAMF 短名**（就是 CMCM_ 的后缀），
 * 顺序照菜单：Standard / Vivid / Neutral / Portrait / Landscape /
 * FCBlue / FCYellow / Cinema / ForestGreen / SunsetRed / Monochrome。
 * （SPP 界面上把它们叫"标准 / 鲜艳 / 中性 / 肖像 / 风景 / FOV 蓝色经典 /
 *   FOV 黄色经典 / 电影院 / 森林绿 / 夕阳红 / 黑白"，另有"原设定"= 用文件里
 *   记录的那一档 —— 那个由宿主先 x3f_render_probe_header_color_mode() 解析。） */
int         x3f_color_mode_count(void);
const char *x3f_color_mode_name(int idx);

/* 把 out 填成"什么都不做"（单位阵 + 无多轴表）。
 *
 * 宿主**必须**用它初始化手里那份表：全零的 x3f_color_mode_t 不是"不做"，
 * 而是一个零矩阵（有人把默认值写成全零就会出黑图）。x3f_edit_defaults()
 * 内部也走这一个。 */
void        x3f_color_mode_default(x3f_color_mode_t *out);

/* 把「模式名 + 强度」解析成 x3f_color_mode_t。
 *
 * mode  = CAMF 短名（大小写敏感，同上）；不认识的模式或文件里没有那张表
 *         → 返回 X3FRENDER_ERR_PARAM，out 落成"单位阵"（= 不改变画面）。
 * mode  = "Standard" 时矩阵就是单位阵、多轴表差分也是 0，所以整段是 no-op。
 * strength = 0..100，50 = 默认档（权重 1.0）。0 = 最柔（权重 0），
 *            100 = 最强（权重 2）。
 *
 * 开销约 40ms（一次文件打开 + CAMF 解码），**别每帧调**：模式/强度变了再调。 */
int         x3f_color_mode_resolve(const char *path, const char *mode,
                                   double strength, x3f_color_mode_t *out);
const char *x3f_color_mode_last_error(void);

/* ---- 导出降噪参数（解码阶段，x3fuse 0.2.0）------------------------
 *
 * 升到 0.2.0 之后，降噪从"一个 0..10 的强度"扩成了参数表 + 执行后端。
 * 它作用在**解码**那一步（BMT 传感器域、x3f_scene_linear 内部），不是 apply
 * 那一步，所以只能随 load 传进来 —— 和 nlm 强度当初放在 load 上是一个道理。
 *
 * 预览 / 放大不走这条路（它们一律 nlm=0），所以这些参数只影响导出。
 *
 * 默认那套（由 x3f_denoise_default 填出）等价于 0.1.6 的行为：
 * NLM / 只压色度 / 倍率 1.0 / 3,3,11,11 窗 / 后端 auto。实测同一 `-denoise N`
 * 下 0.1.6 与 0.2.0 的降噪量一致（暗部相邻色度差 1.69/255 → 1.69/255）。
 *
 * 后端：
 *   auto（默认）—— 能用 GPU 就用，拿不到适配器**静默**退回 CPU
 *   cpu        —— 完全不初始化 GPU
 *   gpu        —— 要求 GPU，拿不到就整个解码失败（X3FRENDER_ERR_DECODE）
 * auto 退回这种事从返回值上看不出来，要告诉用户就得另探一次：
 * x3f_render_gpu_available()。实测（M1，全分辨率 Merrill）主 NLM pass
 * CPU 572.9ms → GPU 131.9ms（4.3x）；Quattro 端到端 5.46s → 1.89s（2.9x）。
 * 两边像素**不会逐字节相同**：上游量过，核内那 ≤1 个码值的差经过后面的色彩 /
 * 色调链会被放大（渲染输出最大差 438~575 个 16bit 码）。我们实测 DP2Q0337
 * （5424x3616，`-ppm`）：NLM 5 有 0.48% 的采样不同、最大 1278/65535；
 * NLM 10 是 0.67%；BM3D 10 是 3.65%。要逐字节可复现就用 cpu。 */
enum {
    X3F_DN_ALGO_NLM  = 0,
    X3F_DN_ALGO_BM3D = 1   /* 两阶段 BM3D，0.2.0 新增 */
};
enum {
    X3F_DN_BACKEND_AUTO = 0,
    X3F_DN_BACKEND_CPU  = 1,
    X3F_DN_BACKEND_GPU  = 2
};

/* `x3f_denoise_t.luminance` 的三个取值。AUTO 是默认：由文件自己的拍摄 ISO
 * 决定（≥ X3F_DN_LUMA_AUTO_ISO 就带上亮度），判据实现在 C 侧一处
 * （见 x3f_render_auto_luma()），免得 Swift 和 C 各写一份阈值。
 *
 * 为什么需要 AUTO：这套 NLM 只压色度是**为 Foveon 定的**（色噪是主要问题），
 * 但高 ISO 下亮度噪点本身就很脏 —— 没有增益的传感器像素在大增益下会出现
 * 大量孤立亮点，色度被抹平之后它们单独显形，就是"满屏白色小点"。
 * 低 ISO 上则相反：亮度通道本来就该原样保留（那是 Foveon 的立身之本），
 * 所以这条默认按 ISO 走，而不是一律开或一律关。 */
enum {
    X3F_DN_LUMA_OFF  = 0,
    X3F_DN_LUMA_ON   = 1,
    X3F_DN_LUMA_AUTO = 2
};

/* AUTO 的门槛（拍摄 ISO）。2026-11-14 由 800 下调到 400：ISO 640 这类
 * 中高感光在预览里已经有肉眼可见的彩噪/亮点，800 才开等于把这一段整个
 * 漏掉（用户实测反馈）。判据只此一处，预览 / 导出 / 命令行共用；
 * X3F_NR_AUTO_LUMA_ISO 可覆盖（lab 用）。
 *
 * 注意：过线不等于踩满 —— 400 是"开始加"，1600（= 4× 门槛）才到满力度，
 * 中间按 ISO 线性渐进（见 x3f_render.c 的 nr_auto_luma_ramp）。原因是亮度
 * NLM 的 h 是绝对值，门槛处一脚踩满会在 ISO 640 上把高频削掉 46%（"涂抹"）。
 * 显式选 X3F_DN_LUMA_ON 不受这个渐进影响（用户自己的决定，给满）。
 * 注意条纹那条 `(estISO < 800) ? 1.0 : 4/3` 是另一套标定，保持 800 不动。 */
#define X3F_DN_LUMA_AUTO_ISO 400.0

typedef struct {
    int    algorithm;      /* X3F_DN_ALGO_*，默认 NLM */
    int    luminance;      /* X3F_DN_LUMA_*（0/1/2），默认 AUTO：亮度通道要不要
                            * 一起降。AUTO 由拍摄 ISO 决定（见上面那组常量）。
                            * 高 ISO 上不开它，色度被抹平后剩下的亮度尖峰就是
                            * "满屏白色小点"。 */
    int    chroma;         /* 0/1：色度通道，默认 1 */
    double strength;       /* 0.1 .. 3.0，默认 1.0（乘在相机自己的噪声档位上） */
    /* **本地新增**：亮度通道**单独的**倍率，只乘在亮度那一路上（色度不受影响）。
     *
     * 为什么需要它：`strength` 是全局的，而两路的合适力度差得远 —— 色度可以下
     * 重手（人眼对色度分辨率不敏感），亮度不行。宿主开亮度通道多半是因为高 ISO
     * （见 X3F_DN_LUMA_AUTO），实测同一个 h 下亮度那一路会把结构一起抹掉。
     * DP0Q2315（ISO 1600，两个降噪滑块都关，Y 五点高通 RMS 0..255）：
     *
     *     亮度倍率   平坦暗部   电扇   布料
     *       0.6       1.64      3.22   3.32   ← 暗部会留下看得见的细颗粒
     *       0.8       1.01      1.99   1.85   ← 宿主默认
     *       1.0       0.77      1.56   1.28
     *
     * 1.0 = 与不开这一项时**完全相同**（老宿主传 1.0 即可保持原行为）。 */
    double luma_scale;     /* 0.1 .. 3.0，默认 1.0 */
    int    luma_patch;     /* NLM 主 pass 窗：1..9 的奇数，默认 3；BM3D 不用 */
    int    chroma_patch;   /* 同上，色度，默认 3 */
    int    luma_search;    /* NLM 搜索窗：1..31 的奇数，默认 11；BM3D 不用 */
    int    chroma_search;  /* 同上，色度，默认 11 */
    int    backend;        /* X3F_DN_BACKEND_*，默认 auto */
} x3f_denoise_t;

/* 填成默认那套。**别拿 {0} 初始化**：0.0 的 strength 会被判成非法，
 * 而窗口填 0 也会被夹到 1。 */
void x3f_denoise_default(x3f_denoise_t *dn);

/* 这张照片的**拍摄 ISO**（CAMF 的 CaptureISO，退回 SensorISO；读不到给 0）。
 *
 * 用它的地方：`x3f_denoise_t.luminance = AUTO` 的判据（x3f_render_auto_luma），
 * 以及宿主自己拼命令行时（DNG / 广色域 TIFF 走外部 x3f_extract，那边表达不了
 * auto，得在 Swift 里先定下来再翻成 -denoise-luma / -no-denoise-luma）。
 * 和降噪用的一律是同一个来源，不是另读一遍 EXIF。 */
double x3f_render_shooting_iso(const char *path);

/* `x3f_denoise_t.luminance` 走 AUTO 时到底开不开（1/0）。
 * path 读不到 ISO 就给 0：**判据不成立时按"关"走**，与 0.1.6 的行为一致。 */
int x3f_render_auto_luma(const char *path);

/* GPU 降噪能不能用。能用返回 1，并把适配器名（"Apple M1 (Metal)"）写进
 * name_out（NUL 结尾，最多 cap 字节；name_out 可传 NULL）。
 * 没编 GPU 后端、或拿不到适配器时返回 0。探测要几十毫秒，结果在本进程内缓存。 */
int  x3f_render_gpu_available(char *name_out, int cap);

/* 解成线性 sRGB。成功返回 X3FRENDER_OK。
 * rgb 由本模块 malloc，用 x3f_render_free() 释放；w*h*3 个 float。
 * wb_out 可传 NULL，否则写入实际使用的白平衡预设名。 */
int  x3f_render_load(const char *path, float **rgb, int *w, int *h, char *wb_out);

/* 同上，但直接按 down×down 做盒式降采样（down=1 等价于 x3f_render_load）。
 * 预览要的是"够看"而不是"够大"：5424x3616 的 1/4 只有 1.2 MP，
 * 拖滑块时每帧渲染的开销从 ~1s 降到 ~20ms。 */
int  x3f_render_load_ex(const char *path, int down,
                        float **rgb, int *w, int *h, char *wb_out);

/* 同 x3f_render_load_ex，但额外返回一张"RAW 饱和分"图（w*h 个 float）。
 * clip[i] = 该像素有几个通道顶到传感器上限的平滑计数（0..3）：
 *   两个以上通道饱和时，白平衡会把偏色推出来（品红），此时才需要高光去饱和。
 * 把这张图传给 x3f_render_apply_ex，去饱和就会按"RAW 实际过曝量"触发，
 * 而不是按显示值是否近白——这样压低曝光/做高光恢复时品红也不会冒出来。
 * clip 由本模块 malloc，用 x3f_render_free() 释放。clip_out 传 NULL 等价于 load_ex。 */
int  x3f_render_load_ex2(const char *path, int down,
                         float **rgb, float **clip,
                         int *w, int *h, char *wb_out);

/* 同上，但**顺手**再返回一张"RAW 域 4×4 块最大值"图（高光恢复用）。
 *
 * 为什么要它：HDR 导出既需要线性 RGB（基础层），又需要一张 RAW 域的亮度图
 * （未切平区拟合相机曲线、切平区反推真实亮度）。那张图原来是单独再解一遍
 * RAW 拿到的 —— 换 x3fuse 之后一次解码要 2 秒上下，同一张照片解两遍纯属浪费。
 * 这里在**同一次**解码里把它算出来：就是 load_pixel 读到的那些 u16（矩阵之前）
 * 逐像素取三通道最大，再按 4×4 块求平均。与"另外解一遍再取 max"逐位相同。
 *
 * rawmax 为 ceil(w/4)×ceil(h/4) 个 float（rawmax==NULL 时忽略）。
 * 只在 down==1 时计算（别的降采样级别拿不到逐像素的 RAW 值，返回 X3FRENDER_ERR_PARAM）。
 *
 * nlm = 导出用的降噪强度（0..10，0 = 关）。
 * dn  = 降噪的算法/通道/倍率/窗口/后端（见 x3f_denoise_t），**可传 NULL**
 *       = 默认那套。它只在 nlm != 0 时有意义。
 *
 * 为什么不放在 x3f_edit_params_t 里：它作用在**解码**这一步（BMT 域的
 * fast_nlm_denoise_controlled），不是 apply 那一步，所以只能随 load 传进去。
 * 而这条入口是**唯一**带 nlm 的：它只有导出会用（要那张 RAW 亮度图），
 * 预览/放大走的 ex/ex2 一律不降噪 —— 全分辨率 NLM 每张要多花十几秒
 * （实测 1.6s → 16s），拖滑块不能背这个成本。
 * 这套 NLM 本来就是只压色度的（亮度权重 0），所以不会动结构。
 * 实测 DP3M0533 暗部相邻色度差降幅：无降噪 0 → NLM 5 → 84%；NLM 10 → 86%
 * （我们原来那套显示域色度低通是 75%）。
 *
 * ex / ex2 / crop 那条路读环境变量 X3F_NLM（只给 lab 工具对拍用，应用不设）。 */
int  x3f_render_load_ex3(const char *path, int down, int nlm,
                         const x3f_denoise_t *dn,
                         float **rgb, float **clip,
                         float **rawmax, int *rmw, int *rmh,
                         int *w, int *h, char *wb_out);

/* 同 ex2，但可以指定**相机自己的白平衡预设**，并且带上导出降噪 —— 宿主
 * （X3F 抽取器的预览与导出）用的就是这一条。
 *
 * wb = "Sunlight" / "Shade" / "Overcast" / "Incandescent" / "Fluorescent" /
 *      "Flash" / "Auto" / "Custom"；NULL 或空串 = 用文件里记录的那个。
 *      名字同时决定用哪一份**每光源色彩校正矩阵**，所以换 WB 是整条色彩
 *      标定跟着换 —— 这也是它只能在这一层传、不能当显示域滑块的原因。
 *      预设名对不上的文件会让解码失败（返回 X3FRENDER_ERR_DECODE），
 *      宿主应当把可选项限制在文件里真有的那几档。
 *
 * nlm = 0..10 的降噪强度（0 = 关；见 ex3）；dn 同 ex3，可传 NULL。
 * rawmax 非空时行为同 ex3：顺手返回 RAW 域亮度图（只在 down==1 时给）。 */
int  x3f_render_load_wb(const char *path, int down, const char *wb, int nlm,
                        const x3f_denoise_t *dn,
                        float **rgb, float **clip,
                        float **rawmax, int *rmw, int *rmh,
                        int *w, int *h, char *wb_out);

/* ---- SFD 合成：X3I 的 7 帧包围曝光合成成一张 --------------------------
 *
 * 照 `SFDComposeWFA`（见 §2/§5）：
 *
 *     LUT_A[i] = 1/(1+exp(0.005·(i−9000)))              高光软门（i 越大权重越小）
 *     LUT_B[i] = max(1/(1+exp(0.015·(300−i))), 1e-10)   只压很暗的一小段
 *     w        = LUT_A[V]·LUT_B[V] / (k²·σ²(V))
 *     acc_val += w·k·V ;  acc_w += w                     → 出图 = acc_val/acc_w
 *     k        = 本帧快门 ÷ 基准帧快门（CAMF 的 CaptureShutter，µs）
 *
 * 三个落地口径（都和 SPP 对得上，写下来免得下次再猜）：
 * · **V 用"DN 当量"**： LUT 是拿信号值本身当索引的（clamp 到 65535，
 *     而它们的信号是 14 bit DN，饱和 ≈16383），所以表中心 9000 ≈ 满量程的 55%。
 *     我们是显示线性域（1.0 = SDR 白），于是按**实测的饱和点**换算：
 *     V = lin · 16383 / LIN_SAT，LIN_SAT 从第 0 帧解码顺手带出来的饱和分
 *     （clip 通道）反解 —— 不写死常数，换机身/换 ISO 都不用改。
 *   · **σ² 用相机自己的噪声模型**（CAMF 的 ColorNoiseModel，逐通道 a+b·V+c·V²；
 * Quattro 的 CAMF 里没有，走 那组常量）—— 与降噪那一路同源。
 *   · **k 精确是 2 的幂**（实测 DP2Q2293：870/1740/3480/6960/13920/27841/55682 µs）
 *     所以曝光归一是干净的整数比；读不到快门时退化成 k 全 1。
 *
 * 降噪只对**基准帧**施加（`nlm`/`dn`）：融合本来就是多帧平均，7 帧全跑 NLM
 * 要多花 6 倍时间而收益很小。App 侧 SFD **默认 nlm = 0**（融合后还有 Stage1），
 * 用户可以在面板上单独拉高；预览、1:1、导出共用同一档。
 *
 * **不再做"缩放到饱和值"**（2026-10-17 撤掉）：显示曲线 kX3FSFDTone 本来就是拿
 * 未归一的合成数据标定的；输出口径是线性 sRGB float，1.0 = SDR 白、高光可以 >1，
 * 最高到各帧曝光范围之和。
 * **权重里"哪些帧算数"**（2026-10-17 补）：软门 LUT_A 只把接近饱和的帧调小、不排除，
 * 于是"七帧同时被压到高光端"时权重只剩 1/k² 的差别，合成值会被**曝光最长、最贴饱和**
 * 的那一帧牵着走（而那正是最不可信的一帧）。现在：像素被切平（clip ≥ 0.9）或该通道
 * 的门值低于 `X3F_SFD_GATE_USABLE`（已在饱和肩上）的帧**不参与加权**，只贡献一个"下界"
 * （k·lin，= 这帧对场景亮度的估计）；一个通道里所有帧都不参与时就用那个下界。
 * 暗部一侧不受影响（那里所有帧都在线性段上，权重照公式走）。
 *
 * **不再做"缩放到饱和值"**（同日撤掉）：显示曲线 kX3FSFDTone 本来就是拿**未归一**的
 * 合成数据标定的，再乘一次 1/max 会让整幅比 导出暗 1.6~3.2 档，而且让映射随
 * 画面内容变。输出口径回到"1.0 = SDR 白、高光可以 >1"。
 *
 * `note` 里回一句诊断（帧数、每帧 k、饱和点、物理峰值），给状态行用。
 * 普通 X3F（帧数 1）返回 X3FRENDER_ERR_PARAM。
 *
 * `norm_out`（可传 NULL）：**恒为 1.0**（归一撤掉之后留下的接口，调用方"除回来"
 * 那一步因此自动变成恒等）。
 */
int  x3f_render_load_sfd(const char *path, int down, const char *wb, int nlm,
                         const x3f_denoise_t *dn, int max_frames,
                         float **rgb, int *w, int *h, char *note, int note_cap,
                         double *norm_out);

/* 逐帧回调：七帧合成里每解出一帧、**还没进累加器之前**调用一次。
 *
 * `rgb` 是该帧的线性数据（1.0 = 该帧自己的 SDR 白），`k` 是
 * 「基准帧快门 ÷ 本帧快门」的曝光归一系数（欠曝帧 k>1）。回调可以直接改 `rgb`
 * （例如把这一帧与它自己的内嵌 JPEG 做高光融合），合成随后用的就是改过的数据。
 * 返回 0 = 继续；非 0 = 放弃这一次合成（`x3f_render_load_sfd_ex` 会把该值当错误码
 * 返回）。`ctx` 原样传回，由宿主决定它是什么。
 *
 * 传 NULL 时行为与 `x3f_render_load_sfd` 完全一样。 */
typedef int (*x3f_sfd_frame_fuse_fn)(void *ctx, int frame, float *rgb,
                                     int w, int h, double k);

/* 七帧合成的进度：done = 已处理帧数（0…total）。只在帧循环里回调；
 * 后面的 Stage1/核心重建不再细分，宿主可把 done==total 当作 ~90%。 */
typedef void (*x3f_sfd_progress_fn)(void *ctx, int done, int total);

int  x3f_render_load_sfd_ex(const char *path, int down, const char *wb, int nlm,
                            const x3f_denoise_t *dn, int max_frames,
                            float **rgb, int *w, int *h, char *note, int note_cap,
                            double *norm_out,
                            x3f_sfd_frame_fuse_fn frame_fuse, void *frame_fuse_ctx,
                            x3f_sfd_progress_fn progress, void *progress_ctx);

/* 一帧的 JPEG 高光融合（给上面的回调用，也可以单独 A/B）。
 *
 * `rgb` 是该帧的线性数据（原地改），`jpeg` 是**传感器朝向**的 8bit sRGB
 * （`stride` 字节/像素，通常 3 或 4）。做法与 App 里单帧那条
 * `fuseJPEGHighlights` 同一个口径：
 *
 *   1. 该帧线性 → 基础曲线显示值 d（1.0 = SDR 白）；
 *   2. 该帧 JPEG 的 sRGB 先反 gamma 成线性，再乘 `k` 归一到基准帧曝光，
 *      过同一条基础曲线得到 guide 显示值 j；
 *   3. 按 d 的亮度和 j 的亮度各给一个平滑权重，把 d 往 j 拉；
 *   4. 拉完的显示值查一次反向基础曲线回到线性，除以 k 交还给合成。
 *
 * 这样每一帧都是「先融合、再合成」，而不是拿合成结果去贴某一张 JPEG。
 * 返回实际改动的像素数；参数不合法返回 -1。 */
long x3f_sfd_fuse_frame_linear(float *rgb, int w, int h,
                               const uint8_t *jpeg, int stride,
                               double k, double strength,
                               double threshold, double jpeg_floor);

/* 只解一块**原生分辨率**的区域（给预览放大看细节用）。
 * x0/y0/w/h 是传感器朝向的像素坐标/尺寸，会自动夹进图像范围。
 * 输出仍是线性 sRGB；w_out/h_out 是**实际**取到的尺寸。
 * 代价与整幅解码相同（RAW 必须整幅解出来），所以调用方要防抖。 */
int  x3f_render_load_crop(const char *path, int x0, int y0, int w, int h,
                          float **rgb, float **clip,
                          int *w_out, int *h_out, char *wb_out);

/* 「正立图像」上的像素矩形 → 传感器坐标矩形（EXIF 方向的换算）。
 *
 * 预览放大时，界面上拿到的是**正立**（按 EXIF 转正后）的坐标，而裁剪切的是
 * 传感器朝向的 RAW，这一步就是两者的桥。推导见 x3f_render.c 里这张表：
 *   1 : 原样              3 : 两个方向翻转
 *   6 : 顺时针 90°        8 : 逆时针 90°（6/8 会交换宽高）
 * 越界坐标会被夹进图像内；输出矩形保证至少有 1×1 个像素。 */
void x3f_render_upright_rect_to_sensor(int orientation, int sensor_w, int sensor_h,
                                       int x, int y, int w, int h,
                                       int *sx, int *sy, int *sw, int *sh);

void x3f_render_free(float *rgb);

/* 把线性 RGB 渲染成 8 位 sRGB（通道交错）。
 * out 至少 h*stride 字节，stride >= w*3。 */
int  x3f_render_apply(const x3f_edit_params_t *p,
                      const float *lin, int w, int h,
                      uint8_t *out, int stride);

/* 同 x3f_render_apply，但带 RAW 饱和分图（可传 NULL，退回按显示值去饱和）。 */
int  x3f_render_apply_ex(const x3f_edit_params_t *p,
                         const float *lin, const float *clip,
                         int w, int h, uint8_t *out, int stride);

/* 同 x3f_render_apply_ex，但输出 **16 位**（每通道 u16，sRGB 编码，0..65535）。
 *
 * 为什么需要它：8 位出口是给屏幕 / JPEG 的，进后期（Lightroom / PS）时那一次
 * 量化就是实打实的损失 —— 暗部一拉、再锐一次，色带和杂色都从这儿来。
 * 这条出口走的是**同一条管线**（基础曲线、高光恢复 / 高光保留、去饱和、
 * 饱和度、锐度 / 细节、填充光全都在），只是最后落到 16 位，所以同一个文件
 * 可以同时出 8 位 JPEG 和 16 位 TIFF，画面一致、精度更高。
 *
 * stride 是**字节**数（= w*3*2）。输出色彩空间由 p->out_space 决定：六个档位
 * 都是完整观感链，转域在最后一步（见 out_space 字段的说明）。 */
int  x3f_render_apply_ex16(const x3f_edit_params_t *p,
                           const float *lin, const float *clip,
                           int w, int h, uint16_t *out, int stride);

/* 同 x3f_render_apply_ex16，但输出**未钳制的 float 场景线性**（RGB 交错，每
 * 通道 float，长度 w*h*3）。仅与 look_defer=1 配套：WB / 填充光 / 细节 /
 * 色彩模式矩阵照常做，但最后一步不钳 [0,1]，把 HDR 高光余量直接交给胶片引擎
 * （Fotufilm 的 halation 由 >1.0 溢出驱动，16 位钳制会让它失效）。 */
int  x3f_render_apply_ex_linear(const x3f_edit_params_t *p,
                                const float *lin, const float *clip,
                                int w, int h, float *out);

/* 只跑「显示域空间算子」（锐度 / 细节 / 阴影高光分区）—— apply_spatial 一个
 * 函数，不碰任何色调曲线、色彩模式、饱和度、换域。
 *
 * 给**两段式的第二段直通**用（2026-11-11）：观感参数全默认时第二段整段不跑
 * （引擎码值就是成品，与官方 SpektraLab 一致），但锐化是显示域算子、引擎不
 * 做，所以单独补这一下。
 *
 * px 是 `p->out_space` 域的 16 位码值，stride 字节数（= w*3*2）。
 * 参数只读 sharpness / sharp_res / detail / shadow / highlight / out_space /
 * hl_soften。参数全 0 时 apply_spatial 直接返回（零开销）。 */
int  x3f_render_sharpen16(const x3f_edit_params_t *p, uint16_t *px,
                          int w, int h, int stride);

/* 把一整幅**已经渲染好**的码值从 `from_space` 搬到 `to_space`（整幅换域）。
 *
 * 2026-11-04（全链路广色域）之后，渲染主链**不再需要它**：转域已经搬进
 * x3f_render_apply_ex / _ex16 的最后一步，在 double 上做、负值奇对称外推，
 * 所以 sRGB 之外的颜色能一路带到目标域（见下面 out_space 段的说明）。
 *
 * 现在只剩一个兜底用途：胶片引擎（SpektraLab host）的 wire 名只有
 * srgb / display-p3 / prophoto 三个，Adobe / Rec.2020 / DCI-P3 三档只能先在
 * ProPhoto 里跑完引擎，出来再换最后一次。那条路上数据是整数码值，
 * **负值与 >1 会被夹到容器能表示的范围** —— 所以只在必要时用。
 *
 * 三个空间之间统一经**线性 sRGB**中转（现有矩阵全是 sRGB→X，反向运行时求逆），
 * 与渲染链上 out_lin_convert / out_encode / out_decode 用的是同一套常数。
 *
 * `from_space` / `to_space` 取 X3F_OUT_*；越界值回落 sRGB。
 * 相同档位直接 memcpy（逐位一致，方便调用方无脑调用）。
 * src / dst 都是通道交错的 RGB，元素数 w*h*3；16 位的 stride 语义与
 * x3f_render_apply_ex16 一致（这里是元素数，不是字节数）。 */
int  x3f_render_reencode8(const uint8_t *src, int w, int h,
                          int from_space, int to_space, uint8_t *dst);
int  x3f_render_reencode16(const uint16_t *src, int w, int h,
                           int from_space, int to_space, uint16_t *dst);

/* 把预先算好的色调 LUT 直接套上去（给曲线拟合工具用）。
 * lut 有 X3F_TONE_LUT_SIZE+1 项，值域 0..65535。 */
#define X3F_TONE_LUT_SIZE 4096
int  x3f_render_apply_lut(const uint16_t *tone_lut,
                          const float *lin, int w, int h,
                          uint8_t *out, int stride);

/* 线性 sRGB 数值上限：LUT 的输入域是 [0, LIN_MAX] */
#define X3F_LIN_MAX 8.0

#ifdef __cplusplus
}
#endif

/*Stage1「多尺度降噪」（三级金字塔，见 x3f_stage1.c）：
 * 就地把线性缓冲（1.0 = SDR 白）收一遍 —— 平坦区按噪声模型平滑、边缘保留。
 * coef = 9 个噪声系数（逐通道 a/b/c），lin_sat = 饱和点的线性值（DN 当量换算用）。
 * 返回实际跑的级数（0 = 关掉 / 太小）。环境变量：X3F_STAGE1=0 关、=2 只跑最粗一级，
 * X3F_STAGE1_STRENGTH=0..1 总强度。 */
int x3f_stage1_denoise(float *rgb, int w, int h, const double coef[9], double lin_sat);

#endif /* X3F_RENDER_H */
