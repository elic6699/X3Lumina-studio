//! M5b: native Rust TRUE entropy decoder.
//!
//! Opt-in via `X3F_RUST_DECODE=1`. The C dispatch in `src/x3f_io.c`'s
//! `true_decode` checks the env var and calls [`x3f_rust_true_decode`]
//! here instead of the per-color C loop. The C side still owns:
//!   - reading the file bytes into `ID->data`
//!   - reading the Huffman table header
//!   - building the Huffman tree (`populate_true_huffman_tree`)
//!   - allocating the `TRU->x3rgb16` and `Q->top16` output buffers
//!   - computing `TRU->plane_address[]` and the per-plane geometry
//!
//! All this Rust code does is the per-color decode loop: walk the prebuilt
//! Huffman tree to read a symbol-length, read that many magnitude bits,
//! sign-extend, accumulate horizontally, and write into the output buffer.
//! It is exact integer math — Tier-3 must show byte-equality between the
//! C and Rust paths.
//!
//! Symbol export: `#[no_mangle] extern "C"` like quattro.rs. A `#[used]`
//! static anchors the symbol so cross-crate DCE does not strip it before
//! the legacy C library's call site is linked.

use std::os::raw::c_void;

// ---------------------------------------------------------------------------
// Mirrored C structs from x3f_io.h. Layout-asserted by the test below; if
// the headers change we want a hard build break, not silent mis-decoding.
// ---------------------------------------------------------------------------

#[repr(C)]
struct Area16 {
    data: *mut u16,
    buf: *mut c_void,
    rows: u32,
    columns: u32,
    channels: u32,
    row_stride: u32,
}

#[repr(C)]
struct HuffNode {
    branch: [*mut HuffNode; 2],
    leaf: u32,
}

#[repr(C)]
pub(crate) struct HuffTree {
    free_node_index: u32,
    nodes: *mut HuffNode,
    /// 快速查表（本项目对上游的本地修改）：一次取 FAST_BITS 位当索引，
    /// 得到 (符号 << 8) | 码长；0 表示未覆盖，退回逐位爬树。
    /// 必须与 csrc/x3f_io.h 的 x3f_hufftree_t 同步——它是内嵌在
    /// x3f_true_t 里的，布局对不上会直接读错内存。
    fast: *mut u16,
}

#[repr(C)]
struct TrueHuffmanElement {
    code_size: u8,
    code: u8,
}

#[repr(C)]
struct TrueHuffmanTable {
    size: u32,
    element: *mut TrueHuffmanElement,
}

#[repr(C)]
struct Table32 {
    size: u32,
    element: *mut u32,
}

#[repr(C)]
struct Table16 {
    size: u32,
    element: *mut u16,
}

#[repr(C)]
struct Area8 {
    data: *mut u8,
    buf: *mut c_void,
    rows: u32,
    columns: u32,
    channels: u32,
    row_stride: u32,
}

#[repr(C)]
struct Huffman {
    mapping: Table16,
    table: Table32,
    tree: HuffTree,
    row_offsets: Table32,
    rgb8: Area8,
    x3rgb16: Area16,
}

const TRUE_PLANES: usize = 3;

#[repr(C)]
struct True {
    seed: [u16; TRUE_PLANES],
    unknown: u16,
    table: TrueHuffmanTable,
    plane_size: Table32,
    plane_address: [*mut u8; TRUE_PLANES],
    tree: HuffTree,
    x3rgb16: Area16,
}

#[repr(C)]
struct QuattroPlane {
    columns: u16,
    rows: u16,
}

#[repr(C)]
struct Quattro {
    plane: [QuattroPlane; TRUE_PLANES],
    unknown: u32,
    quattro_layout: i32, // C `bool_t` is `int`
    top16: Area16,
}

#[repr(C)]
pub(crate) struct ImageData {
    type_: u32,
    format: u32,
    type_format: u32,
    columns: u32,
    rows: u32,
    row_stride: u32,
    huffman: *mut Huffman,
    tru: *mut True,
    quattro: *mut Quattro,
    data: *mut c_void,
    data_size: u32,
}

// Format constants from x3f_io.h.
const X3F_IMAGE_RAW_QUATTRO: u32 = 0x0001_0023;
const X3F_IMAGE_RAW_SDQ: u32 = 0x0001_0025;
const X3F_IMAGE_RAW_SDQH: u32 = 0x0001_0027;

// ---- 快速查表（本项目对上游的本地修改）-------------------------------
//
// 上游是一位一位沿树走，每个符号要 3~5 次指针追逐。这里改成一次取
// FAST_BITS 位当索引、一次得到 (符号, 码长)，再把码长一次性跳过。
// 建表时把码字按「第一位读到的落在最高位」的顺序摆好——与 peek_fast 的
// 下标语义对应。
//
// 纯优化：查表命中与逐位爬树在数学上完全等价（同一棵树、同一段比特流）。
// 实测解码输出与未开表时逐位一致。
pub(crate) const FAST_BITS: u32 = 10;
const FAST_SIZE: usize = 1 << FAST_BITS;

#[inline]
fn is_quattro_format(t: u32) -> bool {
    matches!(
        t,
        X3F_IMAGE_RAW_QUATTRO | X3F_IMAGE_RAW_SDQ | X3F_IMAGE_RAW_SDQH
    )
}

use crate::control::{Control, Error, Result};

pub(crate) struct BitReader<'a> {
    bytes: &'a [u8],
    /// 还没装进 buf 的下一个字节下标
    pos: usize,
    /// 左对齐的比特缓冲：有效的 nbits 位待在最左边
    buf: u64,
    /// buf 里还有多少位有效（从最高位往右数）
    nbits: u32,
}

impl<'a> BitReader<'a> {
    pub(crate) fn new(bytes: &'a [u8]) -> Self {
        let mut reader = Self {
            bytes,
            pos: 0,
            buf: 0,
            nbits: 0,
        };
        reader.refill();
        reader
    }

    /// 把缓冲尽量填满（最多 64 位）。
    ///
    /// 本地修改：上游是「每次 get_bit 都去做一次 bytes[bit/8] 的边界检查 +
    /// 移位」，而幅度位是逐位读的（每个符号要读好几个），于是这部分成了
    /// 解码热路径的主开销——实测比 C 版（32 位缓冲）慢 1.7~2.7x。
    /// 改成一次装 8 字节之后，取位全是寄存器操作，只在快用完时补一次内存。
    #[inline]
    fn refill(&mut self) {
        while self.nbits <= 56 && self.pos < self.bytes.len() {
            self.buf |= (self.bytes[self.pos] as u64) << (56 - self.nbits);
            self.pos += 1;
            self.nbits += 8;
        }
    }
    /// 一次取 FAST_BITS 位当查表下标，不移动读指针。
    ///「第一个读到的位」落在最高位，与 C 版 bs_peek 一致。
    /// 缓冲里不够 FAST_BITS 位时返回 None —— 交给逐位慢路径，
    /// 这样流被截断时的报错行为与原来完全一样。
    #[inline]
    fn peek_fast(&self) -> Option<u32> {
        if self.nbits < FAST_BITS {
            return None;
        }
        Some((self.buf >> (64 - FAST_BITS)) as u32 & (FAST_SIZE as u32 - 1))
    }
    #[inline]
    pub(crate) fn skip(&mut self, n: u32) {
        debug_assert!(n <= self.nbits);
        self.buf <<= n;
        self.nbits -= n;
        self.refill();
    }
    /// 已消费的比特数（refill 的预读通过 nbits 扣回来）。
    pub(crate) fn consumed_bits(&self) -> usize {
        self.pos.saturating_mul(8).saturating_sub(self.nbits as usize)
    }
    #[inline]
    pub(crate) fn get_bit(&mut self) -> Result<u8> {
        if self.nbits == 0 {
            return Err(Error::InvalidData("truncated entropy stream"));
        }
        let result = (self.buf >> 63) as u8;
        self.buf <<= 1;
        self.nbits -= 1;
        self.refill();
        Ok(result)
    }
}

/// 递归填表。`pattern` 是「已读到的位」在 FAST_BITS 位下标里的位置：
/// 第 depth 位对应下标的第 (FAST_BITS-1-depth) 位。
unsafe fn fast_fill(
    node: *const HuffNode,
    table: &mut [u16],
    pattern: u32,
    depth: u32,
    base: usize,
    bytes: usize,
) {
    if node.is_null() {
        return;
    }
    let offset = match (node as usize).checked_sub(base) {
        Some(o) => o,
        None => return,
    };
    if offset >= bytes || offset % std::mem::size_of::<HuffNode>() != 0 {
        return;
    }
    let value = unsafe { &*node };
    if value.branch[0].is_null() && value.branch[1].is_null() {
        // 叶子。叶子值就是「后面还要读多少位幅度」；超过 8 位装不进
        // 表项，留给慢路径（TRUE 的码长实测 ≤ 5，正常进不来）。
        if depth > 0 && depth <= FAST_BITS && value.leaf != u32::MAX && value.leaf <= 0xff {
            let step = 1u32 << (FAST_BITS - depth);
            let entry = ((value.leaf as u16) << 8) | depth as u16;
            for i in 0..step {
                table[(pattern | i) as usize] = entry;
            }
        }
        return;
    }
    if depth >= FAST_BITS {
        return; // 比表还长，交给慢路径
    }
    unsafe {
        fast_fill(value.branch[0], table, pattern, depth + 1, base, bytes);
        fast_fill(
            value.branch[1],
            table,
            pattern | (1u32 << (FAST_BITS - 1 - depth)),
            depth + 1,
            base,
            bytes,
        );
    }
}

/// 建查表。在 `true_tree()` 建完树之后调用。失败不致命——`fast` 留 NULL
/// 就退回逐位爬树（行为与上游一致）。
///
/// 用 libc::calloc：一是表项默认 0（正好是「未覆盖」的哨兵），
/// 二是与 `cleanup_huffman_tree` 的 libc::free 配对。
pub(crate) unsafe fn build_fast_table(tree: *mut HuffTree) -> Result<()> {
    if tree.is_null() {
        return Ok(());
    }
    let t = unsafe { &*tree };
    if t.nodes.is_null() || t.free_node_index == 0 || !t.fast.is_null() {
        return Ok(());
    }
    let table = unsafe { libc::calloc(FAST_SIZE, std::mem::size_of::<u16>()) } as *mut u16;
    if table.is_null() {
        return Ok(()); // 内存不足就退回慢路径
    }
    let base = t.nodes as usize;
    let bytes = t.free_node_index as usize * std::mem::size_of::<HuffNode>();
    {
        let slice = unsafe { std::slice::from_raw_parts_mut(table, FAST_SIZE) };
        unsafe { fast_fill(t.nodes, slice, 0, 0, base, bytes) };
    }
    unsafe { (*tree).fast = table };
    Ok(())
}

#[inline]
unsafe fn read_leaf(br: &mut BitReader<'_>, tree: *const HuffTree) -> Result<u32> {
    if tree.is_null() {
        return Err(Error::InvalidData("missing Huffman tree"));
    }
    let tree = unsafe { &*tree };
    if tree.nodes.is_null() || tree.free_node_index == 0 {
        return Err(Error::InvalidData("empty Huffman tree"));
    }
    let start = tree.nodes as usize;
    let bytes = tree.free_node_index as usize * std::mem::size_of::<HuffNode>();
    let mut node = tree.nodes;
    for _ in 0..=27 {
        let offset = (node as usize)
            .checked_sub(start)
            .ok_or(Error::InvalidData("Huffman node outside tree"))?;
        if offset >= bytes || offset % std::mem::size_of::<HuffNode>() != 0 {
            return Err(Error::InvalidData("Huffman node outside tree"));
        }
        let value = unsafe { &*node };
        if value.branch[0].is_null() && value.branch[1].is_null() {
            if value.leaf == u32::MAX {
                return Err(Error::InvalidData("undefined Huffman code"));
            }
            return Ok(value.leaf);
        }
        node = value.branch[br.get_bit()? as usize];
        if node.is_null() {
            return Err(Error::InvalidData("invalid Huffman code"));
        }
    }
    Err(Error::InvalidData("Huffman tree exceeds maximum depth"))
}

/// 取下一个符号（= 随后要读的幅度位数）：先查表，没覆盖再逐位爬树。
#[inline]
unsafe fn read_leaf_fast(br: &mut BitReader<'_>, tree: *const HuffTree) -> Result<u32> {
    let fast = if tree.is_null() {
        std::ptr::null_mut()
    } else {
        unsafe { (*tree).fast }
    };
    if !fast.is_null() {
        if let Some(index) = br.peek_fast() {
            let entry = unsafe { *fast.add(index as usize) };
            let length = (entry & 0xff) as u32;
            if length != 0 {
                br.skip(length);
                return Ok((entry >> 8) as u32);
            }
        }
    }
    unsafe { read_leaf(br, tree) }
}

#[inline]
pub(crate) unsafe fn true_diff(br: &mut BitReader<'_>, tree: *const HuffTree) -> Result<i32> {
    let bits = unsafe { read_leaf_fast(br, tree) }?;
    if bits > 31 {
        return Err(Error::InvalidData("TRUE difference exceeds 31 bits"));
    }
    if bits == 0 {
        return Ok(0);
    }
    let first = br.get_bit()?;
    let mut difference = first as i64;
    for _ in 1..bits {
        difference = (difference << 1) | br.get_bit()? as i64;
    }
    if first == 0 {
        difference -= (1i64 << bits) - 1;
    }
    Ok(difference as i32)
}

unsafe fn true_decode_one_color(
    id_ptr: *const ImageData,
    color: usize,
    control: Control<'_>,
) -> Result<()> {
    let id = unsafe { &*id_ptr };
    if id.tru.is_null() || id.data.is_null() {
        return Err(Error::InvalidData("missing TRUE data"));
    }
    let tru = unsafe { &*id.tru };
    if tru.plane_size.size != 3 || tru.plane_size.element.is_null() {
        return Err(Error::InvalidData("missing TRUE plane lengths"));
    }
    let mut rows = id.rows;
    let mut cols = id.columns;
    let mut output = &tru.x3rgb16;
    let mut channel = color;
    if is_quattro_format(id.type_format) {
        if id.quattro.is_null() {
            return Err(Error::InvalidData("missing Quattro planes"));
        }
        let q = unsafe { &*id.quattro };
        rows = q.plane[color].rows as u32;
        cols = q.plane[color].columns as u32;
        if q.quattro_layout != 0 && color == 2 {
            output = &q.top16;
            channel = 0;
        }
    }
    if output.data.is_null()
        || rows != output.rows
        || cols < output.columns
        || channel >= output.channels as usize
        || output.row_stride
            < output
                .columns
                .checked_mul(output.channels)
                .ok_or(Error::Allocation)?
    {
        return Err(Error::InvalidData("invalid TRUE output geometry"));
    }
    let start = (tru.plane_address[color] as usize)
        .checked_sub(id.data as usize)
        .ok_or(Error::InvalidData("TRUE plane outside image"))?;
    let length = unsafe { *tru.plane_size.element.add(color) } as usize;
    if start
        .checked_add(length)
        .map_or(true, |end| end > id.data_size as usize)
    {
        return Err(Error::InvalidData("TRUE plane outside image"));
    }
    let bytes = unsafe { std::slice::from_raw_parts(tru.plane_address[color], length) };
    let mut br = BitReader::new(bytes);
    let mut starts = [[tru.seed[color] as i32; 2]; 2];
    for row in 0..rows {
        control.check()?;
        let parity = (row & 1) as usize;
        let mut acc = [0i32; 2];
        for col in 0..cols {
            if col % 4096 == 0 {
                control.check()?;
            }
            let odd = (col & 1) as usize;
            let previous = if col < 2 {
                starts[parity][odd]
            } else {
                acc[odd]
            };
            let diff = unsafe { true_diff(&mut br, &tru.tree) }?;
            let value = previous
                .checked_add(diff)
                .ok_or(Error::InvalidData("TRUE predictor overflow"))?;
            acc[odd] = value;
            if col < 2 {
                starts[parity][odd] = value;
            }
            if col < output.columns {
                let index = row as usize * output.row_stride as usize
                    + col as usize * output.channels as usize
                    + channel;
                unsafe { *output.data.add(index) = value as u16 };
            }
        }
    }
    if std::env::var_os("X3F_HLP_PROBE").is_some() {
        let used = br.consumed_bits();
        let total = bytes.len().saturating_mul(8);
        eprintln!(
            "[hlp-probe] color={} rows={} cols={} plane_bytes={} used_bits={} left_bytes={}",
            color,
            rows,
            cols,
            bytes.len(),
            used,
            total.saturating_sub(used) / 8
        );
    }
    Ok(())
}

pub(crate) unsafe fn true_decode(id: *mut ImageData, control: Control<'_>) -> Result<()> {
    control.check()?;
    if id.is_null() {
        return Err(Error::InvalidData("missing TRUE image"));
    }
    #[derive(Clone, Copy)]
    struct SyncId(*const ImageData);
    unsafe impl Send for SyncId {}
    unsafe impl Sync for SyncId {}
    impl SyncId {
        fn get(self) -> *const ImageData {
            self.0
        }
    }
    let shared = SyncId(id);
    use rayon::prelude::*;
    (0..3usize)
        .into_par_iter()
        .try_for_each(|color| unsafe { true_decode_one_color(shared.get(), color, control) })
}

#[no_mangle]
pub(crate) unsafe extern "C" fn x3f_rust_true_decode(id: *mut ImageData) {
    let _ = unsafe { true_decode(id, Control::none()) };
}

const X3F_IMAGE_RAW_HUFFMAN_X530: u32 = 0x0003_0005;
const X3F_IMAGE_RAW_HUFFMAN_10BIT: u32 = 0x0003_0006;
const X3F_IMAGE_THUMB_HUFFMAN: u32 = 0x0002_000b;

unsafe fn write_huffman_pixel(
    id: &ImageData,
    huf: &Huffman,
    row: u32,
    col: usize,
    color: usize,
    value: u32,
) -> Result<()> {
    let index = 3 * (row as usize * id.columns as usize + col) + color;
    match id.type_format {
        X3F_IMAGE_RAW_HUFFMAN_X530 | X3F_IMAGE_RAW_HUFFMAN_10BIT => {
            if huf.x3rgb16.data.is_null() {
                return Err(Error::InvalidData("missing Huffman output"));
            }
            unsafe { *huf.x3rgb16.data.add(index) = value as u16 };
        }
        X3F_IMAGE_THUMB_HUFFMAN => {
            if huf.rgb8.data.is_null() {
                return Err(Error::InvalidData("missing Huffman thumbnail output"));
            }
            unsafe { *huf.rgb8.data.add(index) = value as u8 };
        }
        _ => return Err(Error::InvalidData("unsupported Huffman format")),
    }
    Ok(())
}

unsafe fn huffman_decode_row(
    id: &ImageData,
    huf: &Huffman,
    row: u32,
    offset: i32,
    minimum: &mut i32,
    control: Control<'_>,
) -> Result<()> {
    let start = unsafe { *huf.row_offsets.element.add(row as usize) } as usize;
    let end = if row + 1 < id.rows {
        (unsafe { *huf.row_offsets.element.add(row as usize + 1) }) as usize
    } else {
        id.data_size as usize
    };
    if start >= end || end > id.data_size as usize {
        return Err(Error::InvalidData("Huffman row outside image"));
    }
    let bytes = unsafe { std::slice::from_raw_parts(id.data.cast::<u8>().add(start), end - start) };
    let mut br = BitReader::new(bytes);
    let mut acc = [offset as i16; 3];
    for col in 0..id.columns as usize {
        if col % 4096 == 0 {
            control.check()?;
        }
        for (color, value) in acc.iter_mut().enumerate() {
            let diff = unsafe { read_leaf(&mut br, &huf.tree) }? as i32;
            *value = value.wrapping_add(diff as i16);
            *minimum = (*minimum).min(*value as i32);
            unsafe {
                write_huffman_pixel(id, huf, row, col, color, (*value as i32).max(0) as u32)
            }?;
        }
    }
    Ok(())
}

pub(crate) unsafe fn huffman_decode(
    id: *mut ImageData,
    _bits: i32,
    control: Control<'_>,
) -> Result<()> {
    control.check()?;
    if id.is_null() {
        return Err(Error::InvalidData("missing Huffman image"));
    }
    let id = unsafe { &*id };
    if id.huffman.is_null() || id.data.is_null() {
        return Err(Error::InvalidData("missing Huffman data"));
    }
    let huf = unsafe { &*id.huffman };
    if huf.row_offsets.size != id.rows || huf.row_offsets.element.is_null() {
        return Err(Error::InvalidData("missing Huffman row offsets"));
    }
    let mut minimum = 0i32;
    let mut offset = unsafe { crate::legacy_offset };
    for row in 0..id.rows {
        control.check()?;
        unsafe { huffman_decode_row(id, huf, row, offset, &mut minimum, control) }?;
    }
    if unsafe { crate::auto_legacy_offset } != 0 && minimum < 0 {
        offset = -minimum;
        for row in 0..id.rows {
            control.check()?;
            unsafe { huffman_decode_row(id, huf, row, offset, &mut minimum, control) }?;
        }
    }
    Ok(())
}

pub(crate) unsafe fn simple_decode(
    id: *mut ImageData,
    bits: i32,
    row_stride: i32,
    control: Control<'_>,
) -> Result<()> {
    control.check()?;
    if id.is_null() || row_stride <= 0 || !(8..=10).contains(&bits) {
        return Err(Error::InvalidData("invalid simple decoder dimensions"));
    }
    let id = unsafe { &*id };
    if id.huffman.is_null() || id.data.is_null() {
        return Err(Error::InvalidData("missing simple image data"));
    }
    let huf = unsafe { &*id.huffman };
    let stride = row_stride as usize;
    if id.columns as usize * 4 > stride
        || stride
            .checked_mul(id.rows as usize)
            .map_or(true, |n| n > id.data_size as usize)
    {
        return Err(Error::InvalidData("simple image raster exceeds input"));
    }
    let mask = (1u32 << bits) - 1;
    for row in 0..id.rows {
        control.check()?;
        let mut acc = [0u16; 3];
        for col in 0..id.columns as usize {
            if col % 4096 == 0 {
                control.check()?;
            }
            let address = unsafe { id.data.cast::<u8>().add(row as usize * stride + col * 4) };
            let packed = unsafe { u32::from_le(ptr_read_u32(address)) };
            for (color, value) in acc.iter_mut().enumerate() {
                let index = ((packed >> (color * bits as usize)) & mask) as usize;
                let diff = if huf.mapping.size == 0 {
                    index as u16
                } else {
                    if index >= huf.mapping.size as usize || huf.mapping.element.is_null() {
                        return Err(Error::InvalidData("simple mapping index outside table"));
                    }
                    unsafe { *huf.mapping.element.add(index) }
                };
                *value = value.wrapping_add(diff);
                let positive = if id.type_format == X3F_IMAGE_THUMB_HUFFMAN {
                    (*value as u8 as i8) > 0
                } else {
                    (*value as i16) > 0
                };
                unsafe {
                    write_huffman_pixel(
                        id,
                        huf,
                        row,
                        col,
                        color,
                        if positive { *value as u32 } else { 0 },
                    )
                }?;
            }
        }
    }
    Ok(())
}

unsafe fn ptr_read_u32(pointer: *const u8) -> u32 {
    unsafe { std::ptr::read_unaligned(pointer.cast::<u32>()) }
}

#[no_mangle]
pub(crate) unsafe extern "C" fn x3f_rust_huffman_decode(
    id: *mut ImageData,
    bits: std::os::raw::c_int,
) {
    let _ = unsafe { huffman_decode(id, bits, Control::none()) };
}

#[no_mangle]
pub(crate) unsafe extern "C" fn x3f_rust_simple_decode(
    id: *mut ImageData,
    bits: std::os::raw::c_int,
    row_stride: std::os::raw::c_int,
) {
    let _ = unsafe { simple_decode(id, bits, row_stride, Control::none()) };
}

#[used]
static _ANCHOR_X3F_RUST_TRUE_DECODE: unsafe extern "C" fn(*mut ImageData) = x3f_rust_true_decode;
#[used]
static _ANCHOR_X3F_RUST_HUFFMAN_DECODE: unsafe extern "C" fn(*mut ImageData, std::os::raw::c_int) =
    x3f_rust_huffman_decode;
#[used]
static _ANCHOR_X3F_RUST_SIMPLE_DECODE: unsafe extern "C" fn(
    *mut ImageData,
    std::os::raw::c_int,
    std::os::raw::c_int,
) = x3f_rust_simple_decode;
#[cfg(test)]
mod tests {
    use super::*;

    /// Layout assertions: the mirrored structs must match the bindgen-
    /// generated FFI structs byte-for-byte. If `x3f_io.h` changes shape,
    /// catch it at test time rather than at first decode of a real file.
    #[test]
    fn struct_layouts_match_bindgen() {
        use std::mem::{align_of, size_of};

        assert_eq!(
            size_of::<Area16>(),
            size_of::<crate::x3f_area16_t>(),
            "Area16"
        );
        assert_eq!(align_of::<Area16>(), align_of::<crate::x3f_area16_t>());

        assert_eq!(size_of::<HuffNode>(), size_of::<crate::x3f_huffnode_s>());
        assert_eq!(align_of::<HuffNode>(), align_of::<crate::x3f_huffnode_s>());

        assert_eq!(size_of::<HuffTree>(), size_of::<crate::x3f_hufftree_t>());

        assert_eq!(
            size_of::<TrueHuffmanElement>(),
            size_of::<crate::x3f_true_huffman_element_t>()
        );
        assert_eq!(
            size_of::<TrueHuffmanTable>(),
            size_of::<crate::x3f_true_huffman_t>()
        );
        assert_eq!(size_of::<Table32>(), size_of::<crate::x3f_table32_t>());

        assert_eq!(size_of::<True>(), size_of::<crate::x3f_true_t>());
        assert_eq!(align_of::<True>(), align_of::<crate::x3f_true_t>());

        assert_eq!(size_of::<Quattro>(), size_of::<crate::x3f_quattro_t>());

        assert_eq!(size_of::<Table16>(), size_of::<crate::x3f_table16_t>());
        assert_eq!(size_of::<Area8>(), size_of::<crate::x3f_area8_t>());
        assert_eq!(size_of::<Huffman>(), size_of::<crate::x3f_huffman_t>());
        assert_eq!(align_of::<Huffman>(), align_of::<crate::x3f_huffman_t>());

        assert_eq!(size_of::<ImageData>(), size_of::<crate::x3f_image_data_t>());
        assert_eq!(
            align_of::<ImageData>(),
            align_of::<crate::x3f_image_data_t>()
        );
    }

    /// Round-trip the bit reader on a known byte sequence. The C reads MSB
    /// first within each byte; this test pins that ordering.
    #[test]
    fn bit_reader_msb_first_within_byte() {
        let bytes: Vec<u8> = vec![0b1010_0110, 0b1100_0011];
        let mut br = BitReader::new(&bytes);
        let mut got = 0u32;
        for _ in 0..16 {
            got = (got << 1) | br.get_bit().unwrap() as u32;
        }
        assert_eq!(got, 0b1010_0110_1100_0011);
        assert!(matches!(br.get_bit(), Err(Error::InvalidData(_))));
    }

    #[test]
    fn malformed_codes_and_cancelled_decode_return_errors() {
        for leaf in [32, u32::MAX] {
            let mut nodes = [HuffNode {
                branch: [std::ptr::null_mut(); 2],
                leaf,
            }];
            let tree = HuffTree {
                free_node_index: 1,
                nodes: nodes.as_mut_ptr(),
                fast: std::ptr::null_mut(),
            };
            assert!(unsafe { true_diff(&mut BitReader::new(&[0]), &tree) }.is_err());
        }
        let token = std::sync::atomic::AtomicBool::new(true);
        assert!(matches!(
            unsafe { true_decode(std::ptr::null_mut(), Control::new(&token)) },
            Err(Error::Cancelled)
        ));
    }

    /// Synthesise a tiny TRUE-coded plane and decode it. Tree has one symbol
    /// of length 1: code 0 -> magnitude bits 0 (no bits read, diff = 0).
    /// All bits 0 -> all diffs = 0 -> all values = seed.
    #[test]
    fn decode_constant_seed_plane() {
        // Tree: single root node with two branches both leading to the
        // length-0 leaf (so both bits 0 and 1 yield 0-magnitude diffs).
        let mut nodes = [
            HuffNode {
                branch: [std::ptr::null_mut(); 2],
                leaf: u32::MAX,
            },
            HuffNode {
                branch: [std::ptr::null_mut(); 2],
                leaf: 0,
            },
        ];
        nodes[0].branch = [&mut nodes[1] as *mut HuffNode; 2];
        let tree = HuffTree {
            free_node_index: 2,
            nodes: nodes.as_mut_ptr(),
            fast: std::ptr::null_mut(),
        };

        // 1 byte of bit stream is enough to satisfy a 4x4 plane: 16 reads,
        // 16 bits in 2 bytes (we provide both as 0).
        let stream = vec![0u8; 4];

        let mut buf = vec![0u16; 4 * 4 * 3];
        let area = Area16 {
            data: buf.as_mut_ptr(),
            buf: std::ptr::null_mut(),
            rows: 4,
            columns: 4,
            channels: 3,
            row_stride: 4 * 3,
        };

        let mut sizes = [4u32; 3];
        let mut tru = True {
            seed: [123, 0, 0],
            unknown: 0,
            table: TrueHuffmanTable {
                size: 0,
                element: std::ptr::null_mut(),
            },
            plane_size: Table32 {
                size: 3,
                element: sizes.as_mut_ptr(),
            },
            plane_address: [
                stream.as_ptr() as *mut u8,
                std::ptr::null_mut(),
                std::ptr::null_mut(),
            ],
            tree,
            x3rgb16: area,
        };

        let id = ImageData {
            type_: 3,
            format: 0x1e,
            type_format: 0x0003_001e, // TRUE
            columns: 4,
            rows: 4,
            row_stride: 12,
            huffman: std::ptr::null_mut(),
            tru: &mut tru as *mut True,
            quattro: std::ptr::null_mut(),
            data: stream.as_ptr() as *mut c_void,
            data_size: stream.len() as u32,
        };

        unsafe { true_decode_one_color(&id as *const ImageData, 0, Control::none()) }.unwrap();

        for row in 0..4 {
            for col in 0..4 {
                let v = buf[(row * 4 + col) * 3]; // color 0 of pixel (row, col)
                assert_eq!(v, 123, "row {row} col {col}");
            }
        }
    }
}
