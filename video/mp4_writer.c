/**
 * @file mp4_writer.c
 * @brief 最小 MP4 写入器（只支持 1 路 H.264 视频）。
 *
 * 文件布局：  [ftyp][mdat：所有帧连续存放][moov：目录，录制结束时才写]
 *
 * 为了简单，所有帧放在同一个 chunk（块）里（它们在 mdat 里本来就是连续的），
 * 所以 stco 只需要 1 个条目：第一帧的位置；
 * 每帧的位置由"第一帧位置 + 前面所有帧的大小之和"算出来。
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>

#include "mp4_writer.h"
#include "nal_parser.h"

#define MP4_VIDEO_TIMESCALE   90000   /* 视频时间刻度：每秒 90000 个刻度，与 RTP 相同 */
#define MP4_MOVIE_TIMESCALE   1000    /* 整部影片的时间刻度：每秒 1000 个刻度（毫秒） */
#define MP4_MAX_MDAT_BYTES    (0xFFFFFFFFu - 0x100000u)   /* stco 里的偏移是 32 位，文件不能超过约 4GB */

/* H.264 的 NAL 类型编号（NAL 头第一个字节的低 5 位） */
#define NAL_TYPE_SLICE_FIRST  1       /* 普通图像切片（P 帧等），1~5 都是图像数据 */
#define NAL_TYPE_SLICE_IDR    5       /* 关键帧切片 */
#define NAL_TYPE_SPS          7       /* 序列参数集 */
#define NAL_TYPE_PPS          8       /* 图像参数集 */
#define NAL_TYPE_AUD          9       /* 访问单元分隔符 */

/* 每一帧需要记住的信息（录完后用来生成 moov 里的各个表） */
typedef struct {
    td_u32 size_bytes;        /* 这一帧在 mdat 中占多少字节（含每个 NAL 前的 4 字节长度） → stsz */
    td_u32 timestamp_ticks;   /* 显示时间，相对第一帧，单位 1/90000 秒 → stts */
    td_u8  is_key_frame;      /* 是否关键帧（IDR）：1 是，0 否 → stss */
} mp4_sample;

struct mp4_writer {
    FILE  *file;              /* 正在写的 MP4 文件 */
    td_u32 video_width;       /* 画面宽（像素） */
    td_u32 video_height;      /* 画面高（像素） */

    td_u8  sps_data[NAL_SPS_MAX_SIZE];   /* SPS，不含起始码 */
    td_u32 sps_length;                                      /* SPS 的字节数 */
    td_u8  pps_data[NAL_PPS_MAX_SIZE];   /* PPS，不含起始码 */
    td_u32 pps_length;                                      /* PPS 的字节数 */

    off_t  mdat_header_offset;    /* mdat 盒子头在文件中的位置，结束时回头填长度 */
    td_u64 mdat_payload_bytes;    /* mdat 里已写入的数据字节数（不含 8 字节盒子头） */

    mp4_sample *samples;          /* 动态数组：每一帧一项 */
    td_u32 sample_count;          /* 已经记录的帧数 */
    td_u32 sample_capacity;       /* 数组当前能放多少帧 */

    td_u64 first_pts_us;          /* 第一帧的时间戳（微秒），用来把时间换算成"从 0 开始" */
};

/* ====================================================================
 * 一、字节缓冲区：在内存里拼 moov，所有数字按"大端序"写入
 * ==================================================================== */
typedef struct {
    td_u8 *data;       /* 指向堆上的内存 */
    size_t length;     /* 已经写入的字节数，也是下一个字节要写的位置 */
    size_t capacity;   /* 这块内存总共能放多少字节 */
    int    has_error;  /* 内存不足时置 1，之后的写入都不再进行，最后统一检查 */
} byte_buffer;

/* 在末尾追加 count 个字节；source 为 NULL 表示追加 count 个 0 */
static void buffer_put_bytes(byte_buffer *buffer, const void *source, size_t count)
{
    if (buffer->has_error) {
        return;
    }
    /*
     * 剩余空间装不下这 count 个字节，就要扩容。
     * 内存是向系统一次次申请的，先估一个新容量，再用 realloc 换一块更大的。
     */
    if (buffer->length + count > buffer->capacity) {

        /*
         * 新容量的起点：第一次（capacity 为 0）给 1024 字节；以后从当前容量开始。
         * 再配合下面的 while 不断翻倍，直到装得下为止。
         * 成倍增长而不是"缺多少补多少"，是为了减少 realloc 的次数（每次 realloc 都可能把整块数据搬一遍）。
         */
        size_t new_capacity ;
        if(buffer->capacity == 0) {
            new_capacity = 1024;
        } else {
            new_capacity = buffer->capacity;
        }
        td_u8 *new_data;

        while (new_capacity < buffer->length + count) {
            new_capacity *= 2;
        }
        /*
         * realloc 可能原地扩大，也可能搬到新地址并释放旧的。
         * 返回值先放进临时变量 new_data：失败会返回 NULL，如果直接赋给 buffer->data，旧指针就丢了（泄漏）。
         * 注意 buffer 里的 data 指针因此可能变化，所以别处不能长期保存指向 data 内部的指针（见 buffer_box_end）。
         */
        new_data = (td_u8 *)realloc(buffer->data, new_capacity);
        if (new_data == NULL) {
            buffer->has_error = 1;                          /* 记下“内存不足”，之后的写入都会被跳过 */
            return;
        }
        buffer->data = new_data;                            /* 申请成功：换用新的内存块 */
        buffer->capacity = new_capacity;                    /* 更新记录的总容量 */
    }

    /*
     * source 有值：把数据拷贝到缓冲区末尾；
     * source 为 NULL：我们约定它表示"写 count 个 0"，用 memset 填 0（不需要源数据）。
     * 最后 length 前进 count，下一次就接着写。
     */
    if (source != NULL) {
        memcpy(buffer->data + buffer->length, source, count);
    } else {
        memset(buffer->data + buffer->length, 0, count);
    }
    buffer->length += count;                                /* 已写字节数增加 count，下次从这里接着写 */
}

/* 按大端序（高字节在前）写 1 / 2 / 4 字节整数 */
static void buffer_put_u8(byte_buffer *buffer, td_u32 value)
{
    /*
     * 把 32 位的 value 强制转成 8 位，只保留最低的 1 个字节（高位被丢掉）。
     * 调用者要保证 value 不超过 255，或者本来就只想要低 8 位。
     */
    td_u8 byte = (td_u8)value;                              /* 强制转成 8 位，只留最低的 1 个字节 */
    buffer_put_bytes(buffer, &byte, 1);                     /* 把这 1 个字节追加到缓冲区末尾 */
}

static void buffer_put_u16(byte_buffer *buffer, td_u32 value)
{
    /*
     * 大端序：高字节在前。例如 value = 0x0102，先写 0x01，再写 0x02，缓冲区里就是 01 02。
     * value >> 8 把高 8 位挪到低 8 位上，再由 buffer_put_u8 截成 1 个字节。
     */
    buffer_put_u8(buffer, value >> 8);                      /* 先写高字节 */
    buffer_put_u8(buffer, value);                           /* 再写低字节 */
}

static void buffer_put_u32(byte_buffer *buffer, td_u32 value)
{
    /*
     * 同样的拆法：先写高 16 位，再写低 16 位。例如 0x00000200 → 00 00 02 00。
     * （第二次调用 buffer_put_u16 时传入的仍是完整的 value，多余的高位最终会在 buffer_put_u8 的截断里被丢掉，只剩低 16 位。）
     */
    buffer_put_u16(buffer, value >> 16);                    /* 先写高 16 位 */
    buffer_put_u16(buffer, value);                          /* 再写低 16 位 */
}

/* 写 count 个 0（用于保留字段） */
static void buffer_put_zeros(byte_buffer *buffer, size_t count)
{
    buffer_put_bytes(buffer, NULL, count);                  /* source 传 NULL 表示写 count 个 0 */
}

/* 写 4 个字符的名字，如 "moov"、"vide" */
static void buffer_put_fourcc(byte_buffer *buffer, const char *four_chars)
{
    buffer_put_bytes(buffer, four_chars, 4);                /* 把 4 个字符原样拷进缓冲区 */
}

/**
 * 开始一个新的盒子（box），并返回该盒子的起始位置,前四个字节先写 0 占位，结束时回头填长度，后四个自己写入盒子类型
 * @param buffer 字节缓冲区
 * @param box_type 盒子类型
 * @return 盒子中的起始位置
 */
static size_t buffer_box_begin(byte_buffer *buffer, const char *box_type)
{
    size_t box_start = buffer->length;                      /* 记下盒子起点：此刻的 length 就是盒子第一个字节的位置 */

    buffer_put_u32(buffer, 0);  /* 先写 0 占位，结束时回头填长度 */
    buffer_put_bytes(buffer, box_type, 4);                  /* 写 4 字节盒子类型，例如 moov */
    return box_start;                                       /* 把起点交给调用者，buffer_box_end 回头填长度时要用 */
}

/*
 * 结束一个盒子：现在才知道盒子总长度（当前长度 - 起点），回头把它填进去。
 * 注意用"下标"访问而不是保存指针：realloc 可能搬家，指针会失效。
 */
static void buffer_box_end(byte_buffer *buffer, size_t box_start)
{
    td_u32 box_size;

    if (buffer->has_error) {
        return;
    }
    box_size = (td_u32)(buffer->length - box_start);        /* 盒子总长度 = 现在的末尾位置 - 盒子起点 */
    /* 填写盒子长度 ，大端序要求高字节在前。拆法是"右移 + 截断"*/
    buffer->data[box_start]     = (td_u8)(box_size >> 24);  /* 长度的最高字节 */
    buffer->data[box_start + 1] = (td_u8)(box_size >> 16);  /* 次高字节 */
    buffer->data[box_start + 2] = (td_u8)(box_size >> 8);   /* 次低字节 */
    buffer->data[box_start + 3] = (td_u8)box_size;          /* 最低字节 */
}

/* "完整盒子"比普通盒子多 1 字节版本号 + 3 字节标志位 */
static size_t buffer_full_box_begin(byte_buffer *buffer, const char *box_type,
                                    td_u32 version, td_u32 flags)
{
    size_t box_start = buffer_box_begin(buffer, box_type);  /* 先按普通盒子写好“长度占位 + 类型” */

    buffer_put_u8(buffer, version);                         /* 1 字节版本号 */
    buffer_put_u8(buffer, flags >> 16);                     /* 标志位共 3 字节：先写最高的那个字节 */
    buffer_put_u16(buffer, flags);                          /* 再写低 2 个字节 */
    return box_start;                                       /* 返回盒子起点，供 buffer_box_end 使用 */
}

/* ====================================================================
 * 二、构造 moov（目录）
 * ==================================================================== */

/* 3x3 变换矩阵，这里用"单位矩阵"（不旋转、不缩放） */
static void buffer_put_identity_matrix(byte_buffer *buffer)
{
    /*
     * 矩阵一共 9 个数，每个 4 字节，按行排列：
     *     1 0 0
     *     0 1 0
     *     0 0 1
     * 其中前两列的数是"16.16 定点数"：高 16 位是整数部分，低 16 位是小数部分，
     * 所以 0x00010000 表示 1.0；最后一个数是"2.30 定点数"，1.0 写成 0x40000000。
     * 单位矩阵 = 不旋转、不缩放、不平移。
     */
    buffer_put_u32(buffer, 0x00010000); buffer_put_u32(buffer, 0); buffer_put_u32(buffer, 0);
    buffer_put_u32(buffer, 0); buffer_put_u32(buffer, 0x00010000); buffer_put_u32(buffer, 0);
    buffer_put_u32(buffer, 0); buffer_put_u32(buffer, 0); buffer_put_u32(buffer, 0x40000000);
}

/*
 * 第 sample_index 帧要显示多久：下一帧的时间 减 本帧的时间。
 * 最后一帧没有"下一帧"，沿用上一帧的时长。
 */
static td_u32 sample_duration(const mp4_writer *writer, td_u32 sample_index)
{
    /*
     * 情况 1：有下一帧（大多数帧）。
     * 时长 = 下一帧的显示时间 - 本帧的显示时间。
     * 例：30fps 时每帧相差 3000 个刻度（90000 / 30），也就是每帧显示 1/30 秒。
     */
    if (sample_index + 1 < writer->sample_count) {
        return writer->samples[sample_index + 1].timestamp_ticks
             - writer->samples[sample_index].timestamp_ticks;
    }
    /*
     * 情况 2：这是最后一帧（没有下一帧可以比），就沿用上一帧的时长，假设帧率没有突变。
     */
    if (sample_index > 0) {
        return writer->samples[sample_index].timestamp_ticks
             - writer->samples[sample_index - 1].timestamp_ticks;
    }
    return MP4_VIDEO_TIMESCALE / 30;    /* 只有 1 帧时，随便给一个 30fps 的时长 */
}

/* avc1：视频样本描述，里面嵌套 avcC（H.264 解码器配置，含 SPS/PPS） */
static void build_avc1_box(byte_buffer *buffer, const mp4_writer *writer)
{
    size_t avc1_start = buffer_box_begin(buffer, "avc1");   /* 开始 avc1 盒子，记下起点 */
    size_t avcc_start;                                      /* avcC 盒子的起点（后面才赋值） */

    buffer_put_zeros(buffer, 6);                 /* 保留 */
    buffer_put_u16(buffer, 1);                   /* 数据引用序号 */
    buffer_put_zeros(buffer, 16);                /* 预定义 + 保留 */
    buffer_put_u16(buffer, writer->video_width);
    buffer_put_u16(buffer, writer->video_height);
    buffer_put_u32(buffer, 0x00480000);          /* 水平分辨率 72dpi */
    buffer_put_u32(buffer, 0x00480000);          /* 垂直分辨率 72dpi */
    buffer_put_u32(buffer, 0);                   /* 保留 */
    buffer_put_u16(buffer, 1);                   /* 每个样本含 1 帧 */
    buffer_put_zeros(buffer, 32);                /* 压缩器名字，留空 */
    buffer_put_u16(buffer, 0x0018);              /* 颜色深度 24 */
    buffer_put_u16(buffer, 0xFFFF);              /* 预定义 = -1 */

    /* avcC：播放器靠它初始化 H.264 解码器 */
    avcc_start = buffer_box_begin(buffer, "avcC");
    buffer_put_u8(buffer, 1);                    /* 配置版本号 */
    buffer_put_u8(buffer, writer->sps_data[1]);  /* profile（SPS 的第 2 个字节） */
    buffer_put_u8(buffer, writer->sps_data[2]);  /* profile 兼容标志（SPS 的第 3 个字节） */
    buffer_put_u8(buffer, writer->sps_data[3]);  /* level（SPS 的第 4 个字节） */
    buffer_put_u8(buffer, 0xFF);                 /* 高 6 位固定为 1，低 2 位 = 3：表示 NAL 长度前缀占 3+1 = 4 字节 */
    buffer_put_u8(buffer, 0xE1);                 /* 高 3 位固定为 1，低 5 位 = 1：表示有 1 个 SPS */
    buffer_put_u16(buffer, writer->sps_length);             /* SPS 的长度（2 字节） */
    buffer_put_bytes(buffer, writer->sps_data, writer->sps_length);  /* SPS 内容原样拷入 */
    buffer_put_u8(buffer, 1);                    /* 有 1 个 PPS */
    buffer_put_u16(buffer, writer->pps_length);             /* PPS 的长度（2 字节） */
    buffer_put_bytes(buffer, writer->pps_data, writer->pps_length);  /* PPS 内容原样拷入 */
    buffer_box_end(buffer, avcc_start);                     /* avcC 结束：回填它的长度 */

    buffer_box_end(buffer, avc1_start);                     /* avc1 结束：回填它的长度 */
}

/* stbl（样本表）：整个目录最核心的部分，告诉播放器每一帧的大小、时间、位置、是否关键帧 */
static void build_sample_table_box(byte_buffer *buffer, const mp4_writer *writer,
                                   td_u32 first_sample_file_offset)
{
    size_t sample_table_start = buffer_box_begin(buffer, "stbl");  /* 开始 stbl 盒子，记下起点 */
    size_t box_start;                                       /* 各个子盒子的起点（反复复用这个变量） */
    td_u32 sample_index;                                    /* 循环用的帧下标，从 0 开始 */
    td_u32 duration_run_count;                              /* 时长一共分成几段（stts 的项数） */
    td_u32 key_frame_count;                                 /* 关键帧的个数（stss 的项数） */

    /*
     * stbl 里依次写 6 张表：stsd、stts、stss、stsc、stsz、stco。
     * 它们都是"完整盒子"（带版本号和标志位），写法一样：
     *     box_start = buffer_full_box_begin(...);   → 写内容 →   buffer_box_end(..., box_start);
     */
    /* stsd（样本描述）：编码格式和参数 */
    box_start = buffer_full_box_begin(buffer, "stsd", 0, 0);  /* 开始 stsd 盒子，记下起点 */
    buffer_put_u32(buffer, 1);                   /* 描述项数量：1 */
    build_avc1_box(buffer, writer);                         /* 写入 avc1 子盒子（编码参数） */
    buffer_box_end(buffer, box_start);                      /* stsd 结束：回填长度 */

    /* stts（时长表）：连续相同的时长合并成一项 (连续多少帧, 每帧时长) */
    /*
     * 先扫第 1 遍：数一数时长一共分成几段（连续相同的算一段）。
     * 为什么要扫两遍？因为 stts 盒子开头要先写"一共几项"，而项数要把所有帧看完才知道。
     */
    duration_run_count = 0;                                 /* 段数计数器清零 */
    for (sample_index = 0; sample_index < writer->sample_count; sample_index++) {
        td_u32 current_duration;    /* 这一帧的时长 */
        td_u32 previous_duration;   /* 上一帧的时长 */

        /* 第 0 帧前面没有帧，它一定是第 1 段的开头，直接计数 */
        if (sample_index == 0) {
            duration_run_count++;                           /* 段数加 1 */
            continue;                                       /* 跳过下面的比较，直接处理下一帧 */
        }
        /* 先把两帧的时长分别算出来存进变量，再比较；和上一帧不同，说明从这一帧开始了新的一段 */
        current_duration = sample_duration(writer, sample_index);  /* 这一帧显示多久 */
        previous_duration = sample_duration(writer, sample_index - 1);  /* 上一帧显示多久 */
        if (current_duration != previous_duration) {
            duration_run_count++;                           /* 时长变了：开始新的一段，段数加 1 */
        }
    }
    box_start = buffer_full_box_begin(buffer, "stts", 0, 0);  /* 开始 stts 盒子，记下起点 */
    buffer_put_u32(buffer, duration_run_count);             /* 先写项数（上一遍数出来的） */
    /*
     * 再扫第 2 遍：每一段写成 (连续多少帧, 每帧时长) 一项，内层 while 数出这一段有多长。
     * 注意这个 for 的第三部分是空的，sample_index 是在循环体最后一行 "+= same_duration_count" 才前进的。
     * 例：30fps 时每帧约 3000 刻度，262 帧理想情况下只要写 1 项 (262, 3000)；
     * 实际微秒换算有舍入，会出现少量 2999 与 3000 交替，项数多一些，但仍远少于帧数。
     */
    for (sample_index = 0; sample_index < writer->sample_count;) {
        td_u32 duration = sample_duration(writer, sample_index);  /* 这一段里每帧的时长 */
        td_u32 same_duration_count = 1;                     /* 这一段已经有几帧：当前这帧自己算 1 帧 */

        /*
         * 往后数：后一帧的时长和这一段相同，就把它并进这一段。
         * 先把后一帧的时长存进 next_duration 再判断；遇到不同的时长就 break，这一段到此结束。
         */
        while (sample_index + same_duration_count < writer->sample_count) {
            td_u32 next_duration;                           /* 后一帧的时长 */

            next_duration = sample_duration(writer, sample_index + same_duration_count);  /* 取出后一帧的时长 */
            if (next_duration != duration) {
                break;                                      /* 时长不同：这一段到此结束，跳出 while */
            }
            same_duration_count++;                          /* 时长相同：并入这一段，帧数加 1 */
        }
        buffer_put_u32(buffer, same_duration_count);        /* 写：这一段连续多少帧 */
        buffer_put_u32(buffer, duration);                   /* 写：每帧时长 */
        sample_index += same_duration_count;                /* 跳到下一段的第一帧 */
    }
    buffer_box_end(buffer, box_start);                      /* stts 结束：回填长度 */

    /* stss（关键帧表）：关键帧的序号（从 1 开始数）。没有它，播放器会认为每一帧都能跳转 */
    /*
     * 同样先数一遍有几个关键帧，因为 stss 盒子开头要先写项数。
     */
    key_frame_count = 0;                                    /* 关键帧计数器清零 */
    for (sample_index = 0; sample_index < writer->sample_count; sample_index++) {
        /* 是关键帧就计数加 1 */
        if (writer->samples[sample_index].is_key_frame) {
            key_frame_count++;                              /* 关键帧个数加 1 */
        }
    }
    box_start = buffer_full_box_begin(buffer, "stss", 0, 0);  /* 开始 stss 盒子，记下起点 */
    buffer_put_u32(buffer, key_frame_count);                /* 先写关键帧个数 */
    for (sample_index = 0; sample_index < writer->sample_count; sample_index++) {
        if (writer->samples[sample_index].is_key_frame) {
            buffer_put_u32(buffer, sample_index + 1);       /* 写关键帧的序号：MP4 从 1 开始数，所以下标要 +1 */
        }
    }
    buffer_box_end(buffer, box_start);                      /* stss 结束：回填长度 */

    /* stsc（样本到块的映射）：我们只有 1 个块，包含全部样本 */
    box_start = buffer_full_box_begin(buffer, "stsc", 0, 0);  /* 开始 stsc 盒子，记下起点 */
    buffer_put_u32(buffer, 1);                   /* 映射项数量：1 */
    buffer_put_u32(buffer, 1);                   /* 从第 1 个块开始 */
    buffer_put_u32(buffer, writer->sample_count);/* 每个块含有的样本数 */
    buffer_put_u32(buffer, 1);                   /* 使用第 1 个样本描述 */
    buffer_box_end(buffer, box_start);                      /* stsc 结束：回填长度 */

    /* stsz（样本大小表）：第一个字段为 0 表示"大小各不相同，下面逐个列出" */
    box_start = buffer_full_box_begin(buffer, "stsz", 0, 0);  /* 开始 stsz 盒子，记下起点 */
    buffer_put_u32(buffer, 0);                              /* 0 表示各帧大小不同，下面逐个列出 */
    buffer_put_u32(buffer, writer->sample_count);           /* 一共多少帧 */
    for (sample_index = 0; sample_index < writer->sample_count; sample_index++) {
        buffer_put_u32(buffer, writer->samples[sample_index].size_bytes);  /* 第 sample_index 帧的大小（字节） */
    }
    buffer_box_end(buffer, box_start);                      /* stsz 结束：回填长度 */

    /* stco（块偏移表）：只有 1 个块，位置就是 mdat 数据区的起点（第一帧开始处） */
    box_start = buffer_full_box_begin(buffer, "stco", 0, 0);  /* 开始 stco 盒子，记下起点 */
    buffer_put_u32(buffer, 1);                              /* 偏移表只有 1 项（只有 1 个块） */
    buffer_put_u32(buffer, first_sample_file_offset);       /* 这个块在文件里的起始位置（= 第一帧的位置） */
    buffer_box_end(buffer, box_start);                      /* stco 结束：回填长度 */

    buffer_box_end(buffer, sample_table_start);             /* stbl 结束：回填长度 */
}

/* moov：整个目录。盒子嵌套关系见下面各层的 begin/end 配对 */
static void build_moov_box(byte_buffer *buffer, const mp4_writer *writer,
                           td_u32 first_sample_file_offset)
{
    /*
     * 最后一帧的下标。调用者必须保证 sample_count > 0（mp4_close 里已检查），
     * 否则 0 - 1 会变成一个巨大的无符号数，下面访问 samples[last_index] 就越界了。
     */
    td_u32 last_index = writer->sample_count - 1;           /* 最后一帧的下标 = 总帧数 - 1 */
    /* 总时长 = 最后一帧的显示时间 + 最后一帧的时长（单位 1/90000 秒） */
    td_u64 total_duration_ticks = (td_u64)writer->samples[last_index].timestamp_ticks  /* 总时长（刻度）= 最后一帧的开始时间 + 它自己的时长 */
                                + sample_duration(writer, last_index);
    /*
     * 同一段总时长，用两种单位表示：
     *     movie_duration：毫秒（时间刻度 1000），写进 mvhd 和 tkhd
     *     media_duration：1/90000 秒（时间刻度 90000），写进 mdhd
     * 例：8.733 秒 → movie_duration = 8733，media_duration = 786000。
     */
    td_u32 movie_duration = (td_u32)(total_duration_ticks * MP4_MOVIE_TIMESCALE / MP4_VIDEO_TIMESCALE);  /* 换算成毫秒：刻度数 × 1000 ÷ 90000 */
    td_u32 media_duration = (td_u32)total_duration_ticks;   /* 直接用刻度数（1/90000 秒） */

    size_t moov_start = buffer_box_begin(buffer, "moov");   /* 开始 moov 盒子，记下起点 */
    size_t trak_start;                                      /* trak（轨道）盒子的起点 */
    size_t mdia_start;                                      /* mdia（媒体）盒子的起点 */
    size_t minf_start;                                      /* minf（媒体信息）盒子的起点 */
    size_t dinf_start;                                      /* dinf（数据信息）盒子的起点 */
    size_t dref_start;                                      /* dref（数据引用）盒子的起点 */
    size_t url_start;                                       /* url 盒子的起点 */
    size_t box_start;                                       /* 各个小盒子的起点（反复复用这个变量） */

    /* mvhd（影片头）：整部影片的信息 */
    box_start = buffer_full_box_begin(buffer, "mvhd", 0, 0);  /* 开始 mvhd 盒子，记下起点 */
        buffer_put_u32(buffer, 0);                   /* 创建时间（不重要，填 0） */
        buffer_put_u32(buffer, 0);                   /* 修改时间 */
        buffer_put_u32(buffer, MP4_MOVIE_TIMESCALE); /* 时间刻度 */
        buffer_put_u32(buffer, movie_duration);      /* 总时长 */
        buffer_put_u32(buffer, 0x00010000);          /* 播放速度 1.0 */
        buffer_put_u16(buffer, 0x0100);              /* 音量 1.0 */
        buffer_put_zeros(buffer, 10);                /* 保留 */
        buffer_put_identity_matrix(buffer);
        buffer_put_zeros(buffer, 24);                /* 预定义 */
        buffer_put_u32(buffer, 2);                   /* 下一个可用的轨道编号 */
    buffer_box_end(buffer, box_start);                      /* mvhd 结束：回填长度 */

    trak_start = buffer_box_begin(buffer, "trak");          /* 开始 trak 盒子；它里面还嵌套别的盒子，所以先不结束 */

    /* tkhd（轨道头）：flags = 3 表示"已启用 + 属于影片" */
    box_start = buffer_full_box_begin(buffer, "tkhd", 0, 3);  /* 开始 tkhd 盒子，记下起点 */
    buffer_put_u32(buffer, 0);                   /* 创建时间 */
    buffer_put_u32(buffer, 0);                   /* 修改时间 */
    buffer_put_u32(buffer, 1);                   /* 轨道编号 */
    buffer_put_u32(buffer, 0);                   /* 保留 */
    buffer_put_u32(buffer, movie_duration);      /* 轨道时长 */
    buffer_put_zeros(buffer, 8);                 /* 保留 */
    buffer_put_u16(buffer, 0);                   /* 图层 */
    buffer_put_u16(buffer, 0);                   /* 备选组 */
    buffer_put_u16(buffer, 0);                   /* 音量（视频为 0） */
    buffer_put_u16(buffer, 0);                   /* 保留 */
    buffer_put_identity_matrix(buffer);
    buffer_put_u32(buffer, writer->video_width << 16);   /* 宽，16.16 定点数 */
    buffer_put_u32(buffer, writer->video_height << 16);  /* 高，16.16 定点数 */
    buffer_box_end(buffer, box_start);                      /* tkhd 结束：回填长度 */

    mdia_start = buffer_box_begin(buffer, "mdia");          /* 开始 mdia 盒子（里面嵌套 mdhd / hdlr / minf） */

    /* mdhd（媒体头）：这条轨道的时间刻度和时长 */
    box_start = buffer_full_box_begin(buffer, "mdhd", 0, 0);  /* 开始 mdhd 盒子，记下起点 */
    buffer_put_u32(buffer, 0);                   /* 创建时间 */
    buffer_put_u32(buffer, 0);                   /* 修改时间 */
    buffer_put_u32(buffer, MP4_VIDEO_TIMESCALE); /* 时间刻度 */
    buffer_put_u32(buffer, media_duration);      /* 时长（用视频刻度） */
    buffer_put_u16(buffer, 0x55C4);              /* 语言 "und"（未指定） */
    buffer_put_u16(buffer, 0);                   /* 预定义 */
    buffer_box_end(buffer, box_start);                      /* mdhd 结束：回填长度 */

    /* hdlr（处理器声明）：声明这是视频轨道 */
    box_start = buffer_full_box_begin(buffer, "hdlr", 0, 0);  /* 开始 hdlr 盒子，记下起点 */
    buffer_put_u32(buffer, 0);                   /* 预定义 */
    buffer_put_fourcc(buffer, "vide");           /* 轨道类型：视频 */
    buffer_put_zeros(buffer, 12);                /* 保留 */
    buffer_put_bytes(buffer, "VideoHandler", 13);/* 名字，13 字节含结尾的 '\0' */
    buffer_box_end(buffer, box_start);                      /* hdlr 结束：回填长度 */

    minf_start = buffer_box_begin(buffer, "minf");          /* 开始 minf 盒子（里面嵌套 vmhd / dinf / stbl） */

    /* vmhd（视频媒体头） */
    box_start = buffer_full_box_begin(buffer, "vmhd", 0, 1);  /* 开始 vmhd 盒子，记下起点 */
    buffer_put_zeros(buffer, 8);                 /* 图形模式 + 颜色，都填 0 */
    buffer_box_end(buffer, box_start);                      /* vmhd 结束：回填长度 */

    /* dinf / dref / url：数据在哪里？flags = 1 表示"就在本文件里" */
    dinf_start = buffer_box_begin(buffer, "dinf");          /* 开始 dinf 盒子 */
    dref_start = buffer_full_box_begin(buffer, "dref", 0, 0);  /* 开始 dref 盒子（在 dinf 里面） */
    buffer_put_u32(buffer, 1);                   /* 数据引用数量：1 */
    url_start = buffer_full_box_begin(buffer, "url ", 0, 1);  /* 开始 url 盒子（在 dref 里面） */
    buffer_box_end(buffer, url_start);                      /* url 结束（它没有内容，长度就是 12） */
    buffer_box_end(buffer, dref_start);                     /* dref 结束 */
    buffer_box_end(buffer, dinf_start);                     /* dinf 结束 */

    build_sample_table_box(buffer, writer, first_sample_file_offset);  /* 写入 stbl（样本表），它是目录里最核心的部分 */

    buffer_box_end(buffer, minf_start);                     /* minf 结束（对应 534 行的 begin） */
    buffer_box_end(buffer, mdia_start);                     /* mdia 结束（对应 514 行的 begin） */
    buffer_box_end(buffer, trak_start);                     /* trak 结束（对应 495 行的 begin） */
    buffer_box_end(buffer, moov_start);                     /* moov 结束（对应 472 行的 begin），整个目录拼完 */
}

/* ====================================================================
 * 三、对外接口
 * ==================================================================== */

/* 直接往文件写 4 字节大端整数 */
static int write_u32_big_endian(FILE *file, td_u32 value)
{
    td_u8 bytes[4] = { (td_u8)(value >> 24), (td_u8)(value >> 16),  /* 把 32 位整数拆成 4 个字节，高字节在前 */
                       (td_u8)(value >> 8),  (td_u8)value };
    size_t write_ret;   /* fwrite 的返回值：实际写入了多少个元素（这里每个元素 1 字节） */

    write_ret = fwrite(bytes, 1, 4, file);                  /* 写 4 个字节，返回值是实际写进去几个 */
    if (write_ret != 4) {
        return -1;      /* 没写满 4 字节，说明写盘出错（磁盘满、SD 卡拔出等） */
    }
    return 0;                                               /* 写成功 */
}

/*
 * 【mp4_open】创建 MP4 文件，写好 ftyp 和 mdat 的盒子头，之后就可以一帧一帧写数据了。
 *
 * 参数：
 *   path          要创建的文件路径，例如 "rec.mp4"
 *   width/height  画面大小（像素），写进 moov 里，播放器据此确定画面尺寸
 *   sps/pps       不含起始码的 SPS、PPS（第 1 个字节是 NAL 头，如 0x67/0x68）。
 *                 MP4 里它们只放在 avcC 盒子中，所以这里先保存起来，到 mp4_close 才用
 * 返回：成功返回句柄（以后传给 mp4_write_frame / mp4_close），失败返回 NULL
 *
 * 执行完后文件内容是：[ftyp 32字节][mdat 盒子头 8字节，长度暂时是 0]
 */
mp4_writer *mp4_open(const char *path, td_u32 width, td_u32 height,
                     const td_u8 *sps, td_u32 sps_len,
                     const td_u8 *pps, td_u32 pps_len)
{
    mp4_writer *writer;
    byte_buffer header_buffer = {0};                        /* 内存缓冲区，先在里面拼好 ftyp；{0} 表示所有成员初始为 0 */
    size_t ftyp_start;                                      /* ftyp 盒子的起点 */
    size_t write_ret;   /* fwrite 的返回值：实际写入的字节数 */
    td_s32 ret;         /* 其他函数的返回值：0 / TD_SUCCESS 表示成功 */

    /*
     * 参数检查。sps_len 必须 >= 4，是因为 avcC 里要读 sps_data[1]、[2]、[3]（profile / 兼容标志 / level）；
     * 长度也不能超过结构体里数组的大小，否则下面的 memcpy 会写出界。
     */
    if (path == NULL || sps == NULL || pps == NULL ||
        sps_len < 4 || sps_len > NAL_SPS_MAX_SIZE || pps_len == 0 || pps_len > NAL_PPS_MAX_SIZE) {
        printf("mp4: invalid argument\n");
        return NULL;
    }

    /*
     * calloc 会把分配的内存全部清零：指针成员（samples）是 NULL，计数（sample_count 等）是 0。
     * 这样后面任何一步出错需要 free 时都是安全的。
     */
    writer = (mp4_writer *)calloc(1, sizeof(*writer));
    if (writer == NULL) {
        return NULL;
    }
    writer->video_width = width;                            /* 保存画面宽度（像素），写 moov 时要用 */
    writer->video_height = height;                          /* 保存画面高度（像素），写 moov 时要用 */
    memcpy(writer->sps_data, sps, sps_len);                 /* 把 SPS 内容拷进 writer 自己的数组（调用者的数据以后可能被覆盖） */
    writer->sps_length = sps_len;                           /* 记下 SPS 有多长 */
    memcpy(writer->pps_data, pps, pps_len);                 /* 把 PPS 内容拷进 writer 自己的数组 */
    writer->pps_length = pps_len;                           /* 记下 PPS 有多长 */

    /*
     * "wb" = 以二进制方式写（会清空已有文件）。MP4 是二进制文件，
     * 在 Windows 上如果用文本模式，会把 0x0A 悄悄换成 0x0D 0x0A，文件就坏了。
     */
    writer->file = fopen(path, "wb");
    if (writer->file == NULL) {
        perror("mp4: fopen");
        free(writer);                                       /* 打开文件失败：释放 writer，避免泄漏 */
        return NULL;
    }
    
    /* ftyp（文件类型盒子）：声明这个文件的格式，以及兼容哪些格式 */
    ftyp_start = buffer_box_begin(&header_buffer, "ftyp");  /* 开始 ftyp 盒子，记下起点 */
        buffer_put_fourcc(&header_buffer, "isom");   /* 主品牌 */
        buffer_put_u32(&header_buffer, 512);         /* 版本号 */
        buffer_put_fourcc(&header_buffer, "isom");   /* 以下是兼容品牌 */
        buffer_put_fourcc(&header_buffer, "iso2");
        buffer_put_fourcc(&header_buffer, "avc1");
        buffer_put_fourcc(&header_buffer, "mp41");
    buffer_box_end(&header_buffer, ftyp_start);             /* ftyp 结束：回填长度 */
    /*
     * 检查 ftyp 有没有写成功，分两步、各自判断：
     *   1. has_error：在内存里拼 ftyp 时有没有发生内存不足；
     *   2. write_ret：把拼好的 ftyp 写进文件，实际写入的字节数不等于长度就是写盘失败。
     * 出错路径要把已经申请的资源逐个释放：缓冲区、文件、writer 本身，少一个就是泄漏。
     */
    if (header_buffer.has_error) {
        printf("mp4: build ftyp failed (out of memory)\n");
        free(header_buffer.data);                           /* 释放拼 ftyp 用的缓冲区 */
        fclose(writer->file);                               /* 关闭文件 */
        free(writer);                                       /* 释放 writer 结构体 */
        return NULL;
    }
    write_ret = fwrite(header_buffer.data, 1, header_buffer.length, writer->file);  /* 把拼好的 ftyp 写进文件，write_ret 是实际写入的字节数 */
    if (write_ret != header_buffer.length) {
        printf("mp4: write ftyp failed\n");
        free(header_buffer.data);                           /* 释放拼 ftyp 用的缓冲区 */
        fclose(writer->file);                               /* 关闭文件 */
        free(writer);                                       /* 释放 writer 结构体 */
        return NULL;
    }
    free(header_buffer.data);                               /* ftyp 已写完，缓冲区不再需要，释放 */

    /*
     * mdat 的盒子头：[4字节长度][4字节 "mdat"]。
     * 总长度要等所有帧写完才知道，所以先写 0 占位，并用 ftello 记下这个位置（此刻 = ftyp 之后 = 32），
     * mp4_close 时再 fseek 回来填真正的长度。
     */
    writer->mdat_header_offset = ftello(writer->file);      /* 记下当前文件位置（= mdat 盒子头的位置），mp4_close 要回来填长度 */

    /*
     * 写入 mdat 盒子头，分两步、各自判断：
     *   1. 4 字节长度，先写 0，mp4_close 时更新；
     *   2. 4 字节类型 "mdat"。
     * 这时 header_buffer 已经释放了，出错时只需要关文件、释放 writer。
     */
    ret = write_u32_big_endian(writer->file, 0);            /* 写 4 字节长度，先占位写 0 */
    if (ret != 0) {
        printf("mp4: write mdat size failed\n");
        fclose(writer->file);                               /* 关闭文件 */
        free(writer);                                       /* 释放 writer 结构体 */
        return NULL;
    }
    write_ret = fwrite("mdat", 1, 4, writer->file);         /* 写 4 字节盒子类型 mdat */
    if (write_ret != 4) {
        printf("mp4: write mdat type failed\n");
        fclose(writer->file);                               /* 关闭文件 */
        free(writer);                                       /* 释放 writer 结构体 */
        return NULL;
    }
    return writer;                                          /* 成功：把句柄交给调用者 */
}                 

/*
 * 【mp4_write_frame】写入一帧。
 *
 * 参数：
 *   frame_data / frame_length  编码器给的一整帧，带起始码，可能含 SPS/PPS + IDR，或只有一个 P 切片
 *   pts_us                     这一帧的时间戳，单位微秒（编码器给的绝对时间即可，内部会换算成从 0 开始）
 * 返回：TD_SUCCESS 成功；TD_FAILURE 表示这一帧没有写入
 *       （参数错误 / 没有图像数据 / 第一帧不是关键帧 / 写盘失败）。
 *       失败时调用者可以丢掉这帧，继续送下一帧。
 *
 * 做的事分三步：
 *   1. 扫一遍，判断这帧有没有图像数据、是不是关键帧
 *   2. 把每个 NAL 改写成 [4字节长度][NAL本体] 写进 mdat
 *   3. 在 samples 数组里记一笔（大小、时间、是否关键帧），留给 mp4_close 生成 moov
 *
 * 已知限制：如果第 2 步写到一半磁盘出错，已经写进文件的那几个 NAL 不会计入大小，
 * 后面帧的位置会整体错位。目前直接放弃整个文件即可，B2 阶段再做更稳妥的处理。
 */
td_s32 mp4_write_frame(mp4_writer *writer, const td_u8 *frame_data, td_u32 frame_length, td_u64 pts_us)
{
    nal_frame_info nal_list;                                /* 存放拆出来的 NAL 列表 */
    td_u32 nal_index;                                       /* 遍历 NAL 列表用的下标 */
    td_u32 frame_size_in_file = 0;      /* 这一帧写进 mdat 后一共占多少字节 */
    td_u8 is_key_frame = 0;                                 /* 这一帧是不是关键帧：先假设不是（0） */
    td_u8 has_picture_slice = 0;                            /* 这一帧有没有图像数据：先假设没有（0） */
    td_u64 timestamp_ticks;                                 /* 这一帧的显示时间（单位 1/90000 秒） */
    mp4_sample *sample;                                     /* 指向 samples 数组里这一帧的“账本”项 */
    td_s32 ret;         /* 函数返回值：0 / TD_SUCCESS 表示成功 */
    size_t write_ret;   /* fwrite 的返回值：实际写入的字节数 */

    if (writer == NULL || frame_data == NULL || frame_length == 0) {
        return TD_FAILURE;
    }
    /*
     * 把整帧拆成 NAL 列表。nal_list 里只记录每个 NAL 的"偏移 / 长度 / 类型"，并不拷贝数据，
     * 真正的数据还在 frame_data 里，后面按偏移去取。
     */
    ret = nal_parse_frame(frame_data, frame_length, &nal_list);
    if (ret != TD_SUCCESS) {
        return TD_FAILURE;
    }

    /* 先扫一遍：这一帧有没有图像切片（类型 1~5），是不是关键帧（类型 5） */
    for (nal_index = 0; nal_index < nal_list.count; nal_index++) {
        td_u8 nal_type = nal_list.nals[nal_index].type;     /* 这个 NAL 的类型编号 */

        if (nal_type == NAL_TYPE_SLICE_IDR) {
            is_key_frame = 1;                               /* 类型 5：这一帧是关键帧 */
        }
        if (nal_type >= NAL_TYPE_SLICE_FIRST && nal_type <= NAL_TYPE_SLICE_IDR) {
            has_picture_slice = 1;                          /* 类型 1~5 都是图像数据 */
        }
    }
    /*
     * 一帧里没有图像数据（比如只有 SPS/PPS）就不算一帧，不能记录。
     */
    if (!has_picture_slice) {
        return TD_FAILURE;
    }
    /* 文件必须从关键帧开始，否则开头的 P 帧没有参考帧，无法解码 */
    if (writer->sample_count == 0 && !is_key_frame) {
        return TD_FAILURE;
    }

    /*
     * 再写一遍：把每个 NAL 改成"4 字节长度 + NAL 本体"（AVCC 格式）。
     * SPS / PPS / AUD 已经放在 avcC 里了，不再写进 mdat。
     */
    for (nal_index = 0; nal_index < nal_list.count; nal_index++) {
        td_u8 nal_type = nal_list.nals[nal_index].type;     /* 这个 NAL 的类型编号 */
        td_u32 nal_length = nal_list.nals[nal_index].len;   /* 这个 NAL 本体的字节数（不含起始码） */

        if (nal_type == NAL_TYPE_SPS || nal_type == NAL_TYPE_PPS || nal_type == NAL_TYPE_AUD) {
            continue;                                       /* 跳过：SPS / PPS / AUD 不写进 mdat */
        }
        /*
         * 写之前先检查：已写的数据 + 这帧已写部分 + 这个 NAL 的 (4 + 长度) 会不会超过上限。
         * stco 里的偏移只有 32 位，文件超过约 4GB 就表示不了了。
         */
        if (writer->mdat_payload_bytes + frame_size_in_file + 4 + nal_length > MP4_MAX_MDAT_BYTES) {
            printf("mp4: file too large\n");
            return TD_FAILURE;
        }
        /* 先写 4 字节长度前缀 */
        ret = write_u32_big_endian(writer->file, nal_length);
        if (ret != 0) {
            printf("mp4: write nal length failed\n");
            return TD_FAILURE;
        }
        /* 再写 NAL 本体：从这一帧里按偏移取出，起始码不写 */
        write_ret = fwrite(frame_data + nal_list.nals[nal_index].offset, 1, nal_length, writer->file);
        if (write_ret != nal_length) {
            printf("mp4: write nal data failed\n");
            return TD_FAILURE;
        }
        /*
         * +4 是每个 NAL 前面那 4 字节长度前缀。它们也占了 mdat 的空间，
         * 而 stsz 里记的正是"这一帧在 mdat 里真正占多少字节"，所以必须算进去。
         */
        frame_size_in_file += 4 + nal_length;               /* 累计这一帧在文件里占的字节数 */
    }

    /*
     * 记录这一帧的信息（大小、时间、是否关键帧），录完后生成 moov 要用。
     * 先确保 samples 数组有空位：满了就把容量翻倍（第一次开 1024 项）。
     * 这里先 realloc 到临时指针 new_samples，成功后才赋给 writer->samples：
     * 如果直接 writer->samples = realloc(...) 而失败返回 NULL，原来的数组指针就丢了，造成内存泄漏。
     */
    if (writer->sample_count == writer->sample_capacity) {
        td_u32 new_capacity;                                /* 新的数组容量（能放多少项） */
        mp4_sample *new_samples;                            /* realloc 的临时返回值 */

        /* 第一次还没有数组（容量为 0）就先开 1024 项，以后每次容量翻倍 */
        if (writer->sample_capacity == 0) {
            new_capacity = 1024;                            /* 第一次：先开 1024 项 */
        } else {
            new_capacity = writer->sample_capacity * 2;     /* 以后每次翻倍 */
        }
        new_samples = (mp4_sample *)realloc(writer->samples, new_capacity * sizeof(*new_samples));  /* 把数组扩大到 new_capacity 项（字节数 = 项数 × 每项大小） */
        if (new_samples == NULL) {
            return TD_FAILURE;                              /* 内存不足：这一帧没记上，放弃 */
        }
        writer->samples = new_samples;                      /* 成功：换用新数组 */
        writer->sample_capacity = new_capacity;             /* 更新容量记录 */
    }
    if (writer->sample_count == 0) {
        writer->first_pts_us = pts_us;                      /* 第一帧：记下它的时间戳，作为时间的 0 点 */
    }
    /* 微秒 → 90kHz 刻度（乘 90000 除 1000000 = 乘 9 除 100）；并保证严格递增，时间戳异常时不让帧时长变成 0 或负数 */
    timestamp_ticks = (pts_us - writer->first_pts_us) * 9 / 100;  /* 相对第一帧的时间，换算成 90kHz 刻度 */
    if (writer->sample_count > 0 &&
        timestamp_ticks <= writer->samples[writer->sample_count - 1].timestamp_ticks) {
        timestamp_ticks = (td_u64)writer->samples[writer->sample_count - 1].timestamp_ticks + 1;  /* 时间没有前进：强行设成“上一帧 + 1” */
    }
    /*
     * 取数组里下一个空位的地址，同时 sample_count 加 1。
     * （count++ 是"先用后加"，所以 sample 指向的是加之前的那个下标。）
     */
    sample = &writer->samples[writer->sample_count++];
    sample->size_bytes = frame_size_in_file;                /* 记：这一帧占多少字节（stsz 用） */
    /*
     * 注意这里转成 32 位：90000 刻度/秒，u32 最多约 13.25 小时，超过就会溢出。
     * 单个文件录得太长要在那之前分段（B2 会做）。
     */
    sample->timestamp_ticks = (td_u32)timestamp_ticks;      /* 记：显示时间（stts 用） */
    sample->is_key_frame = is_key_frame;                    /* 记：是不是关键帧（stss 用） */
    writer->mdat_payload_bytes += frame_size_in_file;       /* mdat 里已写的数据量增加，mp4_close 回填长度时要用 */
    return TD_SUCCESS;                                      /* 这一帧写成功 */
}

/*
 * 【mp4_frame_count】已经成功写入的帧数。writer 为 NULL 时返回 0。
 */
td_u32 mp4_frame_count(const mp4_writer *writer)
{
    if (writer == NULL) {
        return 0;
    }
    return writer->sample_count;                            /* 返回已经记录的帧数 */
}

/*
 * 【mp4_close】结束录制：回填 mdat 长度，把 moov 写到文件末尾，关闭文件。
 *
 * 返回：TD_SUCCESS 表示得到了一个完整可播放的 MP4；TD_FAILURE 表示文件不可用
 *       （一帧都没录到 / 写盘出错 / 关文件出错）。
 * 重要：无论成功还是失败，返回后 writer 都已经被释放，调用者不能再使用这个指针。
 */
td_s32 mp4_close(mp4_writer *writer)
{
    td_s32 result = TD_FAILURE;     /* 整个函数最终的返回值，只有全部步骤成功才改成 TD_SUCCESS */
    byte_buffer moov_buffer = {0};                          /* 内存缓冲区，用来拼 moov；{0} 表示所有成员初始为 0 */
    td_s32 ret;                     /* 每一步调用的返回值：0 表示成功 */
    size_t write_ret;               /* fwrite 的返回值：实际写入的字节数 */

    if (writer == NULL) {
        return TD_FAILURE;
    }
    /*
     * 一帧都没有就不写 moov（build_moov_box 要用最后一帧的信息，没有帧会出错）。
     * 这时直接跳到 cleanup 去关文件、释放内存，返回失败。
     */
    if (writer->sample_count == 0) {
        printf("mp4: no frames recorded\n");
        goto cleanup;
    }

    /*
     * 第 1 步：回头填 mdat 的总长度（= 8 字节盒子头 + 所有帧的数据）。
     * 做法：fseeko 跳回 mp4_open 时记下的 mdat 头位置 → 写 4 字节长度 → 再 fseeko 回到文件末尾，
     * 这样接下来写的 moov 就紧跟在最后一帧后面。
     */
    ret = fseeko(writer->file, writer->mdat_header_offset, SEEK_SET);    /* 1.1 跳回 mdat 头 */
    if (ret != 0) {
        printf("mp4: seek to mdat header failed\n");
        goto cleanup;
    }
    ret = write_u32_big_endian(writer->file, (td_u32)(writer->mdat_payload_bytes + 8));   /* 1.2 写真实长度 */
    if (ret != 0) {
        printf("mp4: patch mdat size failed\n");
        goto cleanup;
    }
    ret = fseeko(writer->file, 0, SEEK_END);    /* 1.3 回到文件末尾，准备写 moov */
    if (ret != 0) {
        printf("mp4: seek to file end failed\n");
        goto cleanup;
    }

    /*
     * 第 2 步：在内存里拼好 moov，一次写到文件末尾。第一帧的位置 = mdat 头位置 + 8。
     * 同样分两次判断：先看拼 moov 时有没有内存不足，再看写盘写全了没有。
     */
    build_moov_box(&moov_buffer, writer, (td_u32)(writer->mdat_header_offset + 8));  /* 在内存里拼好整个 moov */
    if (moov_buffer.has_error) {
        printf("mp4: build moov failed (out of memory)\n");
        goto cleanup;
    }
    write_ret = fwrite(moov_buffer.data, 1, moov_buffer.length, writer->file);  /* 把 moov 一次性写到文件末尾 */
    if (write_ret != moov_buffer.length) {
        printf("mp4: write moov failed\n");
        goto cleanup;
    }
    result = TD_SUCCESS;                                    /* 全部步骤都成功，才把结果改成成功 */

/*
 * 统一出口：无论上面哪一步成功或失败，都要走到这里关闭文件、释放内存。
 * 用 goto 跳到一个出口，比在每个出错分支里重复写一遍释放代码更不容易漏掉。
 * moov_buffer.data 即使从未分配（还是 NULL），free(NULL) 也是安全的。
 */
cleanup:
    /*
     * fclose 也要检查：文件缓冲区里还没真正写到磁盘的数据，是在 fclose 时才写出去的，
     * 这一步失败同样说明文件不完整。
     */
    ret = fclose(writer->file);                             /* 关闭文件；缓冲区里还没落盘的数据在这里写出 */
    if (ret != 0) {
        perror("mp4: fclose");
        result = TD_FAILURE;
    }
    free(moov_buffer.data);                                 /* 释放拼 moov 用的缓冲区 */
    free(writer->samples);                                  /* 释放“每帧账本”数组 */
    free(writer);                                           /* 释放 writer 结构体本身 */
    return result;                                          /* 返回最终结果 */
}

