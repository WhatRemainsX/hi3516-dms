/**
 * @file mp4_writer.h
 * @brief 最小 MP4 写入器的对外接口：只支持 1 路 H.264 视频，没有音频。
 *
 * 【怎么用】按下面的顺序调用，每个录像文件对应一个 writer：
 *
 *     mp4_writer *w = mp4_open("rec.mp4", 1280, 720, sps, sps_len, pps, pps_len);
 *     mp4_write_frame(w, frame, frame_len, pts_us);     // 每来一帧调一次，可以调很多次
 *     mp4_close(w);                                     // 结束；之后 w 不能再用
 *
 * 【文件怎么形成】
 *     mp4_open        → 写 ftyp，再写 mdat 的盒子头（长度先占位 0）
 *     mp4_write_frame → 把每一帧写进 mdat，同时在内存里记下每帧的大小/时间/是否关键帧
 *     mp4_close       → 回填 mdat 长度，把 moov（目录）写到文件末尾
 *   所以必须调用 mp4_close，否则文件没有目录，播放器打不开。
 *
 * 【线程】writer 内部没有加锁，同一个 writer 只能在同一个线程里使用。
 */
#ifndef MP4_WRITER_H
#define MP4_WRITER_H

#include "ot_type.h"   /* SDK 的基本类型：td_u8 / td_u32 / td_u64 / td_s32 等 */

/*
 * 不透明类型：这里只声明"有一个叫 mp4_writer 的结构体"，里面的成员定义在 mp4_writer.c 里。
 * 外部代码只能拿到指针，没法直接访问或修改成员，只能通过下面的函数操作。
 * 好处：以后改内部实现（比如换成 fMP4）时，调用者的代码不用动。
 */
typedef struct mp4_writer mp4_writer;

/*
 * 创建 MP4 文件并写好开头。
 *
 * path          文件路径；文件已存在会被清空覆盖
 * width/height  画面大小（像素），例如 1280、720
 * sps/pps       不含起始码的 SPS、PPS，第 1 个字节是 NAL 头（SPS 通常是 0x67，PPS 通常是 0x68）。
 *               可以用 nal_get_sps_pps() 取得。MP4 里它们只存放在 moov 的 avcC 盒子中，
 *               所以这里只是先保存一份，到 mp4_close 才写进文件
 * sps_len/pps_len  两者的长度；sps_len 至少 4，且不能超过 NAL_SPS_MAX_SIZE / NAL_PPS_MAX_SIZE
 *
 * 返回：成功返回 writer 指针；失败（参数不对、文件打不开、写盘失败）返回 NULL
 */
mp4_writer *mp4_open(const char *path, td_u32 width, td_u32 height,
                     const td_u8 *sps, td_u32 sps_len,
                     const td_u8 *pps, td_u32 pps_len);

/*
 * 写入一帧。
 *
 * data/len  编码器给的一整帧，带起始码（00 00 00 01），
 *           可以含 SPS/PPS/IDR（关键帧），也可以只有一个 P 切片。内部会自动去掉起始码并改成"4 字节长度"格式
 * pts_us    这一帧的时间戳，单位微秒。可以是绝对时间（比如开机以来的时间），
 *           内部会减去第一帧的时间，换算成从 0 开始
 *
 * 返回：TD_SUCCESS 成功；TD_FAILURE 表示这一帧没有被写入，调用者丢掉它、继续送下一帧即可。
 *       常见原因：第一帧不是关键帧（IDR）。文件必须从关键帧开始，否则开头的 P 帧没有参考帧无法解码，
 *       所以刚开始录像时，要一直等到遇上 IDR 才会真正开始
 */
td_s32 mp4_write_frame(mp4_writer *w, const td_u8 *data, td_u32 len, td_u64 pts_us);

/*
 * 已经成功写入的帧数（被拒绝的帧不计）。writer 为 NULL 时返回 0。
 * 常用来在录像结束时打印"录了多少帧"，或判断是否一帧都没录到。
 */
td_u32 mp4_frame_count(const mp4_writer *w);

/*
 * 结束录制：回填 mdat 的长度，把 moov（目录）写到文件末尾，关闭文件。
 *
 * 返回：TD_SUCCESS 表示得到了一个完整可播放的 MP4；
 *       TD_FAILURE 表示文件不可用（一帧都没录到 / 写盘出错 / 关文件出错）。
 *
 * 注意：无论成功还是失败，返回后 w 指向的内存都已经被释放，
 *       调用者不能再使用这个指针（建议调用后立刻把自己的变量置为 NULL，避免重复关闭）。
 */
td_s32 mp4_close(mp4_writer *w);

#endif

