/*
 * npu_model：把一个 .om 模型包装成"填输入 → 执行 → 读输出"。
 *   不关心输入是什么画面、输出代表什么，DMS 的检测、关键点、眼睛模型都用它。
 *
 * 调用顺序：
 *   npu_model_global_init()      进程里只调一次，必须在 media_system_init() 之后
 *   npu_model_load()             每个 .om 调一次
 *     循环：写 get_input_buffer() → npu_model_execute() → 读 get_output_buffer()
 *   npu_model_unload()
 *   npu_model_global_deinit()    所有模型都 unload 之后
 *
 * 线程：一个 npu_model 只能被一个线程使用，模块内部不加锁。
 */
#ifndef NPU_MODEL_H
#define NPU_MODEL_H

#include <stddef.h>
#include "ot_type.h"

#define NPU_MODEL_MAX_DIMS 4

/* 一块输入或输出数据的形状和内存排布 */
typedef struct {
    td_u32 dim_count;                   /* 维度个数，比如 [1,3,352,640] 是 4 */
    td_s64 dims[NPU_MODEL_MAX_DIMS];    /* 各维度大小 */
    size_t size;                        /* 整块缓冲区的字节数 */
    size_t stride;                      /* 最后一维"一行"在内存里实际占的字节数（含对齐补位） */
} npu_tensor_info;

typedef struct npu_model npu_model;

td_s32 npu_model_global_init(void);
void npu_model_global_deinit(void);

/* 加载失败返回 NULL。只支持"1 个真正输入"、输入输出都是 FP32 的模型（DMS 的三个模型都是） */
npu_model *npu_model_load(const char *om_path);
void npu_model_unload(npu_model *model);

/* 输入：先查形状，再直接往 buffer 里写数据 */
td_s32 npu_model_get_input_info(const npu_model *model, npu_tensor_info *info);
void *npu_model_get_input_buffer(npu_model *model);

/* 阻塞执行一次推理，返回时输出已经可以读了 */
td_s32 npu_model_execute(npu_model *model);

/* 输出：按下标遍历，用形状来认 */
td_u32 npu_model_get_output_count(const npu_model *model);
td_s32 npu_model_get_output_info(const npu_model *model, td_u32 index, npu_tensor_info *info);
const void *npu_model_get_output_buffer(const npu_model *model, td_u32 index);

/* 每个要调用 npu_model_execute() 的线程，必须先调用这个函数一次。
 * 原因：ACL 的"当前设备"是线程局部的，npu_model_global_init() 只给调用它的那个
 * 线程设置好了设备；别的线程要自己再设一次，否则 execute 会报 "get cur ctx failed"。 */
td_s32 npu_model_thread_attach(void);

#endif
