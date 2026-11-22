/*
 * npu_model.c：npu_model.h 的实现。
 *   参考：SDK sample/svp/common/sample_common_svp_npu_model.c，以及 npu_study/bench/bench_npu.c。
 *
 * 内存怎么分配（见"CPU 缓存"那一课）：
 *   - 真正的输入、所有输出：带缓存的内存（CPU 要大量读写，带缓存才快），
 *     由 npu_model_execute() 负责执行前、执行后各刷新一次。
 *   - task_buf、work_buf：NPU 自己用的工作区，CPU 从不读写，用不带缓存的内存，省得操心刷新。
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "npu_model.h"
#include "svp_acl.h"
#include "svp_acl_mdl.h"
#include "svp_acl_rt.h"

#define NPU_DEVICE_ID 0
/* 每个模型的输入列表最后都多出 2 项：task_buf 和 work_buf（SDK 要求，不是真正的模型输入） */
#define NPU_EXTRA_INPUT_NUM 2
/* 真正的输入固定是第 0 项 */
#define NPU_REAL_INPUT_INDEX 0

/*
 * NPU 模型运行时对象。
 * 结构体只在这个 .c 里定义，外部模块只能通过 npu_model 指针和公开接口访问。
 * 对象负责保存模型资源、输入输出数据集以及执行推理所需的工作内存。
 */
struct npu_model {
    td_u32 model_id;                    /* ACL 加载模型后分配的模型编号 */
    td_bool model_loaded;               /* 模型是否已成功加载，卸载时据此决定是否调用 unload */
    void *model_memory;                 /* .om 模型文件内容的内存，模型卸载前必须保持有效 */
    svp_acl_mdl_desc *desc;             /* 模型描述信息，用于查询输入输出数量、形状和大小 */
    svp_acl_mdl_dataset *input_dataset; /* 输入数据集，包含真实输入以及 task_buf、work_buf */
    svp_acl_mdl_dataset *output_dataset;/* 输出数据集，保存推理产生的全部输出 */
    td_u32 output_count;                /* 模型输出数量，用于校验和遍历输出下标 */
};

/*
 * 初始化 NPU 推理所需的全局 ACL 运行环境。
 * 该函数只应在进程启动时调用一次，并且必须先于模型加载。
 * 初始化失败时会按相反顺序释放已经成功建立的设备和 ACL 资源。
 */

td_s32 npu_model_global_init(void)
{
    svp_acl_error ret;
    svp_acl_rt_run_mode run_mode;

    /* 1. 初始化 ACL 库。空字符串：不使用配置文件，全部用默认值 */
    ret = svp_acl_init(NULL);
    if (ret != SVP_ACL_SUCCESS) {
        printf("npu: init failed: %d\n", ret);
        return TD_FAILURE;
    }

    /* 2. 打开 0 号 NPU 设备 */
    ret = svp_acl_rt_set_device(NPU_DEVICE_ID);
    if (ret != SVP_ACL_SUCCESS) {
        printf("npu: set device failed: %d\n", ret);
        goto finalize;          /* 第 1 步已成功，要撤销它 */
    }

    /* 3. 确认运行在板端（DEVICE 模式），而不是 PC 仿真 */
    ret=svp_acl_rt_get_run_mode(&run_mode);
    if (ret != SVP_ACL_SUCCESS || run_mode != SVP_ACL_DEVICE) {
        printf("npu: get run mode failed: %d, mode=%d\n", ret, (int)run_mode);
        goto reset_device;      /* 第 1、2 步都已成功，都要撤销 */
    }

    printf("npu: acl initialized\n");
    return TD_SUCCESS;

reset_device:
    (void)svp_acl_rt_reset_device(NPU_DEVICE_ID);   /* 撤销第 2 步 */
finalize:
    (void)svp_acl_finalize();                       /* 撤销第 1 步 */
    return TD_FAILURE;
}

/* 和 global_init 反序释放：先关设备，再关 ACL 库。进程退出前调一次 */
void npu_model_global_deinit(void)
{
    svp_acl_error ret;

    /* 和 init 反序：先关设备，再关库 */
    ret = svp_acl_rt_reset_device(NPU_DEVICE_ID);
    if (ret != SVP_ACL_SUCCESS) {
        printf("npu: reset device failed: %d\n", ret);
        /* 关闭失败也继续往下关，不提前 return */
    }
    ret = svp_acl_finalize();
    if (ret != SVP_ACL_SUCCESS) {
        printf("npu: finalize failed: %d\n", ret);
    }
}

/* 分配一块内存，包成 data_buffer，加进数据集。失败时自己负责释放已分配的部分 */
static td_s32 add_buffer(svp_acl_mdl_dataset *dataset, size_t size, size_t stride, td_bool cached)
{
    void *memory = NULL;
    svp_acl_data_buffer *buffer = NULL;
    svp_acl_error ret;

    /* 大小或 stride 为 0，说明模型描述查出来的数据有问题 */
    if (size == 0 || stride == 0) {
        printf("npu: invalid buffer size %zu / stride %zu\n", size, stride);
        return TD_FAILURE;
    }

    /* 1. 申请 NPU 能访问的内存，按 cached 决定带不带缓存 */
    if (cached == TD_TRUE) {
        ret = svp_acl_rt_malloc_cached(&memory, size, SVP_ACL_MEM_MALLOC_NORMAL_ONLY);
    } else {
        ret = svp_acl_rt_malloc(&memory, size, SVP_ACL_MEM_MALLOC_NORMAL_ONLY);
    }
    if (ret != SVP_ACL_SUCCESS) {
        printf("npu: malloc %zu bytes failed: %d\n", size, ret);
        return TD_FAILURE;
    }

    /* 2. 清零，避免第一次推理时读到上一个程序留下的垃圾数据 */
    memset(memory, 0, size);
    if (cached == TD_TRUE) {
        /* 刚清零的内容还在 CPU 缓存里，刷回 DDR，保证 NPU 看到的也是 0 */
        (void)svp_acl_rt_mem_flush(memory, size);
    }

    /* 3. 给这块内存贴上"标签"：地址 + 大小 + stride */
    buffer = svp_acl_create_data_buffer(memory, size, stride);
    if (buffer == NULL) {
        printf("npu: create data buffer failed\n");
        (void)svp_acl_rt_free(memory);          /* 撤销第 1 步 */
        return TD_FAILURE;
    }

    /* 4. 把这个 buffer 加进数据集 */
    ret = svp_acl_mdl_add_dataset_buffer(dataset, buffer);
    if (ret != SVP_ACL_SUCCESS) {
        printf("npu: add dataset buffer failed: %d\n", ret);
        (void)svp_acl_destroy_data_buffer(buffer);  /* 撤销第 3 步 */
        (void)svp_acl_rt_free(memory);              /* 撤销第 1 步 */
        return TD_FAILURE;
    }
    return TD_SUCCESS;

}

/* 释放数据集里每一块内存和 data_buffer，再释放数据集本身。允许传 NULL */
static void destroy_dataset(svp_acl_mdl_dataset *dataset)
{
    size_t count;
    size_t index;
    svp_acl_data_buffer *buffer;

    /* 加载中途失败时，数据集可能还没创建，所以要允许 NULL */
    if (dataset == NULL) {
        return;
    }

    count = svp_acl_mdl_get_dataset_num_buffers(dataset);
    for (index = 0; index < count; index++) {
        buffer = svp_acl_mdl_get_dataset_buffer(dataset, index);
        if (buffer != NULL) {
            /* 先通过标签找到地址，释放内存；再撕掉标签 */
            (void)svp_acl_rt_free(svp_acl_get_data_buffer_addr(buffer));
            (void)svp_acl_destroy_data_buffer(buffer);
        }
    }

    /* 最后销毁"列表"本身 */
    (void)svp_acl_mdl_destroy_dataset(dataset);
}

/* 把 SDK 的维度结构转成我们自己的 npu_tensor_info */
static td_s32 fill_info(const svp_acl_mdl_io_dims *dims, size_t size, size_t stride, npu_tensor_info *info)
{
    td_u32 index;

    /* 我们只留了 4 个维度的位置，超过就装不下，报错 */
    if (dims->dim_count > NPU_MODEL_MAX_DIMS) {
        printf("npu: too many dims: %zu\n", dims->dim_count);
        return TD_FAILURE;
    }

    memset(info, 0, sizeof(*info));/*先全部清零*/

    info->dim_count = (td_u32)dims->dim_count;
    for (index = 0; index < info->dim_count; index++) {
        info->dims[index] = dims->dims[index];
    }

    info->size = size;
    info->stride = stride;
    return TD_SUCCESS;

}

/* 把 .om 文件整个读进 NPU 能访问的内存 */
static td_s32 read_model_file(npu_model *model, const char *om_path, size_t *model_size)
{
    FILE *fp;
    long file_size;
    int seek_ret;
    size_t read_count;
    td_s32 result = TD_FAILURE;     /* 先假设失败，全部成功才改成 SUCCESS */
    svp_acl_error ret;

    fp = fopen(om_path, "rb");
    if (fp == NULL) {
        printf("npu: open %s failed\n", om_path);
        return TD_FAILURE;
    }

    /* 先跳到文件末尾，ftell 得到的位置就是文件大小，再跳回开头 */
    seek_ret = fseek(fp, 0L, SEEK_END);
    if (seek_ret != 0) {
        goto close_file;
    }
    file_size = ftell(fp);
    if (file_size <= 0) {
        printf("npu: bad model file %s\n", om_path);
        goto close_file;
    }
    seek_ret = fseek(fp, 0L, SEEK_SET);
    if (seek_ret != 0) {
        goto close_file;
    }

    /* 申请和文件一样大的 NPU 内存，地址直接存进结构体 */
    ret = svp_acl_rt_malloc(&model->model_memory, (size_t)file_size, SVP_ACL_MEM_MALLOC_NORMAL_ONLY);
    if (ret != SVP_ACL_SUCCESS) {
        printf("npu: malloc model memory failed: %d\n", ret);
        goto close_file;
    }

    /* 整个文件作为"1 块"读进来，成功时返回 1 */
    read_count = fread(model->model_memory, (size_t)file_size, 1, fp);
    if (read_count != 1) {
        printf("npu: read %s failed\n", om_path);
        goto close_file;    /* model_memory 交给 npu_model_unload 释放 */
    }

    *model_size = (size_t)file_size;    /* 通过指针参数把文件大小交给调用者 */
    result = TD_SUCCESS;

close_file:
    fclose(fp);     /* 不管成功失败，文件都要关 */
    return result;
}

npu_model *npu_model_load(const char *om_path)
{
    npu_model *model;               /* 要返回给调用者的句柄 */
    size_t model_size = 0;          /* .om 文件的字节数，由 read_model_file 填 */
    size_t input_num;               /* 模型有几个输入（含 task_buf、work_buf） */
    size_t index;                   /* 循环下标 */
    size_t batch_index;             /* "batch 控制"这个特殊输入在第几项 */
    size_t buffer_size;             /* 当前这一块内存要多大 */
    size_t buffer_stride;           /* 当前这一块每行占多少字节 */
    td_bool cached;                 /* 当前这一块要不要带缓存 */
    td_s32 result;                  /* 我们自己函数的返回值（TD_SUCCESS / TD_FAILURE） */
    svp_acl_data_type data_type;    /* 数据类型，比如 SVP_ACL_FLOAT */
    svp_acl_error ret;              /* SDK 函数的返回值（SVP_ACL_SUCCESS / 错误码） */

    /* calloc：所有字段先清零，失败清理时靠"是不是 NULL/FALSE"判断哪些资源已经分配 */
    model = calloc(1, sizeof(*model));
    if (model == NULL) {
        printf("npu: out of memory\n");
        return NULL;
    }

    result = read_model_file(model, om_path, &model_size);
    if (result != TD_SUCCESS) {
        goto fail;
    }
    /*
        svp_acl_mdl_load_from_mem(内存, 大小, &编号)：SDK 解析这块内存里的模型（网络结构、权重、NPU 指令），
        在 NPU 那边登记好，然后给你一个编号 model_id。之后执行、查说明书都用这个编号指定是哪个模型
    */
    ret = svp_acl_mdl_load_from_mem(model->model_memory, model_size, &model->model_id);
    if (ret != SVP_ACL_SUCCESS) {
        printf("npu: load model %s failed: %d\n", om_path, ret);
        goto fail;
    }
    model->model_loaded = TD_TRUE;      /* 记下来：卸载时要调 svp_acl_mdl_unload*/

    /* ---------- 2. 取模型描述，检查是不是我们支持的形式 ---------- */
    /*先创建一个空白描述，在写进去这个模型的信息*/
    model->desc = svp_acl_mdl_create_desc();
    if (model->desc == NULL) {
        printf("npu: create desc failed\n");
        goto fail;
    }
    ret = svp_acl_mdl_get_desc(model->desc, model->model_id);
    if (ret != SVP_ACL_SUCCESS) {
        printf("npu: get desc failed: %d\n", ret);
        goto fail;
    }

    /* 输入个数必须是 1（真正输入）+ 2（task_buf、work_buf）= 3 */
    input_num = svp_acl_mdl_get_num_inputs(model->desc);
    if (input_num != 1 + NPU_EXTRA_INPUT_NUM) {
        printf("npu: %s has %zu inputs, only 1 real input is supported\n", om_path, input_num);
        goto fail;
    }

    /* 预处理代码会往输入里写 float，所以输入必须是 FP32 */
    data_type = svp_acl_mdl_get_input_data_type(model->desc, NPU_REAL_INPUT_INDEX);
    if (data_type != SVP_ACL_FLOAT) {
        printf("npu: input of %s is not FP32\n", om_path);
        goto fail;
    }
    model->output_count = (td_u32)svp_acl_mdl_get_num_outputs(model->desc);

    /* ---------- 3. 建输入数据集 ---------- */
    model->input_dataset = svp_acl_mdl_create_dataset();
    model->output_dataset = svp_acl_mdl_create_dataset();
    if (model->input_dataset == NULL || model->output_dataset == NULL) {
        printf("npu: create dataset failed\n");
        goto fail;
    }

    for (index = 0; index < input_num; index++) {
        /* 只有真正的输入要 CPU 写，用带缓存的内存；task_buf、work_buf 不带缓存 */
        if (index == NPU_REAL_INPUT_INDEX) {
            cached = TD_TRUE;
        } else {
            cached = TD_FALSE;
        }
        /* 每一项多大、stride 多少，都向模型描述查，代码里不写死 */
        buffer_size = svp_acl_mdl_get_input_size_by_index(model->desc, index);
        buffer_stride = svp_acl_mdl_get_input_default_stride(model->desc, index);
        result = add_buffer(model->input_dataset, buffer_size, buffer_stride, cached);
        if (result != TD_SUCCESS) {
            goto fail;
        }
    }

    /* ---------- 4. 建输出数据集 ---------- */
    for (index = 0; index < model->output_count; index++) {
        data_type = svp_acl_mdl_get_output_data_type(model->desc, index);
        if (data_type != SVP_ACL_FLOAT) {
            printf("npu: output %zu of %s is not FP32\n", index, om_path);
            goto fail;
        }
        buffer_size = svp_acl_mdl_get_output_size_by_index(model->desc, index);
        buffer_stride = svp_acl_mdl_get_output_default_stride(model->desc, index);
        result = add_buffer(model->output_dataset, buffer_size, buffer_stride, TD_TRUE);
        if (result != TD_SUCCESS) {
            goto fail;
        }
    }

    /* ---------- 5. 动态 batch 设为 1 ---------- */
    ret = svp_acl_mdl_get_input_index_by_name(model->desc, SVP_ACL_DYNAMIC_TENSOR_NAME, &batch_index);
    if (ret == SVP_ACL_SUCCESS) {
        ret = svp_acl_mdl_set_dynamic_batch_size(model->model_id, model->input_dataset, batch_index, 1);
        if (ret != SVP_ACL_SUCCESS) {
            printf("npu: set dynamic batch failed: %d\n", ret);
            goto fail;
        }
    }

    printf("npu: loaded %s (%zu bytes), outputs=%u\n", om_path, model_size, model->output_count);
    return model;

fail:
    /* 不管失败在哪一步，unload 都能只释放已经分配过的东西 */
    npu_model_unload(model);
    return NULL;
}

/* 卸载一个模型，释放它占用的全部资源。顺序和加载时相反，允许传 NULL（load 失败时的清理也会调它） */
void npu_model_unload(npu_model *model)
{
    svp_acl_error ret;

    if (model == NULL) {
        return;
    }

    /* 和加载顺序相反：先数据集，再描述，再卸载模型，最后释放模型文件内存 */
    destroy_dataset(model->input_dataset);      /* 内部会处理 NULL */
    destroy_dataset(model->output_dataset);

    if (model->desc != NULL) {
        (void)svp_acl_mdl_destroy_desc(model->desc);
    }

    if (model->model_loaded == TD_TRUE) {
        ret = svp_acl_mdl_unload(model->model_id);
        if (ret != SVP_ACL_SUCCESS) {
            printf("npu: unload model failed: %d\n", ret);
        }
    }

    /* 必须在 unload 之后：模型加载期间 NPU 一直在用这块内存 */
    if (model->model_memory != NULL) {
        (void)svp_acl_rt_free(model->model_memory);
    }

    free(model);    /* 最后释放结构体本身 */
}

/*让调用者查到输入的形状，好往里写数据。*/
td_s32 npu_model_get_input_info(const npu_model *model, npu_tensor_info *info)
{
    svp_acl_mdl_io_dims dims;
    size_t buffer_size;
    size_t buffer_stride;
    td_s32 result;
    svp_acl_error ret;

    if (model == NULL || info == NULL) {
        return TD_FAILURE;
    }

    /* 查形状：比如检测模型是 [1,3,352,640] */
    memset(&dims, 0, sizeof(dims));
    ret = svp_acl_mdl_get_input_dims(model->desc, NPU_REAL_INPUT_INDEX, &dims);
    if (ret != SVP_ACL_SUCCESS) {
        printf("npu: get input dims failed: %d\n", ret);
        return TD_FAILURE;
    }

    /* 查大小和 stride，一起填进 info */
    buffer_size = svp_acl_mdl_get_input_size_by_index(model->desc, NPU_REAL_INPUT_INDEX);
    buffer_stride = svp_acl_mdl_get_input_default_stride(model->desc, NPU_REAL_INPUT_INDEX);
    result = fill_info(&dims, buffer_size, buffer_stride, info);
    return result;
}

/*让调用者拿到输入内存的地址，好往里写数据。*/
void *npu_model_get_input_buffer(npu_model *model)
{
    svp_acl_data_buffer *buffer;
    void *data;

    if (model == NULL) {
        return NULL;
    }

    /* 从输入数据集里取出第 0 个数据块（真正的输入），再从"标签"上读出地址 */
    buffer = svp_acl_mdl_get_dataset_buffer(model->input_dataset, NPU_REAL_INPUT_INDEX);
    if (buffer == NULL) {
        return NULL;
    }
    data = svp_acl_get_data_buffer_addr(buffer);
    return data;
}

/* 对数据集里第 index 块内存做缓存刷新 */
static td_s32 flush_buffer(const svp_acl_mdl_dataset *dataset, size_t index)
{
    svp_acl_data_buffer *buffer;
    void *data;
    size_t size;
    svp_acl_error ret;

    buffer = svp_acl_mdl_get_dataset_buffer(dataset, index);
    if (buffer == NULL) {
        return TD_FAILURE;
    }
    /* 从"标签"上读出地址和大小，对整块做刷新 */
    data = svp_acl_get_data_buffer_addr(buffer);
    size = svp_acl_get_data_buffer_size(buffer);
    ret = svp_acl_rt_mem_flush(data, size);
    if (ret != SVP_ACL_SUCCESS) {
        printf("npu: flush buffer %zu failed: %d\n", index, ret);
        return TD_FAILURE;
    }
    return TD_SUCCESS;
}

/* 对外的唯一"推理"入口：刷新输入 -> 执行 -> 刷新输出，返回时所有输出都已经可以读 */
td_s32 npu_model_execute(npu_model *model)
{
    td_u32 index;
    td_s32 result;
    svp_acl_error ret;

    if (model == NULL) {
        return TD_FAILURE;
    }

    /* 1. 执行前：把 CPU 写在缓存里的输入刷回 DDR，NPU 才能读到最新数据 */
    result = flush_buffer(model->input_dataset, NPU_REAL_INPUT_INDEX);
    if (result != TD_SUCCESS) {
        return TD_FAILURE;
    }

    /* 2. 执行推理，阻塞到 NPU 算完才返回 */
    ret = svp_acl_mdl_execute(model->model_id, model->input_dataset, model->output_dataset);
    if (ret != SVP_ACL_SUCCESS) {
        printf("npu: execute failed: %d\n", ret);
        return TD_FAILURE;
    }

    /* 3. 执行后：NPU 把结果写进了 DDR，刷新每个输出，丢掉 CPU 缓存里的旧副本（官方样例同样做法） */
    for (index = 0; index < model->output_count; index++) {
        result = flush_buffer(model->output_dataset, index);
        if (result != TD_SUCCESS) {
            return TD_FAILURE;
        }
    }
    return TD_SUCCESS;
}

/* 这个模型有几个输出，调用者据此决定要遍历几次 get_output_info/get_output_buffer */
td_u32 npu_model_get_output_count(const npu_model *model)
{
    if (model == NULL) {
        return 0;
    }
    return model->output_count;
}

/* 查第 index 个输出的形状信息，用法和 npu_model_get_input_info 对称 */
td_s32 npu_model_get_output_info(const npu_model *model, td_u32 index, npu_tensor_info *info)
{
    svp_acl_mdl_io_dims dims;
    size_t buffer_size;
    size_t buffer_stride;
    td_s32 result;
    svp_acl_error ret;

    if (model == NULL || info == NULL || index >= model->output_count) {
        return TD_FAILURE;
    }
    memset(&dims, 0, sizeof(dims));
    ret = svp_acl_mdl_get_output_dims(model->desc, index, &dims);
    if (ret != SVP_ACL_SUCCESS) {
        printf("npu: get output %u dims failed: %d\n", index, ret);
        return TD_FAILURE;
    }
    buffer_size = svp_acl_mdl_get_output_size_by_index(model->desc, index);
    buffer_stride = svp_acl_mdl_get_output_default_stride(model->desc, index);
    result = fill_info(&dims, buffer_size, buffer_stride, info);
    return result;
}

const void *npu_model_get_output_buffer(const npu_model *model, td_u32 index)
{
    svp_acl_data_buffer *buffer;
    const void *data;

    if (model == NULL || index >= model->output_count) {
        return NULL;
    }
    buffer = svp_acl_mdl_get_dataset_buffer(model->output_dataset, index);
    if (buffer == NULL) {
        return NULL;
    }
    data = svp_acl_get_data_buffer_addr(buffer);
    return data;
}

/*
 * ACL 的设备/上下文是"线程局部"的：npu_model_global_init() 只在主线程里设过一次，
 * 别的线程（比如 DMS 线程）要用这个模型，必须自己先调一次这个函数，
 * 否则会报 "get cur ctx failed" 之类的错误。只需要在线程刚启动时调一次。
 */
td_s32 npu_model_thread_attach(void)
{
    svp_acl_error ret;

    ret = svp_acl_rt_set_device(0);
    if (ret != SVP_ACL_SUCCESS) {
        printf("npu: thread attach failed: %d\n", ret);
        return TD_FAILURE;
    }
    return TD_SUCCESS;
}

