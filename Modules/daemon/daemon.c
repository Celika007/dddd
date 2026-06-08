
/********************************包含头文件************************************/
/* 依赖守护模块的头文件声明 */
#include "daemon.h"
/* 时间测量/系统时间接口（后续可通过定时器计时） */
#include "bsp_dwt.h" // 后续通过定时器来计时?
/* C 标准库：动态内存管理 */
#include "stdlib.h"
/* C 标准库：内存/内存拷贝操作 */
#include "memory.h"
/* 蜂鸣器控制接口，用于离线报警提示 */
//#include "buzzer.h"

// 用于保存所有的daemon instance
/* 保存所有已注册的守护实例指针数组 */
static DaemonInstance *daemon_instances[DAEMON_MX_CNT] = {NULL};
/* 当前已注册的守护实例数量，用于遍历与回调 */
static uint8_t idx; // 用于记录当前的daemon instance数量,配合回调使用

/* 注册守护实例并返回指针 */
DaemonInstance *DaemonRegister(Daemon_Init_Config_s *config)
{
    /* 分配守护实例内存 */
    DaemonInstance *instance = (DaemonInstance *)malloc(sizeof(DaemonInstance));
    /* 清零结构体，确保字段处于已知初始状态 */
    memset(instance, 0, sizeof(DaemonInstance));
    /* 保存拥有者ID，便于在离线回调中定位具体实例 */
    instance->owner_id = config->owner_id;
    /* 设置重载计数值；若传入为0则采用默认值100 */
    instance->reload_count = config->reload_count == 0 ? 100 : config->reload_count; // 默认值为100
    /* 记录离线回调函数指针 */
    instance->callback = config->callback;
    /* 初始化当前计数；若传入为0则采用默认值100 */
    instance->temp_count = config->init_count == 0 ? 100 : config->init_count; // 默认值为100,初始计数
    /* 将当前计数覆盖为重载值，统一上线后的计数初始值 */
    instance->temp_count = config->reload_count;
    /* 写入全局实例数组，并递增实例数量计数 */
    daemon_instances[idx++] = instance;
    /* 返回实例指针给调用方 */
    return instance;
}

/* "喂狗"函数：当模块收到数据或发生动作时重载计数 */
void DaemonReload(DaemonInstance *instance)
{
    /* 将当前计数设置为重载值，表示模块在线 */
    instance->temp_count = instance->reload_count;
}

/* 判断守护实例是否在线：当前计数大于0则在线 */
uint8_t DaemonIsOnline(DaemonInstance *instance)
{
    /* 返回在线状态：大于0为真（1），否则为假（0） */
    return instance->temp_count > 0;
}

/* 守护任务：周期递减计数，超时触发离线回调 */
void DaemonTask()
{
    /* 局部缓存指针，提升可读性并减少数组访存开销 */
    DaemonInstance *dins; // 提高可读性同时降低访存开销
    /* 遍历所有已注册的守护实例 */
    for (size_t i = 0; i < idx; ++i)
    {
        /* 读取当前实例指针 */
        dins = daemon_instances[i];
        /* 若当前计数仍大于0，说明未超时，进行递减 */
        if (dins->temp_count > 0) // 如果计数器还有值,说明上一次喂狗后还没有超时,则计数器减一
            dins->temp_count--;
        /* 若已超时且存在离线回调函数，则进行处理 */
        else if (dins->callback) // 等于零说明超时了,调用回调函数(如果有的话)
        {
            /* 调用离线回调，传入owner_id用于模块内部区分具体实例 */
            dins->callback(dins->owner_id); // module内可以将owner_id强制类型转换成自身类型从而调用特定module的offline callback
            /* 后续扩展：蜂鸣器/LED离线报警功能入口 */
            // @todo 为蜂鸣器/led等增加离线报警的功能,非常关键!
        }
    }
}
// (需要id的原因是什么?) 下面是copilot的回答!
// 需要id的原因是因为有些module可能有多个实例,而我们需要知道具体是哪个实例offline
// 如果只有一个实例,则可以不用id,直接调用callback即可
// 比如: 有一个module叫做"电机",它有两个实例,分别是"电机1"和"电机2",那么我们调用电机的离线处理函数时就需要知道是哪个电机offline
