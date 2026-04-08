#include "main_ahrs.h"

#if defined(PLATFORM_WINDOWS) || defined(PLATFORM_LINUX)
#include "ahrs_post.h"
#endif

int main(void)
{
#ifdef PLATFORM_MCU
    // MCU平台: 实时运行模式
    // Relocation of VectTable
    SCB->VTOR = APP_FLASH_BASE + VECT_TAB_OFFSET;

    // 初始化硬件和端口
    hal_Init();

    // 初始化任务和同步对象
    task_Init();

    // 启动调度器
    task_start();

    while (1)
    {
        // 主循环逻辑，一般不会到达这里
        algo_sleep_ms(1000); // 每秒执行一次
    }
#else
    // Windows/Linux平台: 后处理模式
    task_start_post("post_config.txt");
    printf("finish!");
    getchar();
#endif
}
