#include "stm32f10x.h"
#include "main.h"
#include "cpp_main.h" 

int main(void)
{
    // 1. 初始化
    Serial_Init(2000000);
    ICM20602_Init_Bare();
    ICM20602_PreInit_PacketBuffers(p_ping_buffer, p_pong_buffer);
    
    // 2. 移交控制权给 C++
    cpp_entry();

    return 0;
}
