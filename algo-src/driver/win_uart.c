#include "win_uart.h"

// ===================== Windows实现 =====================
#ifdef PLATFORM_WINDOWS

UartHandle *uart_init( UartConfig config)
{
    UartHandle *handle = (UartHandle *)malloc(sizeof(UartHandle));
    if (!handle)
        return NULL;

    // 存储配置信息
    //handle->config = config;
    memcpy(&handle->config, &config, sizeof(UartConfig));

    handle->hCom = INVALID_HANDLE_VALUE;

    return handle;
}

int uart_open(UartHandle *handle)
{
    if (!handle)
        return -1;

    char port_name[16];
    snprintf(port_name, sizeof(port_name), "\\\\.\\%s", (char *)handle->config.port_str);

    // 打开串行端口
    handle->hCom = CreateFileA(
        port_name,
        GENERIC_READ | GENERIC_WRITE,
        0,                     // 独占模式
        NULL,                  // 默认安全属性
        OPEN_EXISTING,         // 打开现有设备
        FILE_ATTRIBUTE_NORMAL, // 默认属性
        NULL                   // 无模板
    );

    if (handle->hCom == INVALID_HANDLE_VALUE)
    {
        return GetLastError(); // 返回错误代码
    }

    // 配置串口参数
    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

    if (!GetCommState(handle->hCom, &dcbSerialParams))
    {
        CloseHandle(handle->hCom);
        return GetLastError();
    }

    // 设置波特率
    dcbSerialParams.BaudRate = handle->config.baud_rate;

    // 设置数据位
    switch (handle->config.data_bits)
    {
    case 5:
        dcbSerialParams.ByteSize = 5;
        break;
    case 6:
        dcbSerialParams.ByteSize = 6;
        break;
    case 7:
        dcbSerialParams.ByteSize = 7;
        break;
    case 8:
    default:
        dcbSerialParams.ByteSize = 8;
    }

    // 设置停止位
    switch (handle->config.stop_bits)
    {
    case 1:
        dcbSerialParams.StopBits = ONESTOPBIT;
        break;
    case 2:
        dcbSerialParams.StopBits = TWOSTOPBITS;
        break;
    default:
        dcbSerialParams.StopBits = ONESTOPBIT;
    }

    // 设置校验位
    switch (handle->config.parity)
    {
    case 'o':
    case 'O':
        dcbSerialParams.Parity = ODDPARITY;
        break;
    case 'e':
    case 'E':
        dcbSerialParams.Parity = EVENPARITY;
        break;
    case 'n':
    case 'N':
    default:
        dcbSerialParams.Parity = NOPARITY;
    }

    // 应用配置
    if (!SetCommState(handle->hCom, &dcbSerialParams))
    {
        CloseHandle(handle->hCom);
        return GetLastError();
    }

    // 设置超时参数 - 非阻塞模式
    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = MAXDWORD;   // 立即返回已有的字符
    timeouts.ReadTotalTimeoutConstant = 0;     // 不等待，立即返回
    timeouts.ReadTotalTimeoutMultiplier = 0;   // 不等待，立即返回
    timeouts.WriteTotalTimeoutConstant = 50;   // 写入操作固定超时
    timeouts.WriteTotalTimeoutMultiplier = 10; // 写入操作每字节超时

    if (!SetCommTimeouts(handle->hCom, &timeouts))
    {
        CloseHandle(handle->hCom);
        return GetLastError();
    }

    // 清空缓冲区
    PurgeComm(handle->hCom, PURGE_RXCLEAR | PURGE_TXCLEAR);

    return 0; // 成功
}

int uart_close(UartHandle *handle)
{
    if (!handle || handle->hCom == INVALID_HANDLE_VALUE)
        return -1;

    // 关闭串口并重置句柄
    CloseHandle(handle->hCom);
    handle->hCom = INVALID_HANDLE_VALUE;

    return 0;
}

int uart_read(UartHandle *handle, char *buffer, size_t size)
{
    if (!handle || handle->hCom == INVALID_HANDLE_VALUE || !buffer || size == 0)
        return -1;

    DWORD bytesRead = 0;
    BOOL status = ReadFile(
        handle->hCom, // 串口句柄
        buffer,       // 接收缓冲区
        (DWORD)size,  // 要读取的字节数
        &bytesRead,   // 实际读取的字节数
        NULL          // 不使用重叠I/O
    );

    if (!status)
    {
        return -GetLastError(); // 返回错误代码（负值）
    }

    return (int)bytesRead;
}

int uart_write(UartHandle *handle, const char *data, size_t length)
{
    if (!handle || handle->hCom == INVALID_HANDLE_VALUE || !data || length == 0)
        return -1;

    DWORD bytesWritten = 0;
    BOOL status = WriteFile(
        handle->hCom,  // 串口句柄
        data,          // 要发送的数据
        (DWORD)length, // 要写入的字节数
        &bytesWritten, // 实际写入的字节数
        NULL           // 不使用重叠I/O
    );

    if (!status)
    {
        return -GetLastError(); // 返回错误代码（负值）
    }

    return (int)bytesWritten;
}

void uart_release(UartHandle *handle)
{
    if (!handle)
        return;

    // 确保关闭串口
    uart_close(handle);

    // 释放内存
    free(handle);
}

#endif // PLATFORM_WINDOWS
