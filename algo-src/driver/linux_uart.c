#include "linux_uart.h"

// ===================== Linux实现 =====================
#ifdef PLATFORM_LINUX
UartHandle *uart_init(void *port_identifier, UartConfig config)
{
    UartHandle *handle = malloc(sizeof(UartHandle));
    if (!handle)
        return NULL;
    handle->fd = -1;
    handle->config = config;
    return handle;
}

int uart_open(UartHandle *handle)
{
    const char *port = (const char *)handle;
    handle->fd = open(port, O_RDWR | O_NOCTTY);
    if (handle->fd < 0)
        return -1;

    struct termios options;
    tcgetattr(handle->fd, &options);

    // 波特率设置
    speed_t baud;
    switch (handle->config.baud_rate)
    {
    case 9600:
        baud = B9600;
        break;
    case 19200:
        baud = B19200;
        break;
    case 38400:
        baud = B38400;
        break;
    case 57600:
        baud = B57600;
        break;
    case 115200:
        baud = B115200;
        break;
    case 460800:
        baud = B460800;
        break;
    default:
        baud = B9600;
    }
    cfsetispeed(&options, baud);
    cfsetospeed(&options, baud);

    // 数据位
    options.c_cflag &= ~CSIZE;
    switch (handle->config.data_bits)
    {
    case 5:
        options.c_cflag |= CS5;
        break;
    case 6:
        options.c_cflag |= CS6;
        break;
    case 7:
        options.c_cflag |= CS7;
        break;
    case 8:
        options.c_cflag |= CS8;
        break;
    default:
        options.c_cflag |= CS8;
    }

    // 停止位
    options.c_cflag &= ~CSTOPB;
    if (handle->config.stop_bits == 2)
        options.c_cflag |= CSTOPB;

    // 奇偶校验
    switch (handle->config.parity)
    {
    case 'o':
    case 'O':
        options.c_cflag |= PARENB;
        options.c_cflag |= PARODD;
        break;
    case 'e':
    case 'E':
        options.c_cflag |= PARENB;
        options.c_cflag &= ~PARODD;
        break;
    default:
        options.c_cflag &= ~PARENB;
    }

    // 应用设置
    options.c_cflag |= CLOCAL | CREAD;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tcsetattr(handle->fd, TCSANOW, &options);
    return 0;
}

int uart_close(UartHandle *handle)
{
    if (handle->fd >= 0)
    {
        close(handle->fd);
        handle->fd = -1;
    }
    return 0;
}

int uart_read(UartHandle *handle, char *buffer, size_t size)
{
    return read(handle->fd, buffer, size);
}

int uart_write(UartHandle *handle, const char *data, size_t length)
{
    return write(handle->fd, data, length);
}

void uart_release(UartHandle *handle)
{
    uart_close(handle);
    free(handle);
}
#endif
