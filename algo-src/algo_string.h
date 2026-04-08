#ifndef __ALGO_STRING_H__
#define __ALGO_STRING_H__

#include <stdint.h>

/**
 * @brief 移除字符串开头和结尾的所有空格、\r和\n字符
 * @param str 要处理的字符串（会被修改）
 * @return 返回处理后的字符串（与输入相同指针）
 */
uint8_t* trim(uint8_t* str);

/**
 * @brief 移除字符串中的所有空格、\r和\n字符（包括中间的空格）
 * @param str 要处理的字符串（会被修改）
 * @return 返回处理后的字符串（与输入相同指针）
 */
uint32_t trim_all(uint8_t* str);

#endif
