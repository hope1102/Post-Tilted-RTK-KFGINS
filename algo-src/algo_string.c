#include "algo_string.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>


/**
 * @brief 移除字符串开头和结尾的所有空格、\r和\n字符
 * @param str 要处理的字符串（会被修改）
 * @return 返回处理后的字符串（与输入相同指针）
 */
uint8_t* trim(uint8_t* str)
{
	if (str == NULL)
	{
		return NULL;
	}

	// 空字符串直接返回
	if (*str == '\0')
	{
		return str;
	}

	uint8_t* start = str;
	uint8_t* end = str + strlen(str) - 1;

	// 从开头找到第一个不是空格、\r或\n的字符
	while (*start && (*start == ' ' || *start == '\r' || *start == '\n'))
	{
		start++;
	}

	// 如果整个字符串都是空白字符
	if (*start == '\0')
	{
		*str = '\0';
		return str;
	}

	// 从结尾找到第一个不是空格、\r或\n的字符
	while (end > start && (*end == ' ' || *end == '\r' || *end == '\n'))
	{
		end--;
	}

	// 移动非空白字符到字符串开头
	if (start != str)
	{
		uint8_t* p = str;
		while (p <= end)
		{
			*p++ = *start++;
		}
		*p = '\0';
	}
	else
	{
		// 直接在原字符串上截断
		*(end + 1) = '\0';
	}

	return str;
}

/**
 * @brief 移除字符串中的所有空格、\r和\n字符（包括中间的空格）
 * @param str 要处理的字符串（会被修改）
 * @return 返回处理后的字符串（与输入相同指针）
 */
uint32_t trim_all(uint8_t* str)
{
	if (str == NULL)
	{
		return 0;
	}

	uint8_t* read_ptr = str;
	uint8_t* write_ptr = str;

	while (*read_ptr)
	{
		if (*read_ptr != ' ' && *read_ptr != '\r' && *read_ptr != '\n')
		{
			*write_ptr++ = *read_ptr;
		}
		read_ptr++;
	}

	*write_ptr = '\0';

	return strlen(str);
}

/**
 * @brief 测试函数
 */
int utest_string()
{
	// 测试用例
	char test1[] = "  Hello World  \r\n";
	char test2[] = "\r\n\r\n  \r\n";
	char test3[] = "NoSpacesHere";
	char test4[] = "   ";
	char test5[] = "Line1\r\nLine2\r\nLine3";

	printf("原始字符串: '%s'\n", test1);
	printf("Trim后: '%s'\n\n", trim(test1));

	printf("原始字符串: '%s'\n", test2);
	printf("Trim后: '%s'\n\n", trim(test2));

	printf("原始字符串: '%s'\n", test3);
	printf("Trim后: '%s'\n\n", trim(test3));

	printf("原始字符串: '%s'\n", test4);
	printf("Trim后: '%s'\n\n", trim(test4));

	printf("原始字符串: '%s'\n", test5);
	printf("Trim后: '%s'\n\n", trim(test5));

	// 测试trim_all函数
	char test6[] = "  Hello \r\n World  \r\n";
	printf("原始字符串: '%s'\n", test6);
	int len = trim_all(test6);
	printf("Trim_all后(%d): '%s'\n\n", len, test6);

	return 0;
}