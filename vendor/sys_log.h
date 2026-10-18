#pragma once
/* minimal logging stub for linoragx build */
#include <cstdio>
#define LOG_INFO(...)  do {} while (0)
#define LOG_WARN(...)  do {} while (0)
#define LOG_ERROR(...) do { fprintf(stderr, __VA_ARGS__); fputc('\n', stderr); } while (0)
