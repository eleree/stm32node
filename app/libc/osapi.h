#ifndef _OSAPI_H_
#define _OSAPI_H_

#include <stdlib.h>
#include <string.h>

const char * os_strchr(const char *str, int c);
void * os_memset(void * s, int c, size_t n);
int os_strncmp(const char * s1, const char * s2, size_t n);
int os_strcmp(const char * s1, const char * s2);
char * os_strncpy(char *  s1, const char *  s2, size_t n);
char * os_strcpy(char * s1, const char *  s2);
char * c_strdup(const char * s1);
size_t os_strlen(const char * s);

void c_exit(int code);
void *os_memcpy(void * s1, const void *  s2, size_t n);
int os_memcmp(const void * s1,const void * s2, size_t n);
char *os_strcat(char * s1, const char *  s2);


#define c_sprintf sprintf

#endif
