#include <stdlib.h>
#include <string.h>
#include "osapi.h"

const char * os_strchr(const char *str, int c)
{
	return NULL;
}

void * os_memset(void * s, int c, size_t n) 
{
	return memset(s,c,n);
}

int os_strncmp(const char * s1, const char * s2, size_t n)
{
	return strncmp(s1,s2,n);
}

int os_strcmp(const char * s1, const char * s2)
{
	return  strcmp(s1,s2);
}

char * os_strncpy(char *  s1, const char *  s2, size_t n)
{
	return os_strncpy(s1, s2, n);
}

char * os_strcpy(char * s1, const char *  s2)
{
	return strcpy(s1,s2);
}

char * c_strdup(const char * s1)
{
	return NULL;
}

size_t os_strlen(const char * s)
{
	return strlen(s);
}


