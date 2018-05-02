/*
** $Id: linit.c,v 1.14.1.1 2007/12/27 13:02:25 roberto Exp $
** Initialization of libraries for lua.c
** See Copyright Notice in lua.h
*/


#define linit_c
#define LUA_LIB
#define LUAC_CROSS_FILE

#include "lua.h"

#include "lualib.h"
#include "lauxlib.h"
#include "luaconf.h"
#include "lrotable.h"

#ifdef LUA_CROSS_COMPILER
//const luaL_Reg lua_libs[] = {{NULL, NULL}};
static const luaL_Reg lua_libs[] = {
  {"", luaopen_base},
  {LUA_LOADLIBNAME, luaopen_package},
  {LUA_TABLIBNAME, luaopen_table},
  {LUA_IOLIBNAME, luaopen_io},
  {LUA_OSLIBNAME, luaopen_os},
  {LUA_STRLIBNAME, luaopen_string},
  {LUA_MATHLIBNAME, luaopen_math},
  {LUA_DBLIBNAME, luaopen_debug},
  {NULL, NULL}
};

const luaR_table lua_rotable[] = {{NULL, NULL}};
#else 
extern const luaL_Reg lua_libs[];
#endif



void luaL_openlibs (lua_State *L) {
  const luaL_Reg *lib = lua_libs;
  for (; lib->name; lib++) {
    if (lib->func)
    {
      lua_pushcfunction(L, lib->func);
      lua_pushstring(L, lib->name);
      lua_call(L, 1, 0);
    }
  }
}

