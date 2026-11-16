#ifndef PORT_H
#define PORT_H

#include <cstdio>

#ifndef _WIN32
#include <strings.h>
#include <cstring>
#ifndef stricmp
#define stricmp strcasecmp
#endif
#ifndef _snprintf
#define _snprintf snprintf
#endif
#ifndef _MAX_EXT
#define _MAX_EXT 256
#endif
inline void _splitpath(const char* path, char*, char*, char*, char* ext)
{
	ext[0] = 0;
	if (!path)
		return;
	const char* dot = strrchr(path, '.');
	const char* slash = strrchr(path, '/');
	if (dot && dot != path && (!slash || dot > slash)) {
		strncpy(ext, dot, _MAX_EXT - 1);
		ext[_MAX_EXT - 1] = 0;
	}
}

typedef unsigned int DWORD;
#ifndef COLOR_BTNFACE
#define COLOR_BTNFACE 15
#endif
inline DWORD GetSysColor(int) { return 0x00d4d0c8; }
inline int GetRValue(DWORD c) { return (int)(c & 0xff); }
inline int GetGValue(DWORD c) { return (int)((c >> 8) & 0xff); }
inline int GetBValue(DWORD c) { return (int)((c >> 16) & 0xff); }
#endif

#endif
