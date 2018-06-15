#if !defined(LOGOUT_H)
#define LOGOUT_H

#include <stdio.h>
#include <stdarg.h>

#ifdef _WINDOWS
#include <process.h>
#endif	
extern FILE* logstr;

#ifdef _WINDOWS 
    #include <direct.h> 
    #define GetCurrentDir _getcwd 
#else 
    #include <unistd.h> 
    #define GetCurrentDir getcwd 
 #endif 

inline char* GET_LOGOUT_DIR()
{ 
	static char cCurrentPath[FILENAME_MAX]; 
	char* logoutdir=cCurrentPath;
 
	if (!GetCurrentDir(cCurrentPath, sizeof(cCurrentPath))) 
     { 
		 return NULL; 
     } 
 
	cCurrentPath[sizeof(cCurrentPath) - 1] = '\0'; /* not really required */ 
 
//	printf ("The current working directory is %s", cCurrentPath); 

	return logoutdir;
}

///WARNING: Having this enabled prevent from loading the plugin in MAX in some systems.
//#define LOGOUT_ENABLED

#ifdef LOGOUT_ENABLED
#pragma message("WARNNING:***** LOG FILE enabled **************")

inline bool ISLOGOUTFILE()
{
	if( logstr)return true;	

    return false;
}
inline bool LOGOUT_START( FILE * logfile)
{
#ifdef _WINDOWS	
	fopen_s(&logstr,"E:\\temp\\log.out", "w" );
#else
	logstr = fopen("log.out", "w" );
#endif
	if( logstr)return true;

	return false;
}
inline void LOGOUT( FILE * logfile, const char * str, ...)
{
	char buffer[256];
	va_list vap;
	va_start( vap, str );

#ifdef _WINDOWS
	vsprintf_s( buffer, str, vap );
#else
	vsprintf( buffer, str, vap );
#endif

	fprintf( logfile, buffer);

	va_end( vap );

	fflush( logstr);

}
inline void LOGOUT_END( FILE * logfile)
{
	if(logstr)
	{
		fflush( logstr);
		fclose(logstr);
	}
}

#else
inline bool ISLOGOUTFILE()
{
    return false;
}

inline bool LOGOUT_START( FILE * logfile)
{
	return false;
}
inline void LOGOUT( FILE * logfile, const char * str, ...)
{
}
inline void LOGOUT_END( FILE * logfile)
{
}
#endif

#endif
