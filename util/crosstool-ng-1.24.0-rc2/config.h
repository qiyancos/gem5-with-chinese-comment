/* config.h.  Generated from config.h.in by configure.  */
/* config.h.in.  Generated from configure.ac by autoheader.  */

/* Define to one of `_getb67', `GETB67', `getb67' for Cray-2 and Cray-YMP
   systems. This function is required for `alloca.c' support on those systems.
   */
/* #undef CRAY_STACKSEG_END */

/* Define to 1 if using `alloca.c'. */
/* #undef C_ALLOCA */

/* Define to 1 if translation of program messages to the user's native
   language is requested. */
#define ENABLE_NLS 1

/* Define to 1 if you have `alloca', as a function or macro. */
#define HAVE_ALLOCA 1

/* Define to 1 if you have <alloca.h> and it should be used (not on Ultrix).
   */
#define HAVE_ALLOCA_H 1

/* Define to 1 if you have the Mac OS X function CFLocaleCopyCurrent in the
   CoreFoundation framework. */
/* #undef HAVE_CFLOCALECOPYCURRENT */

/* Define to 1 if you have the Mac OS X function CFPreferencesCopyAppValue in
   the CoreFoundation framework. */
/* #undef HAVE_CFPREFERENCESCOPYAPPVALUE */

/* Define to 1 if a SysV or X/Open compatible Curses library is present */
#define HAVE_CURSES 1

/* Define to 1 if library supports color (enhanced functions) */
#define HAVE_CURSES_COLOR 1

/* Define to 1 if library supports X/Open Enhanced functions */
/* #undef HAVE_CURSES_ENHANCED */

/* Define to 1 if <curses.h> is present */
/* #undef HAVE_CURSES_H */

/* Define to 1 if library supports certain obsolete features */
#define HAVE_CURSES_OBSOLETE 1

/* Define if the GNU dcgettext() function is already present or preinstalled.
   */
#define HAVE_DCGETTEXT 1

/* Define if the GNU gettext() function is already present or preinstalled. */
#define HAVE_GETTEXT 1

/* Define if you have the iconv() function and it works. */
/* #undef HAVE_ICONV */

/* Define to 1 if you have the <inttypes.h> header file. */
#define HAVE_INTTYPES_H 1

/* Define to 1 if your system has a GNU libc compatible `malloc' function, and
   to 0 otherwise. */
#define HAVE_MALLOC 1

/* Define to 1 if you have the <memory.h> header file. */
#define HAVE_MEMORY_H 1

/* Define to 1 if the Curses Menu library is present */
#define HAVE_MENU 1

/* Define to 1 if <menu.h> is present */
#define HAVE_MENU_H 1

/* Define to 1 if the Ncurses library is present */
#define HAVE_NCURSES 1

/* Define to 1 if the NcursesW library is present */
/* #undef HAVE_NCURSESW */

/* Define to 1 if <ncursesw/curses.h> is present */
/* #undef HAVE_NCURSESW_CURSES_H */

/* Define to 1 if <ncursesw.h> is present */
/* #undef HAVE_NCURSESW_H */

/* Define to 1 if <ncursesw/menu.h> is present */
/* #undef HAVE_NCURSESW_MENU_H */

/* Define to 1 if <ncursesw/panel.h> is present */
/* #undef HAVE_NCURSESW_PANEL_H */

/* Define to 1 if <ncurses/curses.h> is present */
/* #undef HAVE_NCURSES_CURSES_H */

/* Define to 1 if <ncurses.h> is present */
#define HAVE_NCURSES_H 1

/* Define to 1 if <ncurses/menu.h> is present */
/* #undef HAVE_NCURSES_MENU_H */

/* Define to 1 if <ncurses/panel.h> is present */
/* #undef HAVE_NCURSES_PANEL_H */

/* Define to 1 if the Curses Panel library is present */
#define HAVE_PANEL 1

/* Define to 1 if <panel.h> is present */
#define HAVE_PANEL_H 1

/* Define to 1 if your system has a GNU libc compatible `realloc' function,
   and to 0 otherwise. */
#define HAVE_REALLOC 1

/* Define to 1 if you have the <stdint.h> header file. */
#define HAVE_STDINT_H 1

/* Define to 1 if you have the <stdlib.h> header file. */
#define HAVE_STDLIB_H 1

/* Define to 1 if you have the <strings.h> header file. */
#define HAVE_STRINGS_H 1

/* Define to 1 if you have the <string.h> header file. */
#define HAVE_STRING_H 1

/* Define to 1 if you have the <sys/stat.h> header file. */
#define HAVE_SYS_STAT_H 1

/* Define to 1 if you have the <sys/types.h> header file. */
#define HAVE_SYS_TYPES_H 1

/* Define to 1 if you have the <unistd.h> header file. */
#define HAVE_UNISTD_H 1

/* Name of package */
#define PACKAGE "crosstool-ng"

/* Define to the address where bug reports for this package should be sent. */
#define PACKAGE_BUGREPORT "crossgcc@sourceware.org"

/* Define to the full name of this package. */
#define PACKAGE_NAME "crosstool-NG"

/* Define to the full name and version of this package. */
#define PACKAGE_STRING "crosstool-NG 1.24.0-rc2"

/* Define to the one symbol short name of this package. */
#define PACKAGE_TARNAME "crosstool-ng"

/* Define to the home page for this package. */
#define PACKAGE_URL "http://crosstool-ng.org"

/* Define to the version of this package. */
#define PACKAGE_VERSION "1.24.0-rc2"

/* If using the C implementation of alloca, define if you know the
   direction of stack growth for your system; otherwise it will be
   automatically deduced at runtime.
	STACK_DIRECTION > 0 => grows toward higher addresses
	STACK_DIRECTION < 0 => grows toward lower addresses
	STACK_DIRECTION = 0 => direction of growth unknown */
/* #undef STACK_DIRECTION */

/* Define to 1 if you have the ANSI C header files. */
#define STDC_HEADERS 1

/* Version number of package */
#define VERSION "1.24.0-rc2"

/* Define to 1 if `lex' declares `yytext' as a `char *' by default, not a
   `char[]'. */
#define YYTEXT_POINTER 1

/* Define to `__inline__' or `__inline' if that's what the C compiler
   calls it, or to nothing if 'inline' is not supported under any name.  */
#ifndef __cplusplus
/* #undef inline */
#endif

/* Define to rpl_malloc if the replacement function should be used. */
/* #undef malloc */

/* Define to rpl_realloc if the replacement function should be used. */
/* #undef realloc */

/* Define to `unsigned int' if <sys/types.h> does not define. */
/* #undef size_t */


/* Select the correct curses/menu/panel headers */
#if defined HAVE_NCURSESW_CURSES_H
#  define CURSES_LOC <ncursesw/curses.h>
#elif defined HAVE_NCURSESW_H
#  define CURSES_LOC <ncursesw.h>
#elif defined HAVE_NCURSES_CURSES_H
#  define CURSES_LOC <ncurses/curses.h>
#elif defined HAVE_NCURSES_H
#  define CURSES_LOC <ncurses.h>
#elif defined HAVE_CURSES_H
#  define CURSES_LOC <curses.h>
#else
#  /* not an error - maybe a configuration didn't need curses */
#endif

#if defined HAVE_NCURSESW_PANEL_H
#  define PANEL_LOC <ncursesw/panel.h>
#elif defined HAVE_NCURSES_PANEL_H
#  define PANEL_LOC <ncurses/panel.h>
#elif defined HAVE_PANEL_H
#  define PANEL_LOC <panel.h>
#else
#  /* not an error */
#endif

#if defined HAVE_NCURSESW_MENU_H
#  define MENU_LOC <ncursesw/menu.h>
#elif defined HAVE_NCURSES_MENU_H
#  define MENU_LOC <ncurses/menu.h>
#elif defined HAVE_MENU_H
#  define MENU_LOC <menu.h>
#else
#  /* not an error */
#endif

