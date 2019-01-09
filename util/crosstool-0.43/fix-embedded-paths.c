/*--------------------------------------------------------------------------
 Program to fix embedded paths in files.
 Useful especially for gcc < 3.0 and binutils < 2.14, which
 do not work if you move them after installation;
 running this program fixes the paths and lets the programs work again.
 By Clay Wood and Dan Kegel
 Copyright 2005 Google Inc. 
 Licensed under the GPL.

 Example:
 You generated a gcc/glibc/binutils toolchain using http://kegel.com/crosstool, 
 installed it to /tmp/myverylongpath, and later moved it with the command
   mv /tmp/myverylongpath /usr/crosstool
 Sadly, running gcc now fails because gcc still looks for internal files 
 that no longer exist, using absolute paths like
   /tmp/myverylongpath/gcc-3.4.3-glibc-2.3.2/lib/gcc/i686-unknown-linux-gnu/3.4.3/specs
 embedded in its binaries and data files.

 Fortunately for you, this program implements the appropriate simple
 search-and-replace on the files.  Running it with arguments
 "/tmp/myverylongpath /usr/crosstool /usr/crosstool" changes the example
 string above to
   /usr/crosstool/gcc-3.4.3-glibc-2.3.2/lib/gcc/i686-unknown-linux-gnu/3.4.3/specs

 It carefully preserves long paths and string terminators (which
 vary from file type to file type).  It detects string terminators by
 assuming that any character not in a small set known to be used as
 gcc/glibc/binutils filenames is a terminator.  This is a fragile hack,
 but in practice it seems to work for the binary and ascii files that
 make up gcc/glibc/binutils.

 Changelog: 

 Sat Apr  9 PDT 2005 Dan Kegel <dank at kegel.com>
 * Created

 Sun Jul 17 PDT 2005 Dan Kegel
 * Added optional 4th parameter to let you re-relocate
   e.g. if you relocated to a too-short path, and now
   want to relocate to a longer path (but still not
   longer than the original one)
--------------------------------------------------------------------------*/

#ifdef _WIN32
/* Under msvc2003, PATH_MAX is only defined if _POSIX_ is set */
#define _POSIX_ 1	/* for PATH_MAX */
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <limits.h>
#include <ctype.h>
/* #include <unistd.h> */
#include <sys/stat.h>

/* The first few functions are wrappers around os-specific stuff
 * Hopefully this will make it easier to port to win32
 */

/*------- fatalError --------------------------------------------------------*/

/* Display an error to the user, and abort.  
 * Use a macro so we can get line number (for debugging).
 */
#define fatalError(s, arg) do { \
	fprintf(stderr, s, arg); \
	fprintf(stderr, "Failed at line %d\n", __LINE__); \
	exit(1);\
    } while (0)

/*------- struct permissions -----------------------------------------------*/

/* Tiny class to let you make a file writable, then restore its permissions later. */

struct permissions {
    struct stat perms;
};

/* Make the given file writable, and save its old permissions state in *perms.
 */
static void permissions_makeWriteable(struct permissions *perms,
                                     const char *file_name)
{
    assert(NULL != perms);

    if (0 != stat(file_name, &perms->perms))
        fatalError("Can't get stats for %s\n", file_name);

    if ((perms->perms.st_mode & S_IWRITE) == 0) {
        /* Change the permissions to allow writes */
        if (chmod(file_name, perms->perms.st_mode | S_IWRITE) != 0)
            fatalError("Can't make writeable: %s\n", file_name);
    }
}

/* Restore the given file's permissions.
 */
static void permissions_restore(const struct permissions *perms,
                               const char *file_name)
{
    assert(NULL != perms);
    assert(NULL != file_name);

    if ((perms->perms.st_mode & S_IWRITE) == 0) {
        if (chmod(file_name, perms->perms.st_mode) != 0)
            fatalError("Could not make change permissions back: %s\n", file_name);
    }
}

/*----- struct filefinder --------------------------------------------------*/

/* Tiny class to search a directory tree for files containing a substring */

struct filefinder {
    FILE *file_list;
};

/* Start searching a directory tree for files containing the given string 
 */
static void filefinder_init(struct filefinder *finder,
                                   const char *oldroot,
                                   const char *search_path)
{
    char search_cmd[PATH_MAX * 3];

    assert(NULL != finder);
    assert(NULL != oldroot);
    assert(NULL != search_path);

    finder->file_list = NULL;

    /* Assume we have GNU grep.  (Could have used xargs and standard grep.) */
    sprintf(search_cmd, "grep -lr '%s' '%s'", oldroot, search_path);

    finder->file_list = popen(search_cmd, "r");
    if (NULL == finder->file_list)
        fatalError("Can't run %s\n", search_cmd);
}

/* Stop searching.
 */
static void filefinder_end(struct filefinder *finder)
{
    if (finder->file_list) {
        fclose(finder->file_list);
        finder->file_list = NULL;
    }
}

/* Get the path to the next matching file.
 * file_name must be PATH_MAX bytes or longer.
 * Return 1 on success, 0 if no more files
 */
static int filefinder_GetNextFile(struct filefinder *finder,
                                  char *file_name)
{
    int i;

    assert(NULL != finder);
    assert(NULL != file_name);

    if (NULL == fgets(file_name, PATH_MAX, finder->file_list))
        return 0;

    /* Strip newline */
    for (i = 0; i < PATH_MAX && file_name[i] != 0; ++i) {
        if (file_name[i] == '\n') {
            file_name[i] = 0;
            break;
        }
    }

    return 1;
}

/*----- the business end ---------------------------------------------------*/

/* Return TRUE iff c is likely to be part the name of a file or directory
 * within the gnu gcc, glibc, or binutils trees.
 * There are generally no colons or spaces in these filenames even
 * on Windows (though there may be in the old or new root prefixes,
 * which aren't tested with this function).
*/
static int isLikelyFilenameChar(char c)
{
    return isalnum(c) ||
        c == '_' || c == '.' ||
        c == '\\' || c == '/' || c == '-' || c == '+';
}

/* Replace oldroot with newroot in 'path' buffer.
 * On entry,
 *  path[0..path_len-2] contains the absolute path of some file in the old tree
 *  path[path_len-1] is the terminator for the string (NUL, space, eol, etc)
 * On exit,
 *  path has been adjusted to contain the absolute path of the same file in the new tree
 *  followed by the terminator.  Any extra space after the terminator
 *  is filled with ascii 0x20 (space).
 * Returns number of chars to write
 */
static int
replaceOneString(char *path,
                 unsigned int path_len,
                 const char *newroot,
                 unsigned int newroot_len,
                 unsigned int oldroot_len)
{
    assert(oldroot_len <= path_len);
    assert(NULL != path);
    assert(NULL != newroot);

    if (newroot_len < oldroot_len) {
        unsigned int delta = oldroot_len - newroot_len;

        /* Overwrite the beginning of the path with the new root, padded with
         * leading slashes.
         */
        memset(path, '/', delta);
        memcpy(path + delta, newroot, newroot_len);

        return path_len;
    } else {
        unsigned int delta = newroot_len - oldroot_len;

        /* Scoot path (and its terminator!) up to fit */
        memmove(path + delta, path, path_len + 1); 

        /* Overwrite the beginning of the path with the new root */
        memcpy(path, newroot, newroot_len);

        return path_len + delta;
    }
}

/* Opens the give file, searches for all occurences of oldroot,
 * replaces them with newroot, carefully scooting the following
 * string (if any) and terminator down to fit.
 */
static void ReplaceStringsInOneFile(const char *file_name,
                                   const char *oldroot,
                                   const char *newroot,
                                   const int old_oldroot_len)
{
    FILE *file = NULL;
    char buffer[PATH_MAX * 2];
    const int newroot_len = strlen(newroot);
    const int oldroot_len = strlen(oldroot);
    int nmatched;
    int c;

    assert(old_oldroot_len < PATH_MAX);
    assert(NULL != file_name);
    assert(NULL != oldroot);
    assert(NULL != newroot);

    file = fopen(file_name, "rb+");
    if (NULL == file)
        fatalError("Could not open file %s for update\n", file_name);

    nmatched = 0;
    /* Match the old root name.  Whenever a full match is detected, overwrite. */
    while (EOF != (c = fgetc(file))) {
        if (nmatched < oldroot_len) {
            /* See if we can match the next character */
            if (c == oldroot[nmatched]) {
		/* Yes.  Save it and move on. */
                buffer[nmatched++] = c;
            } else {
                /* No.  Start search again one past where we started last time. */
                fseek(file, -nmatched, SEEK_CUR);
                nmatched = 0;
	    }
        } else {
	    int nwrite;

	    /* No matter what, we want to save this char. */
            buffer[nmatched++] = c;

            /* Is it a terminating character? */
            if (!isLikelyFilenameChar(c)) {

                /* Yes. We've found a match.  Modify the buffer. */
                nwrite = replaceOneString(buffer,
                                 nmatched,
                                 newroot,
                                 newroot_len,
                                 oldroot_len);

                /* Rewind file to start of matched path and write modified version. */
                fseek(file, -nmatched, SEEK_CUR);
                if (nwrite != fwrite(buffer, 1, nwrite, file))
                    fatalError("Error writing to %s ?!\n", file_name);
                /* File is now past end of modified version of matched path. */

                nmatched = 0;
            }
        }
    }

    if (fclose(file))
	fatalError("Error flushing to %s ?!\n", file_name);
}

/* This method takes a source and replacement string and searches
 * through all files under the given path to see which files the
 * source string exists.  It then replaces that string with the replacement
 * string.
 * Returns number of files changed.
 */
int ReplaceStringsInMatchingFiles(const char *oldroot,
                                  const char *newroot,
                                  const char *path,
                                  int old_oldroot_len)
{
    char buffer[PATH_MAX * 2];
    int nchanges;

    struct filefinder finder;

    if (old_oldroot_len < strlen(newroot) )
        fatalError("Error: newroot %s must be shorter or same length as oldroot.\n", newroot);

    filefinder_init(&finder, oldroot, path);

    nchanges = 0;
    while (filefinder_GetNextFile(&finder, buffer)) {
        struct permissions oldperms;
        permissions_makeWriteable(&oldperms, buffer);
        ReplaceStringsInOneFile(buffer, oldroot, newroot, old_oldroot_len);
        permissions_restore(&oldperms, buffer);
        nchanges++;
    }

    filefinder_end(&finder);

    return nchanges;
}

/*------- unit test -----------------------------------------------------------*/

void hexdump(char *buf, int len)
{
	int i;
	for (i=0; i<len; i++)
		printf("%c", isprint(buf[i]) ? buf[i] : '_');
	printf("\n");
}

#define MAXTESTLEN 256

void testit()
{
    struct mytest {
	const char *oldroot;
	const char *newroot;
	const char *oldfile;   /* underscores represent nul */
	const char *newfile;
        int old_oldrootlen;    /* 0 to use strlen(oldroot) */
    };
    static struct mytest all_tests[] = {
        /* normal runs */
	{"/oldpath", "/boo",     "/oldpath/bletch'asdf", "/////boo/bletch'asdf", 0},
	{"/oldpath", "/boo",     "/oldpath/bletch_asdf", "/////boo/bletch_asdf", 0},

        /* make sure we can use the old_oldpath_len argument to map to a *longer* path */
	{"/boo",     "/oldpath", "/boo/bletch_    asdf", "/oldpath/bletch_asdf", strlen("/oldpath")},
	{NULL, NULL, NULL, NULL, 0}
    };

    int i;
    for (i=0; all_tests[i].oldfile; i++) {
	FILE *fp;
	struct mytest *p = all_tests+i;
	int filelen = strlen(p->oldfile);
	char *s;
	char buf[MAXTESTLEN];
	char oldfile[MAXTESTLEN];
	char newfile[MAXTESTLEN];

	if (strlen(p->newfile) != filelen)
		fatalError("Bad test %d\n", i);

        /* Turn the underscores to nul's */
	memcpy(oldfile, p->oldfile, filelen+1);
	for (s=oldfile; *s != 0; s++)
	    if (*s == '_')
	        *s = 0;
	memcpy(newfile, p->newfile, filelen+1);
	for (s=newfile; *s != 0; s++)
	    if (*s == '_')
	        *s = 0;

	remove("test-reembed.dat");
	fp = fopen("test-reembed.dat", "wb");
	if (!fp)
	    fatalError("can't create test-reembed.dat at test %d\n", i);
	fwrite(oldfile, 1, filelen, fp);
	fclose(fp);

	ReplaceStringsInOneFile("test-reembed.dat", p->oldroot, p->newroot, (p->old_oldrootlen ? p->old_oldrootlen : strlen(p->oldroot)));
        
	fp = fopen("test-reembed.dat", "rb");
	if (!fp)
	    fatalError("can't open test-reembed.dat at test %d\n", i);
	if (filelen != fread(buf, 1, filelen, fp))
	    fatalError("can't read test-reembed.dat at test %d\n", i);
	fclose(fp);
#if 0
	printf("expect: "); hexdump(newfile, filelen);
	printf("   got: "); hexdump(buf, filelen);
#endif
	if (memcmp(buf, newfile, filelen))
	    fatalError("test %d compare failed\n", i);

	remove("test-reembed.dat");
    }
    printf(__FILE__ " test passed\n");
}

/*------- main ---------------------------------------------------------------*/

int main(int argc, char **argv)
{
    int nchanged;
    int old_oldroot_len;

    if (argc == 2 && strcmp(argv[1], "--test") == 0) {
	testit();
	exit(0);
    }

    if (argc < 4)
        fatalError("Usage: %s <old root> <new root> <directoryToSearch> [<old_oldrootlen>]\n", argv[0]);

    old_oldroot_len = strlen(argv[1]);
    if (argc == 5)
        old_oldroot_len = atoi(argv[4]);	/* only if it was previously relocated from a loooong path */

    nchanged = ReplaceStringsInMatchingFiles(argv[1], argv[2], argv[3], old_oldroot_len);

    if (nchanged > 0) {
        printf("%d files updated.\n", nchanged);
        return 0;
    }
    printf("No files changed!\n");
    return 1;
}
