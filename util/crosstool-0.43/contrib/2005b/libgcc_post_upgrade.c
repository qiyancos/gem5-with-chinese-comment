#ifdef __sparc__
register void *__thread_self __asm ("g7");
#endif
#include <sys/types.h>
#include <sys/wait.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <sys/time.h>
#include <dirent.h>
#include <stddef.h>
#include <fcntl.h>
#include <string.h>

int main(void)
{
  pid_t pid;
  int status;
  char initpath[256];

  if (access ("/sbin/ldconfig", X_OK))
    _exit (0);
  execl ("/sbin/ldconfig", "/sbin/ldconfig", NULL);
  _exit (110);
}

int __libc_multiple_threads __attribute__((nocommon));
int __libc_enable_asynccancel (void) { return 0; }
void __libc_disable_asynccancel (int x) { }
void __libc_csu_init (void) { }
void __libc_csu_fini (void) { }
pid_t __fork (void) { return -1; }
char thr_buf[65536];

#ifndef __powerpc__
int __libc_start_main (int (*main) (void), int argc, char **argv,
		       void (*init) (void), void (*fini) (void),
		       void (*rtld_fini) (void), void * stack_end)
#else
struct startup_info
{
  void *sda_base;
  int (*main) (int, char **, char **, void *);
  int (*init) (int, char **, char **, void *);
  void (*fini) (void);
};

int __libc_start_main (int argc, char **ubp_av,
		       char **ubp_ev,
		       void *auxvec, void (*rtld_fini) (void),
		       struct startup_info *stinfo,
		       char **stack_on_entry)
#endif
{
#if defined __ia64__ || defined __powerpc64__
  register void *r13 __asm ("r13") = thr_buf + 32768;
  __asm ("" : : "r" (r13));
#elif defined __sparc__
  register void *g6 __asm ("g6") = thr_buf + 32768;
  __thread_self = thr_buf + 32768;
  __asm ("" : : "r" (g6), "r" (__thread_self));
#elif defined __s390__ && !defined __s390x__
  __asm ("sar %%a0,%0" : : "d" (thr_buf + 32768));
#elif defined __s390x__
  __asm ("sar %%a1,%0; srlg 0,%0,32; sar %%a0,0" : : "d" (thr_buf + 32768) : "0");
#elif defined __powerpc__ && !defined __powerpc64__
  register void *r2 __asm ("r2") = thr_buf + 32768;
  __asm ("" : : "r" (r2));
#endif
  main();
  return 0;
}
