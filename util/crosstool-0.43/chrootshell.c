/*--------------------------------------------------------------------------
 chrootshell.c, take 73

 This is a painfully long C replacement for a two-line shell script to
 get around some quoting problems I ran into when trying to rcp into
 the chroot jail.
 See http://www.tjw.org/chroot-login-HOWTO/ for the shell script version.
 
 Program to give specific users chroot jails for the purpose of using
 different versions of glibc than the main system.
 To be installed as /sbin/chrootshell, owned by root, setuid, 
 and registered in /etc/shells.

 For each user you want to give access to a jail, create a new /etc/passwd
 entry for each user/jail combination, with home directory set to the
 jail's top directory, and the shell set to /sbin/chrootshell, e.g.
  fred2:x:1000:1000:Fred Smith's Jail:/home/fred/jail:/sbin/chrootshell

 Only users who have the root password can set up jails, partly because
 setting up a jail requires mounting /proc inside the jail.  Users with
 the root password can set up jails on behalf of lesser users (though
 you might want to take away their ability to run setuid root programs
 if you do that, else they might be able to get out of the jail...)

 Each jail must include their own /etc/passwd, /bin/sh, shared libraries,
 and a mounted /proc.  The jail's /etc/passwd file must include an entry
 for the jail user; this entry should point to a real home directory and
 real shell, so it'll be different from the one in the system /etc/passwd.
 e.g.
  fred2:x:1000:1000:Fred Smith In Jail:/home/fred2:/bin/sh

 When a jail user logs in, chrootshell will chroot to the jail user's
 home directory (aka the jail), look up the user's home directory in
 the jail's /etc/passwd, and cd there.

 The program aborts if any of the following are true:
 1. getenv(LOGNAME) != "" && (getenv(USER) != getenv(LOGNAME))
 2. getuid() != getpwname(getenv(USER))->pw_uid
 3. jail's /etc/passwd does not contain an entry for the user

 The program probably should do more than it does, but it seems to do
 enough to let me rcp, rsh, and rlogin into the jail.

 Ideas and code snippets taken variously from 
 SVR4's login's feature whereby a "*" in the shell field of /etc/passwd triggered a chroot,
  http://www.mcsr.olemiss.edu/cgi-bin/man-cgi?login+1
 Martin P. Ibert's 1993 post,
  http://groups.google.com/groups?selm=HNLAB98U%40heaven7.in-berlin.de
 Ulf Bartelt's 1994 post,
  http://groups.google.com/groups?selm=1994Jun5.144526.9091%40solaris.rz.tu-clausthal.de
 Tony White's chroot-login-HOWTO from 2001,
  http://www.tjw.org/chroot-login-HOWTO/
 Mike Makonnen's June 2003 post,
  http://groups.google.com/groups?selm=bbeuh2%2416j0%241%40FreeBSD.csie.NCTU.edu.tw

 Portions Copyright 2003, Ixia Communications, by dkegel@ixiacom.com
 Licensed under the GPL; see http://www.gnu.org/copyleft/gpl.html
----------------------------------------------------------------*/

#include <errno.h>
#include <pwd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>

extern char **environ;

#define MAXENV 20
int myenv_used=0;
char *myenv[MAXENV+1];

/* #define DEBUG_PRINTS */

FILE *fp = NULL;

void _die(int line, const char *fmt, const void *val)
{
	int err = errno;
	fprintf(stderr, "chrootshell: ");
	fprintf(stderr, fmt, val);
#ifdef DEBUG_PRINTS
	fprintf(stderr, "; died on line %d, errno %d\n", line, err);
#endif
	if (fp) {
		fprintf(fp, "chrootshell: ");
		fprintf(fp, fmt, val);
		fprintf(fp, "; died on line %d, errno %d\n", line, err);
	}
	exit(1);
}

#define die(msg) _die(__LINE__, msg, 0)
#define die2(fmt, val) _die(__LINE__, fmt, val)

void mysetenv(const char *name, const char *value)
{
	int len;

	if (myenv_used >= MAXENV) {
		die("Environment overflow");
	}
	len = strlen(name) + strlen(value) + 2;
	myenv[myenv_used] = malloc(len);
	if (!myenv[myenv_used])
		die("Out of memory");
	sprintf(myenv[myenv_used], "%s=%s", name, value);
	myenv_used++;
}

int main(int argc, char **argv)
{
	const char *user;
	const char *logname;
	const struct passwd *pw;
	char **newargv;
	char envsh[BUFSIZ];
	char envhome[BUFSIZ];
	int i, j;
	char *neweviron[10];
	char *shell;
	char *argv0;
	char *newargv0;

#ifdef DEBUG_PRINTS
	fp = fopen("/var/log/chrootshell.log", "a");
#endif
	user = getenv("USER");
	if (!user)
		die("USER not set?!");
	logname = getenv("LOGNAME");
	if (logname && *logname && strcmp(user, logname))
		die("USER does not match LOGNAME\n");
	/* Look up user in outer /etc/passwd */
	pw = getpwnam(user);
	if (!pw)
		die2("no such user %s\n", user);
	shell = strrchr(pw->pw_shell, '/');
	if (!shell)
		die("shell contains no / ?");
	shell++;	/* skip slash */
	argv0 = argv[0];
	if (*argv0 == '-') {
		/* it's a login shell */
		argv0++;	/* skip dash */
	}
	if (strcmp(shell, argv0)) {
		fprintf(fp, "shell '%s', argv[0] '%s'\n", shell, argv[0]);
		die2("%s not chrootshell\n", shell);
	}
	/* Enter jail */
	if (chdir(pw->pw_dir))
		die2("chdir(%s) fails", pw->pw_dir);
	if (chroot(pw->pw_dir))
		die2("chroot(%s) fails", pw->pw_dir);
	/* Permanently discard root privs */
	if (setuid(pw->pw_uid))
		die2("setuid(%d) fails", (void *)pw->pw_uid);
	/* Look up user in jail's /etc/passwd */
	endpwent();
	pw = getpwnam(user);
	if (!pw)
		die2("no such user %s in jail\n", user);
	/* Go to his home directory */
	if (chdir(pw->pw_dir))
		die2("chdir(%s) fails", pw->pw_dir);

	/* Fix up the environment. 
	 * Clear the whole thing out for security reasons, and give him a minimal one.
	 */
	mysetenv("SHELL", pw->pw_shell);
	mysetenv("HOME", pw->pw_dir);
	mysetenv("PATH", "/bin:/usr/bin");
	mysetenv("USER", user);
	/* Note: rshd does not set LOGNAME, as the user hasn't really logged in... */
	if (logname && *logname)
		mysetenv("LOGNAME", user);
	myenv[myenv_used] = 0;
	/* yes, this is the posix way of replacing the entire environment */
	environ = myenv;

	/* Close the handle to the jail's /etc/passwd */
	endpwent();
	/* Finally, run the original command. */
	newargv = malloc((argc + 3) * sizeof(argv[0]));
	if (!newargv)
		die("malloc fails?!\n");
	j = 0;
	i = 1;
	if (argc == 1) {
		/* Case 1: interactive login; argv[0] is the shell, there are no args */
		char *buf = malloc(strlen(pw->pw_shell) + 2);
		newargv0 = pw->pw_shell;
		sprintf(buf, "-%s", pw->pw_shell);
		newargv[j++] = buf;
	} else if (argc > 1 && !strcmp(argv[1], "-c")) {
		/* Case 2: non-interactive; argv[0] is the shell, argv[1] is -c, argv[2] is the command */
		newargv[j++] = pw->pw_shell;
		newargv0 = pw->pw_shell;
		newargv[j++] = "-c";
		newargv[j++] = argv[2];
	} else
		die("Expected argc==1 || (argc==3 && argv[1] == '-c')");
	newargv[j++] = 0;
	execvp(newargv0, newargv);
	die2("exec %s fails", newargv[0]);
	/*notreached*/
	return 1;
}
