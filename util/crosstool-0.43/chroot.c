/* In case your system doesn't have the chroot program,
 * here's a trivial (and pretty awful) implementation.
 * Copyright abandoned, 2004, Dan Kegel.  No rights reserved.
 */
#include <stdio.h>
#include <unistd.h>

main(int argc, char **argv)
{
	if (chdir(argv[1])) {
		perror("chdir");
		exit(1);
	}
	if (chroot(".")) {
		perror("chroot");
		exit(1);
	}
	if (argv[2]) {
		execvp(argv[2], &argv[2]);
		perror("exec argv1");
		exit(1);
	}

	execlp("/bin/sh", "/bin/sh", "-i", NULL);
	perror("exec sh");
	exit(1);
}
