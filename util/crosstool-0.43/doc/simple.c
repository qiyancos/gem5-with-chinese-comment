#include <stdio.h>
int main(int argc, char **argv)
{
    char buf[256];
    buf[0]=0;
    gethostname(buf, sizeof(buf));
    printf("hostname is %s\n", buf);
    printf("Exiting with status %d\n", atoi(argv[1]));
    return atoi(argv[1]);
}
