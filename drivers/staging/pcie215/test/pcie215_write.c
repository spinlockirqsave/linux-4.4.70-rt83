#include <stdio.h>
#include <stdlib.h>

#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <sys/mman.h>
#include <getopt.h>

#include <ctype.h>              /* isprint */

/* ioctl */
#include "../pcie215_ioctl.h"


#define PCIE215	"/dev/pcie215"	/* character device file */


void usage(char *name)
{
    if (name == NULL)
        return;

    fprintf(stderr, "\nusage:\t %s --help\n\n", name);
    fprintf(stderr, "     \t No arguments. Program will call \n");
    fprintf(stderr, "     \t \tfd = open(PCIE215, O_RDWR);\n");
    fprintf(stderr, "     \t \tres = write(fd, 1 byte of 0x01);\n");
    fprintf(stderr, "\n");
	exit(EXIT_FAILURE);
}

int main(int argc, char *argv[])
{
	int		c = 0;
	int		fd = -1;	/* character device file descriptor */

	int res = -1;

    if (argc > 1)
		usage(argv[0]);

    int option_index = 0;
    struct option long_options[] = {
        { "help", no_argument, 0, 1 },
        { 0 }
    };


    while ((c = getopt_long(argc, argv, "", long_options, &option_index)) != -1) {		/* parse optional command line arguments */

        switch (c) {

            case 1:
                usage(argv[0]);															/* --help */
                break;

            case '?':
                if (optopt == 'r')
                    fprintf(stderr, "Option -%c requires an argument.\n", optopt);
                else if (isprint(optopt))
                    fprintf(stderr,"Unknown option '-%c'.\n", optopt);
                else {
                    fprintf(stderr, "Are there any long options? "
                            "Please check that you have typed them correctly.\n");
                }
                usage(argv[0]);

            default:
                exit(EXIT_FAILURE);
        }
    }

	fd = open(PCIE215, O_RDWR);
	if (fd < 0) {
		fprintf(stderr, "Err, can't open [%s]\n", PCIE215);
		exit(EXIT_FAILURE);
	}

	unsigned char b = 0x1;
	res = write(fd, &b, 1);
	if (res < 0) {
		fprintf(stderr, "Err, write failed [%d]\n", res);
		exit(EXIT_FAILURE);
	}
	fprintf(stderr, "OK, [%d] written [%02x]\n", res, b);

	return EXIT_SUCCESS;
}
