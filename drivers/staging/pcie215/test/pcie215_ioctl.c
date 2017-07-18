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

    fprintf(stderr, "\nusage:\t %s [-i ioctl] [-a argument] --help\n\n", name);
    fprintf(stderr, "     \t -i           : I/O call to perform\n");
	fprintf(stderr, "     \t                where\n");
    fprintf(stderr, "     \t                -i 0         : PCIE215_IOCTL_IRQ_ENABLE (argument mandatory)\n");
    fprintf(stderr, "     \t                                   -i 0 -a 0  Disable IRQ triggering\n");
    fprintf(stderr, "     \t                                   -i 0 -a 1  Enable IRQ triggering\n");
    fprintf(stderr, "     \t                -i 1         : PCIE215_IOCTL_IRQ_TRIGGERS_ENABLE, mandatory argument\n");
    fprintf(stderr, "     \t                               is a flag with enabled IRQ sources set (0x00 - 0x3F).\n");
    fprintf(stderr, "     \t                               This will clear and set new mask.\n");
    fprintf(stderr, "     \t                               To enable i-th trigger - set bit i-th to 1.\n");
    fprintf(stderr, "     \t                               Example:\n");
    fprintf(stderr, "     \t                                   -i 1 -a 1  Enable PPI X C0\n");
    fprintf(stderr, "     \t                                   -i 1 -a 2  Enable PPI X C3\n");
    fprintf(stderr, "     \t                                   -i 1 -a 4  Enable PPI Y C0\n");
    fprintf(stderr, "     \t                                   -i 1 -a 8  Enable PPI Y C3\n");
    fprintf(stderr, "     \t                                   -i 1 -a 16 Enable CTRZ1 OUT1 O/P\n");
    fprintf(stderr, "     \t                                   -i 1 -a 32 Enable CTRZ2 OUT1 O/P\n");
    fprintf(stderr, "     \t                -i 2         : PCIE215_IOCTL_IRQ_TRIGGERS_DISABLE (argument mandatory)\n");
    fprintf(stderr, "     \t                               This will clear specified triggers from current mask.\n");
    fprintf(stderr, "     \t -a           : argument to ioctl\n");
    fprintf(stderr, "\n");
	exit(EXIT_FAILURE);
}

int main(int argc, char *argv[])
{
	int		c = 0;
	int		fd = -1;	/* character device file descriptor */
	char                *pCh = NULL;
	unsigned long long  helper = 0;

	int				io = -1;			/* ioctl number */
	unsigned long	arg = 0;			/* ioctl argument */
	int res = -1;

    if (argc < 2)
		usage(argv[0]);

    int option_index = 0;
    struct option long_options[] = {
        { "help", no_argument, 0, 1 },
        { 0 }
    };


    while ((c = getopt_long(argc, argv, "i:a:", long_options, &option_index)) != -1) {		/* parse optional command line arguments */

        switch (c) {

            case 1:
                usage(argv[0]);															/* --help */
                break;

            case 'i':
                helper = strtoul(optarg, &pCh, 10);
                if (helper > 2) {
                    fprintf(stderr, "Err, nonexistent ioctl number\n");
                    exit(EXIT_FAILURE);
                }
                if ((pCh == optarg) || (*pCh != '\0')) {								/* check */
                    fprintf(stderr, "Invalid argument\n");
                    fprintf(stderr, "Parameter conversion error, nonconvertible part is: [%s]\n", pCh);
                    exit(EXIT_FAILURE);
                }

                io = helper;
                break;

            case 'a':
                helper = strtoul(optarg, &pCh, 10);
                if (helper > 0x100000000 - 1) {
                    fprintf(stderr, "Err, range error\n");
                    exit(EXIT_FAILURE);
                }
                if ((pCh == optarg) || (*pCh != '\0')) {								/* check */
                    fprintf(stderr, "Invalid argument\n");
                    fprintf(stderr, "Parameter conversion error, nonconvertible part is: [%s]\n", pCh);
                    exit(EXIT_FAILURE);
                }

                arg = helper;
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
				fprintf(stderr, "Err, getopt\n");
                exit(EXIT_FAILURE);
        }
    }

	if (io == -1) {
		fprintf(stderr, "Err, please give I/O call number as -i argument\n");
		exit(EXIT_FAILURE);
	}

	fd = open(PCIE215, O_RDWR);
	if (fd < 0) {
		fprintf(stderr, "Err, can't open [%s]\n", PCIE215);
		exit(EXIT_FAILURE);
	}

	switch (io) {

		case 0:
			if (arg > 1) {
				fprintf(stderr, "Err, bad argument to PCIE215_IOCTL_IRQ_ENABLE [%ld]\n", arg);
				exit(EXIT_FAILURE);
			}
			res = ioctl(fd, PCIE215_IOCTL_IRQ_ENABLE, arg);
			if (res < 0) {
				fprintf(stderr, "Err, ioctl failed [%d]\n", res);
				exit(EXIT_FAILURE);
			}
			fprintf(stderr, "OK, PCIE215_IOCTL_IRQ_ENABLE [%16lx] [%ld]\n", PCIE215_IOCTL_IRQ_ENABLE, arg);
			break;

		case 1:
			if (arg > 0x3F) {
				fprintf(stderr, "Err, bad argument to PCIE215_IOCTL_IRQ_TRIGGERS_ENABLE [%ld]\n", arg);
				exit(EXIT_FAILURE);
			}
			res = ioctl(fd, PCIE215_IOCTL_IRQ_TRIGGERS_ENABLE, arg);
			if (res < 0) {
				fprintf(stderr, "Err, ioctl failed [%d]\n", res);
				exit(EXIT_FAILURE);
			}
			fprintf(stderr, "OK, PCIE215_IOCTL_IRQ_TRIGGERS_ENABLE [%16lx] [%ld]\n", PCIE215_IOCTL_IRQ_TRIGGERS_ENABLE, arg);
			break;

		case 2:
			if (arg > 0x3F) {
				fprintf(stderr, "Err, bad argument to PCIE215_IOCTL_IRQ_TRIGGERS_DISABLE [%ld]\n", arg);
				exit(EXIT_FAILURE);
			}
			res = ioctl(fd, PCIE215_IOCTL_IRQ_TRIGGERS_DISABLE, arg);
			if (res < 0) {
				fprintf(stderr, "Err, ioctl failed [%d]\n", res);
				exit(EXIT_FAILURE);
			}
			fprintf(stderr, "OK, PCIE215_IOCTL_IRQ_TRIGGERS_DISABLE [%16lx] [%ld]\n", PCIE215_IOCTL_IRQ_TRIGGERS_DISABLE, arg);
			break;

		default:
			fprintf(stderr, "Err, wrong I/O call number [%lu]\n", arg);
			exit(EXIT_FAILURE);
	}

	return EXIT_SUCCESS;
}
