#include <stdio.h>
#include <stdlib.h>

#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <sys/types.h>
#include <signal.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <sys/mman.h>
#include <getopt.h>
#include <sys/time.h>

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
    fprintf(stderr, "     \t \tres = read(fd, NULL, 0);\n");
	fprintf(stderr, "     \t and will be blocked waiting for the interrupt on any\n");
	fprintf(stderr, "     \t of the enabled pins (use pcie215_ioctl program to configure them).\n");
    fprintf(stderr, "\n");
	exit(EXIT_FAILURE);
}

static void tmrAOs(int sig)
{
    fprintf(stderr, "Signal delivered [%d] [%s]\n", sig, strsignal(sig));
}

void initTimerSignalHandler()
{
    struct sigaction sSigAction;

    sigemptyset(&sSigAction.sa_mask);
    sSigAction.sa_flags = SA_NODEFER;
    sSigAction.sa_handler = &tmrAOs;
    sigaction(SIGALRM, &sSigAction, NULL);
}


int main(int argc, char *argv[])
{
	int		c = 0;
	int		fd = -1;	/* character device file descriptor */
	char buf[10] = { 0 };

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

    /* setup SIGALRM handler */
    initTimerSignalHandler();

	fd = open(PCIE215, O_RDWR);
	if (fd < 0) {
		fprintf(stderr, "Err, can't open [%s]\n", PCIE215);
		exit(EXIT_FAILURE);
	}

	/*
	 * Use pcie215_ioctl program to enable IRQ generation and enabling triggers
	 * To enable interrupts generation
	 *		res = ioctl(fd, PCIE215_IOCTL_IRQ_ENABLE, 1);
	 * To specify enabled triggers
	 *		res = ioctl(fd, PCIE215_IOCTL_IRQ_TRIGGERS_ENABLE, arg);
	 */
	res = read(fd, NULL, 0);
	if (res) {
        if (errno == EINTR)
            fprintf(stderr, "Err, interrupted by signal, read [%d] errno [%d][%s]\n", res, errno, strerror(errno));
        else
            fprintf(stderr, "Err, read failed [%d] [%s]\n", res, strerror(errno));
        exit(EXIT_FAILURE);
	}

	fprintf(stderr, "OK, read [%d] buf[0]=[%02x] buf[1]=[%02x]\n", res, buf[0], buf[1]);

	return EXIT_SUCCESS;
}
