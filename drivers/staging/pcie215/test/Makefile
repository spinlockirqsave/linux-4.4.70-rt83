CC = gcc
CFLAGS	=
CFLAGS_RELEASE = -c -o3 -DNDEBUG -Wall -Wextra -std=c99 -pedantic -Wformat -Wno-unused-function -Wfatal-errors -Werror
CFLAGS_DEBUG = -c -g3 -O0 -DDEBUG -Wall -Wextra -std=c99 -pedantic -Wformat -Wno-unused-function -Wfatal-errors -Werror
CPPFLAGS += #compiler flags
LDFLAGS = #-lpcap
LDFLAGS_D = -g #-lpcap
LDLIBS := -pthread
INCLUDES = -I.

all:	pcie215_ioctl pcie215_read pcie215_write
debug:	pcie215_ioctl-debug pcie215_read-debug pcie215_write-debug

pcie215_ioctl-debug:		CFLAGS += $(CFLAGS_DEBUG)
pcie215_ioctl-debug:		pcie215_ioctl.c pcie215_ioctl-debug.o

pcie215_ioctl:	CFLAGS += $(CFLAGS_RELEASE)
pcie215_ioctl: 	pcie215_ioctl.c pcie215_ioctl.o

pcie215_read-debug:		CFLAGS += $(CFLAGS_DEBUG)
pcie215_read-debug:		pcie215_read.c pcie215_read-debug.o

pcie215_read:	CFLAGS += $(CFLAGS_RELEASE)
pcie215_read: 	pcie215_read.c pcie215_read.o

pcie215_write-debug:		CFLAGS += $(CFLAGS_DEBUG)
pcie215_write-debug:		pcie215_write.c pcie215_write-debug.o

pcie215_write:	CFLAGS += $(CFLAGS_RELEASE)
pcie215_write: 	pcie215_write.c pcie215_write.o

install:	all
	cp pcie215_ioctl /usr/bin
	cp pcie215_read /usr/bin
	cp pcie215_write /usr/bin


# ioctl
pcie215_ioctl-debug: pcie215_ioctl-debug.o 
	$(CC) $(LDFLAGS_D) pcie215_ioctl-debug.o -o $@ $(LDLIBS)

pcie215_ioctl: pcie215_ioctl.o 
	$(CC) $(LDFLAGS) pcie215_ioctl.o -o $@ $(LDLIBS)

pcie215_ioctl-debug.o: pcie215_ioctl.c
	$(CC) $(CFLAGS) $(INCLUDES) $< -o $@ $(LDLIBS)

pcie215_ioctl.o: pcie215_ioctl.c
	$(CC) $(CFLAGS) $(INCLUDES) $< -o $@ $(LDLIBS)


# read
pcie215_read-debug: pcie215_read-debug.o 
	$(CC) $(LDFLAGS_D) pcie215_read-debug.o -o $@ $(LDLIBS)

pcie215_read: pcie215_read.o 
	$(CC) $(LDFLAGS) pcie215_read.o -o $@ $(LDLIBS)

pcie215_read-debug.o: pcie215_read.c
	$(CC) $(CFLAGS) $(INCLUDES) $< -o $@ $(LDLIBS)

pcie215_read.o: pcie215_read.c
	$(CC) $(CFLAGS) $(INCLUDES) $< -o $@ $(LDLIBS)

# write
pcie215_write-debug: pcie215_write-debug.o 
	$(CC) $(LDFLAGS_D) pcie215_write-debug.o -o $@ $(LDLIBS)

pcie215_write: pcie215_write.o 
	$(CC) $(LDFLAGS) pcie215_write.o -o $@ $(LDLIBS)

pcie215_write-debug.o: pcie215_write.c
	$(CC) $(CFLAGS) $(INCLUDES) $< -o $@ $(LDLIBS)

pcie215_write.o: pcie215_write.c
	$(CC) $(CFLAGS) $(INCLUDES) $< -o $@ $(LDLIBS)

.c.o:
	$(CC) $(CFLAGS) $(INCLUDES) $< -o $@ $(LDLIBS)

.PHONY: clean
clean:
	rm -rf pcie215_ioctl-debug pcie215_read-debug pcie215_write-debug pcie215_ioctl pcie215_read pcie215_write *.o

.PHONY: clean-all
clean-all:
	make clean
