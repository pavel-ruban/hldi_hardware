#
#
DESTDIR =
VPATH = ../../../.././libgloss/arm ../../../.././libgloss/arm/..
srcdir = ../../../.././libgloss/arm
objdir = .
srcroot = $(srcdir)/../..
objroot = $(objdir)/../..

prefix = /usr/local
exec_prefix = ${prefix}

host_alias = arm-none-eabi
target_alias = arm-none-eabi

bindir = ${exec_prefix}/bin
libdir = ${exec_prefix}/lib
tooldir = $(exec_prefix)/$(target_alias)

objtype = elf-

INSTALL = /usr/bin/install -c
INSTALL_PROGRAM = /usr/bin/install -c
INSTALL_DATA = /usr/bin/install -c -m 644

# Multilib support variables.
# TOP is used instead of MULTI{BUILD,SRC}TOP.
MULTISRCTOP = 
MULTIBUILDTOP = ../
MULTIDIRS = 
MULTISUBDIR = /armv6-m
MULTIDO = true
MULTICLEAN = true

SHELL =	/bin/sh

CC = arm-none-eabi-gcc -B/sources/newlib/newlib-1.18.0/arm-none-eabi/armv6-m/newlib/ -isystem /sources/newlib/newlib-1.18.0/arm-none-eabi/armv6-m/newlib/targ-include -isystem /sources/newlib/newlib-1.18.0/newlib/libc/include -B/sources/newlib/newlib-1.18.0/arm-none-eabi/armv6-m/libgloss/arm -L/sources/newlib/newlib-1.18.0/arm-none-eabi/armv6-m/libgloss/libnosys -L/sources/newlib/newlib-1.18.0/libgloss/arm  -mthumb -march=armv6s-m

AS = arm-none-eabi-as
AR = arm-none-eabi-ar
LD = arm-none-eabi-ld
RANLIB = arm-none-eabi-ranlib

OBJDUMP = `if [ -f ${objroot}/../binutils/objdump ] ; \
	then echo ${objroot}/../binutils/objdump ; \
	else t='$(program_transform_name)'; echo objdump | sed -e $$t ; fi`
OBJCOPY = `if [ -f ${objroot}/../binutils/objcopy ] ; \
	then echo ${objroot}/../binutils/objcopy ; \
	else t='$(program_transform_name)'; echo objcopy | sed -e $$t ; fi`

CRT0		= crt0.o
CRT0_INSTALL	= install-crt0
#CRT0		= 
#CRT0_INSTALL	= 

LINUX_CRT0    = linux-crt0.o
LINUX_BSP     = libgloss-linux.a
LINUX_OBJS    = linux-syscalls0.o linux-syscalls1.o
LINUX_SCRIPTS = linux.specs
LINUX_INSTALL = install-linux

REDBOOT_CRT0	= redboot-crt0.o 
REDBOOT_OBJS	= redboot-syscalls.o
REDBOOT_SCRIPTS	= redboot.ld redboot.specs
REDBOOT_INSTALL	= install-redboot

RDPMON_CRT0	= rdpmon-crt0.o
RDPMON_BSP	= librdpmon.a
RDPMON_OBJS	= syscalls.o libcfunc.o trap.o _exit.o _kill.o
RDPMON_SCRIPTS	= rdpmon.specs
RDPMON_INSTALL	= install-rdpmon

RDIMON_CRT0	= rdimon-crt0.o
RDIMON_BSP	= librdimon.a
RDIMON_OBJS	= $(patsubst %,rdimon-%,$(RDPMON_OBJS))
RDIMON_SCRIPTS	= rdimon.specs
RDIMON_INSTALL	= install-rdimon

CFLAGS		= -g

# Here is all of the eval board stuff
PID_SCRIPTS	= pid.specs
PID_INSTALL	= install-pid

IQ80310_SCRIPTS	= iq80310.specs
IQ80310_INSTALL	= install-iq80310


# Host specific makefile fragment comes in here.
NEWLIB_CFLAGS = `if [ -d ${objroot}/newlib ]; then echo -I${objroot}/newlib/targ-include -I${srcroot}/newlib/libc/include; fi`
NEWLIB_LDFLAGS = `if [ -d ${objroot}/newlib ]; then echo -B${objroot}/newlib/ -L${objroot}/newlib/; fi`

INCLUDES = -I. -I$(srcdir)/..
# Note that when building the library, ${MULTILIB} is not the way multilib
# options are passed; they're passed in $(CFLAGS).
CFLAGS_FOR_TARGET = -O2 -g ${MULTILIB} ${INCLUDES} ${NEWLIB_CFLAGS}
LDFLAGS_FOR_TARGET = ${MULTILIB} ${NEWLIB_LDFLAGS}
AR_FLAGS = qc

.c.o:
	$(CC) $(CFLAGS_FOR_TARGET) -O2 $(INCLUDES) -c $(CFLAGS) $<

.C.o:
	$(CC) $(CFLAGS_FOR_TARGET) -O2 $(INCLUDES) -c $(CFLAGS) $<
.s.o:
	$(AS) $(ASFLAGS_FOR_TARGET) $(INCLUDES) $(ASFLAGS) -o $*.o $<

#
# GCC knows to run the preprocessor on .S files before it assembles them.
#
.S.o:
	$(CC) $(CFLAGS_FOR_TARGET) $(INCLUDES) $(CFLAGS) -c $<

#
# this is a bogus target that'll produce an assembler from the
# C source with the right compiler options. this is so we can
# track down code generation or debug symbol bugs.
#
.c.s:
	$(CC) $(CFLAGS_FOR_TARGET) -S $(INCLUDES) $(CFLAGS) $<

#
# build a test program for each target board. Just trying to get
# it to link is a good test, so we ignore all the errors for now.
#
all: ${CRT0} ${LINUX_CRT0} ${LINUX_BSP} ${REDBOOT_CRT0} ${REDBOOT_OBJS} ${RDPMON_CRT0} ${RDPMON_BSP} ${RDIMON_CRT0} ${RDIMON_BSP}

#
# here's where we build the test programs for each target
#
.PHONY: test
test:

#
crt0.o: crt0.S
redboot-crt0.o: redboot-crt0.S
redboot-syscalls.o: redboot-syscalls.c $(srcdir)/../syscall.h

rdpmon-crt0.o: crt0.S
	$(CC) $(CFLAGS_FOR_TARGET) $(INCLUDES) -DARM_RDP_MONITOR -o $@ -c $<

rdimon-crt0.o: crt0.S
	$(CC) $(CFLAGS_FOR_TARGET) $(INCLUDES) -DARM_RDI_MONITOR -o $@ -c $<

rdimon-trap.o: trap.S
	$(CC) $(CFLAGS_FOR_TARGET) $(INCLUDES) -DARM_RDI_MONITOR -o $@ -c $<

rdimon-_exit.o: _exit.c
	$(CC) $(CFLAGS_FOR_TARGET) $(INCLUDES) -DARM_RDI_MONITOR -o $@ -c $<

rdimon-_kill.o: _kill.c
	$(CC) $(CFLAGS_FOR_TARGET) $(INCLUDES) -DARM_RDI_MONITOR -o $@ -c $<

rdimon-syscalls.o: syscalls.c
	$(CC) $(CFLAGS_FOR_TARGET) $(INCLUDES) -DARM_RDI_MONITOR -o $@ -c $<

rdimon-libcfunc.o: libcfunc.c
	$(CC) $(CFLAGS_FOR_TARGET) $(INCLUDES) -DARM_RDI_MONITOR -o $@ -c $<

$(LINUX_BSP): $(LINUX_OBJS)
	${AR} ${ARFLAGS} $@ $^
	${RANLIB} $@

$(RDPMON_BSP): $(RDPMON_OBJS)
	${AR} ${ARFLAGS} $@ $^
	${RANLIB} $@

$(RDIMON_BSP): $(RDIMON_OBJS)
	${AR} ${ARFLAGS} $@ $^
	${RANLIB} $@

clean mostlyclean:
	rm -f a.out core *.i *.o *-test *.srec *.dis *.x $(SIM_BSP) $(IQ80310_BSP)

distclean maintainer-clean realclean: clean
	rm -f Makefile config.status *~

.PHONY: install info install-info clean-info
install: ${CRT0_INSTALL} ${LINUX_INSTALL} ${REDBOOT_INSTALL} ${RDPMON_INSTALL} ${RDIMON_INSTALL} ${IQ80310_INSTALL}  ${PID_INSTALL}

install-crt0:
	${INSTALL_DATA} ${CRT0} $(DESTDIR)${tooldir}/lib${MULTISUBDIR}/$$x

install-linux:
	set -e; for x in ${LINUX_CRT0} ${LINUX_BSP}; do ${INSTALL_DATA} $$x $(DESTDIR)${tooldir}/lib${MULTISUBDIR}/$$x; done
	set -e; for x in ${LINUX_SCRIPTS}; do ${INSTALL_DATA} ${srcdir}/${objtype}$$x $(DESTDIR)${tooldir}/lib${MULTISUBDIR}/$$x; done

install-redboot:
	set -e; for x in ${REDBOOT_CRT0} ${REDBOOT_OBJS}; do ${INSTALL_DATA} $$x $(DESTDIR)${tooldir}/lib${MULTISUBDIR}/$$x; done
	set -e; for x in ${REDBOOT_SCRIPTS}; do ${INSTALL_DATA} ${srcdir}/${objtype}$$x $(DESTDIR)${tooldir}/lib${MULTISUBDIR}/$$x; done

install-rdpmon:
	set -e; for x in ${RDPMON_CRT0} ${RDPMON_BSP}; do ${INSTALL_DATA} $$x $(DESTDIR)${tooldir}/lib${MULTISUBDIR}/$$x; done
	set -e; for x in ${RDPMON_SCRIPTS}; do ${INSTALL_DATA} ${srcdir}/${objtype}$$x $(DESTDIR)${tooldir}/lib${MULTISUBDIR}/$$x; done

install-rdimon:
	set -e; for x in ${RDIMON_CRT0} ${RDIMON_BSP}; do ${INSTALL_DATA} $$x $(DESTDIR)${tooldir}/lib${MULTISUBDIR}/$$x; done
	set -e; for x in ${RDIMON_SCRIPTS}; do ${INSTALL_DATA} ${srcdir}/${objtype}$$x $(DESTDIR)${tooldir}/lib${MULTISUBDIR}/$$x; done


install-pid:
	set -e; for x in ${PID_SCRIPTS}; do ${INSTALL_DATA} ${srcdir}/${objtype}$$x $(DESTDIR)${tooldir}/lib${MULTISUBDIR}/$$x; done

install-iq80310:
	set -e; for x in ${IQ80310_SCRIPTS}; do ${INSTALL_DATA} ${srcdir}/${objtype}$$x $(DESTDIR)${tooldir}/lib${MULTISUBDIR}/$$x; done


doc:
info:
install-info:
clean-info:

Makefile: Makefile.in config.status ../../../.././libgloss/arm/../config/default.mh
	$(SHELL) config.status

config.status: configure
	$(SHELL) config.status --recheck
