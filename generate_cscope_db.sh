#!/bin/sh


# file      generate_cscope_db.sh
# author    Piotr Gregor
# purpose   Generate cscope database from kernel sources for x86 arch.
# details   Skip other architectures.
#           Use 'set tags=' and 'cs add' in vim for out of tree modules.
# date      13/12/2016 14:22


# fail the whole script on first error
set -e

# param 1 - > Number of step
# param 2 - > Step description
report_step () {
        echo "\n---->\tRunning step $1 : [$2]"
}

# exit if invalid number of arguments passed to script
usage () {
	# compare the value passed to this function with expected number of args
	if [ $1 -lt 2 -o -z $LNX ]; then
		echo "Usage:\n\tsh `basename "$0"` [-f folder] [-t tmpdir]."
        echo "\t\t-f absolute path to kernel sources (result created in that path, no slash at the end!)"
        echo "\t\t-t absolute path to directory used to store intermediate results"
		exit 1
	fi
}

# linux sources dir
LNX=
while getopts "f:t:" opt; do
	case $opt in
	f)
		LNX=$OPTARG
		echo "kernel sources directory [$LNX]" ;;
	t)
		TMPDIR=$OPTARG
		export TMPDIR
		echo "directory for intermeduate results [$TMPDIR]" ;;
	\?)
		echo "Invalid option: -$OPTARG." >&2
		usage $#
		exit 1 ;;
	:)
		echo "Option: -$OPTARG requires argument." >&2
		usage $#
		exit 1 ;;
	esac
done
# call usage with the number of arguments passed to this script
usage $#

# generate cscope.files containing absolute paths
report_step "1" "Generating cscope.files"
cd /
find $LNX \
    -path "$LNX/arch*"              -prune -o		\
    -path "$LNX/drivers*"           -prune -o		\
    -path "$LNX/tmp*"               -prune -o		\
    -path "$LNX/Documentation*"     -prune -o		\
    -path "$LNX/scripts*"           -prune -o		\
    -path "$LNX/tools*"             -prune -o		\
    -path "$LNX/include/config*"    -prune -o		\
    -path "$LNX/usr/include*"       -prune -o		\
    -type f											\
    -not -name '*.mod.c'							\
    -name "*.[chsS]" -print > $LNX/cscope.files
find $LNX/arch/x86									\
    -path "$LNX/arch/x86/configs"    -prune -o		\
    -path "$LNX/arch/x86/kvm"        -prune -o		\
    -path "$LNX/arch/x86/lguest"     -prune -o		\
    -path "$LNX/arch/x86/xen"        -prune -o		\
    -type f											\
    -not -name '*.mod.c'							\
    -name "*.[chsS]" -print >> $LNX/cscope.files
find $LNX/drivers/pci								\
    -type f											\
    -not -name '*.mod.c'							\
    -name "*.[chsS]" -print >> $LNX/cscope.files

# generate scope db, -q for additonal index (speeds up searching for large projects), -k for kernel mode (excludes code from /usr/include that is included in our kernel sources already)
report_step "2" "Generating cscope database"
cd $LNX
cscope -b -q -k

report_step "3" "Generating ctags reusing cscope.files"
ctags -L cscope.files
