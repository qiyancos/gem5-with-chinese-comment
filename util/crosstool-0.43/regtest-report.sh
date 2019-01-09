#!/bin/sh
# Generate report showing matrix of build results for a run of regtest-run.sh
# Copyright (C) 2005, Dan Kegel, Google
# License: GPL
# This badly needs rewriting in perl.

set -e

cd jobdir

LANG=C
export LANG

# Generating simple machine-readable report
# put GCC_CORE_DIR after GLIBC_DIR to get column order right in report
for file in *.dat.txt; do
    echo -n "NAME=$file	"
    cat $file | sed 's/GCC_CORE/GLIBC_ZZZ/' | sort | egrep 'TARGET=|GCC_DIR|GLIBC_ZZZ|GLIBC_DIR|BINUTILS_DIR|LINUX|PASS|FAIL|ICE' | sed 's/GLIBC_ZZZ/GCC_CORE/' | tr '\012' '\011' 
    echo ""
done | sed 's/\.dat\.txt//' > all.dats.txt

# Generate fancy index.html report

# Figure out which tool combinations were used
# Extract TLS from NAME and move it to end, so it's the most minor sort key
sed 's/NAME=[-_a-zA-Z0-9\.]*tls\(.*\)/\1	TLS=tls/;s/NAME=[-_a-zA-Z0-9\.]*nptl\(.*\)/\1	TLS=nptl/;s/NAME=[-_a-zA-Z0-9\.]*//;s/TARGET=[_a-z0-9\-]*//;s/toolchain=[A-Z]*//;s/kernel=[A-Z]*//;s/gdb=[A-Z]*//;s/gdbserver=[A-Z]*//' < all.dats.txt | sort -u | tr '\011' ':' | grep ':.*:.*:' | sed 's/::*/:/g' | sort -u | grep GCC | grep GLIBC > all-tools.tmp
ALL_CPUS=`cat all.dats.txt | tr '\011' '\012' | grep TARGET= | sed 's/TARGET=//;s/-unknown//;s/-linux-gnu//' | sort -u`

OUT=index.html
cat  > $OUT <<_EOF_
<html>
<head>
<style type="text/css">
a:visited { color: black; } 
</style>
<title>Crosstool build results</title>
</head>
<body>
<h2>Crosstool build results</h2>
The following matrix shows whether the given combinations of
gcc, glibc, binutils, and linux kernel headers, lightly patched,
can build a cross-toolchain and compile a kernel for the given CPUs. 
(It doesn't say anything about whether the resulting toolchain works!)
<br>
See <a href="http://kegel.com/crosstool">crosstool</a> for the patches
and scripts used.
<br>
Click on a matrix entry for the last few lines of that build log file.
<br>
The label 'cgcc' refers to the 'core' version of gcc, i.e. the one used to build glibc
before building the final gcc.  (Older glibc's prefer to be built by older gcc's.)
_EOF_

echo "<table>" >> $OUT

echo "<tr><th>" >> $OUT
for tools in `cat all-tools.tmp`; do
	echo $tools | tr ':' '\012' > tools.tmp
	TLS=; grep tls tools.tmp > /dev/null && TLS=tls; grep nptl tools.tmp > /dev/null && TLS=nptl
	BINUTILS_DIR=`awk -F= '/BINUTILS_DIR/ {print $2}' tools.tmp`
	GCC_CORE_DIR=`awk -F= '/GCC_CORE_DIR/ {print $2}' tools.tmp`
	GCC_DIR=`awk -F= '/GCC_DIR/ {print $2}' tools.tmp`
	GLIBC_DIR=`awk -F= '/GLIBC_DIR/ {print $2}' tools.tmp`
	LINUX_DIR=`awk -F= '/LINUX_DIR/ {print $2}' tools.tmp`
	LINUX_SANITIZED_HEADER_DIR=`awk -F= '/LINUX_SANITIZED_HEADER_DIR/ {print $2}' tools.tmp`
	if test -n "$LINUX_SANITIZED_HEADER_DIR"; then
	    LINUX_SANITIZED_HEADER_DIR=hdrs-`echo $LINUX_SANITIZED_HEADER_DIR | sed 's/.*-//'`
	fi
	echo "<th>$GCC_DIR<br>c$GCC_CORE_DIR<br>$GLIBC_DIR<br>$BINUTILS_DIR<br>$LINUX_DIR<br>$LINUX_SANITIZED_HEADER_DIR<br>$TLS<br></th>" >> $OUT
done
echo '</tr>' >> $OUT

for cpu in $ALL_CPUS; do
   # kludge until I use uniform names
   case $cpu in
   arm-9tdmi) cpu=arm9tdmi;;
   armv5b-softfloat-linux) cpu=armv5b-softfloat;;
   powerpc64) cpu=powerpc-970;;
   i686-piii) cpu=pentium3;;       # hrm, that seems off somehow
   esac
   echo '<tr><th>'$cpu'</th>'
   for tools in `cat all-tools.tmp`; do
	echo $tools | tr ':' '\012' > tools.tmp
	TLS=; grep tls tools.tmp > /dev/null && TLS=tls; grep nptl tools.tmp > /dev/null && TLS=nptl
	BINUTILS_DIR=`awk -F= '/BINUTILS_DIR/ {print $2}' tools.tmp`
	GCC_DIR=`awk -F= '/GCC_DIR/ {print $2}' tools.tmp`
	GCC_CORE_DIR=`awk -F= '/GCC_CORE_DIR/ {print $2}' tools.tmp`
	GLIBC_DIR=`awk -F= '/GLIBC_DIR/ {print $2}' tools.tmp`
	LINUX_DIR=`awk -F= '/LINUX_DIR/ {print $2}' tools.tmp`
	LINUX_SANITIZED_HEADER_DIR=`awk -F= '/LINUX_SANITIZED_HEADER_DIR/ {print $2}' tools.tmp`
	toolcombo=$GCC_DIR-$GLIBC_DIR
	if test -n "$LINUX_SANITIZED_HEADER_DIR"; then
	    LINUX_SANITIZED_HEADER_DIR=hdrs-`echo $LINUX_SANITIZED_HEADER_DIR | sed 's/.*-//'`
	    #toolcombo=$toolcombo-$LINUX_SANITIZED_HEADER_DIR
	fi
        case $TLS in
        *tls*) toolcombo=$toolcombo-tls ;;
        *nptl*) toolcombo=$toolcombo-nptl ;;
        *) ;;
        esac
	
      echo '<td ' 
      if test -f $cpu-$toolcombo.dat.txt; then
        status=`cat $cpu-$toolcombo.dat.txt`
        case "$status" in
        *kernel=PASS*)    echo 'style="background-color: green"><a href="'$cpu-$toolcombo.log.txt'">ok</a>'           ;;
        *kernel=ICE*)     echo 'style="background-color: yellow"><a href="'$cpu-$toolcombo.log.txt'">kernel ICE</a>' ;;
        *toolchain=PASS*) echo 'style="background-color: yellow"><a href="'$cpu-$toolcombo.log.txt'">kernel fail</a>' ;;
        *toolchain=ICE*)  echo 'style="background-color: orange"><a href="'$cpu-$toolcombo.log.txt'">ICE</a>'         ;;
        *toolchain=FAIL*) echo 'style="background-color: red"><a href="'$cpu-$toolcombo.log.txt'">FAIL</a>'           ;;
	*) echo 'bug in regtest-report.sh' ;;
        esac
        case "$status" in
        *gdb=PASS*) echo '<br><a href="'$cpu-$toolcombo.log.txt'" style="background-color: green">gdb ok</a>'           ;;
        *gdb=FAIL*) echo '<br><a href="'$cpu-$toolcombo.log.txt'" style="background-color: red">gdb FAIL</a>'           ;;
        *gdb=n/a*) ;;
	*) echo 'bug in regtest-report.sh' ;;
        esac
      fi
      echo '</td>' 
   done
   # Repeat left headings on right
   echo '<th>'$cpu'</th>'
   echo '</tr>' 
done >> $OUT

# Repeat top headings on bottom
echo "<tr><th>" >> $OUT
for tools in `cat all-tools.tmp`; do
	echo $tools | tr ':' '\012' > tools.tmp
	TLS=; grep tls tools.tmp > /dev/null && TLS=tls; grep nptl tools.tmp > /dev/null && TLS=nptl
	BINUTILS_DIR=`awk -F= '/BINUTILS_DIR/ {print $2}' tools.tmp`
	GCC_DIR=`awk -F= '/GCC_DIR/ {print $2}' tools.tmp`
	GCC_CORE_DIR=`awk -F= '/GCC_CORE_DIR/ {print $2}' tools.tmp`
	GLIBC_DIR=`awk -F= '/GLIBC_DIR/ {print $2}' tools.tmp`
	LINUX_DIR=`awk -F= '/LINUX_DIR/ {print $2}' tools.tmp`
	LINUX_SANITIZED_HEADER_DIR=`awk -F= '/LINUX_SANITIZED_HEADER_DIR/ {print $2}' tools.tmp`
	if test -n "$LINUX_SANITIZED_HEADER_DIR"; then
	    LINUX_SANITIZED_HEADER_DIR=hdrs-`echo $LINUX_SANITIZED_HEADER_DIR | sed 's/.*-//'`
	fi
	echo "<th>$GCC_DIR<br>c$GCC_CORE_DIR<br>$GLIBC_DIR<br>$BINUTILS_DIR<br>$LINUX_DIR<br>$LINUX_SANITIZED_HEADER_DIR<br>$TLS<br></th>" >> $OUT
done
echo '</tr>' >> $OUT

echo '</table>' >> $OUT
echo '</body></html>' >> $OUT

#rm -f *.tmp
