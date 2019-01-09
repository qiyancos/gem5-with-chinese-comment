#!/bin/sh
set -ex

cd $PREFIX
if test '!' -d tmp; then
    mkdir tmp
else
    # Cleanout test files for this target.
    rm -rf tmp/$TARGET-*
fi
cd tmp

# check for canadian cross
if test x"$GCC_HOST" != x; then
    echo "GCC_HOST set, so we can't run the compilers on the build machine, not testing"
    exit 0
fi

# Test the C compiler

cat > hello.c <<_eof_
#include <stdio.h>
#include <limits.h>
static long x = LONG_MIN;
int main() { printf("Hello, world!  LONG_MIN is %ld, PATH_MAX is %d\n", x, PATH_MAX); return 0; }
_eof_

$PREFIX/bin/$TARGET-gcc -static hello.c -o $TARGET-hello-static
$PREFIX/bin/$TARGET-gcc hello.c -o $TARGET-hello

# Test the C++ compiler if it was built
if test -x $PREFIX/bin/$TARGET-g++; then

    cat > hello2.cc <<_eof_
#include <iostream>
int main() { std::cout << "Hello, c++!\n"; return 0; }
_eof_

    $PREFIX/bin/$TARGET-g++ -static hello2.cc -o $TARGET-hello2-static
    $PREFIX/bin/$TARGET-g++ hello2.cc -o $TARGET-hello2

else
    echo "No c++ compiler found; c++ compiler not tested."
fi
echo testhello: C compiler can in fact build a trivial program.
