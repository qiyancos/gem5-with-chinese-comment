#!/bin/bash

build/X86/gem5.opt configs/example/fs.py --cpu-type=DerivO3CPU --caches --l1d_size=64kB --l1d_assoc=8 --l1i_size=32kB --l1i_assoc=4 --mem-size=2GB -n 2 --kernel=x86_64-vmlinux-2.6.22.9.smp --disk-image=linux-x86.img
