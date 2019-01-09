#!/usr/bin/perl
# Trivial macro preprocessor
# Passes lines through unchanged, except that each range of lines bracketed as follows:
#   __FOR var val1 val2 val3 ... valN
#   ...
#   __ENDFOR
# is expanded N times by replacing all instances of var with one of the given values.
# Loops may not be nested.
# (Trivia: I tried to do this in sed, but couldn't figure out how.)
# 
# CHEEZY HACK: if var is __CPU__, expand __TARGET__ as well, according to
#  TARGET=`cat $CPU.dat | grep TARGET= | sed 's/.*=//'`
# This requires user to pass directory containing $CPU.dat as argument on commandline 
#
# Dan "I am not proud" Kegel

# kludge:
$datadir=$ARGV[0];
if (! -f "$datadir/i686.dat") {
	die "can't find data file $datadir/i686.dat; did you give crosstool directory as first argument?"
}

$in = 0;
while (<STDIN>) {
	if (/^__FOR (\S*) (.*)/) {
		$in = 1;
		$var = $1;
		$vals = $2;
	} elsif (/^__ENDFOR/) {
		$in = 0;
		# output saved buffer once for each val
		foreach $val (split(" ", $vals)) {
			$out = $buf;
			$out =~ s/$var/$val/g;
			if ($var eq "__CPU__") {
				# oh, this is a kludge...
				$realcpu = $val;
				$realcpu =~ s/_/-/;
				$realcpu =~ s/x86-64/x86_64/;
				$target=`cat $datadir/$realcpu.dat | grep TARGET=`;
				$target =~ s/.*=//;
				chomp $target;
				$out =~ s/__TARGET__/$target/g;
			}
			print $out;
		}
		$buf = "";
	} elsif ($in) {
		$buf .= $_;
	} else {
		print $_;
	}
}
