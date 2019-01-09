#!/usr/bin/perl -w
# Generate a HTML table/matrix of build results from logs results.
#
# Copyright (c) 2006 Paul Schulz <paul@mawsonlakes.org>
# Licence: GPLv2 or later
#
# Description
#   This script can be used to display a variety crosstool results in
#   configurable rows and columns, including narrowing the displayed
#   information by adding constraints.
#
#   The columns and rows to be displayed (pivoted on) are set 
#   in @col and @row. 
#
# Todo
# - Generalise the 'returned' result. eg. Display number of matches,
#     with the colour being the 'worst' outcome for that cell. 
# - Label the column variables (first column?).
#

$legacy=1;

if($legacy){
    open(FILES,'ls buildlogs/*.dat.txt |');
}else{
    $logdir="log";
    open(FILES,"ls $logdir/* | ");
}

#-------------------------------------------------------
# Add the desired rows and columns

push @row, 'TARGET';       # Row variable(s)
push @col, 'GCC_DIR';      # Column variable(s)
push @col, 'GCC_CORE_DIR';
push @col, 'GLIBC_DIR';
push @col, 'GDB_DIR';
push @col, 'BINUTILS_DIR';
push @col, 'LINUX_DIR';

# Set constraints
push @con, 'GCC_DIR=gcc-4.0.2';    # Constaints

# What results should be returned?
# Need to implement this...
# push @val, 'return $keyvalues{toolchain}'; # Result(s)
#-------------------------------------------------------
$sep=':';

my(%values);

while($file=<FILES>){
    chomp $file;
    open(LFILE,"$file");

    while($line=<LFILE>){
	chomp $line;
	if($line=~/=/){
	    ($key,$value)=split('=',$line);

	    push(@keys,$key);
	    $values{$key}=$value;

	    # Keep track of values seen.
	    # In hash of hashes.
 	    if( ! defined $keyvalues{$key} ){
 		$keyvalues{$key}={$value=>1};
 	    }else{
 		${$keyvalues{$key}}{$value}++;
 	    }		
	}
    }

    if(defined @con){
	$cons=0;
	for $constraint (@con){
	    ($key,$value)=split('=',$constraint);

	    if( defined $values{$key} 
		&& $values{$key} eq $value){
		$cons=1;
	    }
	}
    }else{
	$cons=1;
    }
    if($cons){
	$rowid=':';
	for $r (@row){
	    if( defined $values{$r}){
		$rowid.=$values{$r};
	    }
	    $rowid.=$sep;
	}
	
	$colid=':';
	for $c (@col){
	    if( defined $values{$c}){
		$colid.=$values{$c}
	    }
	    $colid.=$sep;
	}
	
	$cellid=$rowid."-".$colid;
	
	if( ! defined $rowid{$rowid}){
	    push @rowid,$rowid;
	    $rowid{$rowid}=1;
	}
	
	if( ! defined $colid{$colid}){
	    push @colid,$colid;
	    $colid{$colid}=1;
	}
	
	if( ! defined $cellid{$cellid}){
	    push @cellid,$cellid;
	    $cellid{$cellid}=1;
	}
	
    # What should be recorded?
#    if( ! defined $cellvalue{$cellid} ){
#	$cellvalue{$cellid}=1;
#    }else{
#	$cellvalue{$cellid}++;
#    }

	if( defined $values{toolchain}){
	    $cellvalue{$cellid}=$values{toolchain};
	}else{
	    $cellvalue{$cellid}="-";
	}
    }

    undef %values;
    undef @keys;
}

@rowid = sort(@rowid);
@colid = sort(@colid);

sub output_cell {
    my($val)=@_;

    my($opt) = '';;

    if( defined $val ){
	$opt .= "align=\"center\" ";
	for ($val){
	    /FAIL/ && do {
		$opt .= "bgcolor=\"red\" ";
		last;
	    };
	    /PASS/ && do {
		$opt .= "bgcolor=\"green\" ";
		last;
	    };
	}
    }else{
	$val = '&nbsp;';
    }

    $str = "<td $opt>";
    $str .= $val;
    $str .= "</td>\n";
    
    return $str;
}

sub output_html {

    print "<p>";
    print "COLUMNS: ";
    for $c (@col){
	print "$c ";
    }
    print "</p>\n";

    print "<p>";
    print "ROWS: ";
    for $r (@row){
	print "$r ";
    }
    print "</p>\n";    

    print "<p>";
    print "Constraints:<br/>";
    for $c (@con){
	print "&nbsp;$c<br/>";
    }
    print "</p>\n";    

    print "<table border=\"1\" cellspacing=\"0\">\n";

    print "<tr>\n";
    print "<th>&nbsp;</th>\n";
    for $c (@colid){
	$clabel=$c;
	$clabel=~s/\:/ /g;
	print "<th>";
	print "$clabel";
	print "</th>\n";
    }

    print "</tr>\n";

    for $r (@rowid){
	$rlabel=$r;
	$rlabel=~s/\:/ /g;
	print "<tr>\n";
	print "<td>$rlabel</td>\n";
	for $c (@colid){
	    $cellid="$r-$c";
	    print output_cell($cellvalue{$cellid});
	}
	print "</tr>\n";
    }
    print "</table>\n";
}

output_html();	

# Debug
# $str="TARGET";
# print STDERR $str."\n";
# for $v (keys %{$keyvalues{$str}}){
#     print STDERR "  ".$v."\n";
# }

