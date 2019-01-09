Subject: Re: crosstool-0.28-pre7: cygwin, macosx compatibility
From: "Harry Kaes" <harry.kaes@tijd.com>
Date: Sun, 21 Mar 2004 15:58:09 +0100
To: "Dan Kegel" <dank@kegel.com>

Hi Dan,

I successfully ran the crosstool to create a cross compiler for mips under
cygwin. I adapted the files in the directory contrib/newlib to fit my
purpose and I've included the adapted files in this mail. I had one slight
problem though at the end of the build. It gave a message that the esac
command could not be found. I tried to find an 'esac' tool on the cygwin
site, but there was none. Anyhow the cross compiler is built and works.
Also, I was not able to build the the tool with the latest 3.3.3 version of
gcc because the tar file is not present in the normal directory of the gnu
ftp site, so I build it with the 3.3.2 version.

I'm wondering if I Should also post a message to the crossgcc mailing list,
because I cannot send the changed files to the mailing list?
[ Harry was referring to the list policy discouraging attachments. - dank]

I also succeeded building the cross compiler without the crosstool, but I
had to build it without c++ support, otherwise it kept giving the error
'div_t' not declared as I described in some earlier mails. I received some
advice to resolve this problem but I still have to try it. As soon as I
succeed in building a fully functional mips cross compiler I'll also post
this on the mailing list.

Thanks for all the help.
