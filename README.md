# PTExtendedBasicArduino
Alpha upload
Full desciption in my blog http://www.delectra.com/toys/?p=145&preview=true

This is a port of Reconstructed source code to the Processor Technology Extended Casstte basic from 1977.
It is not warrented to be of any use whatsoever other than for sentimental Nostalga.

from the blog.
Processor technology was one of the first home computer makers.  Starting at the same time as Apple.   Where Apple sold a few hundred, Processor Tech often called PT sold thousands of units.
Like most computers of the era PT Sol-20s  ran BASIC.  This was loaded in from cassette tapes.   PT produced two versions of BASIC 5K  and Extended cassette Basic, which loaded in 15K.  The latter was a full implementation of the language. 

Processor Tech ceased operations in 1979.  The assettes were sold at auction.  Some of the source code was sold in the remaining minutes of the company to member of the users group Proteus.  By 1982 the source code to the basic was lost.  The user group stopped issuing newsletters about that time.

In recent years, a number of websites have made copies of the Manuals avalilable.  Some of these websites have code and sources recovered from the floppy disks.  This is a reconstruction of the Cassette Basic from the DOS Basic sources.  The code was written in 8080 assembly.  It was my desire to see if this code would run on an AVR processor.  Specifically the Mega328 used in the Arduino Uno. The code is optomized for a Palm stowaway folding keyboard which communicated at 9600 baud.  The display is an ADAFruit ILI9341 TFT.  The serial back channel is not currently implemented. 

The 8080 code was copyrighted.  It is unclear if there is anyone left to care about this. I have included the orgional notice. It is unlikley there is anyone at the addresses in these documents.

The AVR version as no copyright on it whatsoever other than that inherited from retaining restoring and recovering the origional notices.  This message was such a part of the experience, that it seems a shame to remove it.
The Dos sources had a number of bug fixes. Some of these are marked by % comments. They are not implemented.  The Nostaigia in this, is the experience of the 1977/1978 era code

The casstte interface has not been coded yet, so there is no filesystem, involking these commands will generate I/O errors.  Probably the best way to implement the I/O will be with SD cards.

Only simple testing has been done.  There may be a number of bugs that remain which is why this is alpha release

To build the AVR source it is neccissary to set the F_CPU and baud definitions in the command line of the assembler.    
