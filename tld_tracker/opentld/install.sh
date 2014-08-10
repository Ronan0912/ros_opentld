#!/bin/sh
INCPATH=$1/include/libopentld
LIBPATH=$1/lib/libopentld

mkdir -p $LIBPATH
cp $PWD/src/libopentld/*.a $LIBPATH

mkdir -p $INCPATH/tld
cp $PWD/src/libopentld/tld/*.h $INCPATH/tld
mkdir -p $INCPATH/mftracker
cp $PWD/src/libopentld/mftracker/*.h $INCPATH/mftracker

