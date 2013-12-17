#!/bin/sh

DIR='./'
count=`git rev-list HEAD | wc -l | sed -e 's/ *//g' | xargs -n1 printf %04d`
commit=`git show --abbrev-commit HEAD | grep '^commit' | sed -e 's/commit //'`
buildno=build."$count.$commit"

echo -n '"' > $1
echo -n $buildno >> $1
echo -n "@" >> $1
date +"%Y-%m-%d/%T\"" >> $1

