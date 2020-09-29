#!/bin/sh

# This runs all the tests. *.t.cpp is compiled into *.t and then the
# whole lot are run through 'prove', which prints a pretty report.
#
# Call with -v to get full test output, and/or with test names.

set -ex

rm -rf t
mkdir t

for cpp in *.t.cpp
do
    c++ -I.. -o t/"${cpp%.cpp}" "$cpp"
done

prove -e '' "$@"
