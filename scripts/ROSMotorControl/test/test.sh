#!/bin/sh

# This runs all the tests. *.t.cpp is compiled into *.t and then the
# whole lot are run through 'prove', which prints a pretty report.
#
# Call with -v to get full test output, and/or with test names.

rm -rf t
mkdir t

find_tag () {
    local tag=$1
    local file=$2

    match="$(grep "^//$tag:" "$file")"
    [ -z "$match" ] && return 0

    match="${match#//$tag:}"
    echo "$match"
}

doit () {
    echo "$*"
    if ! "$@"
    then
        echo "Command failed with '$?'"
        exit 1
    fi
}

for cpp in *.t.cpp
do
    prog="t/${cpp%.cpp}"
    include="$(find_tag INCLUDE "$cpp")"
    sources="$(find_tag SOURCES "$cpp")"

    doit c++ -std=c++17 -I. -I.. $include -o "$prog" "$cpp" tap.cpp $sources
done

prove -e '' "$@"
