#!/bin/bash

# Roman Yepishev <roman.yepishev@ubuntu.com>
# rye @ #ubuntuone

set -e

DIR=${1:-.}

RENAME_LOG=~/rename-u1conflict-files.log

find "$DIR" -depth -name '*.u1conflict' | (
    while read FILE_PATH; do
        BASENAME="${FILE_PATH%.u1conflict}"
        TARGET="$BASENAME"

        COUNTER=0
        while [ -e "$TARGET" ]; do
            COUNTER=$(( $COUNTER+1 ))
            TARGET="$BASENAME.$COUNTER"
        done

        echo "$FILE_PATH:$TARGET" >> $RENAME_LOG

        mv -v "$FILE_PATH" "$TARGET"

    done
)
