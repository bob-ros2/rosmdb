#!/bin/bash

if [ -z "$1" ]; then
    echo "Please provide data."
    echo "Usage: $(basename $0) <data> [<frame_id> [<tags> [<key> <value> ...]]]]"
    echo "  Data format as string: '\"my string\"' or as dict: '{\"complex\": {\"uid\": 32}}'  "
    echo "  Tag list format 1-n: '\"tag1\", \"tag2\"'"
    echo "  Key/value pair format 1-n: \"keyname\" \"value\" ..."
    exit 1
fi

DATA=$1
FRAMEID=rosmdb
[ -n "$2" ] && FRAMEID=$2
TAGS=$3

kv_pairs() {
    shift 3
    KVPAIRS=
    while [ "$#" -gt 1 ]; do
        KVPAIRS="$KVPAIRS,\n          { \"key\": \"$1\", \"value\": \"$2\" }"
        shift 2
    done
    echo -e "$KVPAIRS"
}


cat <<EOF
{
    "metadata": [
          { "key": "stamp", "value": $(date +%s.%N) },
          { "key": "frame_id", "value": "$FRAMEID" },
          { "key": "tags", "value": [$TAGS] }$(kv_pairs $@)
    ],
    "data": $DATA
}
EOF
