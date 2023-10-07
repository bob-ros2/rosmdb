#!/bin/bash

if [ -z "$1" ]; then
    echo "At least the data to produce must be provided!"
    echo "Usage: $(basename $0) <data> [<frame_id> [<tags>]]]"
    echo "  Data format as string: '\"my string\"' or as dict: '{\"complex\": {\"uid\": 32}}'  "
    echo "  Tag list format 1-n: '\"tag1\", \"tag2\"'"
    exit 1
fi

FRAMEID=rosmdb
[ -n "$2" ] && FRAMEID=$2

cat <<EOF
{
    "metadata": [
          { "key": "stamp", "value": $(date +%s.%N) },
          { "key": "frame_id", "value": "$FRAMEID" },
          { "key": "tags", "value": [$3] }
    ],
    "data": $1
}
EOF
