# RosMDB
A generic Meta DB ROS Node using MongoDB as storage.

The intention to create this MongoDB database node was to have a simple way to 
save any kind of JSON data together with metadata.\
On the other hand it should also be possible to store any kind of structure 
without using the pre configured metadata schema template.

This ROS Node meets the following requirements
- Use MongoDB as storage
- Operation modes
  - Can store data alongside with metadata by default in a [basic schema template](#helper-script-genmetash)
  - Can store data also in raw format by configuration without metadata (raw=True)
- Data can be saved as simple String or complex JSON document
- Additional metadata key\/value pairs can be added
- Parallel processing per interface (all use an own ReentrantCallbackGroup)
- Multiple RosMDB Node instances can run parallel (e.g. one node per collection)
- Add automatically insert timestamp
- Debug capabilities
- Collection control
  - Configurable default collection
  - Optionally override default collection
- Insert_one interfaces
  - By topic
  - By ROS action
  - By CLI application
- Find and find_one interfaces
  - By ROS service
  - By ROS action
  - By CLI application

## Installation Prerequisites

- Already installed MongoDB server https://www.mongodb.com

- Use the package manager [pip3](https://pip.pypa.io/en/stable/) 
to install the below dependencies.

```bash
pip3 install pymongo
```

## Setup Node ##

```bash
# run in your ros2_ws/src folder
git clone https://gitlab.com/bob-ros2/rosmdb.git
cd ..
colcon build
. install/setup.bash
```

## ROS Node MetaDB

### Usage

```bash
# start the node.
ros2 run rosmdb metadb.py
```
### Node Parameter

> ~uri (string, default: "mongodb://localhost:27017/")\
URI to access the DB. See MongoDB website how to authenticate.

> ~name (string, default: "metadb")\
Used DB name.

> ~default_collection (string, default: "memories")\
Used default collection to store data.

> ~raw (Bool, default: False)\
If set to True the metada data template is not used and data will be written 
as it is into the MongoDB.\
If input is not JSON format a minimal dict will be created around: { data: "\<input\>" }

### Subscribed Topics

> ~json (std_msgs/String)\
JSON data to be saved in the default collection.\
When it is not in JSON format 
the string is stored in a default data structure.

### Services

> ~query (srv/Query)\
Query MongoDB data. See [srv/Query.srv](srv/Query.srv) for details.

### Actions

> ~query (action/Query)\
Query MongoDB data. See [action/Query.action](action/Query.action) for details.

> ~insert  (action/Insert)\
Insert data into MongoDB data. See [action/Insert.action](action/Insert.action) for details.

### Programming example

How to use the action interface can best be seen in the also provided CLI ROS Node under [scripts/cli.py](scripts/cli.py)
This is the preferable way to communicate with the RosMDB Node.

To insert via the json topic a simple std_msgs/String publisher can be used. 
If the string is JSON parsable in will be handled as object, otherwise it will be treated as string. 
With this method no response is available about the state of the insert_one operation.

How to use a ROS service can be found in the ROS documentation [https://ros.org](https://ros.org)


## ROS Node CLI

### Usage

```bash
# help
ros2 run rosmdb cli.py -h
usage: cli.py [-h] [-c COLLECTION] [-j JSON] [-f FILE] [-t {find,find_one,insert_one}]

CLI for ROS Node RosMDB.

options:
  -h, --help            show this help message and exit
  -c COLLECTION, --collection COLLECTION
                        used MogoDB collection (default: memories)
  -j JSON, --json JSON  JSON to be used for choosen type. Can be passed via stdin. (default: )
  -t {find,find_one,insert_one}, --type {find,find_one,insert_one}
                        type of operation (default: find_one)

# searching for ObjectId
ros2 run rosmdb cli.py --type find_one --json '{"_id": "651de93bbfe1f5c4df74e77a"}'

# insert a new record, if successfull insert_id will be returned
ros2 run rosmdb cli.py --type insert_one --json '{"data": "hello there"}'

# with piped query available for all types
cat json.txt | ros2 run rosmdb cli.py -t find_one

# start in debug mode
ros2 run rosmdb cli.py -t insert_one -j '{"data": "hey"}' --ros-args --log-level debug
```

## Helper script genmeta.sh
```bash
# helper script to produce JSON data
# the output can be used by cli.py
$ ros2 run rosmdb genmeta.sh
Usage: genmeta.sh <data> [<frame_id> [<tags>]]]
  Data format as string: '"my string"' or as dict: '{"complex": {"uid": 32}}'  
  Tag list format 1-n: '"tag1", "tag2"'

# generate JSON with string data
$ ros2 run rosmdb genmeta.sh '"myData"'
{
    "metadata": [
          { "key": "stamp", "value": 1696705342.022020059 },
          { "key": "frame_id", "value": "rosmdb" },
          { "key": "tags", "value": [] }
    ],
    "data": "myData",
}

# generate JSON with nested dictionary data
$ ros2 run rosmdb genmeta.sh '{"complex": {"uid": 32}}' my_frame '"complex"' 
{
    "metadata": [
          { "key": "stamp", "value": 1696710346.734671263 },
          { "key": "frame_id", "value": "my_frame" },
          { "key": "tags", "value": ["complex"] }
    ],
    "data": {"complex": {"uid": 32}}
}

# HINT if output is not JSON conform it will be stored as string by insert_one!
# to validate format the JSON query tool jq can be used
$ ros2 run rosmdb genmeta.sh '{"complex": {"uid": 32}}' | jq .

# insert into MongoDB
$ ros2 run rosmdb genmeta.sh '"hello again"' | ros2 run rosmdb cli.py -t insert_one -c my_collection
```

## Contributing

Pull requests are welcome. For major changes, please open an issue first
to discuss what you would like to change.

Please make sure to update tests as appropriate.

## License

[Apache2.0](https://www.apache.org/licenses/LICENSE-2.0)