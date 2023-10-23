#!/usr/bin/env python3
#
# Copyright 2023 BobRos
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
import sys
import os
import argparse
import json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rosmdb.action import Query
from rosmdb.action import Insert

class MDBActionClient(Node):
    """Basic action client for RosMDB Node."""
    def __init__(self, name, collection, type, query):
        super().__init__(name)
        self.running = True
        self.type = type
        self.action_query = ActionClient(self, Query, 'query')
        self.action_insert = ActionClient(self, Insert, 'insert')

        self.get_logger().debug('JSON: %s' % query)

        if self.type > 1:
            self.send_insert(collection, query)
        else:
            self.send_query(collection, self.type, query)

        while self.running:
            rclpy.spin_once(self, timeout_sec=0.001)

    def send_query(self, collection, type, query):
        goal_msg = Query.Goal()
        goal_msg.type = type
        goal_msg.collection = collection
        goal_msg.query = query
        self.get_logger().debug(str(goal_msg))
        self.action_query.wait_for_server()
        self.send_target_future = self.action_query.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self.send_target_future.add_done_callback(
            self.goal_response_callback)
        
    def send_insert(self, collection, query):
        goal_msg = Insert.Goal()
        goal_msg.collection = collection
        goal_msg.json = query
        self.get_logger().debug(str(goal_msg))
        self.action_query.wait_for_server()
        self.send_target_future = self.action_insert.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self.send_target_future.add_done_callback(
            self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().debug('goal rejected :(')
            return
        self.get_logger().debug('goal accepted :)')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(
            self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().debug(str(result))
        if self.type > 1:
            print(str(result.insert_id))
        else:
            print(str(result.json))
        self.running = False

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().debug('Feedback: %s' % feedback.state)

# main

def handle_regex(v):
    if v.startswith('/') and v.endswith('/'):
        return {'$regex': v[1:-1]}
    return v

def handle_numbers(v):
    if isinstance(v, str):
        try:
            if float(v) and '.' in v: return float(v)
        except ValueError:
            if v.isnumeric(): return int(v)
    return v

class parse_kv(argparse.Action):
    def __call__(self, parser, args, values, option_string=None):
        try: d = dict(map(lambda x: x.split('='),values))
        except ValueError as ex:
            raise argparse.ArgumentError(self, 
                "Could not parse argument as k1=v1 k2=v2 ... format")
        setattr(args, self.dest, d)
    def old__call__(self, parser, args, values, option_string=None):
        try:
            a = []
            for item in values:
                a.append({"key": handle_regex(item.split('=')[0]), 
                          "value": handle_numbers(handle_regex(item.split('=')[1]))})
        except ValueError as ex:
            raise argparse.ArgumentError(self, 
                "Could not parse argument as k1=v1 k2=v2 ... format")
        setattr(args, self.dest, a)


def query(kv, matchtype):
    d = { "metadata": { matchtype: []} }
    for key, value in kv.items():
       d["metadata"][matchtype].append({ "$elemMatch": 
            {"key": handle_regex(key), "value":handle_regex(value)}})
    return d

def main():
    parser = argparse.ArgumentParser(
        description="CLI for ROS Node MetaDB.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '-c', '--collection', type=str, default="memories", 
        help='used MogoDB collection')
    parser.add_argument(
        '-j', '--json', type=str, default="",
        help="JSON to be used for choosen type. Can be passed via stdin")
    parser.add_argument(
        '-t', '--type', type=str, default="find_one",
        choices=['find', 'find_one', 'insert_one'],
        help="type of operation")
    choices = {'find':0, 'find_one':1, 'insert_one':2}
    parser.add_argument("-k", "--keys", default=[], 
        nargs='*', metavar="KEY=VALUE", action=parse_kv,
        help="key value pair(s) to find or find_one, overrides JSON input. " \
             "Can contain also regex in form /regex/ for key or value fields")
    parser.add_argument(
        '-m', '--match', type=str, default="$all",
        choices=['$all', '$and', '$nor', '$or'],
        help="match type of key/value list.")
    parser.add_argument(
        '-v', '--verbose', action='store_true', default=False,
        help="Switch on verbose mode.")
    args, remaining = parser.parse_known_args()
    
    if not os.isatty(sys.stdin.fileno()):
        args.json = ''
        for line in sys.stdin:
            args.json += line

    if args.keys:
        args.json = json.dumps(query(args.keys, args.match))
        if args.verbose: print(args.json)

    if not args.json:
        print("JSON input is missing!")
        parser.print_help()
        sys.exit(1)

    rclpy.init(args=sys.argv)
    n = MDBActionClient(
        'rosmdb_cli', args.collection, choices[args.type], args.json)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
