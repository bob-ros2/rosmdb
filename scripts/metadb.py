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
import sys, rclpy, pymongo, datetime, time, json
from bson import json_util
from rclpy.node import Node
from std_msgs.msg import String, Header
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer
from rosmdb.srv import Query
from rosmdb.action import Insert as InsertAction
from rosmdb.action import Query as QueryAction

# https://learnmongodbthehardway.com/schema/metadata/
# The used minimnal JSON document structure acting as META data record.
meta_schema = {
    "metadata": [
        {"key": "stamp", "value": 0.0},
        {"key": "frame_id", "value": "metadb"},
        {"key": "tags", "value": []},
    ],
    "data": ""
}

class MetaDB():
    """Basic Meta Database class using Mongo DB."""
    def __init__(self, node=None):
        self.node = node

    def connect(self, name, uri):
        """Connect to given Mongo DB."""
        self.name = name
        self.uri = uri
        self.client = pymongo.MongoClient(self.uri)
        self.db = self.client[self.name]
        return self.db

    def insert_one(self, j, collection="memories", raw=False):
        """Insert one JSON record into given Collection"""
        try: data = json.loads(j)
        except Exception as e:
            # if JSON parsing fails use a minmal dict
            if self.node: self.node.get_logger().debug(
                "WARN, input is not JSON parsable! Storing as string...")
            data = {"data": j}
        if raw: # keep as it is
            record = data
        else: # merge data with default template
            record = dict(meta_schema)
            record["_ts"] = datetime.datetime.fromtimestamp(
                time.time_ns() / 1000000000)
            record.update(data)
        
        insert_id = self.db[collection].insert_one(record).inserted_id
        if self.node: 
            self.node.get_logger().debug(
                'InsertId: %s Data: %s' % (insert_id, str(record)))
        return insert_id

    def find_one(self, q, collection="memories"):
        """Mongo DB find_one equivalent."""
        return self.db[collection].find_one(
            self.replace_object_id(q))

    def find(self, q, collection="memories"):
        """Mongo DB find equivalent."""
        return self.db[collection].find(
            self.replace_object_id(q))

    def replace_object_id(self, q):
        from bson.objectid import ObjectId
        if "_id" in q:
            q["_id"] = ObjectId(q["_id"])
        return q

class MetaDBNode(Node):
    """ROS node as basic Mongo DB interface."""
    def __init__(self):
        super().__init__('rosmdb')

        self.declare_parameter('uri', 'mongodb://localhost:27017/')
        self.declare_parameter('name', 'metadb')
        self.declare_parameter('default_collection', 'memories')
        self.declare_parameter('raw', False)

        self.uri = self.get_parameter(
            'uri').get_parameter_value().string_value
        
        self.db_name = self.get_parameter(
            'name').get_parameter_value().string_value
        
        self.default_collection = self.get_parameter(
            'default_collection').get_parameter_value().string_value
        
        self.raw = self.get_parameter(
            'raw').get_parameter_value().bool_value

        try:
            self.metadb = MetaDB(self)
            self.db = self.metadb.connect(self.db_name, uri=self.uri)
        except Exception as e:
            self.get_logger().error("metadb could not be opened! %s" % str(e))
            raise

        self.sub = self.create_subscription(String, 
            'json', self.sub_json_callback, 10,
            callback_group=ReentrantCallbackGroup())
        
        self.srv = self.create_service(Query, 
            'query', self.srv_query_callback,
            callback_group=ReentrantCallbackGroup())
        
        self.action_query = ActionServer(self, QueryAction, 
            'query', self.action_query_callback,
            callback_group=ReentrantCallbackGroup())
        
        self.action_insert = ActionServer(self, InsertAction, 
            'insert', self.action_insert_callback,
            callback_group=ReentrantCallbackGroup())

    def sub_json_callback(self, msg):
        """Callback which listens for insert_one requests. When msg.data is not 
        JSON format the raw data will be stored in a template structure."""
        try: json.loads(msg.data)
        except:
            header = Header() 
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.get_name()
            msg.data = json.dumps({
                "metadata": [
                    {"key": "stamp", "value": float("%d.%09d" 
                        % (header.stamp.sec, msg.header.stamp.nanosec))},
                    {"key": "frame_id", "value": header.frame_id},
                    {"key": "tags", "value": [header.frame_id]},
                ],
                "data": msg.data
            })

        try: insert_id = self.metadb.insert_one(msg.data, self.default_collection, self.raw)
        except Exception as e: self.get_logger().error(str(e))
        self.get_logger().debug('InsertId: %s Data: %s' % (insert_id, msg.data))

    def db_query(self, request):
        """Perform the DB query. Returns None on failure."""
        try:
            if not request.collection:
                request.collection = self.default_collection
            if request.type == request.TYPE_FIND_ONE: 
                return json_util.dumps(self.metadb.find_one(
                    json.loads(request.query), request.collection))
            else: 
                return json_util.dumps(self.metadb.find(
                    json.loads(request.query), request.collection))
        except Exception as e: 
            self.get_logger().error(str(e))
            return str(e) + ' ' + str(request)

    def db_insert(self, request):
        """Perform the DB insert. Returns None on failure."""
        try:
            if not request.collection:
                request.collection = self.default_collection
            return self.metadb.insert_one(
                request.json, request.collection, self.raw)
        except Exception as e: 
            self.get_logger().error(str(e))
            return None

    def srv_query_callback(self, request, response):
        """Callback which listen for find_one service call."""
        self.get_logger().info('Call Query service %s %s' 
            % (str(request.type), request.collection))
        self.get_logger().debug('Query: %s' % request.query)
        response.json = self.db_query(request)
        self.get_logger().debug('Result: %s' % response.json)
        return response

    def action_query_callback(self, target_handle):
        """Callbak  which listen for query actions."""
        self.get_logger().debug('Call Query action')
        feedback_msg = QueryAction.Feedback()
        feedback_msg.state = "Querying..."
        target_handle.publish_feedback(feedback_msg)
        result = QueryAction.Result()
        result.json = self.db_query(target_handle.request)
        target_handle.succeed()
        return result

    def action_insert_callback(self, target_handle):
        """Callbak  which listen for insert actions."""
        self.get_logger().debug('Call Insert action')
        feedback_msg = InsertAction.Feedback()
        feedback_msg.state = "Inserting..."
        target_handle.publish_feedback(feedback_msg)
        insert_id = self.db_insert(target_handle.request)
        target_handle.succeed()
        result = InsertAction.Result()
        result.insert_id = str(insert_id)
        return result


if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    n = MetaDBNode()
    rclpy.spin(n)
    rclpy.shutdown()
