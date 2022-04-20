#!/usr/bin/env python3.8

from const import NodeStatus
from typing import List, Dict
import math

import rospy
from geometry_msgs.msg import Point
from pymongo import MongoClient

from custom_messages import ComandMessage


class Trace:

    def __init__(self, points: List[Point] = None):
        if not points:
            self._points = []
        else:
            self._points = points
    

    def __repr__(self) -> str:
        return str(self.dict())

    
    def __str__(self) -> str:
        return self.__repr__()

    
    def append(self, point: Point):
        self._points.append(point)


    def dict(self) -> List[Dict[str, float]]:
        return [self._point_to_dict(point_dict) for point_dict in self._points]
    

    @staticmethod
    def _point_to_dict(point: Point) -> Dict[str, float]:
        result = dict()
        result['x'] = point.x
        result['y'] = point.y
        result['z'] = point.z
    
        return result
    

    def filter(self, step: float) -> 'Trace':
        '''Фильтрует координаты с указаным шагом'''
        return self


class DBConfig:
    db_url: str = 'mongodb://root:example@127.0.0.1:27017'
    db_name: str = 'bel_tz_db'
    db_collection_name: str = 'bel_tz_coll' 


class TraceNode:

    def __init__(
        self,
        db_config: DBConfig = None 
    ):
        self._status: NodeStatus = NodeStatus.STOP
        self._trace = Trace()
        if db_config:
            self._db_config = db_config
        else:
            self._db_config = DBConfig()


    def coordinates_handler(self, point: Point):
        '''Обрабатывает появляющиеся координаты'''
        print("[INFO] Get point: " + str(point))

        if self._status == NodeStatus.START:
            self._trace.append(point)
    

    def status_handler(self, command_message: ComandMessage):
        '''Обрабатывает команды для ноды'''
        print("[INFO] Get command: " + str(command_message))

        if command_message.command_type == command_message.START:
            self._trace = Trace()
            self._status = NodeStatus.START
        
        #Мы останавливаем запись только если она была начата
        if command_message.command_type == command_message.END and self._status == NodeStatus.START:
            self._status = NodeStatus.STOP
            
            step = command_message.step
            trace_name = command_message.name
            
            filtered_trace = self._trace.filter(step)
            self._save_trace(filtered_trace, trace_name)
    

    def _save_trace(self, trace: Trace, trace_name: str):
        '''Сохраняет траекторию в базу'''
        
        db_data = {
            'trace_name': trace_name,
            'trace': trace.dict()
        }

        client = MongoClient(self._db_config.db_url)
        db = client[self._db_config.db_name]
        coll = db[self._db_config.db_collection_name]
        coll.insert_one(db_data)
    
        print("[INFO] Save trace: ", trace)


    def run(self):
        '''Запускает работу ноды'''
        
        rospy.init_node('trace_node', anonymous = True)        
        rospy.Subscriber('command', ComandMessage, self.status_handler)
        rospy.Subscriber('coordinates', Point, self.coordinates_handler)

        rospy.spin()


if __name__ == '__main__':
    node = TraceNode()
    node.run()