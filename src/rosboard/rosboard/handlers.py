import json
import socket
import time
import tornado
import tornado.web
import tornado.websocket
import traceback
import types
import uuid
import subprocess
import os
from . import __version__
import sqlite3
import queue
import signal
import threading
import inspect
import ctypes
import psutil
import datetime
import hashlib
import base64
from PIL import Image
from .config import dbPath, mapSavePath


def run_shellcommand(*args):
    '''run the provided command and return its stdout'''
    args = sum([(arg if type(arg) == list else [arg]) for arg in args], [])
    print(args)
    possess = subprocess.Popen(args, stdout=subprocess.PIPE)

    return possess


def terminate_process_by_name(process_name):
    for p in psutil.process_iter(['name', 'pid']):
        if p.info['name'] == process_name:
            p.terminate()
            break


def split_words(text):
    """return a list of lines where each line is a list of words"""
    return [line.strip().split() for line in text.split('\n')]


def _async_raise(tid, exctype):
    """raises the exception, performs cleanup if needed"""
    tid = ctypes.c_long(tid)
    if not inspect.isclass(exctype):
        exctype = type(exctype)
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exctype))
    if res == 0:
        raise ValueError("invalid thread id")
    elif res != 1:
        # """if it returns a number greater than one, you're in trouble,
        # and you should call it again with exc=NULL to revert the effect"""
        ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)
        raise SystemError("PyThreadState_SetAsyncExc failed")


def stop_thread(thread):
    _async_raise(thread.ident, SystemExit)


def make_launch_queue():
    launch_queue = queue.Queue()

    def put_data(data):
        # 向队列中添加数据
        launch_queue.put(data)

    def get_data():
        # 从队列中读取数据
        if not launch_queue.empty():
            return launch_queue.get()
        else:
            return None

    return {'put_data': put_data, 'get_data': get_data}


class NoCacheStaticFileHandler(tornado.web.StaticFileHandler):
    def data_received(self, chunk):
        pass

    def set_extra_headers(self, path):
        # Disable cache
        self.set_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')


class AddPoseHandler(tornado.web.RequestHandler):

    def get(self):
        # logger.debug("Received POST request in  EditMapHandler")
        map_name = self.get_argument('map_name', None)
        if map_name is not None:
            try:
                map_name_list = []
                conn = sqlite3.connect(dbPath)
                c = conn.cursor()
                c.execute("SELECT * FROM xgo_map")
                rows = c.fetchall()
                for row in rows:
                    map_info = {"map_name": row[2], "map_path": row[3]}
                    map_name_list.append(map_info)
                c.close()
                conn.close()
                re_data = {"msg": "ok", "data": map_name_list}
                self.write(json.dumps(re_data))
            except sqlite3.IntegrityError as e:
                re_data = {"msg": str(e)}
                self.write(json.dumps(re_data))
                return
        else:
            map_name = self.get_argument('map_name', None)
            if map_name is not None:
                try:
                    map_name_list = []
                    conn = sqlite3.connect(dbPath)
                    c = conn.cursor()
                    c.execute("SELECT * FROM xgo_map")
                    rows = c.fetchall()
                    for row in rows:
                        map_info = {"map_name": row[2], "map_path": row[3]}
                        map_name_list.append(map_info)
                    c.close()
                    conn.close()
                    re_data = {"msg": "ok", "data": map_name_list}
                    self.write(json.dumps(re_data))
                except sqlite3.IntegrityError as e:
                    re_data = {"msg": str(e)}
                    self.write(json.dumps(re_data))
                    return

    def post(self):
        # logger.debug("Received POST request in  EditMapHandler")
        map_name = self.get_argument('map_name', None)
        if map_name is not None:
            try:
                map_name_list = []
                conn = sqlite3.connect(dbPath)
                c = conn.cursor()
                c.execute("SELECT * FROM xgo_map")
                rows = c.fetchall()
                for row in rows:
                    map_info = {"map_name": row[2], "map_path": row[3]}
                    map_name_list.append(map_info)
                c.close()
                conn.close()
                re_data = {"msg": "ok", "data": map_name_list}
                self.write(json.dumps(re_data))
            except sqlite3.IntegrityError as e:
                re_data = {"msg": str(e)}
                self.write(json.dumps(re_data))
                return
        else:
            map_name = self.get_argument('map_name', None)
            if map_name is not None:
                try:
                    map_name_list = []
                    conn = sqlite3.connect(dbPath)
                    c = conn.cursor()
                    c.execute("SELECT * FROM xgo_map")
                    rows = c.fetchall()
                    for row in rows:
                        map_info = {"map_name": row[2], "map_path": row[3]}
                        map_name_list.append(map_info)
                    c.close()
                    conn.close()
                    re_data = {"msg": "ok", "data": map_name_list}
                    self.write(json.dumps(re_data))
                except sqlite3.IntegrityError as e:
                    re_data = {"msg": str(e)}
                    self.write(json.dumps(re_data))
                    return


class GetMapListHandler(tornado.web.RequestHandler):

    def get(self):
        # logger.debug("Received POST request in  EditMapHandler")
        map_name = self.get_argument('map_name', None)
        print(map_name)
        if map_name is not None:
            try:
                conn = sqlite3.connect(dbPath)
                c = conn.cursor()
                c.execute("SELECT * FROM xgo_map WHERE map_name=? ", (map_name,))
                rows = c.fetchall()
                map_info = {"map_name": rows[0][2], "map_path": rows[0][3]}
                c.close()
                conn.close()
                image_path = rows[0][3] + '.png'
                encoded_image = ''

                with open(image_path, 'rb') as f:
                    pgm_data = f.read()
                encoded_image = base64.b64encode(pgm_data)

                self.write(encoded_image)
            except sqlite3.IntegrityError as e:
                re_data = {"msg": str(e)}
                self.write(json.dumps(re_data))
                return
        else:
            try:
                map_name_list = []
                conn = sqlite3.connect(dbPath)
                c = conn.cursor()
                c.execute("SELECT * FROM xgo_map")
                rows = c.fetchall()
                for row in rows:
                    image_path = rows[0][3] + '.png'
                    with open(image_path, 'rb') as f:
                        pgm_data = f.read()
                    encoded_image = base64.b64encode(pgm_data)
                    map_info = {"map_name": row[2], "map_path": row[3], "imagebase64":str(encoded_image)}
                    map_name_list.append(map_info)
                c.close()
                conn.close()
                re_data = {"msg": "ok", "data": map_name_list}
                self.write(json.dumps(re_data))
            except sqlite3.IntegrityError as e:
                re_data = {"msg": str(e)}
                self.write(json.dumps(re_data))
                return

    def post(self):
        # logger.debug("Received POST request in  EditMapHandler")
        map_name = self.get_argument('map_name', None)
        if map_name is not None:
            try:
                map_name_list = []
                conn = sqlite3.connect(dbPath)
                c = conn.cursor()
                c.execute("SELECT * FROM xgo_map")
                rows = c.fetchall()
                for row in rows:
                    map_info = {"map_name": row[2], "map_path": row[3]}
                    map_name_list.append(map_info)
                c.close()
                conn.close()
                re_data = {"msg": "ok", "data": map_name_list}
                self.write(json.dumps(re_data))
            except sqlite3.IntegrityError as e:
                re_data = {"msg": str(e)}
                self.write(json.dumps(re_data))
                return
        else:
            map_name = self.get_argument('map_name', None)
            if map_name is not None:
                try:
                    map_name_list = []
                    conn = sqlite3.connect(dbPath)
                    c = conn.cursor()
                    c.execute("SELECT * FROM xgo_map")
                    rows = c.fetchall()
                    for row in rows:
                        map_info = {"map_name": row[2], "map_path": row[3]}
                        map_name_list.append(map_info)
                    c.close()
                    conn.close()
                    re_data = {"msg": "ok", "data": map_name_list}
                    self.write(json.dumps(re_data))
                except sqlite3.IntegrityError as e:
                    re_data = {"msg": str(e)}
                    self.write(json.dumps(re_data))
                    return


class DeletMapFileHandler(tornado.web.RequestHandler):
    def get(self):
        map_name = self.get_argument("map_name", None)
        if map_name is not None:
            map_name_list = []
            conn = sqlite3.connect(dbPath)
            c = conn.cursor()
            try:
                c.execute('DELETE FROM xgo_map WHERE map_name = ?;', (map_name,))
            except sqlite3.DataError as e:

                re_data = {"msg": str(e)}
                self.write(json.dumps(re_data))
            conn.commit()
            c.execute("SELECT * FROM xgo_map")
            rows = c.fetchall()
            for row in rows:
                map_info = {"map_name": row[2], "map_path": row[3]}
                map_name_list.append(map_info)
            conn.close()
            re_data = {"msg": "ok", "data": map_name_list}
            self.write(json.dumps(re_data))
        else:
            re_data = {"msg": "请输入图片名称"}
            self.write(json.dumps(re_data))

    def post(self):
        map_name = self.get_argument("map_name", None)
        if map_name is not None:
            map_name_list = []
            conn = sqlite3.connect(dbPath)
            c = conn.cursor()
            try:
                c.execute('DELETE FROM xgo_map WHERE map_name = ?;', (map_name,))
            except sqlite3.DataError as e:
                re_data = {"msg": str(e)}
                self.write(json.dumps(re_data))
            conn.commit()
            c.execute("SELECT * FROM xgo_map")
            rows = c.fetchall()
            for row in rows:
                map_info = {"map_name": row[2], "map_path": row[3]}
                map_name_list.append(map_info)
            conn.close()
            re_data = {"msg": "ok", "data": map_name_list}
            self.write(json.dumps(re_data))
        else:
            re_data = {"msg": "请输入图片名称"}
            self.write(json.dumps(re_data))


class SaveMapHandler(tornado.web.RequestHandler):

    def get(self):
        # logger.debug("Received POST request in  EditMapHandler")
        map_name = self.get_argument('map_name', None)
        if map_name is not None:
            map_type = self.get_argument('map_type', None)
            now = datetime.datetime.now()
            str_time = now.strftime("%Y-%m-%d %H:%M:%S.%f")
            map_names = str_time + map_name
            map_id = hashlib.md5(map_names.encode()).hexdigest()
            map_path = mapSavePath + map_name
            command_carto = 'ros2 service call /write_state cartographer_ros_msgs/srv/WriteState' + '"{filename: "{mapSavePath}{map_name}.pbstream"}"'
            if map_type == "Cartographer":
                try:
                    conn = sqlite3.connect(dbPath)
                    c = conn.cursor()
                    c.execute("INSERT INTO xgo_map (map_name, map_id, map_path) VALUES (?, ?, ?)",
                              (map_name, map_id, map_path))
                    run_shellcommand('ros2', 'service', 'call', '/write_state',
                                     'cartographer_ros_msgs/srv/FinishTrajectory', '{"trajectory_id": 0}')

                    #run_shellcommand('ros2', 'service', 'call', '/write_state',
                                     #'cartographer_ros_msgs/srv/WriteState', pbstream_data_json)
                    result = subprocess.run(command_carto, shell=True, capture_output=True)
                    print(result.stdout.decode('utf-8'))

                    run_shellcommand('ros2', 'run', 'cartographer_ros', 'pbstream_to_ros_map_node',
                                     '-map_filestem%s%s' % mapSavePath, map_name,
                                     '-pbstream_filename=%s%s.pbstream' % mapSavePath, map_name,
                                     '-resolution=0.05')

                except sqlite3.IntegrityError as e:
                    re_data = {"msg": str(e)}
                    self.write(json.dumps(re_data))
                    return
                conn.commit()
                conn.close()
                time.sleep(3)
                print("*************************")
                print(map_path)
                print("*************************")
                with open(map_path + ".pgm", 'rb') as f:
                    pgm_data = f.read()
                    pgm_image = Image.open(f)
                    width, height = pgm_image.size

                pgm_image = Image.frombytes('L', (width, height), pgm_data)
                pgm_image.save(map_path + ".png")
                re_data = {"msg": "ok"}
                self.write(json.dumps(re_data))
            else:
                try:
                    conn = sqlite3.connect(dbPath)
                    c = conn.cursor()
                    c.execute("INSERT INTO xgo_map (map_name, map_id, map_path) VALUES (?, ?, ?)",
                              (map_name, map_id, map_path))
                    run_shellcommand('ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                                     '-f', map_path, '--ros-args', '-p', 'save_map_timeout:=10000')
                except sqlite3.IntegrityError as e:
                    re_data = {"msg": str(e)}
                    self.write(json.dumps(re_data))
                    return
                conn.commit()
                conn.close()
                time.sleep(10)
                print("*************************")
                print(map_path)
                print("*************************")
                with open(map_path + ".pgm", 'rb') as f:
                    pgm_data = f.read()
                    pgm_image = Image.open(f)
                    width, height = pgm_image.size
                pgm_image = Image.frombytes('L', (width, height), pgm_data)
                pgm_image.save(map_path + ".png")
                re_data = {"msg": "ok"}
                self.write(json.dumps(re_data))
            #elif map_type == "Gmapping":
            #    re_data = {"msg": "ok"}
            #    self.write(json.dumps(re_data))
            #else:
            #    re_data = {"msg": "不支持"}
            #    self.write(json.dumps(re_data))

    def post(self):
        # logger.debug("Received POST request in  EditMapHandler")
        map_name = self.get_argument('map_name', None)
        if map_name is not None:
            map_type = self.get_argument('map_type', None)
            if map_type == "Cartographer":
                try:
                    now = datetime.datetime.now()
                    str_time = now.strftime("%Y-%m-%d %H:%M:%S.%f")
                    map_names = str_time + map_name
                    map_id = hashlib.md5(map_names.encode()).hexdigest()
                    map_path = "/home/parallels/cartoros2/data/map/" + map_name
                    conn = sqlite3.connect("dbPath")
                    c = conn.cursor()
                    c.execute("INSERT INTO xgo_map (map_name, map_id, map_path) VALUES (?, ?, ?)",
                              (map_name, map_id, map_path))
                    run_shellcommand('ros2', 'service', 'call', '/finish_trajectory',
                                     'cartographer_ros_msgs/srv/FinishTrajectory', '{"trajectory_id": 0}')
                    run_shellcommand('ros2', 'service', 'call', '/write_state', 'cartographer_ros_msgs/srv/WriteState',
                                     '{"filename": "/home/parallels/cartoros/data/maps/%s.pbstream"}' % map_name)
                    run_shellcommand('ros2', 'run', 'cartographer_ros', 'pbstream_to_ros_map_node',
                                     '-map_filestem=/home/parallels/cartoros2/data/maps/%s' % map_name,
                                     '-pbstream_filename=/home/parallels/cartoros2/data/maps/%s.pbstream' % map_name,
                                     '-resolution=0.05')
                except sqlite3.IntegrityError as e:
                    re_data = {"msg": str(e)}
                    self.write(json.dumps(re_data))
                    return

                conn.commit()
                conn.close()
                re_data = {"msg": "ok"}
                self.write(json.dumps(re_data))
            elif map_type == "slamTool":
                re_data = {"msg": "ok"}
                self.write(json.dumps(re_data))
            elif map_type == "Gmapping":
                re_data = {"msg": "ok"}
                self.write(json.dumps(re_data))
            else:
                re_data = {"msg": "不支持"}
                self.write(json.dumps(re_data))


class StartMapHandler(tornado.web.RequestHandler):

    def initialize(self, queue_functions):
        self.put_data = queue_functions['put_data']
        self.get_data = queue_functions['get_data']

    def get(self):
        model = self.get_argument('change_type', None)
        re_data = {"msg": "ok"}
        while True:
            data = self.get_data()
            if data is None:
                break
            if isinstance(data, str):
                print("%%%%%%%%%%%%%%%%%%%%%%%%%%")
                print(data)
                terminate_process_by_name(data)
                time.sleep(2)
                print("%%%%%%%%%%%%%%%%%%%%%%%%%%")
            else:
                tornado.ioloop.IOLoop.current().remove_handler(data.stdout.fileno())
        if model == "mapping":
            mapModel = self.get_argument('mapping_type', None)
            if mapModel == "slamTool":
                process_world = run_shellcommand('ros2', 'launch', 'turtlebot3_gazebo', 'turtlebot3_world.launch.py')
                self.put_data("gzserver")
                self.put_data("gzclient")
                self.put_data("robot_state_publisher")
                self.put_data(process_world)
                print('*********************************')
                print(process_world)
                tornado.ioloop.IOLoop.current().add_handler(
                    process_world.stdout.fileno(), self.handle_subprocess_output, tornado.ioloop.IOLoop.READ)
                time.sleep(1)
                process_slam = run_shellcommand('ros2', 'launch', 'slam_toolbox', 'online_async_launch.py')
                self.put_data("async_slam_toolbox_node")
                self.put_data(process_world)
                self.put_data(process_slam)
                tornado.ioloop.IOLoop.current().add_handler(
                    process_slam.stdout.fileno(), self.handle_subprocess_output, tornado.ioloop.IOLoop.READ)
                self.write(json.dumps(re_data))
            elif mapModel == "Cartographer":
                carType = self.get_argument('carType', None)
                if carType == "xGo":
                    process_world = run_shellcommand('ros2', 'launch', 'xgo_bringup', 'Mapping_bring.launch.py')
                    self.put_data("cartographer_node")
                    self.put_data("occupancy_grid_node")
                    self.put_data("laserscan_to_po")
                    self.put_data(process_world)
                    print('*********************************')
                    print(process_world)
                    tornado.ioloop.IOLoop.current().add_handler(
                        process_world.stdout.fileno(), self.handle_subprocess_output, tornado.ioloop.IOLoop.READ)
                    self.write(json.dumps(re_data))
                elif carType == "rosMasterX1":
                    process_world = run_shellcommand('ros2', 'launch', '/root/rosboard/yahboomCortographerMap.launch.py')
                    self.put_data("joint_state_publisher")
                    self.put_data("robot_state_publisher")   
                    self.put_data("Mcnamu_driver_X3")
                    self.put_data("base_node_X3")
                    self.put_data("imu_filter_madgwick_node")
                    self.put_data("ekf_node")
                    self.put_data("yahboom_joy_X3")
                    self.put_data("sllidar_node")
                    self.put_data("static_transform_publisher")
                    self.put_data("occupancy_grid_node")
                    self.put_data("cartographer_node")
                    self.put_data("laserscan_to_po")
                    self.put_data(process_world)
                    print('*********************************')
                    print(process_world)
                    tornado.ioloop.IOLoop.current().add_handler(
                        process_world.stdout.fileno(), self.handle_subprocess_output, tornado.ioloop.IOLoop.READ)
                    self.write(json.dumps(re_data))
                elif carType == "rosMasterX2":
                    process_world = run_shellcommand('ros2', 'launch', 'xgo_bringup', 'Mapping_bring.launch.py')
                    self.put_data("cartographer_node")
                    self.put_data("occupancy_grid_node")
                    self.put_data("laserscan_to_po")
                    self.put_data(process_world)
                    print('*********************************')
                    print(process_world)
                    tornado.ioloop.IOLoop.current().add_handler(
                        process_world.stdout.fileno(), self.handle_subprocess_output, tornado.ioloop.IOLoop.READ)
                    self.write(json.dumps(re_data))
                elif carType == "rosMasterR2":
                    process_world = run_shellcommand('ros2', 'launch', 'xgo_bringup', 'Mapping_bring.launch.py')
                    self.put_data("cartographer_node")
                    self.put_data("occupancy_grid_node")
                    self.put_data("laserscan_to_po")
                    self.put_data(process_world)
                    print('*********************************')
                    print(process_world)
                    tornado.ioloop.IOLoop.current().add_handler(
                        process_world.stdout.fileno(), self.handle_subprocess_output, tornado.ioloop.IOLoop.READ)
                    self.write(json.dumps(re_data))
                elif carType == "transBot":
                    process_world = run_shellcommand('ros2', 'launch', 'xgo_bringup', 'Mapping_bring.launch.py')
                    self.put_data("cartographer_node")
                    self.put_data("occupancy_grid_node")
                    self.put_data("laserscan_to_po")
                    self.put_data(process_world)
                    print('*********************************')
                    print(process_world)
                    tornado.ioloop.IOLoop.current().add_handler(
                        process_world.stdout.fileno(), self.handle_subprocess_output, tornado.ioloop.IOLoop.READ)
                    self.write(json.dumps(re_data))
                else:
                    self.write(json.dumps(re_data))


            elif mapModel == "Gmapping":
                carType = self.get_argument('carType', None)
                if carType == "xGo":
                    process_world = run_shellcommand('ros2', 'launch', 'xgo_bringup', 'Mapping_bring.launch.py')
                    self.put_data("cartographer_node")
                    self.put_data("occupancy_grid_node")
                    self.put_data("laserscan_to_po")
                    self.put_data(process_world)
                    print('*********************************')
                    print(process_world)
                    tornado.ioloop.IOLoop.current().add_handler(
                        process_world.stdout.fileno(), self.handle_subprocess_output, tornado.ioloop.IOLoop.READ)
                    self.write(json.dumps(re_data))
                elif carType == "rosMasterX1":
                    process_world = run_shellcommand('ros2', 'launch', '/root/rosboard/yahboomGmapping.launch.py')
                    self.put_data("joint_state_publisher")
                    self.put_data("robot_state_publisher")   
                    self.put_data("Mcnamu_driver_X3")
                    self.put_data("base_node_X3")
                    self.put_data("imu_filter_madgwick_node")
                    self.put_data("ekf_node")
                    self.put_data("yahboom_joy_X3")
                    self.put_data("sllidar_node")
                    self.put_data("static_transform_publisher")
                    self.put_data("slam_gmapping")
                    self.put_data("laserscan_to_po")
                    self.put_data(process_world)
                    print('*********************************')
                    print(process_world)
                    tornado.ioloop.IOLoop.current().add_handler(
                        process_world.stdout.fileno(), self.handle_subprocess_output, tornado.ioloop.IOLoop.READ)
                    self.write(json.dumps(re_data))
                elif carType == "rosMasterX2":
                    process_world = run_shellcommand('ros2', 'launch', 'xgo_bringup', 'Mapping_bring.launch.py')
                    self.put_data("cartographer_node")
                    self.put_data("occupancy_grid_node")
                    self.put_data("laserscan_to_po")
                    self.put_data(process_world)
                    print('*********************************')
                    print(process_world)
                    tornado.ioloop.IOLoop.current().add_handler(
                        process_world.stdout.fileno(), self.handle_subprocess_output, tornado.ioloop.IOLoop.READ)
                    self.write(json.dumps(re_data))
                elif carType == "rosMasterR2":
                    process_world = run_shellcommand('ros2', 'launch', 'xgo_bringup', 'Mapping_bring.launch.py')
                    self.put_data("cartographer_node")
                    self.put_data("occupancy_grid_node")
                    self.put_data("laserscan_to_po")
                    self.put_data(process_world)
                    print('*********************************')
                    print(process_world)
                    tornado.ioloop.IOLoop.current().add_handler(
                        process_world.stdout.fileno(), self.handle_subprocess_output, tornado.ioloop.IOLoop.READ)
                    self.write(json.dumps(re_data))
                elif carType == "transBot":
                    process_world = run_shellcommand('ros2', 'launch', 'xgo_bringup', 'Mapping_bring.launch.py')
                    self.put_data("cartographer_node")
                    self.put_data("occupancy_grid_node")
                    self.put_data("laserscan_to_po")
                    self.put_data(process_world)
                    print('*********************************')
                    print(process_world)
                    tornado.ioloop.IOLoop.current().add_handler(
                        process_world.stdout.fileno(), self.handle_subprocess_output, tornado.ioloop.IOLoop.READ)
                    self.write(json.dumps(re_data))
                else:
                    self.write(json.dumps(re_data))
            else:
                self.write(json.dumps(re_data))
        else:
            navigation_type = self.get_argument('navigation_type', None)
            print(navigation_type)
            if navigation_type == 'cartographer':
                map_name = self.get_argument("map_name", None)
                process_navigation = run_shellcommand('ros2', 'launch', 'xgo_bringup', 'Navigation_bringup.launch.py',
                                                      'map_yaml:=/home/parallels/cartoros2/data/maps/' + map_name + ".yaml", 'map:=/home/parallels/cartoros2/data/maps/' + map_name + ".pbstream")
                self.put_data("cartographer_node")
                self.put_data("rviz2")
                self.put_data("map_server")
                self.put_data("amcl")
                self.put_data("lifecycle_manager")
                self.put_data("controller_server")
                self.put_data("planner_server")
                self.put_data("recoveries_server")
                self.put_data("bt_navigator")
                self.put_data("waypoint_follower")
                self.put_data("laserscan_to_po")

                self.put_data(process_navigation)
                print('*********************************')
                tornado.ioloop.IOLoop.current().add_handler(
                    process_navigation.stdout.fileno(), self.handle_subprocess_output, tornado.ioloop.IOLoop.READ)
                time.sleep(1)
                self.write(json.dumps(re_data))
            elif navigation_type == 'Dwa':
                map_name = self.get_argument("map_name", None)
                process_navigation = run_shellcommand('ros2', 'launch', '/root/rosboard/xDwaNavBringUp.launch.py', 'map:=' + mapSavePath + map_name + ".yaml")
                self.put_data("bt_navigator")
                self.put_data("recoveries_server")
                self.put_data("robot_state_publisher")
                self.put_data("joint_state_pub")
                self.put_data("yahboom_joy_X3")
                self.put_data("amcl")
                self.put_data("Mcnamu_driver_X")
                self.put_data("ekf_node")
                self.put_data("lifecycle_manager")
                self.put_data("controller_server")
                self.put_data("planner_server")
                self.put_data("imu_filter_madgwick_node")
                self.put_data("static_transform_publisher")
                self.put_data("waypoint_follower")
                self.put_data("base_node_X3")
                self.put_data("map_server")
                self.put_data("laserscan_to_po")
                self.put_data("lifecycle_manager")
                
                # self.put_data("")
                self.put_data(process_navigation)
                print('*********************************')
                tornado.ioloop.IOLoop.current().add_handler(
                    process_navigation.stdout.fileno(), self.handle_subprocess_output, tornado.ioloop.IOLoop.READ)
                time.sleep(1)
                self.write(json.dumps(re_data))
            elif navigation_type == 'Teb':
                map_name = self.get_argument("map_name", None)
                process_navigation = run_shellcommand('ros2', 'launch', '/root/rosboard/xTebNavBringUp.launch.py', 'map:=' + mapSavePath + map_name + ".yaml")
                self.put_data("bt_navigator")
                self.put_data("recoveries_server")
                self.put_data("robot_state_publisher")
                self.put_data("joint_state_pub")
                self.put_data("yahboom_joy_X3")
                self.put_data("amcl")
                self.put_data("Mcnamu_driver_X")
                self.put_data("ekf_node")
                self.put_data("lifecycle_manager")
                self.put_data("controller_server")
                self.put_data("planner_server")
                self.put_data("imu_filter_madgwick_node")
                self.put_data("static_transform_publisher")
                self.put_data("waypoint_follower")
                self.put_data("base_node_X3")
                self.put_data("map_server")
                self.put_data("laserscan_to_po")
                self.put_data("lifecycle_manager")
                # self.put_data("")
                self.put_data(process_navigation)
                print('*********************************')
                tornado.ioloop.IOLoop.current().add_handler(
                    process_navigation.stdout.fileno(), self.handle_subprocess_output, tornado.ioloop.IOLoop.READ)
                time.sleep(1)
                self.write(json.dumps(re_data))
            else:
                self.write(json.dumps(re_data))

    def post(self):
        model = self.get_argument('change_type', None)
        re_data = {"msg": "ok"}
        while True:
            data = self.get_data()
            if data is None:
                break
            if isinstance(data, str):
                print("%%%%%%%%%%%%%%%%%%%%%%%%%%")
                print(data)
                terminate_process_by_name(data)
                time.sleep(2)
                print("%%%%%%%%%%%%%%%%%%%%%%%%%%")
            else:
                tornado.ioloop.IOLoop.current().remove_handler(data.stdout.fileno())
        if model == "mapping":
            mapModel = self.get_argument('mapping_type', None)
            if mapModel == "slamTool":
                process_world = run_shellcommand('ros2', 'launch', 'turtlebot3_gazebo', 'turtlebot3_world.launch.py')
                self.put_data("gzserver")
                self.put_data("gzclient")
                self.put_data("robot_state_publisher")
                self.put_data(process_world)
                print('*********************************')
                print(process_world)
                tornado.ioloop.IOLoop.current().add_handler(
                    process_world.stdout.fileno(), self.handle_subprocess_output, tornado.ioloop.IOLoop.READ)
                time.sleep(1)
                process_slam = run_shellcommand('ros2', 'launch', 'slam_toolbox', 'online_async_launch.py')
                self.put_data("async_slam_toolbox_node")
                self.put_data(process_world)
                self.put_data(process_slam)
                tornado.ioloop.IOLoop.current().add_handler(
                    process_slam.stdout.fileno(), self.handle_subprocess_output, tornado.ioloop.IOLoop.READ)
                self.write(json.dumps(re_data))
            elif mapModel == "Cartographer":
                process_world = run_shellcommand('ros2', 'launch', 'xgo_bringup', 'Mapping_bring.launch.py')
                self.put_data("cartographer_node")
                self.put_data("occupancy_grid_node")
                self.put_data("laserscan_to_po")
                self.put_data(process_world)
                print('*********************************')
                print(process_world)
                tornado.ioloop.IOLoop.current().add_handler(
                    process_world.stdout.fileno(), self.handle_subprocess_output, tornado.ioloop.IOLoop.READ)
                self.write(json.dumps(re_data))
            elif mapModel == "Gmapping":
                self.write(json.dumps(re_data))
            else:
                self.write(json.dumps(re_data))
        else:
            process_navigation = run_shellcommand('ros2', 'launch', 'nav2_bringup', 'tb3_simulation_launch.py')
            self.put_data("gzserver")
            self.put_data("gzclient")
            self.put_data("robot_state_publisher")
            self.put_data("rviz2")
            self.put_data("amcl")
            self.put_data("lifecycle_manager")
            self.put_data("controller_server")
            self.put_data("planner_server")
            self.put_data("recoveries_server")
            self.put_data("bt_navigator")
            self.put_data("waypoint_follower")
            # self.put_data("")
            self.put_data(process_navigation)
            print('*********************************')
            tornado.ioloop.IOLoop.current().add_handler(
                process_navigation.stdout.fileno(), self.handle_subprocess_output, tornado.ioloop.IOLoop.READ)
            time.sleep(1)
            self.write(json.dumps(re_data))

    def delete(self, *args, **kwargs):
        re_data = {"msg": "ok"}
        while True:
            data = self.get_data()
            if data is None:
                break
        if isinstance(data, str):
            print("%%%%%%%%%%%%%%%%%%%%%%%%%%")
            print(data)
            terminate_process_by_name(data)
            time.sleep(2)
            print("%%%%%%%%%%%%%%%%%%%%%%%%%%")
        else:
            tornado.ioloop.IOLoop.current().remove_handler(data.stdout.fileno())
        self.write(json.dumps(re_data))

    def handle_subprocess_output(self, fd, events):
        # 读取子进程输出并响应请求
        data = os.read(fd, 1024)
        if data:
            print(data)
        else:
            # 子进程输出结束，取消文件描述符的注册
            tornado.ioloop.IOLoop.current().remove_handler(fd)


class ROSBoardSocketHandler(tornado.websocket.WebSocketHandler):
    sockets = set()

    def initialize(self, node):
        """store the instance of the ROS node that created this WebSocketHandler
        so we can access it later"""
        self.node = node

    def get_compression_options(self):
        # Non-None enables compression with default options.
        return {}

    def open(self):
        self.id = uuid.uuid4()  # unique socket id
        self.latency = 0  # latency measurement
        self.last_ping_times = [0] * 1024
        self.ping_seq = 0

        self.set_nodelay(True)

        # polyfill of is_closing() method for older versions of tornado
        if not hasattr(self.ws_connection, "is_closing"):
            self.ws_connection.is_closing = types.MethodType(
                lambda self_: self_.stream.closed() or self_.client_terminated or self_.server_terminated,
                self.ws_connection
            )

        self.update_intervals_by_topic = {}  # this socket's throttle rate on each topic
        self.last_data_times_by_topic = {}  # last time this socket received data on each topic

        ROSBoardSocketHandler.sockets.add(self)

        self.write_message(json.dumps([ROSBoardSocketHandler.MSG_SYSTEM, {
            "hostname": socket.gethostname(),
            "version": __version__,
        }], separators=(',', ':')))

    def on_close(self):
        ROSBoardSocketHandler.sockets.remove(self)

        # when socket closes, remove ourselves from all subscriptions
        for topic_name in self.node.remote_subs:
            if self.id in self.node.remote_subs[topic_name]:
                self.node.remote_subs[topic_name].remove(self.id)

    @classmethod
    def send_pings(cls):
        """
        Send pings to all sockets. When pongs are received they will be used for measuring
        latency and clock differences.
        """

        for socket in cls.sockets:
            try:
                socket.last_ping_times[socket.ping_seq % 1024] = time.time() * 1000
                if socket.ws_connection and not socket.ws_connection.is_closing():
                    socket.write_message(json.dumps([ROSBoardSocketHandler.MSG_PING, {
                        ROSBoardSocketHandler.PING_SEQ: socket.ping_seq,
                    }], separators=(',', ':')))
                socket.ping_seq += 1
            except Exception as e:
                print("Error sending message: %s" % str(e))

    @classmethod
    def broadcast(cls, message):
        """
        Broadcasts a dict-ivied ROS message (message) to all sockets that care about that topic.
        The dict message should contain metadata about what topic it was
        being sent on: message["_topic_name"], message["_topic_type"].
        """

        try:
            if message[0] == ROSBoardSocketHandler.MSG_TOPICS:
                json_msg = json.dumps(message, separators=(',', ':'))
                for socket in cls.sockets:
                    if socket.ws_connection and not socket.ws_connection.is_closing():
                        socket.write_message(json_msg)
            elif message[0] == ROSBoardSocketHandler.MSG_MSG:
                topic_name = message[1]["_topic_name"]
                json_msg = None
                for socket in cls.sockets:
                    if topic_name not in socket.node.remote_subs:
                        continue
                    if socket.id not in socket.node.remote_subs[topic_name]:
                        continue
                    t = time.time()
                    if t - socket.last_data_times_by_topic.get(topic_name, 0.0) < \
                            socket.update_intervals_by_topic.get(topic_name) - 2e-4:
                        continue
                    if socket.ws_connection and not socket.ws_connection.is_closing():
                        if json_msg is None:
                            json_msg = json.dumps(message, separators=(',', ':'))
                        socket.write_message(json_msg)
                    socket.last_data_times_by_topic[topic_name] = t
        except Exception as e:
            print("Error sending message: %s" % str(e))
            traceback.print_exc()

    def on_message(self, message):
        """
        Message received from the client.
        """

        if self.ws_connection.is_closing():
            return

        # JSON decode it, give up if it isn't valid JSON
        try:
            argv = json.loads(message)
        except (ValueError, TypeError):
            print("error: bad: %s" % message)
            return

        # make sure the received argv is a list and the first element is a string and indicates the type of command
        if type(argv) is not list or len(argv) < 1 or type(argv[0]) is not str:
            print("error: bad: %s" % message)
            return

        # if we got a pong for our own ping, compute latency and clock difference
        elif argv[0] == ROSBoardSocketHandler.MSG_PONG:
            if len(argv) != 2 or type(argv[1]) is not dict:
                print("error: pong: bad: %s" % message)
                return

            received_pong_time = time.time() * 1000
            self.latency = (received_pong_time - self.last_ping_times[
                argv[1].get(ROSBoardSocketHandler.PONG_SEQ, 0) % 1024]) / 2
            if self.latency > 1000.0:
                self.node.logwarn("socket %s has high latency of %.2f ms" % (str(self.id), self.latency))

            if self.latency > 10000.0:
                self.node.logerr(
                    "socket %s has excessive latency of %.2f ms; closing connection" % (str(self.id), self.latency))
                self.close()

        # client wants to subscribe to topic
        elif argv[0] == ROSBoardSocketHandler.MSG_SUB:
            if len(argv) != 2 or type(argv[1]) is not dict:
                print("error: sub: bad: %s" % message)
                return

            topic_name = argv[1].get("topicName")
            max_update_rate = float(argv[1].get("maxUpdateRate", 24.0))

            self.update_intervals_by_topic[topic_name] = 1.0 / max_update_rate
            self.node.update_intervals_by_topic[topic_name] = min(
                self.node.update_intervals_by_topic.get(topic_name, 1.),
                self.update_intervals_by_topic[topic_name]
            )

            if topic_name is None:
                print("error: no topic specified")
                return

            if topic_name not in self.node.remote_subs:
                self.node.remote_subs[topic_name] = set()

            self.node.remote_subs[topic_name].add(self.id)
            self.node.sync_subs()

        # client wants to unsubscribe from topic
        elif argv[0] == ROSBoardSocketHandler.MSG_UNSUB:
            if len(argv) != 2 or type(argv[1]) is not dict:
                print("error: unsub: bad: %s" % message)
                return
            topic_name = argv[1].get("topicName")

            if topic_name not in self.node.remote_subs:
                self.node.remote_subs[topic_name] = set()

            try:
                self.node.remote_subs[topic_name].remove(self.id)
            except KeyError:
                print("KeyError trying to remove sub")


ROSBoardSocketHandler.MSG_PING = "p"
ROSBoardSocketHandler.MSG_PONG = "q"
ROSBoardSocketHandler.MSG_MSG = "m"
ROSBoardSocketHandler.MSG_TOPICS = "t"
ROSBoardSocketHandler.MSG_SUB = "s"
ROSBoardSocketHandler.MSG_SYSTEM = "y"
ROSBoardSocketHandler.MSG_UNSUB = "u"

ROSBoardSocketHandler.PING_SEQ = "s"
ROSBoardSocketHandler.PONG_SEQ = "s"
ROSBoardSocketHandler.PONG_TIME = "t"
