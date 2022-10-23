import json
import random
from datetime import datetime
from threading import Lock
from ..db.dbManagement import DbClient
from flask_socketio import SocketIO
from influxdb_client import Point, WritePrecision
from threading import Event

tracking_data = {
    "payload_type": "TRACKING",
    "Longitude": 1.0,
    "Latitude": 1.0
}

motion_data = {
    "payload_type": "MOTION",
    "ID": 1,
    "Time": "",
    "Roll": 1.0,
    "Pitch": 1.0,
    "Yaw": 1.0,
    "Sun": 1.0,
    "Satus": 1,
    "Reserved0": "",
    "Reserved1": "",
    "Reserved2": ""
}

environment_data = {
    "ID": 1,
    "Time": "",
    "Longitude": 1.0,
    "Latitude": 1.0,
    "Pressure": 1.0,
    "Gas_Concentration": 1.0,
    "Servo": 1.0,
    "Temperature": 1.0,
    "Temp": 1.0,
    "Status": 1,
    "Reserved0": "",
    "Reserved1": "",
    "Reserved2": ""
}


def create_payload(payload_type):
    match payload_type:
        case 'tracking':
            return tracking_payload()
        case 'motion':
            return motion_payload()
        case 'environment':
            return environment_payload()


def tracking_payload():
    data = tracking_data
    data["Longitude"] = 11.586111
    data["Latitude"] = 50.927223
    return data


def environment_payload():
    data = environment_data
    data["Time"] = str(datetime.now())
    data["Longitude"] = 11.586111
    data["Latitude"] = 50.927223
    data["Pressure"] = 1.013
    data["Gas_Concentration"] = 20.95
    data["Servo"] = 10
    data["Temperature"] = -20
    data["Status"] = 1
    return data


def motion_payload():
    data = motion_data
    data["Time"] = str(datetime.now())
    data["Roll"] = 1
    data["Pitch"] = 1
    data["Yaw"] = 1
    data["Sun"] = 1
    data["Status"] = 1
    return data


def sim_thread(lock: Lock, sio: SocketIO, terminate_event: Event):
    lock.acquire()
    db_client = DbClient()
    while True:
        msg = create_payload('motion')

        sequence = ["fgu,fgu=motion ID={0}".format(msg["ID"]),
                    "fgu,fgu=motion Roll={0}".format(msg["Roll"]),
                    "fgu,fgu=motion Pitch={0}".format(msg["Pitch"]),
                    "fgu,fgu=motion Yaw={0}".format(msg["Yaw"]),
                    "fgu,fgu=motion Sun={0}".format(msg["Sun"]),
                    "fgu,fgu=motion Status={0}".format(msg["Status"])]

        db_client.write_sequence(sequence=sequence)

        # point = Point("fgu_raw") \
        #    .field("payload", int(count)) \
        #    .time(datetime.utcnow(), WritePrecision.NS)
        # db_client.write_point(point)

        msg = json.dumps(msg, indent=4)
        sio.emit('sim_msg', msg)

        if terminate_event.is_set():
            lock.release()
            break

        sio.sleep(1)
