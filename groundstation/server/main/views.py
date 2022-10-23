# groundstation/server/main/views.py
import threading
from datetime import datetime
from flask import render_template, Blueprint, request
from flask_socketio import emit
from ..db.dbManagement import DbClient
from influxdb_client import Point, WritePrecision
from threading import Thread, Event
from .simulation import sim_thread

from .. import socketio

main_blueprint = Blueprint("main", __name__, )

terminate_sim_event = Event()
_sim_lock = threading.Lock()


@main_blueprint.route("/", methods=["GET"])
def home():
    return render_template("index.html")


'''
def sim_thread():
    _sim_lock.acquire()
    count = 0
    db_client = DbClient()
    while True:
        socketio.sleep(1)
        count += 1
        socketio.emit('sim_msg',
                      {'data': 'Server generated event', 'count': count})
        point = Point("fgu_raw") \
            .field("payload", int(count)) \
            .time(datetime.utcnow(), WritePrecision.NS)
        db_client.write_point(point)

        if terminate_sim_event.is_set():
            _sim_lock.release()
            break
'''


def send_msg(message):
    print(message)


@socketio.on('connect')
def connect(message):
    print('connect ', request.sid)
    emit('server_msg', {'data': 'connected'})


@socketio.on('start_sim')
def start_sim(message):
    print('start_sim request')
    if _sim_lock.locked() is False:
        # thread = Thread(target=sim_thread(), daemon=False, name='sim_thread')
        # thread.start()
        # Todo: was ist besser?
        # socketio.start_background_task(target=sim_thread())
        socketio.start_background_task(
            target=sim_thread(
                lock=_sim_lock,
                sio=socketio,
                terminate_event=terminate_sim_event)
        )


@socketio.on('stop_sim')
def stop_sim(message):
    print('stop_sim request')
    terminate_sim_event.set()
    while _sim_lock.locked():
        pass
    terminate_sim_event.clear()


@socketio.on('send_cmd')
def send_cmd(message):
    thread = Thread(target=send_msg(message))
    thread.daemon = False
    thread.start()
