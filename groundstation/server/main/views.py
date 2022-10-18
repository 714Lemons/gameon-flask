# groundstation/server/main/views.py
import asyncio
from datetime import datetime
from threading import Lock
from flask import render_template, Blueprint, jsonify, request
from flask_socketio import SocketIO, emit
from ..db.dbManagement import DbClient
from influxdb_client import Point, WritePrecision
from threading import Thread

from .. import socketio

main_blueprint = Blueprint("main", __name__, )
loop = asyncio.get_event_loop()


@main_blueprint.route("/", methods=["GET"])
def home():
    return render_template("index.html")


def sim_message():
    count = 0
    db_client = DbClient()
    while True:
        socketio.sleep(1)
        count += 1
        socketio.emit('my_response',
                      {'data': 'Server generated event', 'count': count})
        point = Point("fgu_raw") \
            .field("payload", int(count)) \
            .time(datetime.utcnow(), WritePrecision.NS)
        db_client.write_point(point)


@socketio.on('connect')
def connect(message):
    print('connect ', request.sid)
    thread = Thread(target=sim_message)
    thread.daemon = True
    thread.start()
    emit('my_response', {'data': 'data', 'count': 1}, broadcast=True)
