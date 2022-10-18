import json
import os
from influxdb_client import InfluxDBClient
from influxdb_client.client.flux_table import FluxStructureEncoder
from influxdb_client.client.write_api import SYNCHRONOUS
from dotenv import load_dotenv

load_dotenv('influxdb-docker.env')

ENV_TOKEN = os.getenv('TOKEN')
ENV_ORG = os.getenv('ORG')
ENV_BUCKET = os.getenv('BUCKET')


def encode_query_response(table):
    return json.dumps(table, cls=FluxStructureEncoder, indent=2)


class DbClient:
    def __init__(self):
        self.client = InfluxDBClient(url="http://influxdb:8086", token=ENV_TOKEN, org=ENV_ORG)
        # self.client = InfluxDBClient.from_config_file("config.ini")
        self.write_api = self.client.write_api(write_options=SYNCHRONOUS)

    def write_sequence(self, sequence):
        self.write_api.write(ENV_BUCKET, ENV_ORG, sequence)

    def write_point(self, point):
        self.write_api.write(ENV_BUCKET, ENV_ORG, point)

    def close(self):
        self.client.close()

    def get_last_payload(self, measurement, field):
        query = 'from(bucket: "' + ENV_BUCKET + '") ' \
                '|> range(start: -10s) ' \
                '|> filter(fn: (r) => r._measurement == "' + measurement + '") ' \
                '|> filter(fn: (r) => r._field == "' + field + '") ' \
                '|> last()'
        return encode_query_response(self.client.query_api().query(query))

    def get_payloads(self, timeframe, measurement, field):
        query = 'from(bucket: "' + ENV_BUCKET + '") ' \
                '|> range(start: -' + timeframe + ') ' \
                '|> filter(fn: (r) => r._measurement == "' + measurement + '") ' \
                '|> filter(fn: (r) => r._field == "' + field + '")'
        return encode_query_response(self.client.query_api().query(query))

    def query(self, timeframe):
        query = 'from(bucket: "' + ENV_BUCKET + '") |> range(start: ' + timeframe + ')'
        return encode_query_response(self.client.query_api().query(query))

    def query_last(self):
        query = 'from(bucket: "' + ENV_BUCKET + '") |> range(start: -1h) |> last()'
        return encode_query_response(self.client.query_api().query(query))
