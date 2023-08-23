import os
import random
import influxdb_client
from influxdb_client.client.write_api import SYNCHRONOUS
from dotenv import load_dotenv, find_dotenv
from datetime import datetime

INFLUXDB_TOKEN = "ZjOH7ZBTua9oo9nTwjGK-uL6DQyos-5rhNeMJ6fWndzG4kOtEpFYqT_K6lcjhY2_m9fSdj-GiAkGYSPEsAoGlQ=="
INFLUXDB_ORG = "IPA"
INFLUXDB_BUCKET = "rokit-db"
INFLUXDB_URL = "127.0.1.1:8086"


class writeInfluxDB():
    def __init__(self) -> None:
        load_dotenv(find_dotenv())
        self.token = INFLUXDB_TOKEN
        self.org = INFLUXDB_ORG
        self.bucket = INFLUXDB_BUCKET
        self.url = INFLUXDB_URL
        self.client = influxdb_client.InfluxDBClient(
            url=self.url,
            token=self.token,
            org=self.org,
            bucket=self.bucket)
        self.write_api = self.client.write_api(write_options=SYNCHRONOUS)

    def writePoints(self, data):
        time = datetime.utcnow().strftime('%Y-%m-%dT%H:%M:%SZ')
        p = influxdb_client.Point("Testcase" + str(data.trial_number)).field("floor_type", data.floor_type).field("tracking_object",data.tracking_object).field("robottype", data.robottype).field("humidity", data.humidity).field("inclindation", data.inclination).field("notes", data.notes).field("temperature",data.temperature).tag("testtype", data.testtype).field("velocity_meters_per_second", data.meters_per_second).time(time)
        self.write_api.write(bucket=self.bucket, org=self.org, record=p)


def write_content(data):
    write_db = writeInfluxDB()
    write_db.writePoints(data)


