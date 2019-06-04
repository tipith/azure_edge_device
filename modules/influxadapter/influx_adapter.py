from influxdb import InfluxDBClient
import logging, os, time

logger = logging.getLogger('influx adapter')


class InfluxAdapter:

    def __init__(self, dbname):
        self.influx = InfluxDBClient('influxdb', 8086, 'root', 'root', dbname)
        self._connect(dbname)

    def _connect(self, dbname):
        while True:
            try:
                dbs = self.influx.get_list_database()
                if dbname not in dbs:
                    self.influx.create_database(dbname)
            except Exception:
                logger.exception("Error connecting to InfluxDB. Retrying in 30sec")
                time.sleep(30)
                continue
            else:
                logging.info("connected to influxdb")
                break

    def _write(self, payload):
        while True:
            try:
                self.influx.write_points(payload)
            except Exception:
                logger.exception("Error writing to InfluxDB. Retrying in 30sec")
                time.sleep(30)
                continue
            else:
                break

    def _convert_msg(self, devid, msg):
        fields = { 
            'temp': msg['temp1'], 
            'hum': msg['hum'], 
            'press': msg['press'] }
        return [{'measurement': devid, 'time': msg['timestamp'], 'fields': fields}]

    def add(self, dev, msg):
        print(dev, msg)
        imsg = self._convert_msg(dev, msg)
        self._write(imsg)