import socket
import json
from qgis.core import (
    QgsProject,
    QgsVectorLayer,
    QgsFeature,
    QgsGeometry,
    QgsPointXY
)
from PyQt5.QtCore import QTimer

UDP_IP = "127.0.0.1"
UDP_PORT = 9100

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)

layer = QgsVectorLayer("Point?crs=EPSG:4326", "UDP Points", "memory")
pr = layer.dataProvider()
QgsProject.instance().addMapLayer(layer)

def update():
    try:
        data, addr = sock.recvfrom(1024)
        message = json.loads(data.decode("utf-8"))
        lat = message.get("lat")
        lon = message.get("lon")
        if lat is not None and lon is not None:
            point = QgsFeature()
            point.setGeometry(QgsGeometry.fromPointXY(QgsPointXY(lon, lat)))
            pr.addFeatures([point])
            layer.updateExtents()
            layer.triggerRepaint()
            print(f"Point: {lat}, {lon}")
    except socket.error:
        pass

timer = QTimer()
timer.timeout.connect(update)
timer.start(100)

print("UDP was started...")