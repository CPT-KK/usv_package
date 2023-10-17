import rospy
import socket
import json
import re

client_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

server_host = '192.168.147.50'
server_pot = 9999

request_data = {"get":"radioinfo"}
request_json = json.dumps(request_data).encode('utf-8')

client_socket.sendto(request_json,(server_host,server_pot))

responce, server_address = client_socket.recvfrom(1024)

responce_str = responce.decode('utf-8')

match = re.search(r'{.*}',responce_str)

if match:
    valid_json = match.group(0)
    try:
        responce_data = json.loads(valid_json)

    except json.JSONDecodeError as e:
        print(f"JSON decoding error: {e}")

for sender in responce_data["senders"]:
    dist = sender["dist"]
    ipAddr = sender["ipAddr"]
    print("dist:", dist, "ipAddr:", ipAddr)

client_socket.close()