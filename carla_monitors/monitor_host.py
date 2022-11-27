from pydoc import cli
from socketserver import ThreadingTCPServer,StreamRequestHandler
import socket
from datetime import datetime

HOST = "127.0.0.1"  # The server's hostname or IP address
PORT = 65001  # The port used by the server
#s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_present = False
logfile = "'Monitor Logfile {time}".format(time = str(datetime.now()))
comma = ""

class echohandler(StreamRequestHandler):
    def handle(self):
        print(f'Connected: {self.client_address[0]}:{self.client_address[1]}')
        client_present = False
        global comma
        while True:

            # get message
            msg = self.rfile.readline()
            if not msg:
                print(f'Disconnected: {self.client_address[0]}:{self.client_address[1]}')
                break # exits handler, framework closes socket
            print(f'Received: {msg.decode()}')
            self.wfile.write(msg)
            self.wfile.flush()

            if(not msg.decode().startswith("Turn")):
                f = open(logfile, "a")
                f.write(comma + "{\n\t\"Type\" : \"Broken Rule\"\n\t\"Time\" : \""+str(datetime.now())+"\"\n\t\"Message\" : \""+msg.decode()+"\"\n}")
                f.close()
                comma = ",\n"

            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.connect((HOST, PORT))
                    s.sendall(f"{msg.decode()}".encode())
                    s.close()
                client_present = True
            except:
                client_present = False
            
            client_present = False

ThreadingTCPServer.allow_reuse_address = True
server = ThreadingTCPServer(('',65000),echohandler)
server.serve_forever()