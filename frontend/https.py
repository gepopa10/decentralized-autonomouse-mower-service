#!/usr/bin/env python

import http.server, ssl
import sys, getopt

def main(argv):
    address = '0.0.0.0'
    port = 4443
    try:
        opts, args = getopt.getopt(argv,"ha:p:",["address=","port="])
    except getopt.GetoptError:
        print('https.py -a <address> -p <port>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print('https.py -a <address> -p <port>')
            sys.exit()
        elif opt in ("-a", "--address"):
            address = arg
        elif opt in ("-p", "--port"):
            port = arg

    server_address = (address, int(port))
    httpd = http.server.HTTPServer(server_address, http.server.SimpleHTTPRequestHandler)
    httpd.socket = ssl.wrap_socket(httpd.socket,
                                   server_side=True,
                                   certfile='localhost.pem',
                                   ssl_version=ssl.PROTOCOL_TLS)
    print(f"Serving HTTPS on {address} port {str(port)} (https://{address}:{str(port)}/) ...")
    httpd.serve_forever()

if __name__ == '__main__':
    main(sys.argv[1:])
