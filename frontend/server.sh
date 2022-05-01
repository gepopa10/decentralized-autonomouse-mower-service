#!/bin/bash
python3 -m http.server 8000 & \
lt --port 8000 --subdomain mower && fg
