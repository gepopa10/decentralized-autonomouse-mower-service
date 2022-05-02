#!/bin/bash
python3 -m http.server 8000 & \
ngrok http 8000 && fg
