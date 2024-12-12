#!/bin/bash

BOT_DIR="/home/sirc/Scrivania/rasa-3.x-form-examples-main/02-slots"

cd $BOT_DIR

rasa run -m models --endpoints endpoints.yml --port 5002 --credentials credentials.yml --enable-api
