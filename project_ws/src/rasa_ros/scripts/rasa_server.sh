#!/bin/bash

BOT_DIR="$(dirname "$0")/../../../../chatbot"

cd $BOT_DIR

rasa run -m models --endpoints endpoints.yml --port 5002 --credentials credentials.yml --enable-api
