#!/bin/bash

BOT_DIR="$(dirname "$0")/../../../../chatbot"

cd $BOT_DIR

rasa run actions
