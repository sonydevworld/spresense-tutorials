#!/usr/bin/env python3

#
# Simple command line utility for interacting with the Senseye.io API
# to list existing or create new sensor ids
#

import argparse
import getpass
import json
import os
import requests
import sys
import textwrap

token = ""

#
# Authenticates with the API. Username and password will be prompted if missing.
# Te retrieved token is stored in '.token' to avoid authentication next time.
#
def api_authenticate(user=None, password=None):
    global token

    if user is None:
        user = input("Username: ")
    if password is None:
        password = getpass.getpass("Password: ")

    payload = { 'username' : user, 'password' : password, 'grant_type' : 'password' }

    r = requests.post('https://api.senseye.io/oauth/token', data=payload)
    if r.status_code == requests.codes.ok:
        print("Authentication OK")
        token = r.json()['access_token']
        with open('.token', 'w') as f:
            f.write(token)
        return True
    else:
        print("Authentication failed: {}\nURL: {}".format(r.text), r.url)
    return False

#
# Send a new HTTP GET (default) or POST request to the API with the given body (if any)
#
def api_request(url, is_post=False, body=None):
    global token

    headers = {'Authorization': 'Bearer {}'.format(token)}

    if is_post:
        r = requests.post(url, headers=headers, data=body)
    else:
        r = requests.get(url, headers=headers, data=body)

    if r.status_code == requests.codes.ok:
        return r.json()
    else:
        print("{} failed: {}".format('POST' if is_post else 'GET', r.text))
        print(r.url)
    return None

#
# Retreive the name of the sensor with the given id
#
def api_get_name(sensor_id):
    json = api_request('https://api.senseye.io/v1/hub/sensors/{}'.format(sensor_id))
    if json != None:
        return json['name']
    else:
        return ""

#
# Retreive and print all sensor ids and names
#
def api_list_sensors():
    json = api_request('https://api.senseye.io/v1/hub/sensors')
    if json != None:
        print("Sensors:")
        for s in json['sensors']:
            print("  {} \"{}\"".format(s, api_get_name(s)))

#
# Create a new sensor id with the given name
#
def api_create_sensor(name):
    body = { 'name' : name }
    json = api_request('https://api.senseye.io/v1/hub/sensors', True, body)
    if json != None:
        print("Created:")
        print("  \"{}\" = {}".format(name, json['sensorId']))
        print("{} left".format(json['unactivatedSensorsRemaining']))

#
# Create and eturn a new command line argument parser
#
def get_parser():
    parser = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter,
        description=textwrap.dedent('''Simple utility to interact with the Senseye.io API.\n'''))
    parser.add_argument('-l', '--list', dest='list_sensors', action='store_true',
        help='list all registered sensors')
    parser.add_argument('-c', '--create', dest='create', metavar='NAME', type=str, default=None,
        help='create a new sensor with the given name')
    return parser


if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    # Read token from file if possible, otherwise authenticate
    if os.path.isfile('.token'):
        with open('.token', 'r') as f:
            token = f.read()
    else:
        if not api_authenticate():
            exit(0)

    # Handle commands
    if args.list_sensors:
        api_list_sensors()
    elif args.create:
        api_create_sensor(args.create)
    else:
        parser.print_help(sys.stderr)

