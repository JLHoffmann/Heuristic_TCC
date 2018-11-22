#!/usr/bin/env python3

# To get an unencrypted PEM (without passphrase):
# openssl rsa -in certificate.pem -out certificate_unencrypted.pem

import os
import sys
import time
#import serial
import argparse
import requests
import json

parser = argparse.ArgumentParser(description='EPOS Serial->IoT Gateway')

required = parser.add_argument_group('required named arguments')
required.add_argument('-c','--certificate', help='Your PEM certificate', required=True)

parser.add_argument('-d','--dev', help='EPOSMote III device descriptor file', default='/dev/ttyACM0')
parser.add_argument('-t','--timeout', help='Timeout for reading from mote', default=600)
parser.add_argument('-u','--url', help='Post URL', default='https://iot.lisha.ufsc.br/api/put.php')
parser.add_argument('-a','--attach_url', help='Attach URL', default='https://iot.lisha.ufsc.br/api/attach.php')
parser.add_argument('-j','--json', help='Use JSON API', action='store_true')
parser.add_argument('-D','--domain', help='Data domain', default='')
parser.add_argument('-U','--username', help='Data domain username', default='')
parser.add_argument('-P','--password', help='User password', default='')

args = vars(parser.parse_args())
DEV = args['dev']
TIMEOUT = int(args['timeout'])
URL = args['url']
ATTACH_URL = args['attach_url']
MY_CERTIFICATE = [args['certificate']+'.pem', args['certificate']+'.key']
DOMAIN = args['domain']
USERNAME = args['username']
PASSWORD = args['password']
JSON = args['json']

if JSON:
    import json
    import struct
    from struct import unpack

attached = []

if __name__ == "__main__":

	session = requests.Session()
	session.headers = {'Content-type' : 'application/json'}
	session.cert = MY_CERTIFICATE

	with open('strings.json') as json_data:
	    d = json.load(json_data)
	    
	
	if not d in attached:
			try:
				#print(json.dumps(d))
				response = session.post(ATTACH_URL, json.dumps(d))
	    			#print("[", str(response.status_code), "] (", len(d), ") ", d, sep='')
				print("SEND", str(response.status_code))
				#attached.append(d)
			except Exception as e:
				#print("Exception caught:", e, file=sys.stderr)
				#print("Exception caught:", e)
				print("ERROR", )
	else:
			print("ERROR")
	#print d

