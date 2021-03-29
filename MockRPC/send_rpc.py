import os
import json
import time
import requests


TIME = 45
DEVICE_ID = "a9e821a0-8caf-11eb-950e-efef5c07c810"
BASE = 'https://thingsboard.e-yantra.org/api'

response = requests.post(f'{BASE}/auth/login', 
                json={
                	"username": "cs684student10@e-yantra.org",
					"password": "cs684student10"
				},
                headers={
                    'Accept': 'application/json',
                    'Content-Type': 'application/json'
                }
            )

JWT_TOKEN = response.json()['token']

if __name__ == "__main__":
    with open('rpc.json') as fp:
        rpc_list = json.load(fp)
    for req in rpc_list:
        try:
            response = requests.post(f'{BASE}/plugins/rpc/oneway/{DEVICE_ID}', 
                json=req,
                headers={
                    'X-Authorization': f'Bearer {JWT_TOKEN}',
                    'Content-Type': 'application/json'
                })
            if response.status_code // 100 == 2:
            	print(f'{req} successfully sent.')
            	# print(response.json())
            else:
            	print(f'Failed {response.json()}')
        except requests.exceptions.RequestException as e:
            print(f'Problem in sending request: {req}')
            print(e)
        time.sleep(TIME)
