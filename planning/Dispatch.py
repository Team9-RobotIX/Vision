import requests
import json

class Dispatch:

    def __init__(self):
        self.updates = []

    def add(self, update):
        self.updates.append(update)

    def dispatch(self):
        #print(self.updates)
        if len(self.updates) == 0:
            return
        #print(self.updates[0])
        r =requests.post('http://35.177.199.115/development/batch', data=json.dumps(self.updates))
        self.updates = []
