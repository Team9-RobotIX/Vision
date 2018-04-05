import requests
import json

class Dispatch:

    def __init__(self):
        self.updates = []

    def add(self, update):
        self.updates.append(update)

    def overwrite(self, update):
        for i in range(len(self.updates)):
            if self.updates[i]['robot'] == update['robot']:
                self.updates[i] = update

    def dispatch(self):
        print(self.updates)
        if len(self.updates) == 0:
            return
        try:
            r =requests.post('http://35.177.199.115/development/batch', data=json.dumps(self.updates))
        except Exception as e:
            print('err sending') 
        self.updates = []
