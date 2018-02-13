import requests
url = "http://ec2-18-219-63-23.us-east-2.compute.amazonaws.com/development/instructionsPost"
data = {'instruction': 'MOVE', 'value': '500'}
r = requests.post(url, data = data)
print (r.text)
