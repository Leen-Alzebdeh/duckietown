import os

message = "Hello World! %s!" % os.environ['VEHICLE_NAME']
print(message)