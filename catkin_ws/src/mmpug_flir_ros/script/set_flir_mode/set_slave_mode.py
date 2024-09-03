#!/usr/bin/python3

import os
from Boson_SDK import *
import time

myport = pyClient.Initialize(manualport="/dev/ttyACM0")

# set external trigger slave mode
result = pyClient.bosonSetExtSyncMode(FLR_BOSON_EXT_SYNC_MODE_E.FLR_BOSON_EXT_SYNC_DISABLE_MODE)
print("Set external trigger disable mode: ", result)
time.sleep(1)
result = pyClient.bosonSetExtSyncMode(FLR_BOSON_EXT_SYNC_MODE_E.FLR_BOSON_EXT_SYNC_SLAVE_MODE)
print("Set external trigger slave mode: ", result)

# run boson FFC
# result = pyClient.bosonRunFFC()
# print("FFC result: ", result)

pyClient.Close(myport)
