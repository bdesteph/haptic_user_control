from pythonosc import osc_message_builder
from pythonosc import udp_client
from pythonosc import osc_server
from pythonosc import dispatcher
from pythonosc.osc_server import AsyncIOOSCUDPServer
import asyncio

import numpy as np
import matplotlib.pyplot as plt

# Server and client must have their own port

# We send via the client:
client = udp_client.SimpleUDPClient("127.0.0.1", 1234)

x_positions = []
slider_focused = False

def sliderFocus(unused_addr, slider_number):
# Format is used to format the numbers into string values, instead of objects:
    slider_focused = True
    print("FOCUS OK! ")

def useValue(unused_addr, message1):
# use the slider's value (exemple: make the sound it must do)
    print('got it ')

dispatcher = dispatcher.Dispatcher()

# This maps a message ID to a function.
# When a message with a given ID is received, the given function is run:
dispatcher.map("/sliderFocus",sliderFocus)
dispatcher.map("/effectivePosition",useValue)

async def loop():
  finished = False

  while (finished == False):
    # if the slider is focused in Processing, send the position that it is taking
    if slider_focused:
        client.send_message("/position", "{}".format("1"))
        print("OK ")
    await asyncio.sleep(1)

ip = "127.0.0.1"
port = 5005

#if __name__ == "__main__":
async def init_main():
  server = AsyncIOOSCUDPServer((ip, port), dispatcher, asyncio.get_event_loop())
  transport, protocol = await server.create_serve_endpoint()

  await loop()

  transport.close()

  #print("Serving on {}".format(server.server_address))
  #server.serve_forever()

asyncio.run(init_main())