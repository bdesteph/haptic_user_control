from pythonosc import osc_message_builder
from pythonosc import udp_client
from pythonosc import osc_server
from pythonosc import dispatcher
from pythonosc.osc_server import AsyncIOOSCUDPServer
import asyncio

# Server and client must have their own port

# We send / receive to Processing via this client:
client = udp_client.SimpleUDPClient("127.0.0.1", 1234)
# We send to Max via this client
client2 = udp_client.SimpleUDPClient("127.0.0.1", 4321)

x_positions = []
slider_focused = False

def sliderFocus(unused_addr, slider_number):
# Format is used to format the numbers into string values, instead of objects:
    slider_focused = True
    print("FOCUS OK! ")

def useValue(unused_addr, message1):
# use the slider's value (exemple: make the sound it must do)
    client2.send_message("/value", "{}".format(message1))
    print(message1)

def sendPosition(unused_addr, message1):
# if (slider_focused is True):
    client.send_message("/position", "{}".format(message1))
    print(message1)

dispatcher = dispatcher.Dispatcher()

# This maps a message ID to a function.
# When a message with a given ID is received, the given function is run:
dispatcher.map("/sliderFocus",sliderFocus)
dispatcher.map("/getPosition",sendPosition)
dispatcher.map("/effectivePosition",useValue)

async def loop():
  finished = False

  while (finished == False):
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