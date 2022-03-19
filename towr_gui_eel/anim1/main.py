import eel
from datetime import datetime as dt

@eel.expose
def hello():
  print("Hello world!")

eel.init("www")
eel.start("index.html", block=False)  # don't block on this function call

while True:
  eel.sleep(1.0)