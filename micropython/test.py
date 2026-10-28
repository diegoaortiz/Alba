from machine import Pin, SPI
import gc9a01py

# SPI avec polarity=1, phase=1 (config originale qui marchait)
spi = SPI(1, baudrate=20000000, polarity=1, phase=1, sck=Pin(10), mosi=Pin(11))

dc = Pin(14, Pin.OUT)
cs = Pin(13, Pin.OUT)
rst = Pin(15, Pin.OUT)
bl = Pin(12, Pin.OUT)
bl.on()

display = gc9a01py.GC9A01(spi, dc=dc, cs=cs, reset=rst)

# Test bleu
print("Test ecran bleu...")
display.fill(gc9a01py.color565(0, 0, 255))
print("Ecran bleu?")
