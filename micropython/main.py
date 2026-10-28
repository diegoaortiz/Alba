"""
CO2 Display - Framebuffer Approach
Draw to memory buffer, then blit to screen = SMOOTH
"""

import time
import math
import gc
from machine import Pin, SPI, UART
import gc9a01py
import framebuf

# ============================================================
# HARDWARE
# ============================================================

spi = SPI(1, baudrate=40000000, polarity=1, phase=1, sck=Pin(10), mosi=Pin(11))
dc = Pin(14, Pin.OUT)
cs = Pin(13, Pin.OUT)
rst = Pin(15, Pin.OUT)
bl = Pin(12, Pin.OUT)
bl.on()

lcd = gc9a01py.GC9A01(spi, dc=dc, cs=cs, reset=rst)

uart = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))
CO2_CMD = b"\xFF\x01\x86\x00\x00\x00\x00\x00\x79"

# ============================================================
# FRAMEBUFFER (smaller region to save RAM)
# ============================================================

# We can't fit full 240x240x2 bytes (115KB) in RAM
# So we use a smaller buffer and update in chunks
CHUNK_H = 60  # Height of each chunk
buf = bytearray(240 * CHUNK_H * 2)  # RGB565 = 2 bytes/pixel
fb = framebuf.FrameBuffer(buf, 240, CHUNK_H, framebuf.RGB565)

# ============================================================
# CO2 SENSOR
# ============================================================

last_co2 = 400

def read_co2():
    global last_co2
    uart.write(CO2_CMD)
    time.sleep(0.05)
    if uart.any() >= 9:
        d = uart.read(9)
        if d and len(d) == 9 and d[0] == 0xFF and d[1] == 0x86:
            co2 = d[2] * 256 + d[3]
            if co2 > 300:
                last_co2 = co2
    return last_co2

# ============================================================
# COLORS
# ============================================================

def rgb565(r, g, b):
    # Swap bytes for framebuf (different endianness)
    c = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
    return ((c & 0xFF) << 8) | (c >> 8)

PALETTES = {
    "EXCELLENT": (80, 255, 120),
    "GOOD":      (0, 180, 255),
    "MEDIUM":    (180, 50, 255),
    "BAD":       (255, 60, 30)
}

BLACK = rgb565(0, 0, 0)
WHITE = rgb565(255, 255, 255)

# ============================================================
# CONFIG
# ============================================================

CX, CY = 120, 120
TEXT_R = 35
MAX_R = 115

# ============================================================
# DIGITS for framebuf
# ============================================================

D = {
    '0': [(1,0),(2,0),(0,1),(3,1),(0,2),(3,2),(0,3),(3,3),(0,4),(3,4),(1,5),(2,5)],
    '1': [(2,0),(1,1),(2,1),(2,2),(2,3),(2,4),(1,5),(2,5),(3,5)],
    '2': [(1,0),(2,0),(0,1),(3,1),(3,2),(2,3),(1,4),(0,5),(1,5),(2,5),(3,5)],
    '3': [(1,0),(2,0),(3,1),(2,2),(3,3),(3,4),(1,5),(2,5)],
    '4': [(0,0),(3,0),(0,1),(3,1),(0,2),(1,2),(2,2),(3,2),(3,3),(3,4),(3,5)],
    '5': [(0,0),(1,0),(2,0),(3,0),(0,1),(0,2),(1,2),(2,2),(3,3),(3,4),(0,5),(1,5),(2,5)],
    '6': [(2,0),(1,1),(0,2),(1,2),(2,2),(0,3),(3,3),(0,4),(3,4),(1,5),(2,5)],
    '7': [(0,0),(1,0),(2,0),(3,0),(3,1),(2,2),(2,3),(1,4),(1,5)],
    '8': [(1,0),(2,0),(0,1),(3,1),(1,2),(2,2),(0,3),(3,3),(0,4),(3,4),(1,5),(2,5)],
    '9': [(1,0),(2,0),(0,1),(3,1),(0,2),(3,2),(1,3),(2,3),(3,3),(3,4),(1,5),(2,5)],
}

def draw_digit_fb(fb, d, x, y, s, col):
    if d not in D:
        return 0
    for px, py in D[d]:
        fb.fill_rect(x + px * s, y + py * s, s, s, col)
    return 5 * s

def draw_num_fb(fb, n, cx, cy, s, col):
    text = str(n)
    w = len(text) * 5 * s
    x = cx - w // 2
    y = cy - 3 * s
    for c in text:
        x += draw_digit_fb(fb, c, x, y, s, col)

# ============================================================
# DRAW CHUNK
# ============================================================

def draw_chunk(y_start, breath, color, co2):
    """Draw a horizontal chunk of the display."""
    fb.fill(BLACK)
    
    # Draw glow rings in this chunk
    for band in range(8):
        # Ring radius
        r = TEXT_R + 10 + band * 10 + int(breath * 8)
        
        # Brightness based on breath and distance
        alpha = (0.3 + 0.7 * breath) * (1.0 - band * 0.1)
        
        cr = int(color[0] * alpha)
        cg = int(color[1] * alpha)
        cb = int(color[2] * alpha)
        ring_col = rgb565(cr, cg, cb)
        
        # Draw ring pixels that fall in this chunk
        for a in range(0, 360, 4):
            rad = a * 3.14159 / 180
            px = int(CX + r * math.cos(rad))
            py = int(CY + r * math.sin(rad))
            
            # Check if this pixel is in current chunk
            local_y = py - y_start
            if 0 <= local_y < CHUNK_H and 0 <= px < 240:
                fb.fill_rect(px - 2, local_y - 2, 5, 5, ring_col)
    
    # Draw text if it's in this chunk
    text_y = CY - 10
    if y_start <= text_y < y_start + CHUNK_H:
        # Black background for text
        fb.fill_rect(CX - 35, text_y - y_start - 5, 70, 30, BLACK)
        draw_num_fb(fb, co2, CX, CY - y_start, 3, WHITE)
    
    # Blit to display
    lcd.blit_buffer(buf, 0, y_start, 240, CHUNK_H)

# ============================================================
# MAIN
# ============================================================

print("CO2 Display - Framebuffer Mode")
lcd.fill(gc9a01py.BLACK)

t = 0.0
co2 = read_co2()
last_read = 0

while True:
    now = time.ticks_ms()
    
    # Read CO2 every 3s
    if time.ticks_diff(now, last_read) > 3000:
        co2 = read_co2()
        last_read = now
        gc.collect()
    
    # Palette
    if co2 < 600:
        color = PALETTES["EXCELLENT"]; speed = 0.4
    elif co2 < 800:
        color = PALETTES["GOOD"]; speed = 0.6
    elif co2 < 1200:
        color = PALETTES["MEDIUM"]; speed = 0.9
    else:
        color = PALETTES["BAD"]; speed = 1.3
    
    # Breathing (smooth continuous)
    breath = (math.sin(t * speed) + 1) / 2
    
    # Draw in chunks
    for y in range(0, 240, CHUNK_H):
        draw_chunk(y, breath, color, co2)
    
    t += 0.08
    time.sleep(0.02)
