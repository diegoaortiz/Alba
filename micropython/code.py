"""
CO2 Display - Smooth Breathing Animation
Based on todbot's approach for fluid animation
"""

import time
import math
import board
import busio
import displayio
import gc9a01
from vectorio import Circle
from adafruit_display_text import label
import terminalio

# Release displays
displayio.release_displays()

# SPI - Pico 2W pins
spi = busio.SPI(clock=board.GP10, MOSI=board.GP11)

# Display bus
display_bus = displayio.FourWire(
    spi,
    command=board.GP14,
    chip_select=board.GP13,
    reset=board.GP15
)

# GC9A01 display
display = gc9a01.GC9A01(
    display_bus,
    width=240,
    height=240,
    backlight_pin=board.GP12
)

# Main group
main = displayio.Group()
display.root_group = main

# ============================================================
# CREATE BREATHING CIRCLES (from outer to inner)
# ============================================================

# Color palettes for each ring - we'll animate the colors
ring_palettes = []
ring_circles = []

# 6 rings from outer to inner
radii = [118, 100, 82, 64, 46, 38]

for i, r in enumerate(radii):
    pal = displayio.Palette(1)
    # Start with green
    pal[0] = 0x40FF80
    ring_palettes.append(pal)
    
    circle = Circle(pixel_shader=pal, radius=r, x=120, y=120)
    ring_circles.append(circle)
    main.append(circle)

# Center black circle for text
center_pal = displayio.Palette(1)
center_pal[0] = 0x000000
center = Circle(pixel_shader=center_pal, radius=35, x=120, y=120)
main.append(center)

# CO2 Text
text_area = label.Label(
    terminalio.FONT,
    text="400",
    color=0xFFFFFF,
    scale=3,
    anchor_point=(0.5, 0.5),
    anchored_position=(120, 120)
)
main.append(text_area)

# ============================================================
# CO2 SENSOR
# ============================================================

uart = busio.UART(board.GP0, board.GP1, baudrate=9600)
CO2_CMD = b"\xFF\x01\x86\x00\x00\x00\x00\x00\x79"
last_co2 = 400

def read_co2():
    global last_co2
    uart.write(CO2_CMD)
    time.sleep(0.08)
    if uart.in_waiting >= 9:
        data = uart.read(9)
        if data and len(data) == 9 and data[0] == 0xFF and data[1] == 0x86:
            co2 = data[2] * 256 + data[3]
            if 300 < co2 < 5000:
                last_co2 = co2
    return last_co2

# ============================================================
# COLORS
# ============================================================

def get_rgb(co2):
    if co2 < 600:
        return (0x40, 0xFF, 0x80)   # Green
    elif co2 < 800:
        return (0x00, 0xC8, 0xFF)   # Cyan  
    elif co2 < 1200:
        return (0xC8, 0x32, 0xFF)   # Purple
    else:
        return (0xFF, 0x40, 0x20)   # Red

# ============================================================
# MAIN ANIMATION LOOP
# ============================================================

print("CO2 Display Starting...")

# CRITICAL: Disable auto refresh for smooth animation
display.auto_refresh = False

theta = 0.0
co2 = 400
last_read = time.monotonic()
rgb = get_rgb(co2)

while True:
    now = time.monotonic()
    
    # Read CO2 every 4 seconds  
    if now - last_read > 4:
        co2 = read_co2()
        last_read = now
        text_area.text = str(co2)
        rgb = get_rgb(co2)
    
    # Animate each ring with breathing effect
    for i in range(len(ring_palettes)):
        # Phase offset for wave effect
        phase = theta + i * 0.6
        
        # Smooth sine wave for breathing
        breath = (math.sin(phase) + 1.0) * 0.5  # 0.0 to 1.0
        
        # Intensity modulation
        intensity = 0.15 + 0.85 * breath
        
        # Darker for outer rings, brighter for inner
        ring_fade = 0.5 + 0.5 * (i / len(ring_palettes))
        
        # Final color
        cr = int(rgb[0] * intensity * ring_fade)
        cg = int(rgb[1] * intensity * ring_fade)
        cb = int(rgb[2] * intensity * ring_fade)
        
        # Update palette color (this is fast - no redraw)
        ring_palettes[i][0] = (cr << 16) | (cg << 8) | cb
    
    # Single refresh per frame
    display.refresh()
    
    # Advance animation
    theta += 0.08
    
    # Target ~30 FPS
    time.sleep(0.033)
