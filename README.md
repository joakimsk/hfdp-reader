# hfdp-reader

Hardware and software to read the Honeywell SSC TruStability differential pressure sensor -+100 mbar at up to 2000 Hz sample rate. Assisted a lot by ChatGPT.

Goal is to measure the intake cycles on the inlet filter of a Bauer PE 550 VE compressor, the one owned by SUB-BSI diving club in Bergen.

Hypothesis is that a long intake hose makes the underpressure larger than wanted, and this leads to extensive oil carryover, reducing filter life.

# Usage

1. Set up your sensor using a suitable MCU, for example ESP8266 - Wemos D1 Mini is a good development board.
2. Connect 1mm silicone hose to port 1 and 2 of the Honeywell SSC TruStability sensor. In my case, port 1 allows for liquids.
3. If wanted, convert the 1.5 mm silicone hose to a 4 mm PUR tube by using for example the dohickey adapter.
4. Flash program to your MCU
5. Test using live_plot_mbar_raw.py - ensure you see sensible data
6. Receive raw data using rx_store_to_bin.py storing it to capture.bin or equivalent
7. Plot the data using one of the plot_-scripts OR use live_plot_-scripts to see the data in real-time - this is also where you apply any filters as needed.

NB: Auto-calibration should be implemented, a sensor may seem noisy when it is for very low differential pressures.

NB2: %FSS means we can expect +- 4 mbar accuracy.
NB3: Minmax is +- 100 mbar, and absolute max burst pressure is some 1.4 bar for this sensor (eeek).

# Plumbing
DIP RR sensor variety shows Ø1.93 mm per port.

Silicone tube RS PRO 273-2491 1.5mm ID, 3mm OD seems to fit well.

Tube also fits the AU3 pagoda well.

![DIP RR Sensor variety](media/DIP-RR.jpg | width=100)
![M6 to AU3, pagoda for 1-2mm silicone tube](media/M6-AU3.jpg | width=100)
A simple M6 nut with two small O-rings to build out, or encapsulated in epoxy.
![PC4 to M6, push connector 4mm tube](media/PC4-M6.jpg | width=100)

The final adapter from 1.5mm ID silicone tube to 4mm PUR tube looks like this:
![Dohickey adapter](media/dohickey.jpg | width=100)

# Resources
[Read Honeywell Trustability SSC resources](https://automation.honeywell.com/us/en/products/sensing-solutions/sensors/pressure-sensors/board-mount-pressure-amplified/trustability-ssc-series-board-mount-pressure-sensor#resources)