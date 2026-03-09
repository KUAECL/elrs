# ELRS Python Interface

### Preliminaries
![WhatsApp Image 2026-03-09 at 11 16 33 AM](https://github.com/user-attachments/assets/393e9802-b1ba-4cd1-84ef-e54e08f8daaf)

1. Connect Tx module to the FrSky tandem X20s through the crsf pin.
2. Connect Rx module to Telem 1 in Pixhawk.
3. Connect Pixhawk Fc through USB to QGC, and setup the according to this https://docs.px4.io/main/en/telemetry/crsf_telemetry#expresslrs-radio-systems.
4. Follow this, https://www.youtube.com/watch?v=bO5PWkJr0nU, to keep the FrSky radio up to date and enable extra protocols through the lua script.
5. Once set up, navigate to model "plane icon" in the FrSky radio, go to RF System, enable external module, and disable internal.
6. Type: Elrs, Baud: 921k, also disable backpack and set the frequency to 500Hz.
### Connection
<img width="600" height="338" alt="image" src="https://github.com/user-attachments/assets/39d6babf-f927-4aef-8c75-485498579980" />
<img width="593" height="762" alt="image" src="https://github.com/user-attachments/assets/cd6c0665-47bc-4c91-af38-c9aacc25a852" />
1. Connect the jetson nano uart_a, Rx1 and Tx1 tied together and connected to the crsf pin on the Tx module, and 5V & Gnd to power it up.
2. Use the example.py attached to send rc commands and receive telem.

--------------------------------------------------------------------------------------------------
### (Original branch)

```python
import asyncio
from elrs import ELRS
from datetime import datetime

PORT = "/dev/ttyUSB0"
BAUD = 921600

async def main() -> None:

    def callback(ftype, decoded):
        ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        print(f"[{ts}] {ftype:02X} {decoded}")

    elrs = ELRS(PORT, baud=BAUD, rate=50, telemetry_callback=callback)

    asyncio.create_task(elrs.start())

    value = 1000
    while True:
        channels = [value] * 16
        elrs.set_channels(channels)
        value = (value + 1) % 2048
        await asyncio.sleep(0.1)

if __name__ == "__main__":
    asyncio.run(main())
```

Tested with the Radiomaster Ranger Nano.
### Radiomaster Ranger Nano
- Connect to the Wifi hosted by the Ranger Nano and go to `http://10.0.0.1/hardware.html` and set `RX: 3` `TX: 1`
j after it is configured make sure that after power cycling it, you send commands to it within a certain time. it seems to go into the wifi-hosting configuration mode some time after powerup if it does not receive commands.

### ELRS forwarding 

Use e.g. Radiomaster Ranger Micro. (This is tested with `3.5.5`, flash using the ExpressLRS Configurator):
1. Connect to the WiFi hosted by it
2. Go to 10.0.0.1
3. Set the passphrase to match the one on your desired model
4. Set the same passphrase in betaflight by copying the 6 numbers from the site: `set expresslrs_uid = {6 numbers}` then `save` (both in the CLI)
5. Go to 10.0.0.1/hardware.html
6. Disable the Backpack
7. Set `CSRF.RX=3` and `CSRF.TX=1` (this works for both Radiomaster Ranger Micro and Jumper Aion ELRS 2.4G TX Nano and is reported to work for other modules like the BetaFPV ones as well)
8. After it is configured make sure that after power cycling it, you send commands to it within a certain time. it seems to go into the wifi-hosting configuration mode some time after powerup if it does not receive commands.
9. run `elrs /dev/ttyUSB0 921600 --ch 1337` (replace path with the assigned port on you PC). This should send a `1337` value to the receiver. Note that this is sent in these ranges
```
RC_CHANNEL_MIN = 172
RC_CHANNEL_MID = 992
RC_CHANNEL_MAX = 1811
```
and should be normalized to 1000 - 2000 in betaflight

