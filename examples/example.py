import asyncio
from elrs import ELRS
from datetime import datetime
RC_MIN = 172
RC_MID = 992
RC_MAX = 1811

PORT = "/dev/ttyTHS1"
BAUD = 921600

async def main() -> None:

    def callback(ftype, decoded):
        ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        print(f"[{ts}] {ftype:02X} {decoded}")

    elrs = ELRS(PORT, baud=BAUD, rate=500, telemetry_callback=callback)

    asyncio.create_task(elrs.start())
    elrs.armed = True
    while True:

        channels          = [RC_MID] * 16
        channels[2]       = RC_MIN   # throttle minimum
        channels[4]       = 172      # AUX1
        channels[5]       = 172      # AUX2
        channels[6]       = 172      # AUX3
        channels[7]       = 172      # AUX4
        # channels[8]       = RC_MIN   # disarmed
        # channels[9]       = RC_MIN   # manual flight mode
        channels[8:16]   = [RC_MIN] * 8
        elrs.set_channels(channels)
        await asyncio.sleep(1/elrs.rate)

if __name__ == "__main__":
    asyncio.run(main())