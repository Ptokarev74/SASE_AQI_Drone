import asyncio
import sys
import struct
import logging
from bleak import BleakClient, BleakScanner

# Configuration (Matching config.py and Arduino code)
BLE_DEVICE_NAME = "AQI_Drone"
COMMAND_CHAR_UUID = "19b10012-e8f2-537e-4f6c-d104768a1214"

# Set up basic logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
logger = logging.getLogger(__name__)

def pack_binary(vx: float, vy: float, vz: float, yaw: float, arm: bool) -> bytes:
    """Packs the command into the exact 21-byte struct expected by the Arduino."""
    # Assuming CRC32 is 0 for this basic test, as the Arduino script just prints it
    crc32 = 0 
    return struct.pack("<ffffBI", vx, vy, vz, yaw, 1 if arm else 0, crc32)

async def run_test():
    logger.info(f"Scanning for device named '{BLE_DEVICE_NAME}'...")
    
    device = await BleakScanner.find_device_by_filter(
        lambda d, ad: ad.local_name == BLE_DEVICE_NAME,
        timeout=10.0
    )

    if not device:
        logger.error(f"Could not find device '{BLE_DEVICE_NAME}'. Is the Arduino turned on and advertising?")
        sys.exit(1)

    logger.info(f"Found device: {device.address}. Connecting...")

    try:
        async with BleakClient(device) as client:
            logger.info("Connected!")
            
            # Allow time for connection stabilization
            await asyncio.sleep(1.0)

            commands = [
                {"name": "Arm Drone", "vx": 0.0, "vy": 0.0, "vz": 0.0, "yaw": 0.0, "arm": True},
                {"name": "Forward Pitch", "vx": 50.0, "vy": 0.0, "vz": 0.0, "yaw": 0.0, "arm": True},
                {"name": "Right Strafe", "vx": 0.0, "vy": 50.0, "vz": 0.0, "yaw": 0.0, "arm": True},
                {"name": "Ascend", "vx": 0.0, "vy": 0.0, "vz": 50.0, "yaw": 0.0, "arm": True},
                {"name": "Yaw CW", "vx": 0.0, "vy": 0.0, "vz": 0.0, "yaw": 50.0, "arm": True},
                {"name": "Disarm", "vx": 0.0, "vy": 0.0, "vz": 0.0, "yaw": 0.0, "arm": False},
            ]

            for i, cmd in enumerate(commands):
                logger.info(f"[{i+1}/{len(commands)}] Sending: {cmd['name']}")
                
                payload = pack_binary(cmd['vx'], cmd['vy'], cmd['vz'], cmd['yaw'], cmd['arm'])
                
                # Write without response, as configured in the bridge
                await client.write_gatt_char(COMMAND_CHAR_UUID, payload, response=False)
                logger.info(f"  -> Successfully sent {len(payload)} bytes.")
                
                # Wait before sending the next command so you can read the Arduino serial monitor
                await asyncio.sleep(2.0)

            logger.info("Finished sending all test commands.")

    except Exception as e:
        logger.error(f"Connection or write failed: {e}")

if __name__ == "__main__":
    asyncio.run(run_test())
