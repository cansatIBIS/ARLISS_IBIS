from mavsdk import System
import asyncio

async def run():
    drone = System()
    
    print("-- Waiting for drone to connect...")
    await drone.connect(system_address="udp://:14540")


    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone!")
            break
        
    
    
    print("-- Killng the drone")
    await drone.action.kill()
    print("-- Killed!")
    

if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())