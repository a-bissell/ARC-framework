from typing import Dict, List, Type
from .base_sensor import AuthenticationSensor, SensorReading
import asyncio
import logging

class SensorManager:
    def __init__(self):
        self.sensors: Dict[str, AuthenticationSensor] = {}
        self.logger = logging.getLogger(__name__)
    
    async def register_sensor(self, sensor: AuthenticationSensor, config: Dict) -> bool:
        """Register and initialize a new sensor"""
        if sensor.sensor_id in self.sensors:
            self.logger.warning(f"Sensor {sensor.sensor_id} already registered")
            return False
            
        if await sensor.initialize(config):
            self.sensors[sensor.sensor_id] = sensor
            self.logger.info(f"Registered sensor: {sensor.sensor_id}")
            return True
        return False
    
    async def read_all(self) -> Dict[str, SensorReading]:
        """Read from all registered sensors"""
        readings = {}
        tasks = []
        
        for sensor_id, sensor in self.sensors.items():
            tasks.append(self._read_sensor(sensor_id, sensor))
            
        results = await asyncio.gather(*tasks, return_exceptions=True)
        
        for sensor_id, result in zip(self.sensors.keys(), results):
            if isinstance(result, Exception):
                self.logger.error(f"Error reading from {sensor_id}: {result}")
            elif result is not None:
                readings[sensor_id] = result
                
        return readings
    
    async def _read_sensor(self, sensor_id: str, sensor: AuthenticationSensor) -> Optional[SensorReading]:
        """Helper method to read from a single sensor"""
        try:
            return await sensor.read()
        except Exception as e:
            self.logger.error(f"Error reading from sensor {sensor_id}: {e}")
            return None
    
    async def verify_user(self, user_id: str) -> Dict[str, float]:
        """Verify user against all sensors"""
        confidences = {}
        readings = await self.read_all()
        
        for sensor_id, reading in readings.items():
            try:
                confidence = await self.sensors[sensor_id].verify(user_id, reading)
                confidences[sensor_id] = confidence
            except Exception as e:
                self.logger.error(f"Error verifying {sensor_id}: {e}")
                confidences[sensor_id] = 0.0
                
        return confidences
    
    async def cleanup(self):
        """Cleanup all sensors"""
        for sensor in self.sensors.values():
            await sensor.cleanup()
        self.sensors.clear() 