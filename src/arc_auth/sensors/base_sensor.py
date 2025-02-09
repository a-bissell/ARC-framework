from abc import ABC, abstractmethod
from typing import Dict, Optional, Any
from dataclasses import dataclass
import numpy as np

@dataclass
class SensorReading:
    confidence: float  # 0.0 to 1.0
    raw_data: Any  # Raw sensor data
    processed_data: Any  # Processed/feature extracted data
    metadata: Dict  # Additional sensor-specific information

class AuthenticationSensor(ABC):
    def __init__(self):
        self.sensor_id: str = self.__class__.__name__
        self.is_initialized: bool = False
        self.config: Dict = {}
    
    @abstractmethod
    async def initialize(self, config: Dict) -> bool:
        """Initialize the sensor with given configuration"""
        pass
    
    @abstractmethod
    async def read(self) -> Optional[SensorReading]:
        """Read data from the sensor and return processed results"""
        pass
    
    @abstractmethod
    async def enroll(self, user_id: str, num_samples: int = 3) -> bool:
        """Collect enrollment samples for a user"""
        pass
    
    @abstractmethod
    async def verify(self, user_id: str, reading: SensorReading) -> float:
        """Verify a reading against enrolled user data"""
        pass
    
    @abstractmethod
    async def cleanup(self) -> None:
        """Cleanup sensor resources"""
        pass
    
    def get_sensor_info(self) -> Dict:
        """Get sensor information and capabilities"""
        return {
            "sensor_id": self.sensor_id,
            "is_initialized": self.is_initialized,
            "config": self.config
        } 