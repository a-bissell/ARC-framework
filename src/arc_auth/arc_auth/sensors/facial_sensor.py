import cv2
import numpy as np
from typing import Dict, Optional, List
import face_recognition
import asyncio
from .base_sensor import AuthenticationSensor, SensorReading

class FacialRecognitionSensor(AuthenticationSensor):
    def __init__(self):
        super().__init__()
        self.camera = None
        self.enrolled_encodings: Dict[str, List[np.ndarray]] = {}
        self.min_confidence_threshold = 0.6
        
    async def initialize(self, config: Dict) -> bool:
        try:
            self.camera = cv2.VideoCapture(config.get('camera_id', 0))
            self.min_confidence_threshold = config.get('min_confidence', 0.6)
            self.config = config
            self.is_initialized = True
            return True
        except Exception as e:
            print(f"Failed to initialize facial sensor: {e}")
            return False
    
    async def read(self) -> Optional[SensorReading]:
        if not self.is_initialized:
            return None
            
        ret, frame = self.camera.read()
        if not ret:
            return None
            
        # Convert to RGB for face_recognition library
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Detect faces
        face_locations = face_recognition.face_locations(rgb_frame)
        if not face_locations:
            return None
            
        # Get face encodings
        face_encodings = face_recognition.face_encodings(rgb_frame, face_locations)
        if not face_encodings:
            return None
            
        return SensorReading(
            confidence=1.0,  # Will be adjusted during verification
            raw_data=frame,
            processed_data=face_encodings[0],  # Use first detected face
            metadata={
                "face_location": face_locations[0],
                "multiple_faces": len(face_locations) > 1
            }
        )
    
    async def enroll(self, user_id: str, num_samples: int = 3) -> bool:
        if not self.is_initialized:
            return False
            
        encodings = []
        for _ in range(num_samples):
            reading = await self.read()
            if reading is None:
                continue
            encodings.append(reading.processed_data)
            await asyncio.sleep(1)  # Wait between captures
            
        if len(encodings) < num_samples:
            return False
            
        self.enrolled_encodings[user_id] = encodings
        return True
    
    async def verify(self, user_id: str, reading: SensorReading) -> float:
        if not self.is_initialized or user_id not in self.enrolled_encodings:
            return 0.0
            
        # Compare against all enrolled encodings
        matches = face_recognition.compare_faces(
            self.enrolled_encodings[user_id],
            reading.processed_data,
            tolerance=0.6
        )
        
        if not any(matches):
            return 0.0
            
        # Calculate confidence based on face distance
        face_distances = face_recognition.face_distance(
            self.enrolled_encodings[user_id],
            reading.processed_data
        )
        confidence = 1.0 - min(face_distances)
        
        return max(0.0, min(1.0, confidence))
    
    async def cleanup(self) -> None:
        if self.camera is not None:
            self.camera.release()
        self.is_initialized = False 