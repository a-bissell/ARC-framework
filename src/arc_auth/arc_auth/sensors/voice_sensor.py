import speech_recognition as sr
import numpy as np
from typing import Dict, Optional, List, Tuple
import asyncio
from .base_sensor import AuthenticationSensor, SensorReading
import librosa
from sklearn.preprocessing import normalize
import sounddevice as sd
import threading
import queue
import logging
import difflib

class VoiceRecognitionSensor(AuthenticationSensor):
    def __init__(self):
        super().__init__()
        self.recognizer = sr.Recognizer()
        self.microphone = None
        self.enrolled_voice_prints: Dict[str, List[Tuple[np.ndarray, str]]] = {}  # (features, spoken_text)
        self.audio_queue = queue.Queue()
        self.recording = False
        self.sample_rate = 16000
        self.logger = logging.getLogger(__name__)
        self.passphrase = "authenticate system access"  # Default passphrase
        
    async def initialize(self, config: Dict) -> bool:
        try:
            # Initialize microphone
            self.microphone = sr.Microphone(
                device_index=config.get('device_index', None),
                sample_rate=self.sample_rate
            )
            
            # Adjust for ambient noise
            with self.microphone as source:
                self.recognizer.adjust_for_ambient_noise(source)
            
            self.config = config
            self.passphrase = config.get('passphrase', self.passphrase)
            self.text_dependent = config.get('text_dependent', True)
            self.is_initialized = True
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to initialize voice sensor: {e}")
            return False
    
    def _audio_callback(self, indata, frames, time, status):
        """Callback for audio stream"""
        if status:
            self.logger.warning(f"Audio callback status: {status}")
        if self.recording:
            self.audio_queue.put(indata.copy())
    
    async def read(self) -> Optional[SensorReading]:
        if not self.is_initialized:
            return None
        
        try:
            # Record audio
            audio_data = []
            self.recording = True
            self.audio_queue.queue.clear()
            
            # Start audio stream
            stream = sd.InputStream(
                channels=1,
                samplerate=self.sample_rate,
                callback=self._audio_callback
            )
            
            with stream:
                # Record for 5 seconds (increased for text verification)
                await asyncio.sleep(5)
            
            self.recording = False
            
            # Collect recorded audio
            while not self.audio_queue.empty():
                audio_data.append(self.audio_queue.get())
            
            if not audio_data:
                return None
            
            # Combine audio chunks
            audio_array = np.concatenate(audio_data, axis=0)
            
            # Extract voice features
            features = self._extract_voice_features(audio_array)
            
            # Perform speech recognition
            recognized_text = self._recognize_speech(audio_array)
            
            return SensorReading(
                confidence=1.0,  # Will be adjusted during verification
                raw_data=audio_array,
                processed_data=(features, recognized_text),  # Tuple of features and text
                metadata={
                    "sample_rate": self.sample_rate,
                    "duration": len(audio_array) / self.sample_rate,
                    "recognized_text": recognized_text
                }
            )
            
        except Exception as e:
            self.logger.error(f"Error reading voice data: {e}")
            return None
    
    def _extract_voice_features(self, audio_array: np.ndarray) -> np.ndarray:
        """Extract MFCC features from audio"""
        # Convert to mono if needed
        if len(audio_array.shape) > 1:
            audio_array = audio_array.mean(axis=1)
        
        # Extract MFCCs
        mfccs = librosa.feature.mfcc(
            y=audio_array.flatten(),
            sr=self.sample_rate,
            n_mfcc=20
        )
        
        # Normalize features
        mfccs_normalized = normalize(mfccs, axis=1)
        
        # Calculate statistics over time
        features = np.concatenate([
            np.mean(mfccs_normalized, axis=1),
            np.std(mfccs_normalized, axis=1)
        ])
        
        return features
    
    def _recognize_speech(self, audio_array: np.ndarray) -> str:
        """Convert audio to text using speech recognition"""
        try:
            # Convert numpy array to AudioData
            audio_data = sr.AudioData(
                audio_array.tobytes(),
                sample_rate=self.sample_rate,
                sample_width=2  # 16-bit audio
            )
            
            # Perform speech recognition
            text = self.recognizer.recognize_google(audio_data)
            return text.lower()
        except Exception as e:
            self.logger.error(f"Speech recognition failed: {e}")
            return ""
    
    def _calculate_text_similarity(self, text1: str, text2: str) -> float:
        """Calculate similarity between two texts"""
        return difflib.SequenceMatcher(None, text1, text2).ratio()
    
    async def enroll(self, user_id: str, num_samples: int = 3) -> bool:
        if not self.is_initialized:
            return False
        
        voice_prints = []
        
        # Inform user about the passphrase if text-dependent
        if self.text_dependent:
            self.logger.info(f"Please speak the following phrase: '{self.passphrase}'")
        
        for i in range(num_samples):
            self.logger.info(f"Recording sample {i+1}/{num_samples}")
            reading = await self.read()
            
            if reading is None:
                continue
            
            features, recognized_text = reading.processed_data
            
            if self.text_dependent:
                # Verify the spoken text matches the passphrase
                similarity = self._calculate_text_similarity(recognized_text, self.passphrase)
                if similarity < 0.8:  # 80% similarity threshold
                    self.logger.warning("Spoken text doesn't match the passphrase. Please try again.")
                    continue
            
            voice_prints.append((features, recognized_text))
            await asyncio.sleep(1)  # Wait between samples
        
        if len(voice_prints) < num_samples:
            return False
        
        self.enrolled_voice_prints[user_id] = voice_prints
        return True
    
    async def verify(self, user_id: str, reading: SensorReading) -> float:
        if not self.is_initialized or user_id not in self.enrolled_voice_prints:
            return 0.0
        
        try:
            # Unpack current reading
            current_features, current_text = reading.processed_data
            enrolled_samples = self.enrolled_voice_prints[user_id]
            
            # Voice biometric verification
            distances = [
                np.linalg.norm(current_features - enrolled_features)
                for enrolled_features, _ in enrolled_samples
            ]
            min_distance = min(distances)
            
            # Calculate voice biometric confidence
            max_acceptable_distance = 2.0
            if min_distance > max_acceptable_distance:
                return 0.0
            
            voice_confidence = 1.0 - (min_distance / max_acceptable_distance)
            
            # Text-dependent verification if enabled
            if self.text_dependent:
                # Check if the spoken text matches the passphrase
                text_similarity = self._calculate_text_similarity(current_text, self.passphrase)
                
                # Combine voice and text confidences
                # Weight can be adjusted based on requirements
                combined_confidence = 0.7 * voice_confidence + 0.3 * text_similarity
                return max(0.0, min(1.0, combined_confidence))
            
            return max(0.0, min(1.0, voice_confidence))
            
        except Exception as e:
            self.logger.error(f"Error during voice verification: {e}")
            return 0.0
    
    async def cleanup(self) -> None:
        self.recording = False
        self.is_initialized = False 