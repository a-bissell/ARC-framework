from enum import IntEnum
import time
from typing import Dict, List, Optional
import numpy as np
from dataclasses import dataclass

class AccessLevel(IntEnum):
    NONE = 0
    BASIC = 1
    LIMITED = 2
    STANDARD = 3
    ENHANCED = 4
    PRIVILEGED = 5
    ADMIN = 6
    ROOT = 7
    FULL = 8

@dataclass
class AuthenticationFactor:
    factor_id: str
    confidence: float  # 0.0 to 1.0
    timestamp: float
    metadata: Dict

class AuthenticationCore:
    def __init__(self):
        self.factor_weights = {
            'facial': 0.3,
            'voice': 0.2,
            'fingerprint': 0.4,
            'rfid': 0.1
        }
        
        # Thresholds for each access level
        self.access_thresholds = {
            AccessLevel.BASIC: 0.2,
            AccessLevel.LIMITED: 0.3,
            AccessLevel.STANDARD: 0.4,
            AccessLevel.ENHANCED: 0.5,
            AccessLevel.PRIVILEGED: 0.6,
            AccessLevel.ADMIN: 0.7,
            AccessLevel.ROOT: 0.8,
            AccessLevel.FULL: 0.9
        }
        
        # Retry tracking
        self.max_immediate_retries = 3
        self.retry_counts = {}
        self.lockout_until = {}

    def calculate_confidence_score(self, factors: List[AuthenticationFactor]) -> float:
        if not factors:
            return 0.0
            
        total_weight = 0.0
        weighted_sum = 0.0
        
        for factor in factors:
            weight = self.factor_weights.get(factor.factor_id, 0.0)
            total_weight += weight
            weighted_sum += weight * factor.confidence
            
        if total_weight == 0:
            return 0.0
            
        return weighted_sum / total_weight

    def determine_access_level(self, user_id: str, factors: List[AuthenticationFactor]) -> AccessLevel:
        # Check lockout
        if user_id in self.lockout_until:
            if time.time() < self.lockout_until[user_id]:
                return AccessLevel.NONE
            else:
                del self.lockout_until[user_id]
        
        # Calculate confidence score
        confidence = self.calculate_confidence_score(factors)
        
        # Handle retries
        if confidence < self.access_thresholds[AccessLevel.BASIC]:
            self.retry_counts[user_id] = self.retry_counts.get(user_id, 0) + 1
            
            if self.retry_counts[user_id] > self.max_immediate_retries:
                # Implement exponential backoff
                lockout_duration = min(300 * (2 ** (self.retry_counts[user_id] - self.max_immediate_retries)), 3600)
                self.lockout_until[user_id] = time.time() + lockout_duration
                return AccessLevel.NONE
        else:
            self.retry_counts[user_id] = 0
        
        # Determine access level based on confidence score
        for level in reversed(AccessLevel):
            if level == AccessLevel.NONE:
                continue
            if confidence >= self.access_thresholds[level]:
                return level
                
        return AccessLevel.NONE 