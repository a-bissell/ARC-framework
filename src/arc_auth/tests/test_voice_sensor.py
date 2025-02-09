import asyncio
import pytest
from ..sensors.voice_sensor import VoiceRecognitionSensor
import logging
import sys

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    stream=sys.stdout
)

class TestVoiceRecognition:
    @pytest.fixture
    async def text_dependent_sensor(self):
        """Create a text-dependent voice sensor"""
        sensor = VoiceRecognitionSensor()
        await sensor.initialize({
            'text_dependent': True,
            'passphrase': "verify my identity now",
            'device_index': 0
        })
        yield sensor
        await sensor.cleanup()

    @pytest.fixture
    async def text_independent_sensor(self):
        """Create a text-independent voice sensor"""
        sensor = VoiceRecognitionSensor()
        await sensor.initialize({
            'text_dependent': False,
            'device_index': 0
        })
        yield sensor
        await sensor.cleanup()

    @pytest.mark.asyncio
    async def test_text_dependent_enrollment(self, text_dependent_sensor):
        """Test text-dependent enrollment process"""
        logger = logging.getLogger(__name__)
        
        logger.info("\n=== Text-Dependent Enrollment Test ===")
        logger.info("This test will enroll a user with a specific passphrase")
        logger.info(f"Please speak the passphrase: '{text_dependent_sensor.passphrase}'")
        logger.info("You will need to repeat it 3 times\n")
        
        # Attempt enrollment
        success = await text_dependent_sensor.enroll("test_user_1")
        
        assert success, "Enrollment should succeed with correct passphrase"
        assert "test_user_1" in text_dependent_sensor.enrolled_voice_prints
        assert len(text_dependent_sensor.enrolled_voice_prints["test_user_1"]) == 3

    @pytest.mark.asyncio
    async def test_text_independent_enrollment(self, text_independent_sensor):
        """Test text-independent enrollment process"""
        logger = logging.getLogger(__name__)
        
        logger.info("\n=== Text-Independent Enrollment Test ===")
        logger.info("This test will enroll a user without requiring specific phrases")
        logger.info("Please speak naturally for 5 seconds, 3 times\n")
        
        # Attempt enrollment
        success = await text_independent_sensor.enroll("test_user_2")
        
        assert success, "Enrollment should succeed with any speech"
        assert "test_user_2" in text_independent_sensor.enrolled_voice_prints
        assert len(text_independent_sensor.enrolled_voice_prints["test_user_2"]) == 3

    @pytest.mark.asyncio
    async def test_text_dependent_verification(self, text_dependent_sensor):
        """Test text-dependent verification"""
        logger = logging.getLogger(__name__)
        
        # First enroll the user
        logger.info("\n=== Text-Dependent Verification Test ===")
        logger.info("First, let's enroll a user...")
        await text_dependent_sensor.enroll("verify_user_1")
        
        logger.info("\nNow, let's verify the user")
        logger.info(f"Please speak the passphrase: '{text_dependent_sensor.passphrase}'")
        
        # Attempt verification
        reading = await text_dependent_sensor.read()
        confidence = await text_dependent_sensor.verify("verify_user_1", reading)
        
        logger.info(f"Verification confidence: {confidence:.2f}")
        assert confidence > 0.0, "Verification should return positive confidence for correct user and passphrase"

    @pytest.mark.asyncio
    async def test_text_independent_verification(self, text_independent_sensor):
        """Test text-independent verification"""
        logger = logging.getLogger(__name__)
        
        # First enroll the user
        logger.info("\n=== Text-Independent Verification Test ===")
        logger.info("First, let's enroll a user...")
        await text_independent_sensor.enroll("verify_user_2")
        
        logger.info("\nNow, let's verify the user")
        logger.info("Please speak naturally for 5 seconds")
        
        # Attempt verification
        reading = await text_independent_sensor.read()
        confidence = await text_independent_sensor.verify("verify_user_2", reading)
        
        logger.info(f"Verification confidence: {confidence:.2f}")
        assert confidence > 0.0, "Verification should return positive confidence for correct user"

    @pytest.mark.asyncio
    async def test_wrong_passphrase(self, text_dependent_sensor):
        """Test verification with wrong passphrase"""
        logger = logging.getLogger(__name__)
        
        # First enroll the user
        logger.info("\n=== Wrong Passphrase Test ===")
        logger.info("First, let's enroll a user...")
        await text_dependent_sensor.enroll("verify_user_3")
        
        logger.info("\nNow, please speak a different phrase than the passphrase")
        
        # Attempt verification with wrong passphrase
        reading = await text_dependent_sensor.read()
        confidence = await text_dependent_sensor.verify("verify_user_3", reading)
        
        logger.info(f"Verification confidence: {confidence:.2f}")
        assert confidence < 0.5, "Verification should return low confidence for wrong passphrase"

    @pytest.mark.asyncio
    async def test_different_user(self, text_independent_sensor):
        """Test verification with different user"""
        logger = logging.getLogger(__name__)
        
        # First enroll the user
        logger.info("\n=== Different User Test ===")
        logger.info("First, let's enroll user A...")
        await text_independent_sensor.enroll("user_a")
        
        logger.info("\nNow, let's have a different person (user B) try to verify")
        logger.info("Please have a different person speak")
        
        # Attempt verification with different user
        reading = await text_independent_sensor.read()
        confidence = await text_independent_sensor.verify("user_a", reading)
        
        logger.info(f"Verification confidence: {confidence:.2f}")
        assert confidence < 0.5, "Verification should return low confidence for different user"

def main():
    """Manual test runner"""
    import asyncio
    
    async def run_tests():
        # Create sensors
        text_dep_sensor = VoiceRecognitionSensor()
        await text_dep_sensor.initialize({
            'text_dependent': True,
            'passphrase': "verify my identity now"
        })
        
        text_indep_sensor = VoiceRecognitionSensor()
        await text_indep_sensor.initialize({
            'text_dependent': False
        })
        
        # Create test instance
        test = TestVoiceRecognition()
        
        try:
            # Run tests
            await test.test_text_dependent_enrollment(text_dep_sensor)
            await test.test_text_dependent_verification(text_dep_sensor)
            await test.test_text_independent_enrollment(text_indep_sensor)
            await test.test_text_independent_verification(text_indep_sensor)
            await test.test_wrong_passphrase(text_dep_sensor)
            await test.test_different_user(text_indep_sensor)
            
        finally:
            # Cleanup
            await text_dep_sensor.cleanup()
            await text_indep_sensor.cleanup()

    asyncio.run(run_tests())

if __name__ == "__main__":
    main() 