import RPi.GPIO as GPIO
import time
import atexit
import logging

# ============================================================================
# CONFIGURATION - CẤU HÌNH HỆ THỐNG
# ============================================================================

# Cấu hình chân cảm biến (BCM mode)
SENSOR_CONFIG = {
    'LEFT': 23,
    'CENTER': 24, 
    'RIGHT': 25
}

# Cấu hình chân điều khiển motor L298N
MOTOR_CONFIG = {
    'LEFT': {
        'IN1': 6,   # Hướng motor trái
        'IN2': 5,   # Hướng motor trái
        'ENA': 12   # PWM motor trái
    },
    'RIGHT': {
        'IN3': 22,  # Hướng motor phải
        'IN4': 27,  # Hướng motor phải
        'ENB': 13   # PWM motor phải
    }
}

# Thông số PID Controller
PID_CONSTANTS = {
    'KP': 40.0,   # Proportional gain - độ nhạy với lỗi hiện tại
    'KI': 0.5,    # Integral gain - bù lỗi tích lũy
    'KD': 15.0    # Derivative gain - dự đoán xu hướng lỗi
}

# Thông số tốc độ
SPEED_CONFIG = {
    'BASE_SPEED': 40,     # Tốc độ cơ bản khi đi thẳng
    'MAX_SPEED': 80,      # Tốc độ tối đa
    'MIN_SPEED': 10,      # Tốc độ tối thiểu
    'PWM_FREQUENCY': 1000 # Tần số PWM
}

# Logging configuration
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(message)s')
logger = logging.getLogger(__name__)

# ============================================================================
# HARDWARE INTERFACE CLASS - GIAO DIỆN PHẦN CỨNG
# ============================================================================

class HardwareController:
    """Lớp điều khiển phần cứng GPIO"""
    
    def __init__(self):
        self.setup_gpio()
        self.setup_motors()
        self.setup_sensors()
        
    def setup_gpio(self):
        """Khởi tạo GPIO"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        logger.info("GPIO initialized in BCM mode")
        
    def setup_sensors(self):
        """Cấu hình chân cảm biến"""
        sensor_pins = list(SENSOR_CONFIG.values())
        GPIO.setup(sensor_pins, GPIO.IN)
        logger.info(f"Sensors configured on pins: {sensor_pins}")
        
    def setup_motors(self):
        """Cấu hình chân motor và PWM"""
        # Setup motor direction pins
        motor_pins = []
        for motor in MOTOR_CONFIG.values():
            motor_pins.extend(motor.values())
        GPIO.setup(motor_pins, GPIO.OUT)
        
        # Setup PWM for speed control
        self.pwm_left = GPIO.PWM(MOTOR_CONFIG['LEFT']['ENA'], SPEED_CONFIG['PWM_FREQUENCY'])
        self.pwm_right = GPIO.PWM(MOTOR_CONFIG['RIGHT']['ENB'], SPEED_CONFIG['PWM_FREQUENCY'])
        
        self.pwm_left.start(0)
        self.pwm_right.start(0)
        
        logger.info("Motors and PWM configured")
        
    def read_sensors(self):
        """Đọc trạng thái 3 cảm biến
        
        Returns:
            tuple: (left, center, right) - 1: phát hiện line, 0: không phát hiện
        """
        left = GPIO.input(SENSOR_CONFIG['LEFT'])
        center = GPIO.input(SENSOR_CONFIG['CENTER'])
        right = GPIO.input(SENSOR_CONFIG['RIGHT'])
        return (left, center, right)
        
    def set_motor_speed(self, left_speed, right_speed):
        """Điều khiển tốc độ và hướng motor
        
        Args:
            left_speed (float): Tốc độ motor trái (-100 to 100)
            right_speed (float): Tốc độ motor phải (-100 to 100)
        """
        # Clamp speeds to valid range
        left_speed = max(-100, min(100, left_speed))
        right_speed = max(-100, min(100, right_speed))
        
        # Control left motor
        if left_speed >= 0:
            GPIO.output(MOTOR_CONFIG['LEFT']['IN1'], GPIO.LOW)
            GPIO.output(MOTOR_CONFIG['LEFT']['IN2'], GPIO.HIGH)
        else:
            GPIO.output(MOTOR_CONFIG['LEFT']['IN1'], GPIO.HIGH)
            GPIO.output(MOTOR_CONFIG['LEFT']['IN2'], GPIO.LOW)
        
        # Control right motor
        if right_speed >= 0:
            GPIO.output(MOTOR_CONFIG['RIGHT']['IN3'], GPIO.HIGH)
            GPIO.output(MOTOR_CONFIG['RIGHT']['IN4'], GPIO.LOW)
        else:
            GPIO.output(MOTOR_CONFIG['RIGHT']['IN3'], GPIO.LOW)
            GPIO.output(MOTOR_CONFIG['RIGHT']['IN4'], GPIO.HIGH)
            
        # Set PWM duty cycle
        self.pwm_left.ChangeDutyCycle(abs(left_speed))
        self.pwm_right.ChangeDutyCycle(abs(right_speed))
        
    def stop_motors(self):
        """Dừng tất cả motor"""
        self.set_motor_speed(0, 0)
        
    def cleanup(self):
        """Dọn dẹp GPIO khi kết thúc"""
        self.stop_motors()
        self.pwm_left.stop()
        self.pwm_right.stop()
        GPIO.cleanup()
        logger.info("Hardware cleanup completed")

# ============================================================================
# PID CONTROLLER CLASS - BỘ ĐIỀU KHIỂN PID
# ============================================================================

class PIDController:
    """Bộ điều khiển PID cho line following"""
    
    def __init__(self, kp, ki, kd, output_limits=(-100, 100)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.output_limits = output_limits
        self.reset()
        
    def reset(self):
        """Reset các giá trị PID"""
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        
    def compute(self, error):
        """Tính toán output PID
        
        Args:
            error (float): Lỗi hiện tại (setpoint - current_value)
            
        Returns:
            float: Output điều khiển
        """
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt <= 0.0:
            return 0.0
            
        # Proportional term
        proportional = self.kp * error
        
        # Integral term với anti-windup
        self.integral += error * dt
        integral = self.ki * self.integral
        
        # Derivative term
        derivative = self.kd * (error - self.previous_error) / dt
        
        # Tổng output
        output = proportional + integral + derivative
        
        # Giới hạn output
        output = max(self.output_limits[0], min(self.output_limits[1], output))
        
        # Lưu giá trị cho lần tính tiếp theo
        self.previous_error = error
        self.last_time = current_time
        
        return output
        
    def set_tunings(self, kp, ki, kd):
        """Thay đổi thông số PID"""
        self.kp = kp
        self.ki = ki  
        self.kd = kd

# ============================================================================
# LINE SENSOR PROCESSOR - XỬ LÝ CẢMBIEN LINE
# ============================================================================

class LineSensorProcessor:
    """Xử lý dữ liệu từ cảm biến line"""
    
    # Bảng tra cứu lỗi dựa trên trạng thái cảm biến
    ERROR_TABLE = {
        (0, 0, 0): 0,    # Không thấy line - giữ hướng cũ
        (0, 0, 1): -2,   # Lệch phải mạnh
        (0, 1, 0): 0,    # Đi thẳng - hoàn hảo
        (0, 1, 1): -1,   # Lệch phải nhẹ
        (1, 0, 0): 2,    # Lệch trái mạnh
        (1, 0, 1): 0,    # Trường hợp đặc biệt - intersection hoặc noise
        (1, 1, 0): 1,    # Lệch trái nhẹ
        (1, 1, 1): 0     # Tất cả thấy line - intersection hoặc start line
    }
    
    def __init__(self):
        self.last_error = 0
        
    def calculate_error(self, sensor_values):
        """Tính toán lỗi dựa trên giá trị cảm biến
        
        Args:
            sensor_values (tuple): (left, center, right) sensor readings
            
        Returns:
            float: Lỗi position (-2 to 2)
                  -2: lệch phải mạnh, 0: đúng center, +2: lệch trái mạnh
        """
        error = self.ERROR_TABLE.get(sensor_values, self.last_error)
        
        # Chỉ cập nhật last_error nếu có tín hiệu từ cảm biến
        if sensor_values != (0, 0, 0):
            self.last_error = error
            
        return error

# ============================================================================
# MAIN ROBOT CLASS - LỚP ROBOT CHÍNH
# ============================================================================

class LineFollowingRobot:
    """Robot dò line với điều khiển PID"""
    
    def __init__(self):
        self.hardware = HardwareController()
        self.pid = PIDController(
            PID_CONSTANTS['KP'], 
            PID_CONSTANTS['KI'], 
            PID_CONSTANTS['KD']
        )
        self.sensor_processor = LineSensorProcessor()
        
        # Đăng ký cleanup khi thoát
        atexit.register(self.cleanup)
        
        logger.info("Line Following Robot initialized")
        logger.info(f"PID Parameters: KP={PID_CONSTANTS['KP']}, KI={PID_CONSTANTS['KI']}, KD={PID_CONSTANTS['KD']}")
        
    def run(self):
        """Chạy chương trình chính"""
        logger.info("Starting line following...")
        
        try:
            while True:
                # Đọc cảm biến
                sensor_values = self.hardware.read_sensors()
                
                # Tính toán lỗi
                error = self.sensor_processor.calculate_error(sensor_values)
                
                # Tính toán correction từ PID
                pid_correction = self.pid.compute(error)
                
                # Tính toán tốc độ cho từng motor
                base_speed = SPEED_CONFIG['BASE_SPEED']
                left_speed = base_speed - pid_correction
                right_speed = base_speed + pid_correction
                
                # Giới hạn tốc độ
                left_speed = max(SPEED_CONFIG['MIN_SPEED'], 
                               min(SPEED_CONFIG['MAX_SPEED'], left_speed))
                right_speed = max(SPEED_CONFIG['MIN_SPEED'], 
                                min(SPEED_CONFIG['MAX_SPEED'], right_speed))
                
                # Điều khiển motor
                self.hardware.set_motor_speed(left_speed, right_speed)
                
                # Debug info (có thể comment để tăng performance)
                if time.time() % 1 < 0.01:  # Log mỗi giây
                    logger.debug(f"Sensors: {sensor_values}, Error: {error:.2f}, "
                               f"PID: {pid_correction:.2f}, Motors: L={left_speed:.1f} R={right_speed:.1f}")
                
                # Delay nhỏ để tránh quá tải CPU
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            logger.info("Program interrupted by user")
        except Exception as e:
            logger.error(f"Unexpected error: {e}")
        finally:
            self.cleanup()
            
    def cleanup(self):
        """Dọn dẹp khi kết thúc chương trình"""
        logger.info("Cleaning up...")
        self.hardware.cleanup()
        
    def tune_pid(self, kp=None, ki=None, kd=None):
        """Điều chỉnh thông số PID trong runtime"""
        current_kp = kp if kp is not None else self.pid.kp
        current_ki = ki if ki is not None else self.pid.ki
        current_kd = kd if kd is not None else self.pid.kd
        
        self.pid.set_tunings(current_kp, current_ki, current_kd)
        logger.info(f"PID tuned to: KP={current_kp}, KI={current_ki}, KD={current_kd}")

# ============================================================================
# MAIN EXECUTION - THỰC THI CHƯƠNG TRÌNH
# ============================================================================

def main():
    """Hàm chính"""
    print("=" * 60)
    print("LINE FOLLOWING ROBOT WITH PID CONTROLLER")
    print("=" * 60)
    print("Nhấn Ctrl+C để dừng chương trình")
    print()
    
    # Khởi tạo và chạy robot
    robot = LineFollowingRobot()
    robot.run()

if __name__ == "__main__":
    main()
