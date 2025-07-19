import RPi.GPIO as GPIO
import time
import atexit

# ==== Cảm biến line (3 cái: Trái, Giữa, Phải) ====
SENSOR_PINS = [23, 24, 25]  # GPIO 23 = Trái, 24 = Giữa, 25 = Phải

# ==== Chân điều khiển động cơ ====
MOTOR_PINS = {
    'IN1': 6,
    'IN2': 5,
    'ENA': 12,  # PWM trái
    'IN3': 22,
    'IN4': 27,
    'ENB': 13   # PWM phải
}

# ==== Thiết lập GPIO ====
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(SENSOR_PINS, GPIO.IN)
GPIO.setup(list(MOTOR_PINS.values()), GPIO.OUT)

pwm_ena = GPIO.PWM(MOTOR_PINS['ENA'], 500)
pwm_enb = GPIO.PWM(MOTOR_PINS['ENB'], 500)
pwm_ena.start(0)
pwm_enb.start(0)

# ==== PID Class ====
class PID:
    def __init__(self, kp=20, ki=0.3, kd=50):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
        self.prev_time = time.time()

    def compute(self, error):
        now = time.time()
        dt = now - self.prev_time if now > self.prev_time else 0.01
        self.prev_time = now
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

# ==== Điều khiển động cơ ====
def motor_left(speed):
    if speed >= 0:
        GPIO.output(MOTOR_PINS['IN1'], GPIO.HIGH)
        GPIO.output(MOTOR_PINS['IN2'], GPIO.LOW)
        pwm_ena.ChangeDutyCycle(min(speed, 100))
    else:
        GPIO.output(MOTOR_PINS['IN1'], GPIO.LOW)
        GPIO.output(MOTOR_PINS['IN2'], GPIO.HIGH)
        pwm_ena.ChangeDutyCycle(min(-speed, 100))

def motor_right(speed):
    if speed >= 0:
        GPIO.output(MOTOR_PINS['IN3'], GPIO.HIGH)
        GPIO.output(MOTOR_PINS['IN4'], GPIO.LOW)
        pwm_enb.ChangeDutyCycle(min(speed, 100))
    else:
        GPIO.output(MOTOR_PINS['IN3'], GPIO.LOW)
        GPIO.output(MOTOR_PINS['IN4'], GPIO.HIGH)
        pwm_enb.ChangeDutyCycle(min(-speed, 100))

# ==== Đọc cảm biến và tính toán lỗi ====
def read_sensors():
    return [GPIO.input(pin) for pin in SENSOR_PINS]

def calculate_error(sensors):
    L, C, R = sensors
    print("Sensors: L={}, C={}, R={}".format(L, C, R))

    # Quy đổi cảm biến sang giá trị lỗi cho PID
    if C == 1 and L == 0 and R == 0:
        return 0  # Đúng giữa line
    elif L == 1 and C == 0 and R == 0:
        return 1  # Lệch trái
    elif R == 1 and C == 0 and L == 0:
        return -1  # Lệch phải
    elif L == 1 and C == 1 and R == 0:
        return 0.5  # hơi lệch trái
    elif R == 1 and C == 1 and L == 0:
        return -0.5  # hơi lệch phải
    elif L == 1 and R == 1 and C == 0:
        return 0  # có thể ngã tư
    elif L == 0 and C == 0 and R == 0:
        return 0  # mất line → giữ hướng cũ
    else:
        return 0

# ==== Điều chỉnh tốc độ ====
def speed_control(pid_value, base_speed=60):
    left_speed = base_speed - pid_value
    right_speed = base_speed + pid_value
    left_speed = max(min(left_speed, 100), -100)
    right_speed = max(min(right_speed, 100), -100)
    motor_left(left_speed)
    motor_right(right_speed)

# ==== Dọn dẹp GPIO khi thoát ====
def cleanup():
    pwm_ena.stop()
    pwm_enb.stop()
    GPIO.cleanup()
    print("Đã thoát và dọn dẹp GPIO.")

atexit.register(cleanup)

# ==== Vòng lặp chính ====
pid = PID(kp=20, ki=0.3, kd=50)
print("Khởi động robot dò line với 3 cảm biến...")

try:
    while True:
        sensors = read_sensors()
        error = calculate_error(sensors)
        correction = pid.compute(error)
        speed_control(correction)
        time.sleep(0.01)
except KeyboardInterrupt:
    cleanup()
