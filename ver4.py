import RPi.GPIO as GPIO
import time
import atexit

# === Cấu hình chân cảm biến ===
SENSOR_LEFT = 23
SENSOR_CENTER = 24
SENSOR_RIGHT = 25

# === Cấu hình chân động cơ L298N ===
MOTOR_PINS = {
    'IN1': 6, 'IN2': 5, 'ENA': 12,   # Motor trái
    'IN3': 22, 'IN4': 27, 'ENB': 13  # Motor phải
}

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Cảm biến
GPIO.setup([SENSOR_LEFT, SENSOR_CENTER, SENSOR_RIGHT], GPIO.IN)

# Motor
GPIO.setup(list(MOTOR_PINS.values()), GPIO.OUT)
pwm_ena = GPIO.PWM(MOTOR_PINS['ENA'], 1000)
pwm_enb = GPIO.PWM(MOTOR_PINS['ENB'], 1000)
pwm_ena.start(0)
pwm_enb.start(0)

# === Hàm điều khiển motor ===
def set_motor(l_in1, l_in2, r_in3, r_in4, speed_l, speed_r):
    GPIO.output(MOTOR_PINS['IN1'], l_in1)
    GPIO.output(MOTOR_PINS['IN2'], l_in2)
    GPIO.output(MOTOR_PINS['IN3'], r_in3)
    GPIO.output(MOTOR_PINS['IN4'], r_in4)
    pwm_ena.ChangeDutyCycle(max(0, min(speed_l, 100)))
    pwm_enb.ChangeDutyCycle(max(0, min(speed_r, 100)))

def forward(left_speed, right_speed):
    set_motor(0, 1, 1, 0, left_speed, right_speed)

def stop():
    set_motor(0, 0, 0, 0, 0, 0)

atexit.register(lambda: (stop(), pwm_ena.stop(), pwm_enb.stop(), GPIO.cleanup()))

# === PID Controller ===
class PID:
    def __init__(self, kp=50, ki=0.1, kd=8):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()

    def compute(self, error):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

# === Đọc cảm biến ===
def read_sensors():
    L = GPIO.input(SENSOR_LEFT)
    C = GPIO.input(SENSOR_CENTER)
    R = GPIO.input(SENSOR_RIGHT)
    return L, C, R

# === Tính toán lỗi theo cảm biến 3 kênh ===
def calculate_error(L, C, R):
    if L == 1 and C == 0 and R == 0:
        return 2
    elif L == 1 and C == 1 and R == 0:
        return 1
    elif L == 0 and C == 1 and R == 0:
        return 0
    elif L == 0 and C == 1 and R == 1:
        return -1
    elif L == 0 and C == 0 and R == 1:
        return -2
    elif L == 0 and C == 0 and R == 0:
        return 0  # mất line: tạm thời coi là tiến thẳng
    else:
        return 0

# === Khởi tạo PID ===
pid = PID(kp=50, ki=0.1, kd=8)
base_speed = 12  # tốc độ khi đi thẳng

# === Vòng lặp chính ===
print("Bắt đầu dò line 3 cảm biến (PID)...")
try:
    while True:
        L, C, R = read_sensors()
        error = calculate_error(L, C, R)
        correction = pid.compute(error)

        # Quay gắt nếu lệch hẳn trái/phải, còn lại dùng PID
        if error == 2:
            left_speed = 2
            right_speed = 35
        elif error == 1:
            left_speed = 8
            right_speed = 30
        elif error == -1:
            left_speed = 30
            right_speed = 8
        elif error == -2:
            left_speed = 35
            right_speed = 2
        else:
            left_speed = base_speed - correction
            right_speed = base_speed + correction

        # Giới hạn tốc độ trong khoảng 0–100
        left_speed = max(0, min(100, left_speed))
        right_speed = max(0, min(100, right_speed))

        forward(left_speed, right_speed)
        time.sleep(0.01)

except KeyboardInterrupt:
    print("Kết thúc chương trình.")
