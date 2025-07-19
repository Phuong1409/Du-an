import RPi.GPIO as GPIO
import time
import atexit

# === CẤU HÌNH CHÂN GPIO ===
SENSOR_LEFT = 23
SENSOR_CENTER = 24
SENSOR_RIGHT = 25

MOTOR_PINS = {
    'IN1': 6, 'IN2': 5, 'ENA': 12,    # Motor trái
    'IN3': 22, 'IN4': 27, 'ENB': 13   # Motor phải
}

# === PID HẰNG SỐ VÀ TỐC ĐỘ ===
PID_CONSTANTS = {'KP': 70.0, 'KI': 0.0, 'KD': 25.0}
SPEED_CONFIG = {'BASE_SPEED': 30, 'MAX_SPEED': 80, 'MIN_SPEED': 15, 'PWM_FREQUENCY': 1000}

# === KHỞI TẠO GPIO ===
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup([SENSOR_LEFT, SENSOR_CENTER, SENSOR_RIGHT], GPIO.IN)
GPIO.setup(list(MOTOR_PINS.values()), GPIO.OUT)

pwm_ena = GPIO.PWM(MOTOR_PINS['ENA'], SPEED_CONFIG['PWM_FREQUENCY'])
pwm_enb = GPIO.PWM(MOTOR_PINS['ENB'], SPEED_CONFIG['PWM_FREQUENCY'])
pwm_ena.start(0)
pwm_enb.start(0)

# === HÀM DỌN DẸP ===
def cleanup():
    pwm_ena.stop()
    pwm_enb.stop()
    GPIO.cleanup()

atexit.register(cleanup)

# === HÀM ĐIỀU KHIỂN ĐỘNG CƠ ===
def set_motor(left_forward, left_backward, right_forward, right_backward, left_speed, right_speed):
    GPIO.output(MOTOR_PINS['IN1'], left_forward)
    GPIO.output(MOTOR_PINS['IN2'], left_backward)
    GPIO.output(MOTOR_PINS['IN3'], right_forward)
    GPIO.output(MOTOR_PINS['IN4'], right_backward)
    pwm_ena.ChangeDutyCycle(max(0, min(100, left_speed)))
    pwm_enb.ChangeDutyCycle(max(0, min(100, right_speed)))

def forward(left_speed, right_speed):
    set_motor(0, 1, 1, 0, left_speed, right_speed)

def stop():
    set_motor(0, 0, 0, 0, 0, 0)

# === LỚP PID ===
class PID:
    def __init__(self, kp, ki, kd):
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

# === ĐỌC CẢM BIẾN ===
def read_sensors():
    L = GPIO.input(SENSOR_LEFT)
    C = GPIO.input(SENSOR_CENTER)
    R = GPIO.input(SENSOR_RIGHT)
    return L, C, R

# === TÍNH LỖI PID ===
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
        return 0  # mất line: tiến thẳng tạm thời
    else:
        return 0

# === LỚP CHÍNH ĐIỀU KHIỂN DÒ LINE ===
class LineFollowingRobot:
    def __init__(self):
        self.pid = PID(PID_CONSTANTS['KP'], PID_CONSTANTS['KI'], PID_CONSTANTS['KD'])
        self.base_speed = SPEED_CONFIG['BASE_SPEED']
        self.motor_trim = {'LEFT': 5.0, 'RIGHT': -5.0}  # Điều chỉnh nếu motor lệch

    def run(self):
        print("Khởi động robot dò line PID...")
        try:
            while True:
                L, C, R = read_sensors()
                error = calculate_error(L, C, R)
                correction = self.pid.compute(error)

                left_speed = self.base_speed - correction + self.motor_trim['LEFT']
                right_speed = self.base_speed + correction + self.motor_trim['RIGHT']

                # Giới hạn tốc độ
                left_speed = max(SPEED_CONFIG['MIN_SPEED'], min(SPEED_CONFIG['MAX_SPEED'], left_speed))
                right_speed = max(SPEED_CONFIG['MIN_SPEED'], min(SPEED_CONFIG['MAX_SPEED'], right_speed))

                forward(left_speed, right_speed)
                time.sleep(0.01)
        except KeyboardInterrupt:
            print("Dừng robot.")
            stop()

# === CHẠY ===
if __name__ == "__main__":
    robot = LineFollowingRobot()
    robot.run()
