
/*
 * LineTracer_PID.ino
 * Arduino UNO + 6 Analog IR Sensors + TB6612FNG
 * 초기 PD 제어 테스트
 */

// ===== 센서 핀 =====
const int sensorPins[6] = {A0, A1, A2, A3, A4, A5};

// ===== 모터 드라이버 핀 =====
const int PWMA = 5;
const int AIN1 = 7;
const int AIN2 = 8;

const int PWMB = 6;
const int BIN1 = 9;
const int BIN2 = 10;

const int STBY = 4;

// ===== 제어 변수 =====
float Kp = 0.025;
float Kd = 0.045;

int baseSpeed = 100;
int lastError = 0;

// 실제 센서 특성에 따라 변경
// true: 검은색에서 ADC 값이 낮은 경우
bool blackIsLow = true;

int sensorMin[6];
int sensorMax[6];

unsigned long lastControl = 0;

// ===== 모터 제어 함수 =====
void setMotor(int pwmPin, int in1, int in2, int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  analogWrite(pwmPin, abs(speed));
}

void setMotors(int left, int right) {
  setMotor(PWMA, AIN1, AIN2, left);
  setMotor(PWMB, BIN1, BIN2, right);
}

// ===== 센서 보정 =====
void calibrateSensors() {
  for (int i = 0; i < 6; i++) {
    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }

  // 5초 동안 센서를 흑백 영역 위로 손으로 이동
  unsigned long start = millis();

  while (millis() - start < 5000) {
    for (int i = 0; i < 6; i++) {
      int v = analogRead(sensorPins[i]);
      sensorMin[i] = min(sensorMin[i], v);
      sensorMax[i] = max(sensorMax[i], v);
    }
  }
}

// ===== 라인 위치 계산 =====
int readLinePosition() {
  long weightedSum = 0;
  long total = 0;

  for (int i = 0; i < 6; i++) {
    int raw = analogRead(sensorPins[i]);

    int span = sensorMax[i] - sensorMin[i];

    if (span < 30) continue;

    long value = (long)(raw - sensorMin[i]) * 1000 / span;
    value = constrain(value, 0L, 1000L);

    if (blackIsLow) {
      value = 1000 - value;
    }

    // 약한 반응 제거
    if (value < 100) value = 0;

    weightedSum += value * (i * 1000L);
    total += value;
  }

  // 라인을 완전히 놓쳤으면 정지
  if (total < 200) return -1;

  return weightedSum / total;
}

void setup() {
  Serial.begin(115200);

  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, LOW);
  setMotors(0, 0);

  Serial.println("Calibrating sensors...");
  calibrateSensors();

  Serial.println("Calibration finished");
  delay(2000);

  digitalWrite(STBY, HIGH);
}

void loop() {
  // 약 10ms 간격으로 제어
  if (micros() - lastControl < 10000) return;
  lastControl = micros();

  int position = readLinePosition();

  if (position < 0) {
    setMotors(0, 0);
    lastError = 0;
    return;
  }

  // 목표 중앙 = 2500
  int error = 2500 - position;

  // 초기 PD 제어
  float correction = Kp * error
                   + Kd * (error - lastError);

  lastError = error;

  int leftSpeed  = constrain(
      (int)(baseSpeed + correction), 0, 255);

  int rightSpeed = constrain(
      (int)(baseSpeed - correction), 0, 255);

  setMotors(leftSpeed, rightSpeed);
}
