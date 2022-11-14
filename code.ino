#define MOTOR_POWER 255 // 차량 모터 출력값
#define MOTOR_POWER_ROTATE 150 // 차량 모터 출력값

// 회전시 너무 빠르면 방향을 초과해 회전해 버리므로 회전은 천천히 함
// 혹은 한바퀴 회전 방법 사용 바람
int motor_speed = MOTOR_POWER;
int motor_speed_rotate = MOTOR_POWER_ROTATE;

#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

// GPS 설정
#include <TinyGPS++.h> // 라이브러리를 포함
TinyGPSPlus gps; // GPS를 정의

// 지자계 센서
#define SDA_PIN 17
#define SCL_PIN 18
#include <Wire.h>
#define Addr 0x1E
#define DIRECTION_SEND_PERIOD 500
#define ROTATE_MOTOR_POWER 150 // 지자계 세팅을 위한 회전 출력 설정
#define MAG_SETTING_PERIOD 10000 // 지자계 세팅시 회전하는 시간
unsigned long prev_direction_send_millis;
float magnetic_x, magnetic_y; // 센서에서 감지된 값
float center_magnetic_x , center_magnetic_y; // 평향 값

int mode = 0; // 0 : 수동 제어 모드 , 1 : 위치 지정 모드
/*
  unsigned long obstacle_millis; // 장애물이 감지된 시간
  #define AUTOMATIC_FREE_DRIVING_PERIOD 5000
*/
// 모터드라이버 실드 세팅
#define SHIFT_DATA 14
#define SHIFT_LATCH 25
#define SHIFT_CLOCK 32
#define ENA1 33
#define ENA2 27
#define ENA3 26
#define ENA4 13
#define BAT_READ_PIN 36
#define R1 2200.0 // 분압 저항 값
#define R2 1000.0 // 분압 저항 값
#define VOLT_LIMIT 7000 // 저전압 경보 값
#define BUZZER_PIN 5
uint8_t motor_bit = 0b00000000;

void setup()
{
  Serial.begin(115200);
  SerialBT.begin("Project");
  Serial2.begin(9600, SERIAL_8N1, 2, -1); // RX, TX , GPS 시리얼
  Serial1.begin(115200, SERIAL_8N1, 23, -1); // 라이다 esp32 cam
  // 지자계 센서
  Wire.begin(SDA_PIN, SCL_PIN);
  // Set operating mode to continuous
  Wire.beginTransmission(Addr);
  Wire.write(byte(0x02));
  Wire.write(byte(0x00));
  Wire.endTransmission();

  pinMode(SHIFT_DATA, OUTPUT);
  pinMode(SHIFT_LATCH, OUTPUT);
  pinMode(SHIFT_CLOCK, OUTPUT);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);

  ledcAttachPin(ENA1, 0);
  ledcAttachPin(ENA2, 1);
  ledcAttachPin(ENA3, 2);
  ledcAttachPin(ENA4, 3);
  ledcSetup(0, 5000, 8);
  ledcSetup(1, 5000, 8);
  ledcSetup(2, 5000, 8);
  ledcSetup(3, 5000, 8);
  stop();

  center_magnetic_x = 0;
  center_magnetic_y = 0;
  prev_direction_send_millis = millis();
}

int distance_list[36];

void loop()
{
  while ( Serial1.available() >= 74 )
  {
    int id = Serial1.read();
    if ( id == 20 )
    {
      int cmd = Serial1.read();
      if ( cmd == 6 )
      {
        for ( int i = 0; i < 36; i++)
        {
          distance_list[i] = Serial1.read();
          distance_list[i] = distance_list[i] | ( Serial1.read() << 8 );
          if ( distance_list[i] == 0 )
          {
            distance_list[i] = 20000;
          }
        }
        Serial1.read();
      }
    }
  }

  while (Serial2.available() > 0) // GPS로부터 받은 데이타가 있으면
  {
    if (gps.encode(Serial2.read())) // GPS신호 분석
    {
      send_info(); // 블루투스로 데이타 전송
    }
  }

  if ( ( millis() - prev_direction_send_millis ) > DIRECTION_SEND_PERIOD )
  {
    int direction = get_magnetic_sensor();
    SerialBT.write('k'); // 데이타 송신 시작을 알림
    SerialBT.write(2); // 방향 정보 송신
    SerialBT.write(direction >> 0); // 현재 차량의 방위를 전송
    SerialBT.write(direction >> 8);


    SerialBT.write('k'); // 데이타 송신 시작을 알림
    SerialBT.write(3); // 장애물간 거리 값 전송

    Serial.println("##########");
    Serial.println(distance_list[15]);
    Serial.println(distance_list[16]);
    Serial.println(distance_list[17]);
    Serial.println(distance_list[18]);
    Serial.println(distance_list[19]);
    Serial.println(distance_list[20]);
    Serial.println(distance_list[21]);

    SerialBT.write(distance_list[15]);
    SerialBT.write(distance_list[15] >> 8);
    SerialBT.write(distance_list[16]);
    SerialBT.write(distance_list[16] >> 8);
    SerialBT.write(distance_list[17]);
    SerialBT.write(distance_list[17] >> 8);
    SerialBT.write(distance_list[18]);
    SerialBT.write(distance_list[18] >> 8);
    SerialBT.write(distance_list[19]);
    SerialBT.write(distance_list[19] >> 8);
    SerialBT.write(distance_list[20]);
    SerialBT.write(distance_list[20] >> 8);
    SerialBT.write(distance_list[21]);
    SerialBT.write(distance_list[21] >> 8);

    prev_direction_send_millis = millis();
  }

  cmd_processing();
  if ( mode == 1 ) // GPS 위치 이동 모드인 경우
  {
    if ( distance_list[18] < 1000 ){ // 장애물이 감지된 경우 정지
      stop();
    }
    else if(( distance_list[15] < 1000 ) || ( distance_list[16] < 1000 ) || ( distance_list[17] < 1000 )){
      right();
    }
    else if(( distance_list[19] < 1000 ) || ( distance_list[20] < 1000 ) || ( distance_list[21] < 1000 )){
      left();
    }
  }
}

void cmd_processing()
{
  if ( SerialBT.available() >= 2 ) // 블루투스 데이타를 2바이트 이상 받은 경우
  {
    int id = SerialBT.read(); // 데이타 읽음
    if ( id == 20 ) // 시작 명령이면
    {
      int cmd = SerialBT.read(); // 명령을 읽음
      if ( cmd == 1 ) // FRONT
      {
        mode = 0;
        front();
      }
      if ( cmd == 2 ) // RIGHT
      {
        mode = 0;
        right();
      }
      if ( cmd == 3 ) // LEFT
      {
        mode = 0;
        left();
      }
      if ( cmd == 4 ) // BACK
      {
        mode = 0;
        back();
      }
      if ( cmd == 5 ) // STOP
      {
        mode = 0;
        stop();
      }

      if ( cmd == 11 ) // FRONT
      {
        mode = 1;
        front();
      }
      if ( cmd == 12 ) // RIGHT
      {
        mode = 1;
        right();
      }
      if ( cmd == 13 ) // LEFT
      {
        mode = 1;
        left();
      }
      if ( cmd == 14 ) // BACK
      {
        mode = 1;
        back();
      }
      if ( cmd == 15 ) // STOP
      {
        mode = 1;
        stop();
      }


      if ( cmd == 6 ) // 지자계 센서 보정 명령시
      {
        setting_hmc();
      }
      if ( cmd == 7 ) // 수동 제어 모드
      {
        mode = 0;
        stop();
        //        Serial.println("수동모드");
      }
      if ( cmd == 8 ) // 위치 지정 모드
      {
        mode = 1;
        stop();
        //        Serial.println("위치지정모드");
      }
    }
  }
}

void send_info() // GPS데이타를 전송하는 함수
{
  if (gps.location.isValid()) // GPS데이타가 정상이면
  {
    Serial.println("SEND INFO");
    unsigned long lat;
    unsigned long lng;

    lat = gps.location.lat() * 1000000; // 위도에 1000000을 곱하여 소수자리를 없앰
    lng = gps.location.lng() * 1000000; // 경도에 1000000을 곱하여 소수자리를 없앰
    SerialBT.write('k'); // 데이타 송신 시작을 알림
    SerialBT.write(1); // GPS데이타 송신을 위한 명령
    SerialBT.write(lat >> 0); // 위도 데이타 전송 4 Byte
    SerialBT.write(lat >> 8);
    SerialBT.write(lat >> 16);
    SerialBT.write(lat >> 24);

    SerialBT.write(lng >> 0); // 경도 데이타 전송 4 Byte
    SerialBT.write(lng >> 8);
    SerialBT.write(lng >> 16);
    SerialBT.write(lng >> 24);
  }
  else
  {
  }
}

float get_magnetic_sensor() // 지자계 센서로 방향을 측정 , 지자계 중심을 고려한 angle을 반환
{
  int16_t x, y, z;

  Wire.beginTransmission(Addr);
  Wire.write(byte(0x03));       // Send request to X MSB register
  Wire.endTransmission();

  Wire.requestFrom(Addr, 6);    // Request 6 bytes; 2 bytes per axis
  if (Wire.available() == 6)
  { // If 6 bytes available
    x = Wire.read() << 8 | Wire.read();
    z = Wire.read() << 8 | Wire.read();
    y = Wire.read() << 8 | Wire.read();
  }

  magnetic_x = x;
  magnetic_y = y;

  float angle = atan2(x - center_magnetic_x, y - center_magnetic_y) * 180.0 / 3.141592654; // + magnetic_declination; // 진북을 0도로한 시계방향으로 현재 센서의 각도
  return -angle;
}

void setting_hmc() // 지자계 센서를 보정하는 함수
{
  center_magnetic_x = 0;
  center_magnetic_y = 0;

  int max_value_x = -10000;
  int min_value_x = 10000;
  int max_value_y = -10000;
  int min_value_y = 10000;
  unsigned long start_millis = millis();
  motor_speed = ROTATE_MOTOR_POWER;
  while ( ( millis() - start_millis ) < MAG_SETTING_PERIOD )
  {
    left(); // 지자계 보정을 위하여 좌회전
    get_magnetic_sensor();
    if ( magnetic_x > max_value_x )
    {
      max_value_x = magnetic_x;
    }
    if ( magnetic_x < min_value_x )
    {
      min_value_x = magnetic_x;
    }
    if ( magnetic_y > max_value_y )
    {
      max_value_y = magnetic_y;
    }
    if ( magnetic_y < min_value_y )
    {
      min_value_y = magnetic_y;
    }
  }
  stop();
  motor_speed = MOTOR_POWER;
  center_magnetic_x = ( max_value_x + min_value_x ) / 2;
  center_magnetic_y = ( max_value_y + min_value_y ) / 2;
}

void front()
{
  m1(MOTOR_POWER);
  m2(MOTOR_POWER);
  m3(MOTOR_POWER);
  m4(MOTOR_POWER);
}

void back()
{
  m1(-MOTOR_POWER);
  m2(-MOTOR_POWER);
  m3(-MOTOR_POWER);
  m4(-MOTOR_POWER);
}

void left()
{
  //  m1(-MOTOR_POWER);
  m1(0);
  m2(MOTOR_POWER);
  //  m3(-MOTOR_POWER);
  m3(0);
  m4(MOTOR_POWER);
}

void right()
{
  m1(MOTOR_POWER);
  //  m2(-MOTOR_POWER);
  m2(0);
  m3(MOTOR_POWER);
  //  m4(-MOTOR_POWER);
  m4(0);
}


void stop()
{
  m1(0);
  m2(0);
  m3(0);
  m4(0);
}

void m1(int value)
{
  if ( value > 0 )
  {
    motor_bit = motor_bit & 0b00111111;
    motor_bit = motor_bit | 0b10000000;
    ledcWrite(0, value);
  }
  else
  {
    motor_bit = motor_bit & 0b00111111;
    motor_bit = motor_bit | 0b01000000;
    ledcWrite(0, -value);
  }
  digitalWrite(SHIFT_LATCH, LOW);
  shiftOut(SHIFT_DATA, SHIFT_CLOCK, LSBFIRST, motor_bit);
  digitalWrite(SHIFT_LATCH, HIGH);
}

void m2(int value)
{
  if ( value > 0 )
  {
    motor_bit = motor_bit & 0b11110011;
    motor_bit = motor_bit | 0b00001000;
    ledcWrite(1, value);
  }
  else
  {
    motor_bit = motor_bit & 0b11110011;
    motor_bit = motor_bit | 0b00000100;
    ledcWrite(1, -value);
  }
  digitalWrite(SHIFT_LATCH, LOW);
  shiftOut(SHIFT_DATA, SHIFT_CLOCK, LSBFIRST, motor_bit);
  digitalWrite(SHIFT_LATCH, HIGH);
}

void m3(int value)
{
  if ( value > 0 )
  {
    motor_bit = motor_bit & 0b11001111;
    motor_bit = motor_bit | 0b00100000;
    ledcWrite(2, value);
  }
  else
  {
    motor_bit = motor_bit & 0b11001111;
    motor_bit = motor_bit | 0b00010000;
    ledcWrite(2, -value);
  }
  digitalWrite(SHIFT_LATCH, LOW);
  shiftOut(SHIFT_DATA, SHIFT_CLOCK, LSBFIRST, motor_bit);
  digitalWrite(SHIFT_LATCH, HIGH);
}

void m4(int value)
{
  if ( value > 0 )
  {
    motor_bit = motor_bit & 0b11111100;
    motor_bit = motor_bit | 0b00000010;
    ledcWrite(3, value);
  }
  else
  {
    motor_bit = motor_bit & 0b11111100;
    motor_bit = motor_bit | 0b00000001;
    ledcWrite(3, -value);
  }
  digitalWrite(SHIFT_LATCH, LOW);
  shiftOut(SHIFT_DATA, SHIFT_CLOCK, LSBFIRST, motor_bit);
  digitalWrite(SHIFT_LATCH, HIGH);
}
