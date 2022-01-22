// uncomment this if you do not have MPU6050 gyro connected
#define HAVE_GYRO

#include <Adafruit_ST7735.h>
#include <Servo.h>
#ifdef HAVE_GYRO
#include <Wire.h>
#include <MPU6050.h>
#endif

// put next line after comment, if you have no MPU6050 gyroscope


// pin connections:
// servos:
// touch:
//    1: A2
//    2: A3
// mp3 player:
//    RX:     4
// gyro (MPU-6050)
//    SDA:    A4
//    SCL:    A5


#define MP3_OUTPUT_PIN 4   // connect Rx pin of DFPlayer to A2


#define SERIAL_STATE_IDLE      0
#define SERIAL_STATE_RECEIVING 1
#define SERIAL_BUFFER_LENGTH   20

// Ottis internal arduino communication protocol:
//
// 2 -> 1:
// 'A'  - touch1 up
// 'B'  - touch1 down
// 'C'  - touch2 up
// 'D'  - touch2 down
// '1'..'4'  - test communication
// rXX  - 16bit int binary roll (*100)
// pXX  - 16bit int binary pitch (*100)
// yXX  - 16bit int binary yaw (*100)
//
// 1 -> 2:
//
// 'L'  - turn on LED1
// 'l'  - turn off LED1
// 'E'  - turn on LED2
// 'e'  - turn off LED2
// '1'  - play current song
// '2'  - set next song
// '3'  - set previous song
// '4'  - test display
// '8'  - response with '#' (check connection)
// 'R'/'r'  - turn on/off reporting roll
// 'P'/'p'  - turn on/off reporting pitch
// 'Y'/'y'  - turn on/off reporting yaw

static uint8_t MSG_TOUCH_UP[2] = { 'A', 'C' };
static uint8_t MSG_TOUCH_DOWN[2] = { 'B', 'D' };


#define TFT_CS        10
#define TFT_RST        9 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC         8
#define TFT_MOSI 11  // Data out
#define TFT_SCLK 13  // Clock out

#define TOUCH1 16
#define TOUCH2 17
#define LED1 5
#define LED2 6

#define WARN_LED 13

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

// Pitch, Roll and Yaw values of gyro and timestep
static float pitch = 0;
static float roll = 0;
static float yaw = 0;

uint8_t current_song = 1;
uint8_t volume;
uint8_t gyro_reporting_frequency = 10; // how often gyro reports reading to main board (0=max)

static uint8_t report_roll = 0;
static uint8_t report_pitch = 0;
static uint8_t report_yaw = 0;
static uint8_t report_counter = 0;

#ifdef HAVE_GYRO
MPU6050 mpu;
#endif

void setup() 
{
  volume = 20;
  pitch = roll = yaw = 0;
  
  pinMode(TOUCH1, INPUT);
  pinMode(TOUCH2, INPUT);

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  
  pinMode(WARN_LED, OUTPUT);
  digitalWrite(WARN_LED, LOW);

  Serial.begin(9600);
  
#ifdef HAVE_GYRO  
  if (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))  
    Serial.println(F("gyro init error"));
  else {
    delay(1000);
    mpu.calibrateGyro();
    mpu.setThreshold(3);
  }
#endif
  
  while (Serial.available()) Serial.read();
  delay(1500);
  
  Serial.println(F("Ottis second. Press H for help"));
  init_serial(9600);

  tft.initR(INITR_BLACKTAB);
  
  tft.setRotation(3);
  tft.fillScreen(ST77XX_BLACK);
  testdrawtext(30, 33, "Ottis\n greets\n   you!", ST77XX_GREEN);
  heart(130, 15);
  
  //mp3_play(1);
  delay(10);
  mp3_set_volume(volume);
  
}

void testdrawtext(uint8_t x, uint8_t y, char *text, uint16_t color) 
{
  tft.setCursor(0, 0);
  tft.setTextColor(color);
  tft.setTextSize(3);
  tft.setCursor(x, y);
  tft.setTextWrap(true);
  tft.print(text);
}

void heart(uint8_t x, uint8_t y)
{
  tft.fillCircle(x, y, 4, ST77XX_RED);
  tft.fillCircle(x + 8, y, 4, ST77XX_RED);
  tft.fillCircle(x + 4, y + 4, 4, ST77XX_RED);
  tft.drawPixel(x + 4, y + 9, ST77XX_RED);
}

void refresh_gyro()
{
  //ideally this fn. should be called 100-times per second
#ifdef HAVE_GYRO
  static uint8_t reporting_counter = 0;
  static unsigned long last_time = 0;
  unsigned long this_time = millis();
  while (millis() == this_time);
  if (this_time < last_time) return;
  
  // Read normalized values
  Vector norm = mpu.readNormalizeGyro();

  // Calculate Pitch, Roll and Yaw
  float q = (this_time - last_time) / 1000.0;
  pitch = pitch + norm.YAxis * q;
  roll = roll + norm.XAxis * q;
  yaw = yaw + norm.ZAxis * q;
  last_time = this_time;

  if (report_roll) 
  {
    uint16_t roll16 = (uint16_t)((int16_t)(roll * 100));
    if (report_counter == gyro_reporting_frequency)
    {
      serial_write('r'); 
      serial_write((uint8_t) (roll16 >> 8));
      serial_write((uint8_t) (roll16 & 255));
    }
  }
  if (report_pitch)
  {
    uint16_t pitch16 = (uint16_t)((int16_t)(pitch * 100));
    if (report_counter == gyro_reporting_frequency)
    {
      serial_write('p'); 
      serial_write((uint8_t) (pitch16 >> 8));
      serial_write((uint8_t) (pitch16 & 255));
    }
  }
  if (report_yaw)
  {
    uint16_t yaw16 = (uint16_t)((int16_t)(yaw * 100));
    if (report_counter == gyro_reporting_frequency)
    {
      serial_write('y'); 
      serial_write((uint8_t) (yaw16 >> 8));
      serial_write((uint8_t) (yaw16 & 255));
    }
  }  
  if (report_counter == gyro_reporting_frequency) report_counter = 0;
  else report_counter++;
#endif
}

void react_to_gyro()
{
  refresh_gyro();
}

void debug_menu()
{
  Serial.print(F("1 play current song ("));
  Serial.print(current_song);
  Serial.println(F(")"));
  Serial.println(F("2 next song"));
  Serial.println(F("3 previous song"));
  Serial.println(F("+ increase volume"));
  Serial.println(F("- decrease volume")); 
  Serial.println(F("4 display test"));
  Serial.println(F("5 test LEDs"));
  Serial.println(F("6 test touch"));
  Serial.println(F("7 test gyro"));
  Serial.println(F("8 send to first"));
  Serial.println(F("h show this help"));
}

void display_test()
{
  tft.fillScreen(ST77XX_BLACK); 
  testdrawtext(46, 10, "TEST", ST77XX_ORANGE);   
  tft.drawLine(0, 0, 159, 0, ST77XX_YELLOW);
  tft.drawLine(0, 0, 0, 127, ST77XX_YELLOW);
  tft.drawLine(0, 127, 159, 127, ST77XX_YELLOW);
  tft.drawLine(159, 0, 159, 127, ST77XX_YELLOW);
  tft.drawLine(0, 0, 159, 127, ST77XX_YELLOW);
  tft.drawLine(159, 0, 0, 127, ST77XX_YELLOW);
  heart(76, 105);
}

void led_test()
{
    Serial.println(F("LEDs flashing"));
    for (int i = 0; i < 4; i++)
    {
     led_1_on();
     delay(500);
     led_1_off();
     delay(500);
     led_2_on();
     delay(500);
     led_2_off();
     delay(500);
    }
    Serial.println(F("done"));
}

void touch_test()
{
   for (int i = 0; i < 100; i++)
   {
      Serial.print(F("touch1: ")); Serial.println(digitalRead(TOUCH1));
      Serial.print(F("touch2: ")); Serial.println(digitalRead(TOUCH2));
      delay(200);
   }
}

void gyro_test() 
{
   while (!Serial.available())
   {
     refresh_gyro();
     Serial.print(F("r=")); Serial.print((int)(roll * 100));
     Serial.print(F(", p=")); Serial.print((int)(pitch * 100));
     Serial.print(F(", y=")); Serial.println((int)(yaw * 100));
     delay(7);
   }
   Serial.read();
}

void test_sending()
{
  for (int i = 0; i < 10; i++)
  {
   serial_write('1');
   serial_write('2');
   serial_write('3');
   serial_write('4');
   delay(500);
  }
}

// debugging menu
void control_over_serial()
{
  char c = Serial.read();
  switch (c) {
    case '1': mp3_play(current_song); break;
    case '2': current_song++; 
              Serial.print(F("song ")); 
              Serial.println(current_song); 
              break;
    case '3': if (current_song > 0) current_song--; 
              Serial.print(F("song ")); 
              Serial.println(current_song); 
              break;
    case '+': if (volume < 30) volume++;
              mp3_set_volume(volume);
              Serial.println(volume); 
              break;
    case '-': if (volume > 0) volume--;
              mp3_set_volume(volume);
              Serial.println(volume);               
              break;
    case '4': display_test(); break;
    case '5': led_test(); break;
    case '6': touch_test(); break;
    case '7': gyro_test(); break;
    case '8': test_sending(); break;
    case 'H':
    case 'h': debug_menu(); break;
  }
}

void display_command()
{
  tft.fillScreen(ST77XX_BLACK);  
  testdrawtext(33, 45, "Hello!", ST77XX_YELLOW);  
}

void led_1_on() 
{
  digitalWrite(LED1, HIGH);
}

void led_1_off() 
{
  digitalWrite(LED1, LOW);
}

void led_2_on() 
{
  digitalWrite(LED2, HIGH);
}

void led_2_off() 
{
  digitalWrite(LED2, LOW);
}

void check_connection()
{
  serial_write('#');  
}

void control_by_first()
{
  char c = serial_read();
  switch (c) {
    case '1': mp3_play(current_song); break;
    case '2': current_song++; break;
    case '3': if (current_song > 0) current_song--; break;
    case '4': display_command(); break;
    case 'L': led_1_on(); break;    
    case 'l': led_1_off(); break;    
    case 'E': led_2_on(); break;    
    case 'e': led_2_off(); break;    
    case '8': check_connection(); break;
    case 'R': report_roll = 1; break;
    case 'r': report_roll = 0; break;
    case 'P': report_pitch = 1; break;
    case 'p': report_pitch = 0; break;
    case 'Y': report_yaw = 1; break;
    case 'y': report_yaw = 0; break;
  }
}

void check_touch()
{
  static uint8_t touch_state[2] = {0, 0 };
  static unsigned long last_change[2] = {0, 0 };

  uint8_t touch[2];
  unsigned long tm = millis();
  
  touch[0] = digitalRead(TOUCH1);
  touch[1] = digitalRead(TOUCH2);

  for (int i = 0; i < 2; i++)
  {
    if ((touch[i] != touch_state[i]) && (abs(tm - last_change[i]) > 50))
    {
      if (touch[i]) serial_write(MSG_TOUCH_DOWN[i]);                                                               
      else serial_write(MSG_TOUCH_UP[i]);        
      touch_state[i] = touch[i];
      last_change[i] = tm;  
    }
  }
}

void loop() 
{
  if (serial_available()) control_by_first();
  if (Serial.available()) control_over_serial();

  check_touch();

#ifdef HAVE_GYRO
  react_to_gyro();
#endif
}


// volume 0-30
void mp3_set_volume(uint8_t volume)
{
 mp3_send_packet(0x06, volume);  
}

void mp3_play(uint8_t song_number)
{
 mp3_send_packet(0x03, song_number);  
}

void mp3_send_byte(uint8_t pin, uint8_t val)
{
  pinMode(MP3_OUTPUT_PIN, OUTPUT);
  float start_transmission = micros();
  float one_bit = 1000000 / 9600.0;
  float next_change = start_transmission + one_bit;
  digitalWrite(pin, LOW);
  while (micros() < next_change);
  
  for (int i = 2; i < 10; i++)
  {
    if (val & 1) digitalWrite(pin, HIGH);
    else digitalWrite(pin, LOW);
    next_change = start_transmission + one_bit * i;
    val >>= 1;
    while (micros() < next_change);
  }

  digitalWrite(pin, HIGH);
  next_change = micros() + 2 * one_bit;
  while (micros() < next_change);
  pinMode(MP3_OUTPUT_PIN, INPUT);
}

void mp3_send_packet(uint8_t cmd, uint16_t param)
{
  mp3_send_byte(MP3_OUTPUT_PIN, 0x7E);
  mp3_send_byte(MP3_OUTPUT_PIN, 0xFF);
  mp3_send_byte(MP3_OUTPUT_PIN, 0x06);
  mp3_send_byte(MP3_OUTPUT_PIN, cmd);
  mp3_send_byte(MP3_OUTPUT_PIN, 0x00);
  mp3_send_byte(MP3_OUTPUT_PIN, (uint8_t)(param >> 8));
  mp3_send_byte(MP3_OUTPUT_PIN, (uint8_t)(param & 0xFF));
  uint16_t chksm = 0xFF + 0x06 + cmd + (param >> 8) + (param & 0xFF);
  chksm = -chksm;
  mp3_send_byte(MP3_OUTPUT_PIN, (uint8_t)(chksm >> 8));
  mp3_send_byte(MP3_OUTPUT_PIN, (uint8_t)(chksm & 0xFF));
  mp3_send_byte(MP3_OUTPUT_PIN, 0xEF);
}

// serial connection with first arduino

static volatile uint8_t serial_state;
static uint8_t serial_buffer[SERIAL_BUFFER_LENGTH];
static volatile uint8_t serial_buf_wp, serial_buf_rp;

static volatile uint8_t receiving_byte;

static volatile uint32_t time_startbit_noticed;
static volatile uint8_t next_bit_order;
static volatile uint8_t waiting_stop_bit;
static uint16_t one_byte_duration;
static uint16_t one_bit_duration;
static uint16_t one_bit_write_duration;
static uint16_t half_of_one_bit_duration;

void init_serial(uint32_t baud_rate)
{
  pinMode(2, INPUT);
  pinMode(3, OUTPUT);
  digitalWrite(2, HIGH);
  digitalWrite(3, HIGH);

  serial_state = SERIAL_STATE_IDLE;

  one_byte_duration = 9500000 / baud_rate;
  one_bit_duration = 1000000 / baud_rate;
  one_bit_write_duration = one_bit_duration - 1;
  half_of_one_bit_duration = 500000 / baud_rate;

  PCMSK2 |= 4; //PCINT18;   // PIN 2
  //PCIFR &= ~4; //PCIF2;
  PCICR |= 4; // PCIE2;

  //Serial.print("one_bit_write_duration=");
  //Serial.println(one_bit_write_duration);
}

ISR(PCINT2_vect)
{
  uint32_t tm = micros();
  if (serial_state == SERIAL_STATE_IDLE)
  {
    time_startbit_noticed = tm;
    serial_state = SERIAL_STATE_RECEIVING;
    receiving_byte = 0xFF;
    next_bit_order = 0;
  }
  else if (tm - time_startbit_noticed > one_byte_duration)
  {
    serial_buffer[serial_buf_wp] = receiving_byte;
    serial_buf_wp++;
    if (serial_buf_wp == SERIAL_BUFFER_LENGTH) serial_buf_wp = 0;
    time_startbit_noticed = tm;
    receiving_byte = 0xFF;
    next_bit_order = 0;
  }
  else if (PIND & 4)
  {
    int8_t new_next_bit_order = (tm - time_startbit_noticed - half_of_one_bit_duration) / one_bit_duration;
    while (next_bit_order < new_next_bit_order)
    {
      receiving_byte &= ~(1 << next_bit_order);
      next_bit_order++;
    }
    if (next_bit_order == 8)
    {
      serial_buffer[serial_buf_wp] = receiving_byte;
      serial_buf_wp++;
      if (serial_buf_wp == SERIAL_BUFFER_LENGTH) serial_buf_wp = 0;
      serial_state = SERIAL_STATE_IDLE;
    }
  } else
    next_bit_order = (tm - time_startbit_noticed - half_of_one_bit_duration) / one_bit_duration;  

}

uint8_t serial_available()
{
  cli();
  if (serial_buf_rp != serial_buf_wp)
  { 
    sei();
    return 1;
  }
  if (serial_state == SERIAL_STATE_RECEIVING)
  {
    uint32_t tm = micros();
    if (tm - time_startbit_noticed > one_byte_duration)
    {
      serial_state = SERIAL_STATE_IDLE; 
      serial_buffer[serial_buf_wp] = receiving_byte;
      serial_buf_wp++;
      if (serial_buf_wp == SERIAL_BUFFER_LENGTH) serial_buf_wp = 0;
    sei();
      return 1;
    }
  }
  sei();
  return 0;
}

int16_t serial_read()
{
  cli();
  if (serial_buf_rp != serial_buf_wp)
  {
    uint8_t ch = serial_buffer[serial_buf_rp];
    serial_buf_rp++;
    if (serial_buf_rp == SERIAL_BUFFER_LENGTH) serial_buf_rp = 0;
    sei();
    //if (ch < 250) bt_active = 1; // HC-05 sends weird char on connect     
    return ch;
  }

  if (serial_state == SERIAL_STATE_RECEIVING)
  {
    uint32_t tm = micros();
    if (tm - time_startbit_noticed > one_byte_duration)
    {
      uint8_t ch = receiving_byte;
      serial_state = SERIAL_STATE_IDLE;
      sei();
      return ch;
    }
  }
  sei();
  return -1;
}

int16_t serial_peek()
{
  cli();
  if (serial_buf_rp != serial_buf_wp)
  {
    uint8_t ch = serial_buffer[serial_buf_rp];        
    sei();    
    return ch;
  }

  if (serial_state == SERIAL_STATE_RECEIVING)
  {
    uint32_t tm = micros();
    if (tm - time_startbit_noticed > one_byte_duration)
    {
      uint8_t ch = receiving_byte;
      sei();
      return ch;
    }
  }
  sei();
  return -1;
}

void serial_write(uint8_t ch)
{
  PORTD &= ~8;
  delayMicroseconds(one_bit_write_duration);
  for (uint8_t i = 0; i < 8; i++)
  {
    if (ch & 1) PORTD |= 8;
    else PORTD &= ~8;
    ch >>= 1;
    delayMicroseconds(one_bit_write_duration);
  }
  PORTD |= 8;

  delayMicroseconds(one_bit_write_duration);
  delayMicroseconds(one_bit_write_duration);
  delayMicroseconds(one_bit_write_duration);
  delayMicroseconds(one_bit_write_duration);
  delayMicroseconds(one_bit_write_duration);
}
