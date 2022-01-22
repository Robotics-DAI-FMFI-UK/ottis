#include <EEPROM.h>
#include <Servo.h>

// put next line after comment, if you have no HC-SR04 ultrasonic 
#define HAVE_ULTRASONIC
// pin connections:
// servos:
//    D3 LEFT FOOT
//    D5 LEFT KNEE
//    D6 LEFT HIP
//    D7 LEFT SHOULDER
//    D8 LEFT ARM
//    D9 CHEST
//    D10 RIGHT SHOULDER
//    D11 RIGHT ARM
//    D12 RIGHT HIP
//    D14/A0  RIGHT KNEE
//    D15/A1  RIGHT FOOT

#define PIN_LEFT_FOOT 3
#define PIN_LEFT_KNEE 5
#define PIN_LEFT_HIP 6
#define PIN_LEFT_SHOULDER 7
#define PIN_LEFT_ARM 8
#define PIN_CHEST 9
#define PIN_RIGHT_SHOULDER 10
#define PIN_RIGHT_ARM 11
#define PIN_RIGHT_HIP 12
#define PIN_RIGHT_KNEE 14
#define PIN_RIGHT_FOOT 15


// bluetooth:
//    BT TX:  D2
//    BT RX:  D4
// sirene:
//    +: D13
// ultrasonic HC-SR04
//    TRIG:   A2
//    ECHO:   A3
// battery voltage:
//    Vbat:   A6

// second arduino
//    Rx: A4
//    Tx: A5

#define TRIG 16    // pin A2
#define ECHO 17    // pin A3

#define SECOND_RX  19
#define SECOND_TX  18

#define SERIAL_STATE_IDLE      0
#define SERIAL_STATE_RECEIVING 1
#define SERIAL_BUFFER_LENGTH   20

// left/right front/back 1=upper, 2=lower
#define LEFT_FOOT 0
#define LEFT_KNEE 1
#define LEFT_HIP 2
#define LEFT_SHOULDER 3
#define LEFT_ARM 4
#define CHEST 5
#define RIGHT_SHOULDER 6
#define RIGHT_ARM 7
#define RIGHT_HIP 8
#define RIGHT_KNEE 9
#define RIGHT_FOOT 10

#define MAX_SEQ_LENGTH 28
#define MAX_PROG_SIZE (MAX_SEQ_LENGTH * 12 + 2)

#define WARN_LED 13


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

#define MSG_TOUCH1_UP 'A'
#define MSG_TOUCH1_DOWN 'B'
#define MSG_TOUCH2_UP 'C'
#define MSG_TOUCH2_DOWN 'D'

#define MSG_ROLL 'r'
#define MSG_YAW 'y'
#define MSG_PITCH 'p'

Servo legs[11];
uint8_t legv[11];
char pluskey[] = {'q', 'w', '3', '4', 'a', 'x', 'c', 'f', '<', 'j', 'm'};
char minuskey[] = {'1', '2', 'e', 'r', 'z', 's', 'd', 'v', '>', 'n', 'M'};
char key_step_plus = '+';
char key_step_minus = '-';
int8_t step_size;
int seq_length;
uint8_t seq[MAX_SEQ_LENGTH][11];
uint8_t delaj[MAX_SEQ_LENGTH];
uint8_t del = 0;
uint8_t ultrasonic = 0;
uint8_t quiet = 0;

int16_t pitch;
int16_t roll;
int16_t yaw;
uint8_t roll_subscription, pitch_subscription, yaw_subscription;

uint8_t current_song = 1;

uint8_t usb_active;
uint8_t bt_active;

uint8_t in_edit_mode;
uint8_t debug_mode;
uint8_t debugged_step;

uint8_t initial[] = {75, 106, 98, 91, 120, 136, 76, 75, 100, 87, 62};

#ifdef HAVE_GYRO
MPU6050 mpu;
#endif

int usual_delay = 70;
int inp = 0;
int8_t laying = 0;
int auto_cube = 0;
int auto_safe = 0;
int safe_delay = 500;

static volatile uint8_t serial_state[2];
static uint8_t serial_buffer[2][SERIAL_BUFFER_LENGTH];
static volatile uint8_t serial_buf_wp[2], serial_buf_rp[2];

static volatile uint8_t receiving_byte[2];

static volatile uint32_t time_startbit_noticed[2];
static volatile uint8_t next_bit_order[2];
static volatile uint8_t waiting_stop_bit[2];
static uint16_t one_byte_duration;
static uint16_t one_bit_duration;
static uint16_t one_bit_write_duration;
static uint16_t half_of_one_bit_duration;
static uint8_t ignore_batteries = 0;

void setup() 
{
  analogReference(INTERNAL); 
  analogRead(A6); 

  usb_active = 0;
  bt_active = 0;
  in_edit_mode = 0;
  debug_mode = 0;
  seq_length = 0;
  roll_subscription = pitch_subscription = yaw_subscription = 0;

  legs[LEFT_FOOT].attach(PIN_LEFT_FOOT);
  legs[LEFT_KNEE].attach(PIN_LEFT_KNEE);
  legs[LEFT_HIP].attach(PIN_LEFT_HIP);
  legs[LEFT_SHOULDER].attach(PIN_LEFT_SHOULDER);
  legs[LEFT_ARM].attach(PIN_LEFT_ARM);
  legs[CHEST].attach(PIN_CHEST);
  legs[RIGHT_SHOULDER].attach(PIN_RIGHT_SHOULDER);
  legs[RIGHT_ARM].attach(PIN_RIGHT_ARM);
  legs[RIGHT_HIP].attach(PIN_RIGHT_HIP);
  legs[RIGHT_KNEE].attach(PIN_RIGHT_KNEE);
  legs[RIGHT_FOOT].attach(PIN_RIGHT_FOOT);
  reset_position();

  pinMode(WARN_LED, OUTPUT);
  digitalWrite(WARN_LED, LOW);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  Serial.begin(9600);
  init_tone2();

  while (Serial.available()) Serial.read();
  Serial.println(F("Hi! Press U for USB-powered run."));
  delay(2500);
  if (Serial.available())
  {
    if (Serial.read() == 'U') 
    {
      ignore_batteries = 1;
      Serial.println(F("USB-powered"));
    }
    usb_active = 1;
  }
  else Serial.println(F("Bat.powered"));
  Serial.println(F("Ottis. Press H for help"));
  init_serial(9600);
  
  step_size = 1;
  sound_greeting();
    
  if (load_autostart()) play_sequence(1);
}

void sound_greeting()
{
  tone2(440, 50);
  delay(50);
  tone2(880, 100);
  delay(100);
  tone2(1760, 50);
  delay(50);
}

void position_90()
{
    for (int i = 0; i < 11; i++)
      legv[i] = 90;
       
    for (int i = 0; i < 11; i++)
      legs[i].write(legv[i]);
}

void reset_position()
{
    for (int i = 0; i < 11; i++)
      legv[i] = initial[i];
       
    for (int i = 0; i < 11; i++)
      legs[i].write(legv[i]);
}


// walking - todo
void forward()
{ 
}

void backward()
{
}

void right()
{
}

void left()
{
}

void LED_cheeks_on()
{
  serial_write(1, 'L');
  serial_write(1, 'E');
}

void LED_cheeks_off()
{
  serial_write(1, 'l');
  serial_write(1, 'e');
}

void play_song()
{
  serial_write(1, '1');
}

void next_song()
{
  serial_write(1, '2');
}

void previous_song()
{
  serial_write(1, '3');
}

void test_display()
{
  serial_write(1, '4');
}

void check_connection() // gyro reporting should be off
{
  while (serial_available(1)) serial_read(1);
  serial_write(1, '8');
  delay(10);
  char c = serial_read(1);
  if (c == '#') serial_println_flash(PSTR("conn ok"));
  else serial_println_flash(PSTR("conn fail"));
}

void roll_onoff()
{
  if (roll_subscription) serial_write(1, 'r');
  else serial_write(1, 'R');
  roll_subscription = 1 - roll_subscription;
  serial_print_flash(PSTR("roll: ")); serial_write(0, '0' + roll_subscription);
  serial_println();
}

void yaw_onoff()
{
  if (yaw_subscription) serial_write(1, 'y');
  else serial_write(1, 'Y');
  yaw_subscription = 1 - yaw_subscription;  
  serial_print_flash(PSTR("yaw: ")); serial_write(0, '0' + yaw_subscription);
  serial_println();
}

void pitch_onoff()
{
  if (pitch_subscription) serial_write(1, 'p');
  else serial_write(1, 'P');
  pitch_subscription = 1 - pitch_subscription;
  serial_print_flash(PSTR("pitch: ")); serial_write(0, '0' + pitch_subscription);
  serial_println();
}

void print_gyro()
{
  serial_print_flash(PSTR("r=")); serial_print_num(roll);
  serial_print_flash(PSTR(", y=")); serial_print_num(yaw);
  serial_print_flash(PSTR(", p=")); serial_println_num(pitch);
}

char read_latest_char() 
{
  char c = 0;
  while (serial_available(0) > 0)
    c = serial_read(0);
  return c;
}

void both_modes(char c)
{
  if (c == '/')
    play_sequence(0);
  else if (c == '\\')
    play_sequence(1);
  else if (c == 't'){
    if (load_from_EEPROM(1))
      play_sequence(1);
  }
  else if (c == 'g'){
    if (load_from_EEPROM(2))
      play_sequence(1);
  }
  else if (c == 'b'){
    if (load_from_EEPROM(3))
      play_sequence(1);
  }
  else if (c == '*')
  {
    reset_position();
    laying = 0;
  }
  else if (c == '9')
    position_90();
  else if ((c == 27) || (c == ':')) stop_melody();
  else if (c == 'u')
  {
    ultrasonic = 1 - ultrasonic;
    serial_print_flash(PSTR("ultra:"));
    serial_println_num(ultrasonic);
  }
  else if ((c == 'h') || (c == 'H') || (c == '?')) print_usage();
  else if (c == 'B') serial_println_num(measure_bat());
  else try_melodies(c);
}

void edit_mode(char c)
{
  for (int i = 0; i < 11; i++)
    if (c == pluskey[i])
    {
      if (legv[i] <= 175 - step_size) legv[i] += step_size; 
      legs[i].write(legv[i]); 
      dump_state();
    }
    else if (c == minuskey[i])
    {
      if (legv[i] >= 5 + step_size) legv[i] -= step_size; 
      legs[i].write(legv[i]);
      dump_state(); 
    }
    
  if (c == key_step_plus)
  {
      step_size++;
      serial_print_flash(PSTR("step: "));
      serial_println_num(step_size);
  }
  else if (c == key_step_minus)
  {
      if (step_size > 1) step_size--;
      serial_print_flash(PSTR("step: "));
      serial_println_num(step_size);
  }
  else if (debug_mode)
  {
    if ((c == 9) || (c == 'T'))
    {
      debug_mode = 0;
      serial_println_flash(PSTR("debug: 0"));
    }
    else if ((c == ' ') || (c == 27) || (c == ':') || (c == 'N') || (c == 'D') || (c == 'I'))
    {
      if (c == 'D')
      {
        if (seq_length > 0)
        {
          if (debugged_step == seq_length - 1)
          {
            debugged_step--;
          }        
          else 
          {
             for (int i = debugged_step; i < seq_length - 1; i++)
             {
               for (int j = 0; j < 11; j++)
                 seq[i][j] = seq[i + 1][j];
               delaj[i] = delaj[i + 1];
             }
          }
          seq_length--;
          serial_println_flash(PSTR("deleted"));
        }
        else 
        {
          tone2(2000, 50);
          serial_println_flash(PSTR("only 1 remains"));
        }
      } 
      else if (c == 'I')
      {
         if (seq_length < MAX_SEQ_LENGTH)
         {
           for (int i = seq_length; i > debugged_step; i--)
           {
              for (int j = 0; j < 11; j++)
                seq[i][j] = seq[i - 1][j];
              delaj[i] = delaj[i - 1];
           }
           seq_length++;   
           serial_println_flash(PSTR("inserted"));
         }
         else 
         {
           tone2(2000, 50);
           serial_println_flash(PSTR("full"));
         }
      }
      else if ((c == ' ') || (c == 'N')) 
      {
        debugged_step++;
        if (debugged_step == seq_length) debugged_step = 0;
        serial_print_flash(PSTR("step "));
        serial_println_num(debugged_step);
      }
      else serial_println_flash(PSTR("restored"));
      play_step(debugged_step);
      for (int i = 0; i < 11; i++)
        legv[i] = seq[debugged_step][i];
    }
    else if ((c == 13) || (c == 'W'))
    {
      for (int i = 0; i < 11; i++)
        seq[debugged_step][i] = legv[i];
      serial_println_flash(PSTR("replaced"));
    }
    else if ((c == 'h') || (c == 'H') || (c == '?')) print_usage();
  }
  else if (c == 'C') 
  {
    in_edit_mode = 0;
    serial_println_flash(PSTR("ctrl mode"));
  }
  else if ((c == 9) || (c == 'T'))
  {
    if (seq_length > 0)
    {
      debug_mode = 1;
      debugged_step = 0;
      dump_sequence(1);
      serial_println_flash(PSTR("debug: 1"));
      serial_println_flash(PSTR("step 0"));
      play_step(0);
    }
    else serial_println_flash(PSTR("nothing to debug"));
  }
  else if ((c == 13) || (c == 'W'))
    store_new_point();
  else if ((c == ' ') || (c == 'N'))
    dump_sequence(0);
  else if (c == 'E')
    store_to_EEPROM();
  else if (c == 'O')
    load_from_EEPROM(0);
  else if (c == 'A')
    toggle_autostart();
  else if (c == 'R')
  {
    serial_print_flash(PSTR("Discard? [y/n]: "));
    do { c = anyserial_readchar(); } while ((c != 'y') && (c != 'n'));
    if (c == 'y')
    {
       seq_length = 0;
       serial_println_flash(PSTR("Sequence discarded."));
    }
  }
  else if (c == 'L')
    load_sequence();
  else if (c == 'U')
    undo_step();
  else both_modes(c);
}

void control_mode(char c)
{
  if (c == '1') forward();
  else if (c == '2') backward();
  else if (c == '3') right();
  else if (c == '4') left();
  else if (c == '5') play_song();
  else if (c == '6') next_song();
  else if (c == '7') previous_song();
  else if (c == '0') test_display();
  else if (c == '8') check_connection();
  else if (c == '(') roll_onoff();
  else if (c == ')') yaw_onoff();
  else if (c == '$') pitch_onoff();
  else if (c == '&') print_gyro();
  else if (c == 'E') 
  {
    in_edit_mode = 1;
    serial_println_flash(PSTR("edit mode"));
  }
  else if (c == '{')
  {
    LED_cheeks_on();
  }
  else if (c == '}')
  {
    LED_cheeks_off();
  }
  else both_modes(c);
  delay(300);
}

void control_over_bt()
{
  beep();
  char c = read_latest_char();
  if (in_edit_mode)
    edit_mode(c);  
  else control_mode(c);
}

void process_control_char(char c)
{
  uint8_t b1, b2;
  int16_t v;
  switch (c) {
    case MSG_TOUCH1_UP: serial_println_flash(PSTR("touch 1 up")); break;
    case MSG_TOUCH1_DOWN: serial_println_flash(PSTR("touch 1 down")); break;
    case MSG_TOUCH2_UP: serial_println_flash(PSTR("touch 2 up")); break;    
    case MSG_TOUCH2_DOWN: serial_println_flash(PSTR("touch 2 down")); break;
    case MSG_ROLL: 
    case MSG_YAW:
    case MSG_PITCH:
      while (!serial_available(1)); 
      b1 = serial_read(1);
      while (!serial_available(1));
      b2 = serial_read(1);
      v = (int16_t)((((uint16_t)b1) << 8) | b2);
      switch (c) {
        case MSG_ROLL: roll = v; break;
        case MSG_YAW: yaw = v; break;
        case MSG_PITCH: pitch = v; break;
      }
  }
  if ((c >= '1') && (c <= '4'))
    serial_print_char(c); 
}

void control_over_serial()
{
  char c = Serial.read();
  usb_active = 1;
  if (in_edit_mode)
    edit_mode(c);  
  else control_mode(c);
}
void control_over_other_serial()
{
  char c = serial_read(1);
  process_control_char(c);
}

void try_melodies(char c)
{
  if (c == ',') play_melody(1);
  else if (c == '.') play_melody(2);
  else if (c == ';') play_melody(3);
  else if (c == '[') play_melody(4);
  else if (c == ']') play_melody(5);
  else if (c == '\'') play_melody(6);
}

void loop() 
{
#ifdef HAVE_ULTRASONIC
  if ((measure_US() < 15) && ultrasonic)
  {
    // insert your code to react to obstacle in front of robot here
    Serial.println("US");
    backward();
  }
#endif

  if (serial_available(1)) control_over_other_serial();
  if (serial_available(0)) control_over_bt();
  if (Serial.available()) control_over_serial();
  
  check_battery();
}

void print_usage()
{
  if (in_edit_mode)
  {
    if (debug_mode)
    {
      serial_println_flash(PSTR("Debug mode:"));
      serial_println_flash(PSTR(" TAB/T   - leave debug mode"));
      serial_println_flash(PSTR(" SPACE/N - move to next step"));
      serial_println_flash(PSTR(" ENTER/W - replace with current"));
      serial_println_flash(PSTR(" ESC/:   - restore current step"));
      serial_println_flash(PSTR(" D       - delete current step"));
      serial_println_flash(PSTR(" ESC/:   - insert step before current"));
    }
    else serial_println_flash(PSTR("Edit mode. Hit C for control mode."));
    serial_println_flash(PSTR("Servo control:"));
    for (int i = 0; i < 11; i++)
    {
      serial_print_char(minuskey[i]);
      serial_print_flash(PSTR(" ... "));
      serial_println_char(pluskey[i]);
    }
    serial_println_flash(PSTR("Step size control:"));
    serial_print_char(key_step_minus);
    serial_print_flash(PSTR(" ... "));
    serial_println_char(key_step_plus);
    delay(150);
    if (!debug_mode)
    {
      serial_println_flash(PSTR("Store next point: ENTER/W"));
      serial_println_flash(PSTR("Print the sequence: SPACE/N"));
      serial_println_flash(PSTR("Load (paste in) the sequence: L"));
      serial_println_flash(PSTR("Save to EEPROM: E"));
      serial_println_flash(PSTR("Load from EEPROM: O"));
      serial_println_flash(PSTR("Toggle autostart: A"));
      serial_println_flash(PSTR("Erase sequence: R"));
      serial_println_flash(PSTR("Undo to last saved position: U"));
      serial_println_flash(PSTR("Enter debug mode: TAB/T"));
      serial_println_flash(PSTR("(to insert a break, repeat the same position again with delay)"));   
    }
    else
    {
      serial_println_flash(PSTR("Print help: h"));
      return;
    }
  } else
  {   
    serial_println_flash(PSTR("Control mode. Hit E for edit mode."));    
    serial_println_flash(PSTR(" 1: forward"));
    serial_println_flash(PSTR(" 2: backward"));
    serial_println_flash(PSTR(" 3: right"));
    serial_println_flash(PSTR(" 4: left"));
    serial_println_flash(PSTR(" 5: play song"));
    serial_println_flash(PSTR(" 6: next song"));
    serial_println_flash(PSTR(" 7: prev song"));
    serial_println_flash(PSTR(" 0: test display"));
    serial_println_flash(PSTR(" 8: check connection")); 
    serial_println_flash(PSTR(" (: roll on/off")); 
    serial_println_flash(PSTR(" ): yaw on/off")); 
    serial_println_flash(PSTR(" $: pitch on/off")); 
    serial_println_flash(PSTR(" &: print gyro")); 
    serial_println_flash(PSTR(" {: LED cheeks on"));  
    serial_println_flash(PSTR(" }: LED cheeks off"));  
  } 
  delay(150); 
  serial_println_flash(PSTR("Play the sequence: /"));
  serial_println_flash(PSTR("Repetitive play: \\"));
  serial_println_flash(PSTR("Play eeprom seq: t g b"));
  serial_println_flash(PSTR("Initial position: *"));
  serial_println_flash(PSTR("Position 90: 9"));
  serial_println_flash(PSTR("Ultrasonic ON/OFF: u"));
  serial_println_flash(PSTR("Battery level (*100 V): B"));
  serial_println_flash(PSTR("Play melody: . , ; ' [ ]"));
  serial_println_flash(PSTR("Stop melody: ESC/:"));
  delay(150); 
  serial_println_flash(PSTR("Print help: h"));
}

void store_new_point()
{
  char c = 0;
  if (seq_length == MAX_SEQ_LENGTH) return;
  while ((c != 13) && (c != 'W'))
  {
    serial_print_flash(PSTR(" "));
    serial_print_char((char)13);
    serial_print_flash(PSTR("(+/-/ENTER;W) delay = "));
    serial_print_num(del);
    while ((!Serial.available()) && !serial_available(0));
    if (Serial.available()) c = Serial.read();
    else c = serial_read(0);
    if (c == '+') if (del < 30) del++;
    if (c == '-') if (del > 0) del--;
  }
  serial_println();
  for (int i = 0; i < 11; i++)
    seq[seq_length][i] = legv[i];
  delaj[seq_length] = del;
  dump_row(seq_length);
  seq_length++;
}

void dump_row(int i)
{
    for (int j = 0; j < 11; j++)
    {
      serial_print_num(seq[i][j]);
      serial_print_flash(PSTR(" "));
    }
    serial_println_num(delaj[i]);
}

void dump_sequence(uint8_t step_numbers)
{
  serial_println_flash(PSTR("---"));
  for (int i = 0; i < seq_length; i++)
  {
    if (step_numbers) 
    { 
       serial_print_num(i);
       serial_print_flash(PSTR(". "));
    }
    dump_row(i);
  }
  serial_println_flash(PSTR("---"));
}

void play_step(uint8_t i)
{
   uint8_t index_new = i;
   uint8_t index_old = i - 1;
   if (i == 0) index_old = seq_length - 1;
      
   if (delaj[index_new] == 0)
   for (int j = 0; j < 11; j++)
      legs[j].write(seq[index_new][j]);
   else for (int k = 0; k < 100; k++)
   {
      for (int j = 0; j < 11; j++)        
         legs[j].write(seq[index_old][j] + ((int)(seq[index_new][j] - seq[index_old][j]) * k) / 100);
      delay(delaj[index_new]);
   } 
}

void play_sequence(uint8_t repete)
{
  if (repete) if (!quiet) play_melody(6);
  do 
  {
    for (int i = 0; i < seq_length; i++)
    {
      if (Serial.available()) { Serial.read(); repete = 0; break; }
      if (serial_available(0)) { serial_read(0); repete = 0; break; }
      dump_row(i);
      play_step(i);
      check_battery();
    }
    if (Serial.available()) { Serial.read(); break; }
    if (serial_available(0)) { serial_read(0); break; }
  } while (repete);
  for (int i = 0; i < 11; i++)
    legv[i] = seq[seq_length - 1][i];
  stop_melody();
}

void load_sequence()
{
  serial_print_flash(PSTR("Paste the sequence (or type SPACE/N to cancel): "));
  char c;
  uint8_t ok;
  
  do {
    c = serial_peek(0);
  } while (c == -1);
  
  if ((c == ' ')  || (c == 'N'))
  {
    Serial.read();
    serial_println_flash(PSTR("cancelled."));
    return;
  }
  seq_length = 0;
  serial_println();
  serial_println_flash(PSTR("loading... (terminate with empty line)"));

  do {
    while ((!Serial.available()) && !serial_available(0));
    c = serial_peek(0);
    if ((c == 13) || (c == 10)) break;
    ok = 1;
    for (int i = 0; ok & (i < 11); i++)
      seq[seq_length][i] = read_number(&ok);
    if (!ok) break;   
    delaj[seq_length] = read_number(&ok);
    if (!ok) break;
    serial_println();
    seq_length++;
  } while (1);
  if (!ok) 
  {
    serial_println();
    serial_println_flash(PSTR("loading failed."));
  }
  else 
  {
    Serial.read();
    serial_println_flash(PSTR("Done."));
  }
}

int read_number(uint8_t *ok)
{
    int x = 0;
    *ok = 0;
    while (1) {
    if (Serial.available() || serial_available(0))
    {
      int c;
      if (Serial.available()) c = Serial.read();
      else c = serial_read(0);
      serial_print_char(c);
      if (c == 8) x /= 10;
      if ((c >= '0') && (c <= '9'))
      {
        *ok = 1;
        x = x * 10 + (c - '0');
      } else return x;
    }
  }
}

void dump_state()
{
    for (int j = 0; j < 11; j++)
    {
      serial_print_num(legv[j]);
      serial_print_flash(PSTR(" "));
    }
    serial_println();
}

void undo_step()
{
  uint8_t changed = 0;
  if (seq_length > 0)
  {
    for (int i = 0; i < 11; i++)
      if (seq[seq_length - 1][i] != legv[i]) changed = 1;
  }
  else
  {
    for (int i = 0; i < 11; i++)
    {
      legv[i] = initial[i];
      legs[i].write(legv[i]);
    } 
    return;
  }
  
  if (!changed)
  {
    if (seq_length == 0)
    {
      serial_println_flash(PSTR("Nothing to undo."));
      return; 
    }
    serial_print_flash(PSTR("Nothing changed, erase last saved step? [y/n]: "));
    char c = anyserial_readchar();
    if (c != 'y') return;
    seq_length--; 
    serial_println_flash(PSTR("erased."));
  }
  else
  {
    serial_print_flash(PSTR("Undo? [y/n]: "));
    char c = anyserial_readchar();
    if (c == 'n') return;
  }
  if (seq_length > 0)
    for (int i = 0; i < 11; i++)
    {
      legv[i] = seq[seq_length - 1][i];
      legs[i].write(legv[i]);
    }
  else reset_position();
}

int8_t ask_for_slot_num()
{
  serial_print_flash(PSTR("Seq.slot 1-3: "));
  char c = anyserial_readchar();
  if ((c < '1') || (c > '3')) 
  {
    serial_println_flash(PSTR("range is 1-3"));
    return -1;
  }
  return c - '1';
}

char anyserial_readchar()
{
  char c;
  while ((!Serial.available()) && !serial_available(0));
  if (Serial.available()) c = Serial.read();
  else c = serial_read(0);
  serial_println_char(c);
  return c;
}
  
void store_to_EEPROM()
{
  if (seq_length == 0)
  {
    serial_println_flash(PSTR("nothing to store"));
    return;
  }
  int8_t prognum = ask_for_slot_num();
  if (prognum < 0) return;
  
  serial_print_flash(PSTR("Write sequence to EEPROM? [y/n]: "));
  char c = anyserial_readchar();
  if (c != 'y') return;
  
  EEPROM.write(0 + prognum * MAX_PROG_SIZE, '@'); 
  EEPROM.write(1 + prognum * MAX_PROG_SIZE, (uint8_t)seq_length);
  for (int i = 0; i < seq_length; i++)
  {
    for (int j = 0; j < 11; j++)
      EEPROM.write(prognum * MAX_PROG_SIZE + 2 + i*12 + j, seq[i][j]);
    EEPROM.write(prognum * MAX_PROG_SIZE + 2 + i*12 + 11, delaj[i]);
  }

  serial_println_flash(PSTR("Written."));
}

uint8_t load_autostart()
{
  if (EEPROM.read(1022) == '~')
  {
    uint8_t autostart = EEPROM.read(1023);
    if (autostart) return load_from_EEPROM(autostart);
    return autostart;
  }
  else return 0;  
}

void toggle_autostart()
{
  int8_t autostart = ask_for_slot_num();
  if (autostart < 0)
  {
      EEPROM.write(1023, 0);
      serial_println_flash(PSTR("Autostart is OFF"));
  }
  else
  {
    serial_print_flash(PSTR("Autostart is ON: "));
    serial_println_num(autostart + 1);
    EEPROM.write(1022, '~');
    EEPROM.write(1023, autostart + 1);
  }
}

uint8_t load_from_EEPROM(int8_t prognum)
{
  if (prognum == 0)
  {
    prognum = ask_for_slot_num();
    if (prognum < 0) return 0;
    serial_print_flash(PSTR("Read sequence from EEPROM [y/n]: "));
    char c = anyserial_readchar();
    if (c != 'y') return 0;
  }
  else prognum--;
  if (EEPROM.read(prognum * MAX_PROG_SIZE + 0) != '@')
  {
    serial_println_flash(PSTR("nothing in EEPROM"));
    return 0;
  }
  
  seq_length = EEPROM.read(prognum * MAX_PROG_SIZE + 1);
  for (int i = 0; i < seq_length; i++)
  {
    for (int j = 0; j < 11; j++)
      seq[i][j] = EEPROM.read(prognum * MAX_PROG_SIZE + 2 + i*12 + j);
    delaj[i] = EEPROM.read(prognum * MAX_PROG_SIZE + 2 + i*12 + 11);
  }
  serial_print_num(seq_length);
  serial_println_flash(PSTR(" positions."));
  return 1;
}

int measure_bat()
{
  delay(5);
  float volt = analogRead(A6); 
  // volt * 1.1 * 242 / 22 / 1023  (22 KOhm out of 220+22=242 KOhm)
  return (int)(0.5 + 100.0 * volt * 0.01181640625);   
}

void check_battery()
{
  static long last_measurement = 0;
  static long measurement_count = 0;

  if (ignore_batteries) return;
  long tm = millis();
  if (tm - last_measurement > 500)
  {
    last_measurement = tm;
    if (measure_bat() < 620)  
    {
      measurement_count++;
      if (measurement_count < 5) return;
      delay(50);
      usb_active = 1;
      bt_active = 1;
      serial_println_flash(PSTR("!!!!!!!!!!!!!!!! Replace batteries !!!!!!!!!!!!!!!!!!!!")); 
      for (int i = 0; i < 11; i++)
        legs[i].detach();
      for (int i = 2; i < 20; i++)
        pinMode(i, INPUT);
      pinMode(WARN_LED, OUTPUT);
      while(1) SOS();
    }
    else measurement_count = 0;
  }
}

void SOS()
{
  for (uint8_t i = 0; i < 3; i++)
  {
    digitalWrite(WARN_LED, HIGH);
    tone2(1680, 100);
    delay(100);
    digitalWrite(WARN_LED, LOW);
    delay(50);
  }
  delay(300);
  for (uint8_t i = 0; i < 3; i++)
  {
    tone2(1680, 300);
    digitalWrite(WARN_LED, HIGH);
    delay(300);
    digitalWrite(WARN_LED, LOW);
    delay(50);
  }
  delay(300);
  for (uint8_t i = 0; i < 3; i++)
  {
    digitalWrite(WARN_LED, HIGH);
    tone2(1680, 100);
    delay(100);
    digitalWrite(WARN_LED, LOW);
    delay(50);
  }
  delay(500);
}

void init_serial(uint32_t baud_rate)
{
  pinMode(2, INPUT);
  pinMode(4, OUTPUT);
  pinMode(19, INPUT);     // A5 is input (PCINT13)
  pinMode(18, OUTPUT);    // A4 is output
  digitalWrite(2, HIGH);
  digitalWrite(19, HIGH);

  serial_state[0] = serial_state[1] = SERIAL_STATE_IDLE;

  one_byte_duration = 9500000 / baud_rate;
  one_bit_duration = 1000000 / baud_rate;
  one_bit_write_duration = one_bit_duration - 1;
  half_of_one_bit_duration = 500000 / baud_rate;

  PCMSK1 |= 32; //PCINT13;  // PIN A5
  PCMSK2 |= 4; //PCINT18;   // PIN 2
  PCIFR &= ~6; //PCIF2;
  PCICR |= 6; // PCIE2;
}

void rx_pin_change(uint8_t w, uint8_t state)
{
  uint32_t tm = micros();
  if (serial_state[w] == SERIAL_STATE_IDLE)
  {
    time_startbit_noticed[w] = tm;
    serial_state[w] = SERIAL_STATE_RECEIVING;
    receiving_byte[w] = 0xFF;
    next_bit_order[w] = 0;
  }
  else if (tm - time_startbit_noticed[w] > one_byte_duration)
  {
    serial_buffer[w][serial_buf_wp[w]] = receiving_byte[w];
    serial_buf_wp[w]++;
    if (serial_buf_wp[w] == SERIAL_BUFFER_LENGTH) serial_buf_wp[w] = 0;
    time_startbit_noticed[w] = tm;
    receiving_byte[w] = 0xFF;
    next_bit_order[w] = 0;
  }
  else if (state)
  {
    int8_t new_next_bit_order = (tm - time_startbit_noticed[w] - half_of_one_bit_duration) / one_bit_duration;
    while (next_bit_order[w] < new_next_bit_order)
    {
      receiving_byte[w] &= ~(1 << next_bit_order[w]);
      next_bit_order[w]++;
    }
    if (next_bit_order[w] == 8)
    {
      serial_buffer[w][serial_buf_wp[w]] = receiving_byte[w];
      serial_buf_wp[w]++;
      if (serial_buf_wp[w] == SERIAL_BUFFER_LENGTH) serial_buf_wp[w] = 0;
      serial_state[w] = SERIAL_STATE_IDLE;
    }
  } else
    next_bit_order[w] = (tm - time_startbit_noticed[w] - half_of_one_bit_duration) / one_bit_duration;  
}

ISR(PCINT1_vect)
{
  rx_pin_change(1, PINC & 32);
}

ISR(PCINT2_vect)
{
  rx_pin_change(0, PIND & 4);
}

uint8_t serial_available(uint8_t w)
{
  cli();
  if (serial_buf_rp[w] != serial_buf_wp[w])
  { 
    sei();
    return 1;
  }
  if (serial_state[w] == SERIAL_STATE_RECEIVING)
  {
    uint32_t tm = micros();
    if (tm - time_startbit_noticed[w] > one_byte_duration)
    {
      serial_state[w] = SERIAL_STATE_IDLE; 
      serial_buffer[w][serial_buf_wp[w]] = receiving_byte[w];
      serial_buf_wp[w]++;
      if (serial_buf_wp[w] == SERIAL_BUFFER_LENGTH) serial_buf_wp[w] = 0;
    sei();
      return 1;
    }
  }
  sei();
  return 0;
}

int16_t serial_read(uint8_t w)
{
  cli();
  if (serial_buf_rp[w] != serial_buf_wp[w])
  {
    uint8_t ch = serial_buffer[w][serial_buf_rp[w]];
    serial_buf_rp[w]++;
    if (serial_buf_rp[w] == SERIAL_BUFFER_LENGTH) serial_buf_rp[w] = 0;
    sei();
    if ((ch < 250) && (w == 0)) bt_active = 1; // HC-05 sends weird char on connect     
    return ch;
  }

  if (serial_state[w] == SERIAL_STATE_RECEIVING)
  {
    uint32_t tm = micros();
    if (tm - time_startbit_noticed[w] > one_byte_duration)
    {
      uint8_t ch = receiving_byte[w];
      serial_state[w] = SERIAL_STATE_IDLE;
      sei();
      if ((ch < 250) && (w == 0)) bt_active = 1;
      return ch;
    }
  }
  sei();
  return -1;
}

int16_t serial_peek(uint8_t w)
{
  cli();
  if (serial_buf_rp[w] != serial_buf_wp[w])
  {
    uint8_t ch = serial_buffer[w][serial_buf_rp[w]];        
    sei();    
    return ch;
  }

  if (serial_state[w] == SERIAL_STATE_RECEIVING)
  {
    uint32_t tm = micros();
    if (tm - time_startbit_noticed[w] > one_byte_duration)
    {
      uint8_t ch = receiving_byte[w];
      sei();
      return ch;
    }
  }
  sei();
  if (usb_active) return Serial.peek();
  return -1;
}

void serial_write(uint8_t w, uint8_t ch)
{
  if (w == 0)
  {
    if (usb_active)
      Serial.print((char)ch);
    if (bt_active)
    {
      PORTD &= ~16;
      delayMicroseconds(one_bit_write_duration);
      for (uint8_t i = 0; i < 8; i++)
      {
        if (ch & 1) PORTD |= 16;
        else PORTD &= ~16;
        ch >>= 1;
        delayMicroseconds(one_bit_write_duration);
      }
      PORTD |= 16;
    }
  }
  else
  {
    PORTC &= ~16;
    delayMicroseconds(one_bit_write_duration);
    for (uint8_t i = 0; i < 8; i++)
    {
      if (ch & 1) PORTC |= 16;
      else PORTC &= ~16;
      ch >>= 1;
      delayMicroseconds(one_bit_write_duration);
    }
    PORTC |= 16;
  }
  delayMicroseconds(one_bit_write_duration);
  delayMicroseconds(one_bit_write_duration);
  delayMicroseconds(one_bit_write_duration);
  delayMicroseconds(one_bit_write_duration);
  delayMicroseconds(one_bit_write_duration);
}

uint16_t serial_readln(uint8_t *ln, uint16_t max_length)
{
  uint16_t len;
  int16_t ch;
  do {
    ch = serial_read(0);
    if (ch == 13) continue;
  } while (ch == -1);

  do {
    if ((ch != 13) && (ch != 10) && (ch != -1))
    {
      *(ln++) = ch;
      max_length--;
      len++;
    }
    ch = serial_read(0);
  } while ((ch != 13) && max_length);
  *ln = 0;
  return len;
}

void serial_print_num(int32_t number)
{
  if (number < 0)
  {
    serial_write(0, '-');
    number = -number;
  }
  int32_t rad = 1;
  while (number / rad) rad *= 10;
  if (number > 0) rad /= 10;
  while (rad)
  {
    serial_write(0, (char)('0' + (number / rad)));
    number -= (number / rad) * rad;
    rad /= 10;
  }
}

void serial_print_char(char ch)
{
  serial_write(0, ch);
}

void serial_print(const char *str)
{
  while (*str) serial_write(0, *(str++));
}

void serial_println(const char *str)
{
  serial_print(str);
  serial_write(0, 13);
  serial_write(0, 10);
}

void serial_print_flash(const char *str)
{
  int ln = strlen_P(str);
  for (int i = 0; i < ln; i++)
    serial_write(0, pgm_read_byte(str + i));
}

void serial_println_flash(const char *str)
{
  serial_print_flash(str);
  serial_write(0, 13);
  serial_write(0, 10);
}

void serial_println_num(int32_t number)
{
  serial_print_num(number);
  serial_println();
}

void serial_println_char(char ch)
{
  serial_write(0, ch);
  serial_println();
}

void serial_println()
{
  serial_write(0, 13);
  serial_write(0, 10);
}

int measure_US()
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  unsigned long counter = micros() + 10000;
  while ((digitalRead(ECHO) == 0) & (micros() < counter)) { }
  if (digitalRead(ECHO) == 0) return 255;
  unsigned long tm1 = micros();
  counter = tm1 + 10000; 
  while ((digitalRead(ECHO) == 1) && (micros() < counter)) { }
  if (digitalRead(ECHO) == 1) return 255;
  unsigned long tm2 = micros();
  return (tm2 - tm1) / 58;
}

//-------------------------------- the following code is for playing melodies using timer2 in the background
#define SIRENE_PORT  PORTB
#define SIRENE_DDR   DDRB
#define SIRENE_PIN   5

#define FIS3 2960
#define G3 3136

float octave_4[] = { 2093.00, 2217.46, 2349.32, 2489.02, 2637.02, 2793.83, 2959.96, 3135.96, 3322.44, 3520.00, 3729.31, 3951.07 };

//popcorn
uint16_t melody_len[] = {0, 386, 26, 281, 217, 36, 468};
const uint8_t melody1[] PROGMEM = { 252, 50, 149,  49,
                                     28, 31, 35, 40, 49, 99, 38, 49, 99, 40, 49, 99, 35, 49, 99, 31, 49, 99, 35, 49, 99, 28, 49, 99, 49,
                                     28, 31, 35, 40, 49, 99, 38, 49, 99, 40, 49, 99, 35, 49, 99, 31, 49, 99, 35, 49, 99, 28, 49, 99, 149,
                                     40, 49, 99, 42, 49, 99, 43, 49, 99, 42, 49, 99, 43, 49, 99, 40, 49, 99, 42, 49, 99, 40, 49, 99, 42, 49, 99, 38, 49, 99, 40, 49, 99, 38, 49, 99, 40, 49, 99, 36, 49, 99, 40, 49, 99,
                                     28, 31, 35, 40, 49, 99, 38, 49, 99, 40, 49, 99, 35, 49, 99, 31, 49, 99, 35, 49, 99, 28, 49, 99, 49,
                                     28, 31, 35, 40, 49, 99, 38, 49, 99, 40, 49, 99, 35, 49, 99, 31, 49, 99, 35, 49, 99, 28, 49, 99, 149,
                                     40, 49, 99, 42, 49, 99, 43, 49, 99, 42, 49, 99, 43, 49, 99, 40, 49, 99, 42, 49, 99, 40, 49, 99, 42, 49, 99, 38, 49, 99, 40, 49, 99, 38, 49, 99, 40, 49, 99, 42, 49, 99, 43, 49, 99,
                                     49, 35, 38, 43, 47, 49, 99, 45, 49, 99, 47, 49, 99, 43, 49, 99, 38, 49, 99, 43, 49, 99, 35, 49, 99,
                                     49, 35, 38, 43, 47, 49, 99, 45, 49, 99, 47, 49, 99, 43, 49, 99, 38, 49, 99, 43, 49, 99, 35, 49, 99, 149 ,
                                     47, 49, 99, 254, 49, 99, 255, 49, 99, 254, 49, 99, 255, 49, 99, 47, 49, 99, 254, 49, 99, 47, 49, 99, 254, 49, 99, 45, 49, 99, 47, 49, 99, 45, 49, 99, 47, 49, 99, 43, 49, 99, 47, 49, 99,
                                     49, 35, 38, 43, 47, 49, 99, 45, 49, 99, 47, 49, 99, 43, 49, 99, 38, 49, 99, 43, 49, 99, 35, 49, 99,
                                     49, 35, 38, 43, 47, 49, 99, 45, 49, 99, 47, 49, 99, 43, 49, 99, 38, 49, 99, 43, 49, 99, 35, 49, 99, 149 ,
                                     47, 49, 99, 254, 49, 99, 255, 49, 99, 254, 49, 99, 255, 49, 99, 47, 49, 99, 254, 49, 99, 47, 49, 99, 254, 49, 99, 45, 49, 99, 47, 49, 99, 45, 49, 99, 47, 49, 99, 254, 49, 99, 255, 49, 99
                                   };

//kohutik jarabi
const uint8_t melody2[] PROGMEM = { 252, 150, 119, 121, 173, 174, 124, 124, 124, 123, 171, 173, 123, 123, 123, 121, 169, 171, 121, 121, 121, 123, 171, 169, 119, 119 };

//CAN-CAN
const uint8_t melody3[] PROGMEM = { 252, 100,
                                     251, 1, 184, 1, 32, 126, 149, 251, 1, 184, 1, 32, 126, 149, 251, 1, 184, 1, 32, 126, 251, 1, 184, 1, 32, 126, 251, 1, 184, 1, 32, 126, 251, 1, 184, 1, 32, 126,
                                     64, 71, 71, 73, 71, 69, 69, 73, 74, 78, 81, 78, 78, 76, 251, 1, 184, 1, 32, 126, 78, 68, 68, 78, 76, 69, 69, 73, 73, 71, 73, 71, 85, 83, 85, 83,
                                     64, 71, 71, 73, 71, 69, 69, 73, 74, 78, 81, 78, 78, 76, 251, 1, 184, 1, 32, 126, 78, 68, 68, 78, 76, 69, 69, 73, 73, 71, 73, 71, 71, 69, 119,
                                     135, 131, 128, 126, 75, 76, 78, 80, 81, 76, 80, 76, 81, 76, 80, 76, 81, 76, 80, 76, 81, 76, 80, 76,
                                     251, 2, 11, 3, 16, 19, 251, 1, 4, 3, 16, 19,
                                     251, 1, 4, 3, 16, 19, 251, 1, 4, 3, 16, 19,
                                     251, 1, 4, 3, 16, 19, 251, 1, 4, 3, 16, 19,
                                     251, 1, 4, 3, 16, 19, 251, 1, 4, 3, 16, 19,
                                     174, 88, 91, 90, 88, 143, 143, 93, 95, 90, 91, 138, 138, 88, 91, 90, 88, 86, 86, 85, 83, 81, 79, 78, 76,
                                     174, 88, 91, 90, 88, 143, 143, 93, 95, 90, 91, 138, 138, 88, 91, 90, 88, 86, 93, 89, 90, 136,
                                     64, 71, 71, 73, 71, 69, 69, 73, 74, 78, 81, 78, 78, 76, 251, 1, 184, 1, 32, 126,
                                     78, 76, 126, 78, 76, 126, 78, 76, 126, 78, 76, 126, 78, 76, 126, 78, 76, 126,
                                     78, 76, 78, 76, 78, 76, 78, 76,
                                     131, 119, 119, 119, 169
                                   };

//swan lake
const uint8_t melody4[] PROGMEM = {
  252, 220, 66, 69, 73, 69, 66, 69, 73, 69, 66, 69, 73, 69, 66, 69, 73, 69,
  185, 78, 80, 81, 83, 251, 5, 39, 3, 8, 81, 251, 5, 39, 3, 8, 81, 251, 5, 39, 3, 8, 78, 81, 78, 73, 81, 178,
  99, 83, 81, 80,
  185, 78, 80, 81, 83, 251, 5, 39, 3, 8, 81, 251, 5, 39, 3, 8, 81, 251, 5, 39, 3, 8, 78, 81, 78, 73, 81, 178,
  149, 128, 130, 131, 133, 85, 86, 251, 6, 32, 3, 8, 86, 135, 86, 88, 251, 6, 224, 3, 8, 88, 136, 88, 90, 251, 7, 184, 3, 8, 90, 85, 81, 80, 78,
  130, 131, 133, 85, 86, 251, 6, 32, 3, 8, 86, 135, 86, 88, 251, 6, 224, 3, 8, 88, 136, 88, 90, 251, 7, 73, 3, 8, 86, 133, 86, 91, 251, 7, 184, 3, 8, 87, 251, 7, 184, 3, 8, 85,
  185, 78, 80, 81, 83, 251, 5, 39, 3, 8, 81, 251, 5, 39, 3, 8, 81, 251, 5, 39, 3, 8, 78, 81, 78, 73, 81, 178,
  99, 83, 81, 80,
  185, 78, 80, 81, 83, 251, 5, 39, 3, 8, 81, 251, 5, 39, 3, 8, 81, 251, 5, 39, 3, 8, 78, 81, 78, 73, 81, 178
};

//let it be https://www.musicnotes.com/sheetmusic/mtd.asp?ppn=MN0101556
const uint8_t melody5[] PROGMEM = { 252, 200, 26, 26, 76, 26, 253, 78, 73, 76, 76, 31, 253, 83, 35, 85, 253, 85, 
                                     83, 83, 81, 131, 35, 253, 85, 86, 35, 35, 85, 83, 99, 35, 33, 83, 81, 181 
};

//Turkish march by Mozart
const uint8_t melody6[] PROGMEM = { 80, 78, 77, 78, 131, 149, 83, 81, 80, 81, 135, 149, 86, 85, 84, 85, 92, 90, 
89, 90, 92, 90, 89, 90, 193, 140, 143, 142, 140, 138, 140, 142, 140, 138, 
140, 142, 140, 138, 137, 185, 80, 78, 77, 78, 131, 149, 83, 81, 80, 81, 
135, 149, 86, 85, 84, 85, 92, 90, 89, 90, 92, 90, 89, 90, 193, 140, 143, 
142, 140, 138, 140, 142, 140, 138, 140, 142, 140, 138, 137, 185, 135, 136, 
138, 138, 90, 88, 86, 85, 183, 135, 136, 138, 138, 90, 88, 86, 85, 183, 
131, 133, 135, 135, 86, 85, 83, 81, 180, 131, 133, 135, 135, 86, 85, 83, 
81, 180, 80, 78, 77, 78, 131, 149, 83, 81, 80, 81, 135, 149, 86, 85, 84, 
85, 92, 90, 89, 90, 92, 90, 89, 90, 193, 140, 142, 143, 142, 140, 139, 140, 
135, 136, 133, 181, 130, 78, 80, 178, 128, 130, 182, 128, 130, 132, 130, 
128, 127, 125, 127, 128, 130, 127, 123, 128, 130, 182, 128, 130, 132, 130, 
128, 127, 125, 130, 127, 123, 178, 94, 95, 94, 92, 140, 139, 87, 90, 89, 
87, 136, 139, 82, 84, 85, 82, 137, 139, 90, 89, 90, 92, 144, 93, 94, 95, 
94, 92, 140, 139, 87, 90, 89, 87, 135, 139, 82, 84, 85, 82, 134, 137, 81, 
82, 84, 81, 182, 94, 95, 94, 92, 140, 139, 87, 90, 89, 87, 136, 139, 82, 
84, 85, 82, 137, 139, 90, 89, 90, 92, 144, 93, 94, 95, 94, 92, 140, 139, 
87, 90, 89, 87, 135, 139, 82, 84, 85, 82, 134, 137, 81, 82, 84, 81, 182, 
128, 130, 182, 128, 130, 132, 130, 128, 127, 125, 127, 128, 130, 127, 123, 
128, 130, 182, 128, 130, 132, 130, 128, 127, 125, 130, 127, 123, 178, 80, 
78, 77, 78, 131, 149, 83, 81, 80, 81, 135, 149, 86, 85, 84, 85, 92, 90, 89, 
90, 92, 90, 89, 90, 193, 140, 143, 142, 140, 138, 140, 142, 140, 138, 140, 
142, 140, 138, 137, 185, 80, 78, 77, 78, 131, 149, 83, 81, 80, 81, 135, 
149, 86, 85, 84, 85, 92, 90, 89, 90, 92, 90, 89, 90, 193, 140, 143, 142, 
140, 138, 140, 142, 140, 138, 140, 142, 140, 138, 137, 185, 135, 136, 138, 
138, 90, 88, 86, 85, 183, 135, 136, 138, 138, 90, 88, 86, 85, 183, 131, 
133, 135, 135, 86, 85, 83, 81, 180, 131, 133, 135, 135, 86, 85, 83, 81, 
180, 80, 78, 77, 78, 131, 149, 83, 81, 80, 81, 135, 149, 86, 85, 84, 85, 
92, 90, 89, 90, 92, 90, 89, 90, 193, 140, 142, 143, 142, 140, 139, 140, 
135, 136, 133, 181, 130, 78, 80, 178, 131, 135, 190, 199 };

volatile int16_t music_speed = 800 / 16;
volatile const uint8_t *current_note ;
volatile uint16_t notes_remaining;
uint8_t dotted_note = 0;

void play_melody(uint8_t num)
{
  if (num == 0) {
    stop_melody();
    return;
  }

  if (num == 1) current_note = melody1;
  else if (num == 2) current_note = melody2;
  else if (num == 3) current_note = melody3;
  else if (num == 4) current_note = melody4;
  else if (num == 5) current_note = melody5;
  else if (num == 6) current_note = melody6;
  notes_remaining = melody_len[num];

  next_note();
}

void next_note()
{
  uint16_t freq = 0, dur = 0;
  if (!notes_remaining) return;
  otto_translate_tone_flash(&freq, &dur);
  tone2(freq, dur);
}

void otto_translate_tone_flash(uint16_t *freq, uint16_t *del)
{
  do {
    uint8_t n = pgm_read_byte(current_note);
    if (n > 249)
    {
      if (n == 251)
      {
        current_note++;
        uint8_t f1 = pgm_read_byte(current_note);
        current_note++;
        uint8_t f2 = pgm_read_byte(current_note);
        current_note++;
        uint8_t d1 = pgm_read_byte(current_note);
        current_note++;
        uint8_t d2 = pgm_read_byte(current_note);
        *freq = (f1 << 8) + f2;
        *del = (music_speed * 16 * (long)d1) / d2;
        notes_remaining -= 4;
      }
      else if (n == 252)
      {
        current_note++;
        music_speed = pgm_read_byte(current_note);
        current_note++;
        notes_remaining -= 2;
        continue;
      }
      else if (n == 253) 
      {
        dotted_note = 1;
        current_note++;
        notes_remaining--;        
        continue;
      }
      else if (n == 254)
      {
        *freq = FIS3;
        *del = music_speed;
      }
      else if (n == 255)
      {
        *freq = G3;
        *del = music_speed;
      }
    }
    else
    {
      uint8_t len = n / 50;
      *del = music_speed;
      while (len--) *del *= 2;
      n = n % 50;
      if (n != 49)
      {
        uint8_t octave = (n + 5) / 12;
        n = (n + 5) % 12;
        float ffreq = octave_4[n];
        octave = 4 - octave;
        while (octave > 0)
        {
          ffreq /= 2.0;
          octave--;
        }
        *freq = (uint16_t)ffreq;
      }
      else *freq = 0;
    }
    notes_remaining--;
    current_note++;
    break;
  } while (1);
  if (dotted_note) 
  {
    *del += (*del) / 2;
    dotted_note = 0;
  }
}

static volatile uint8_t tone2_state;
static volatile uint8_t tone2_pause;
static volatile uint32_t tone2_len;

void init_tone2()
{
  notes_remaining = 0;
  tone2_pause = 0;
  dotted_note = 0;
  TCCR2A = 2;
  TCCR2B = 0;
  TIMSK2 = 2;
  SIRENE_DDR |= (1 << SIRENE_PIN);
}

ISR(TIMER2_COMPA_vect)
{
  if (!tone2_pause)
  {
    if (tone2_state)
    {
      SIRENE_PORT |= (1 << SIRENE_PIN);
      tone2_state = 0;
    }
    else
    {
      SIRENE_PORT &= ~(1 << SIRENE_PIN);
      tone2_state = 1;
    }
  }
  if ((--tone2_len) == 0)
  {
    TCCR2B = 0;
    tone2_pause = 0;
    next_note();
  }
}

void tone2(uint16_t freq, uint16_t duration)
{
  uint32_t period = ((uint32_t)1000000) / (uint32_t)freq;

  if (freq >= 977)  // prescaler 32
  {
    tone2_state = 0;
    tone2_len = ((uint32_t)duration * (uint32_t)1000) * 2 / period;
    if (tone2_len == 0) tone2_len++;
    TCNT2 = 0;
    OCR2A = (uint8_t) (250000 / (uint32_t)freq);
    TCCR2B = 3;
  }
  else if (freq >= 488) // prescaler 64
  {
    tone2_state = 0;
    tone2_len = ((uint32_t)duration * (uint32_t)1000) * 2 / period;
    if (tone2_len == 0) tone2_len++;
    TCNT2 = 0;
    OCR2A = (uint8_t) (125000 / (uint32_t)freq);
    TCCR2B = 4;
  }
  else if (freq >= 244) // prescaler 128
  {
    tone2_state = 0;
    tone2_len = ((uint32_t)duration * (uint32_t)1000) * 2 / period;
    if (tone2_len == 0) tone2_len++;
    TCNT2 = 0;
    OCR2A = (uint8_t) (62500 / (uint32_t)freq);
    TCCR2B = 5;
  }
  else if (freq >= 122) //prescaler 256
  {
    tone2_state = 0;
    tone2_len = ((uint32_t)duration * (uint32_t)1000) * 2 / period;
    if (tone2_len == 0) tone2_len++;
    TCNT2 = 0;
    OCR2A = (uint8_t) (31250 / (uint32_t)freq);
    TCCR2B = 6;
  }
  else if (freq >= 30) //prescaler 1024
  {
    tone2_state = 0;
    tone2_len = ((uint32_t)duration * (uint32_t)1000) * 2 / period;
    if (tone2_len == 0) tone2_len++;
 
   TCNT2 = 0;
    OCR2A = (uint8_t) (7813 / (uint32_t)freq);
    TCCR2B = 7;
  }
  else if (freq == 0)
  {
    tone2_pause = 1;
    tone2_state = 0;

    period = 1000000 / 500;
    tone2_len = ((uint32_t)duration * (uint32_t)1000) * 2 / period;
    TCNT2 = 0;
    OCR2A = (uint8_t) (125000 / (uint32_t)500);
    TCCR2B = 4;
  }
  else
  {
    TCCR2B = 0;
  }
}

void beep()
{
  if (notes_remaining == 0) tone2(698, 30);
}

void stop_melody()
{
  notes_remaining = 0;
}
