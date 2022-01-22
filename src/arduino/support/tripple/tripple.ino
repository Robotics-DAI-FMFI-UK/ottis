int state;
int who;
int cnt;

void setup() {
   init_serial(9600);  // our custom serial on pins 2, 4
   Serial.begin(9600);
   state = 1;
   cnt = 0;
   pinMode(13, OUTPUT);   
   analogRead(7);
   who = analogRead(7);
}

void loop() {
  if (who == 0) loop0();
  else loop1();
  delay(50);
  cnt++;

  if (cnt == 20)
  { 
    digitalWrite(13, state);
    state = 1 - state;
    cnt = 0;
  }
}

void loop0()
{
  if (serial_available(0))
  {
    char c = serial_read(0);
    serial_write(1, c);
    Serial.write(c);
  }
  if (serial_available(1))
  {
    char c = serial_read(1);
    serial_write(0, c);
    Serial.write(c);
  }
  if (Serial.available())
  {
    char c = Serial.read();    
    serial_write(1, c);
    serial_write(0, c);
  }
}

void loop1()
{
  if (serial_available(0))
  {
    char c = serial_read(0);
    Serial.write(c);
  }
  if (Serial.available())
  {
    char c = Serial.read();    
    serial_write(0, c);
  }
  delay(50);

  if (cnt == 20)
  { 
    digitalWrite(13, state);
    state = 1 - state;
  }
}


#define SERIAL_STATE_IDLE      0
#define SERIAL_STATE_RECEIVING 1
#define SERIAL_BUFFER_LENGTH   20

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
    //if (ch < 250) bt_active = 1; // HC-05 sends weird char on connect     
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
