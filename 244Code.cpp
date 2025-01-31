#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>


#define MPU6050_ADDR 0x68

#define Trigger_pin PB5
#define Echo_pin PD3
#define SERVO_PIN PB1

// UART printing
uint16_t ubrr = 103; // 9600 baud rate with 16 MHz clock

// front US
volatile uint32_t timer_overflow_count = 0;
volatile uint32_t start_time = 0;
volatile uint32_t end_time = 0;

volatile int duration = 0;
volatile int distance = 0;
volatile bool wall = false;

volatile int left_dist;
volatile int right_dist;
volatile int front_dist;

// top US
volatile int distance_top = 0;
volatile int duration_top = 0;

// mpu
#define GYRO_SCALE 131.0
volatile int pre_yaw = 0;
volatile int yaw = 0;

// mpu readings
int16_t gyro_proc_z, prev_gyro_z = 0;
int16_t ax, ay, az;
int16_t gx, gy, gyro_z;
int stuck = 0;

// turning angles for servo
long OCR_min = 32;
long OCR_ctr = 89;
long OCR_max = 154;

// IR
const float AVREF = 3.1;
float voltage_avg = 0;
float voltage = 0;
float bar = 0;
const int trials =
    50; // measures 50 times, so we divide by 50 later. we can also do 100
int ADCvalue = 0;
int adc_avg = 0;

bool stop = false;

//------------------------------------IMU
//FUNCTIONS------------------------------------// initialize I2C (TWI)
void I2C_init(void) {
  TWBR = 72;   // SCL frequency of 100kHz with 8MHz system clock
  TWSR = 0x00; // Set prescaler to 1
}

// send start condition
void I2C_start(void) {
  TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT);
  while (!(TWCR & (1 << TWINT)))
    ;
}

// send stop condition
void I2C_stop(void) { TWCR = (1 << TWSTO) | (1 << TWEN) | (1 << TWINT); }

// write data to I2C
void I2C_write(uint8_t data) {
  TWDR = data;
  TWCR = (1 << TWEN) | (1 << TWINT);
  while (!(TWCR & (1 << TWINT)))
    ;
}

// read data with acknowledgment
uint8_t I2C_read_ack(void) {
  TWCR = (1 << TWEN) | (1 << TWINT) | (1 << TWEA);
  while (!(TWCR & (1 << TWINT)))
    ;
  return TWDR;
}

// read data without acknowledgment
uint8_t I2C_read_nack(void) {
  TWCR = (1 << TWEN) | (1 << TWINT);
  while (!(TWCR & (1 << TWINT)))
    ;
  return TWDR;
}

// initialize MPU6050 sensor
void MPU6050_init(void) {
  I2C_start();
  I2C_write(MPU6050_ADDR << 1);
  I2C_write(0x6B); // Power management register
  I2C_write(0x00); // Set to zero to wake up the MPU6050
  I2C_stop();
}

// read gyroscope data from MPU6050
void MPU6050_read(int16_t *gx, int16_t *gy, int16_t *gyro_z) {
  I2C_start();                  // start operating
  I2C_write(MPU6050_ADDR << 1); // access address
  I2C_write(0x43); // start reading at register 0x43 (gyroscope data), according
                   // to datasheet
  I2C_stop();                            // stop
  _delay_us(10);                         // restart delay
  I2C_start();                           // restartI2C
  I2C_write((MPU6050_ADDR << 1) | 0x01); // read mode

  *gx = (I2C_read_ack() << 8) | I2C_read_ack();      // read x
  *gy = (I2C_read_ack() << 8) | I2C_read_ack();      // read y
  *gyro_z = (I2C_read_ack() << 8) | I2C_read_nack(); // read z

  I2C_stop(); // stop operation
}

// convert IMU readings
void convert_raw_data() {
  // each function call resets the temp value.
  float temp;

  // gz reading / Scale is how IMU converts to angular velocity
  // adding previous readings from gyro imitates somewhat of a numerical
  // integration. provides readings combined over time. 0.25 is a compensation
  // factor for the fact that the readings are not accurate at all. Without
  // using filters, this is the simplest approach trial and error gives readings
  // that are decently accurate for usage. during testing, range can go from 0.1
  // to 0.25
  //  + 0.5 is depending on the reading. Through testing, values range from 0 to
  //  -70 (rarely -70, often -60) 60/131 is =~ 0.45 so we counter this drift
  //  with +0.45 to counter the -a/g values reading.
  temp = (float)gyro_z / GYRO_SCALE * 0.12 + prev_gyro_z + 0.45;

  // using the temp values we get,
  // using this method, our angle values will range from 0 to 180 only. This
  // will be useful to determine CCW or CW turning. if value printed shows 179 -
  // > 90, we are turning CCW, while 0 -> 90 is turning CW
  gyro_proc_z = int abs((int16_t)(temp + 180) % 180);
}

//------------------------------------SERVO
//FUNCTIONS------------------------------------// servo initialization
void timer1_init() {
  TCCR1A = 0;
  TCCR1B = (1 << CS11);  // Prescaler 8
  TIMSK1 = (1 << TOIE1); // Enable Timer1 overflow interrupt
  sei();
}

//------------------------------------ IR FUNCTIONS
//------------------------------------// IR read
int analogRead_A0() {
  // enable adc, means it can start reading
  ADCSRA |= (1 << ADEN);

  // start the conversion
  ADCSRA |= (1 << ADSC);

  // wait for it to be done. ADSC will become 0 when it's done
  while (ADCSRA & (1 << ADSC))
    ;

  // read the values of ADC and get the final values measured by the sensor.
  int adc_value = ADC;

  // return the value, it should give a value that is around 0 to
  // 1023;1024;1025.
  return adc_value;
}
//------------------------------------ULTRASONIC
//Functions------------------------------------// uses 8-bit readings, which is
// worse than timer 1 but still decent.
void timer2_init() {
  TCCR2A = 0;
  TCCR2B = 0;

  // prescaler of 32
  TCCR2B |= (1 << CS21);
  TCCR2B |= (1 << CS20);

  TIMSK2 = (1 << TOIE2);
  sei();
}

uint32_t get_timer_value() {
  uint32_t overflows;
  uint8_t counts; // Note: 8-bit for Timer2

  cli();          // Disable interrupts
  counts = TCNT2; // extra number of bit counted, tick number is under 256.
  overflows = timer_overflow_count; // number of times it has overflows 256
                                    // ticks, when TCNT2 > 256.
  sei(); // Enable interrupts

  // Timer2 is 8-bit, so multiply overflows by 256 (2^8 bits)
  // the number of overflows, meaning how many times it goes past 256
  // the final value is the duration reading.
  return (overflows * 256) + counts;
}

// calls when timer2 overflows.
ISR(TIMER2_OVF_vect) { timer_overflow_count++; }

// initialize front ultrasonic.
void HCSR04_setup() {
  DDRD &= ~(1 << Echo_pin);   // set Echo pin as input
  DDRB |= (1 << Trigger_pin); // set Trigger pin as output
  // timer1_init();             //initialize timer once
  timer2_init();
}
// ultrasonic reads the duration
void HCSR04_read() {
  // reset variables for duration measurement
  timer_overflow_count = 0; // overflow reset
  TCNT2 = 0;                // timer reset "so it's as if we start at t = 0"

  // send trigger pulse
  PORTB &= ~(1 << Trigger_pin); // set low, make sure it's off
  _delay_us(2);                 // give it 2us to prepare
  PORTB |= (1 << Trigger_pin);  // set high, turn on
  _delay_us(10);                // give it 10us to read
  PORTB &= ~(1 << Trigger_pin); // set low, turn off

  // wait for echo to start, the pulse has been sent
  while (!(PIND & (1 << Echo_pin))) {
    if (timer_overflow_count > 20)
      return; // adjusted timeout for Timer2
  }

  uint32_t start_time = get_timer_value(); // time that the pulse was first sent

  // wait for echo to end, the puslse reached
  while (PIND & (1 << Echo_pin)) {
    if (timer_overflow_count > 20)
      return; // adjusted timeout for Timer2
  }

  uint32_t end_time = get_timer_value(); // time that the pulse was received

  // calculate duration in microseconds
  // timer runs at 16MHz/8 = 2MHz, each tick is 0.5us
  duration = ((end_time - start_time) / 2);
}

// ultrasonic on top for the bar
int top_hcsr04() {
  PORTB &= ~(1 << PB3); // set low, make sure it's off
  _delay_us(2);         // give 2us to prep
  PORTB |= (1 << PB3);  // set high, turn on.
  _delay_us(10);        // give 10us to read
  PORTB &= ~(1 << PB3); // set low, turn off.

  // wait until pulse sent out
  while (!(PIND & (1 << PD2)))
    ;

  // start counting duration while PD2 is HIGH
  // this is as accurate as possible for now without any other timers
  // this is going to be a method we can use for now to calculate timing between
  // detections for bar add duration while pulse is coming back (wait for
  // reception)
  while (PIND & (1 << PD2)) {
    duration_top++;
  }
  // 2MHZ so each duration (tick) is 0.5us, which we will just divide by 2 to
  // compensate. 2 ticks per us basically.
  return duration_top / 2;
}

void top_distance() {
  distance_top = ((duration_top * 0.034) / 2); // calcutation in cm
  // distance = time / 74 / 2            //calcutation in inches
}

void HCSR04_distance() {
  // speed of sound is ~343m/s = 34300cm/s
  // distance = (time in microseconds × 34300) / (2 × 1000000)
  // because of our prescaler, we will multiply by 4. It also helps get better
  // readings by testing.
  distance = (duration * 4 * 34300) / 20000 / 100;
}

//------------------------------------FAN
//FUNCTIONS------------------------------------//
void fan_init() {
  // Thrust
  DDRD |= (1 << DDD4);    // enable PD4, P17 on board
  PORTD |= (1 << PORTD4); // set it high
  // Lift
  DDRD |= (1 << DDD6);    // enable port PD6, P4 on board
  PORTD |= (1 << PORTD6); // set to high
  // BACKUP thrust port.
  DDRD |= (1 << DDD7);    // enable PD7, P18 on board
  PORTD |= (1 << PORTD7); // set to high
}

// Thrust fan will use timer0
void timer0_init() {
  // for controling the thrust fan
  TCCR0A |= (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);
  TCCR0B |= (1 << CS01);
}

// wall detection algorithm
void control_thrust_lift(int distance) {
  // wall detected if reading is under 15 cm
  if (distance <= 15) {
    PORTD &= ~(1 << PORTD6); // turn off each fan
    PORTD &= ~(1 << PORTD4);
    PORTD &= ~(1 << PORTD7);
    wall = true; // wall detected is true
    yaw = 0; // reset the yaw (so it does not cuase issues after turning) yaw is
             // reset after each turn.
  }
  // wall not detected if under 15cm.
  // if over 42, keep going,
  // if it is in between, it will most likely re-do the algorithm and run into a
  // wall. if there is nothing in front, will always read over 42 cm.
  else if (distance > 42) {
    OCR0A = 250;
  }
}

void make_decision(int left, int right, int front) {
  // algorithm to determine the longest distance
  int max = left;
  if (left < right)
    max = right;
  else
    max = left;
  if (max < front)
    max = front;

  // in case the front is a wall, we need to turn
  if (front < 20) {
    if (left == max) {
      OCR1A = 32;             // servo angle
      OCR0A = 230;            // thrust fan speeed
      PORTD |= (1 << PORTD6); // restart thrust fan
      PORTD |= (1 << PORTD7); // lift fan on
      PORTD |= (1 << PORTD4); // lift fan on (Backup)

      // turn until it is approx. 75 degrees, due to thrust not stopping
      // instantly, there is drift to compensate, which nearly turns 90 at the
      // end of 75 yaw
      while (yaw < 75) {
        MPU6050_read(&gx, &gy, &gyro_z);
        UART_print("a/g:\t");
        UART_print_int(gyro_z);
        UART_print("\n");
        convert_raw_data();
        if (gyro_proc_z > 90) {
          yaw += gyro_proc_z - 180;
        } else {
          yaw += gyro_proc_z;
        }
        UART_print("Yaw: ");
        UART_print_int(yaw);
        UART_print("\n");
        UART_print("\n");

        // failsafe, if after 5 seconds, the craft can't turn to 75 yaw, it will
        // stop and go forward/check again
        if (stuck >= 5000)
          break;

        // read every 100ms and update
        stuck += 100;
        _delay_ms(100);
        // basically only 5 seconds to turn 90 degrees.
      }
      // when loop is done, turn off everything
      PORTD &= ~(1 << PORTD6);
      PORTD &= ~(1 << PORTD4);
      PORTD &= ~(1 << PORTD7);

      stuck = 0;      // reset the failsafe timer
      _delay_ms(500); // delay 0.5s and return to loop
    }
    // same as left turn.
    if (right == max) {
      OCR1A = 154;
      OCR0A = 230;
      PORTD |= (1 << PORTD6);
      PORTD |= (1 << PORTD7);
      PORTD |= (1 << PORTD4);
      while (yaw > -75) {
        MPU6050_read(&gx, &gy, &gyro_z);
        UART_print("a/g:\t");
        UART_print_int(gyro_z);
        UART_print("\n");

        convert_raw_data();
        if (gyro_proc_z > 90) {
          yaw += gyro_proc_z - 180;
        } else {
          yaw += gyro_proc_z;
        }
        UART_print("Yaw: ");
        UART_print_int(yaw);
        UART_print("\n");
        UART_print("\n");

        if (stuck >= 5000)
          break;
        stuck += 100;
        _delay_ms(100);
      }
      PORTD &= ~(1 << PORTD6);
      PORTD &= ~(1 << PORTD4);
      PORTD &= ~(1 << PORTD7);
      stuck = 0;
      _delay_ms(500);
    }
    // backup in case of mis-read and glitches. if the front is max, but under
    // 20, still go in front instead of random turns. it will loop back to
    // re-read and make a new decision after detecting the wall again. it will
    // attempt to retry for a better turn.
    if (front == max) {
      OCR1A = 89;             // servo straight
      OCR0A = 230;            // speed up thrust
      PORTD |= (1 << PORTD6); // restart each fan
      PORTD |= (1 << PORTD7);
      PORTD |= (1 << PORTD4);
    }
  }

  // if front > 20, there is no wall. continue in front.
  else {
    OCR1A = 89;
    OCR0A = 255;
    PORTD |= (1 << PORTD6); // restart lift fan
    PORTD |= (1 << PORTD7);
    PORTD |= (1 << PORTD4);
  }
}

// This is to control the drift that the hovercraft board has.
// The lift fan turns the hovercraft itself, but can be compensated if we know
// what degree.
void drifting_control() {
  OCR0A = 255; // max speed for decent adjustments while moving forward
  OCR1A = OCR_map(
      yaw, -90, 90, OCR_min,
      OCR_max); // correction by turning servo depending on the yaw IMU reading
  _delay_ms(100); // delay so we don't move servo too fast.
}

// similar mapping function to arduino, using linear approximation.
long OCR_map(int16_t angle, long min_angle, long max_angle, long OCR_min,
             long OCR_max) {
  return (angle - min_angle) * (OCR_max - OCR_min) / (max_angle - min_angle) +
         OCR_min;
}
//------------------------------------UART
//printing------------------------------------// initialize UART for serial
// communication
void UART_init(void) {
  UBRR0H = (unsigned char)(ubrr >> 8);
  UBRR0L = (unsigned char)ubrr;
  UCSR0B = (1 << TXEN0);
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// transmit a single byte over UART
void UART_transmit(unsigned char data) {
  while (!(UCSR0A & (1 << UDRE0)))
    ;
  UDR0 = data;
}

// print a string over UART
void UART_print(const char *str) {
  while (*str) {
    UART_transmit(*str++);
  }
}

// print a number over UART
void UART_print_int(int16_t num) {
  char buffer[10];
  itoa(num, buffer, 10);
  UART_print(buffer);
}

// print float
void UART_print_float(float num) {
  char buffer[10];
  dtostrf(num, 6, 4, buffer);
  UART_print(buffer);
}

//--------------------------ALGORITHM for hovercraft starts
//here---------------------//
void setup() {
  UART_init();
  // Start lift fan.
  DDRD |= (1 << DDD6);
  PORTD |= (1 << PORTD6);

  I2C_init();     // MPU
  MPU6050_init(); // MPU
  fan_init();     // Both fans

  HCSR04_setup(); // Sensor
  timer0_init();  // thrust fan control (timer0)

  // servo init
  DDRB |= (1 << PB1); // enables pin  d9
  TCCR1A = (1 << COM1A1) | (1 << WGM11) | (1 << WGM10);
  TCCR1B = (1 << WGM12) | (1 << CS12);

  // top bar HCSR04
  DDRD &= ~(1 << PD2); // set Echo pin as input
  DDRB |= (1 << PB3);  // set Trigger pin as output

  // IR
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  ADMUX = (1 << REFS0);
  // clear channels selected in case there are selected ones for other uses.
  ADMUX &= 0xF0;
  // 0x00 is PORTA 0, which is A0 for us, or ADC0
  ADMUX |= 0x00;

  OCR1A = 89;     // set the servo to look at the front
  _delay_ms(200); // prevent spinning fan while turning servo
  OCR0A = 255;    // set the thrust fan to max speed
}

void loop() {
  // if the bar is detected, stop indefinitely
  while (stop)
    ;

  // IR method for bar
  voltage_avg = 0;
  for (int i = 0; i < trials; i++) {
    // the function will output a ADC reading from the pins.
    ADCvalue = analogRead_A0();
    _delay_ms(1);

    // the value will be summed up in the voltage_avg
    // we use 1023, 1024 and 1025 as we wish but we chose 1023 according to how
    // others did it on tutorials AVREF is using the external reference voltage
    // which we set via the potentiometer 3.1 volts.
    voltage_avg += ADCvalue * (AVREF / 1023);
    // sum the ADC values so that we can output the adc values.
    adc_avg = ADCvalue;
  }
  voltage = voltage_avg / trials;
  // calculate distance of the bar.
  bar = 29.888 * pow(voltage, -1.00);

  // US method for bar, which ever best works at the day of competition
  // top_hcsr04();
  // top_distance();
  // bar = distance_top;

  // servo
  // 41 is -90? //left
  // 89 is 0
  // 154 is 90 //right
  HCSR04_read();
  HCSR04_distance();
  UART_print("normal: \t");
  UART_print_int(distance);
  UART_print("\n");
  control_thrust_lift(distance);

  // if a wall is detected
  if (wall) {

    // read left
    OCR1A = 32;        // servo looks left
    _delay_ms(1000);   // delay 1 second to turn
    HCSR04_read();     // Front  US sensor reads
    HCSR04_distance(); // calculate distance
    UART_print("left: \t");
    UART_print_int(distance);
    UART_print("\n");
    left_dist = distance; // set distance to left side reading
    _delay_ms(1000);      // wait 1 second again.

    // read front
    OCR1A = 84; // algorithm repeats from prev. same algorithm. Reading front
    _delay_ms(1000);
    HCSR04_read();
    HCSR04_distance();
    UART_print("front: \t");
    UART_print_int(distance);
    UART_print("\n");
    front_dist = distance;
    _delay_ms(1000);

    // read right
    OCR1A = 154; // reading right.
    _delay_ms(1000);
    HCSR04_read();
    HCSR04_distance();
    UART_print("right: \t");
    UART_print_int(distance);
    UART_print("\n");
    right_dist = distance;
    _delay_ms(1000);

    // after reading all, we make a decision
    make_decision(left_dist, right_dist, front_dist);
    _delay_ms(2000); // after making a decision, wait for extra 2 seconds.

    wall = false; // reset the wall detection
    yaw = 0;      // reset IMU readings
    _delay_ms(250);
    PORTD |= (1 << PORTD6); // restart lift fan
    PORTD |= (1 << PORTD7);
    PORTD |= (1 << PORTD4);
  }

  // read raw accel/gyro data from the module
  MPU6050_read(&gx, &gy, &gyro_z);

  // print the obtained data on the defined format
  UART_print("a/g:\t");
  UART_print_int(gyro_z);
  UART_print("\n");

  // conver readings
  convert_raw_data();
  if (gyro_proc_z > 90) {
    yaw += gyro_proc_z - 180;
  } else {
    yaw += gyro_proc_z;
  }
  // print yaw to check
  UART_print("Yaw: ");
  UART_print_int(yaw);
  UART_print("\t");
  UART_print("IRBAR: ");
  UART_print_float(bar);
  UART_print("\n");

  // drifting control for the hovercraft's moment of inertia
  drifting_control();

  // algorithm if the bar is detected
  if (bar < 20) {
    PORTD &= ~(1 << PORTD6); // turn off thrust fan completely
    PORTD &= ~(1 << PORTD4);
    PORTD &= ~(1 << PORTD7);
    stop = true; // this prevents the loop from continuing on the next
                 // iteration.
  }

  _delay_ms(100); // add delay between readings
}