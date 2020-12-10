#include <LiquidCrystal.h>
#include <SPI.h>

#define MAX_TEMP  700
#define MIN_TEMP  30

#define PID_P   2.8
#define PID_I   0.6
#define PID_D   0.2

#define MAX_VAL 93
#define MIN_VAL 4

#define BUTTON_TIMER  250 // in miliseconds
#define ENC_TIMER     5000  // in miliseconds

#define CS          PA8
#define HEATER_PIN  PC13
#define ZERO_DETECT PA4
#define LED_PIN     PC14

#define ENC_A       PA1
#define ENC_B       PA2
#define ENC_SW      PB15
encoder_t *enc = NULL;

#define SENSOR_PERIOD 250 // in miliseconds

struct encoder_t {
    uint8_t a_pin;
    uint8_t b_pin;
    uint8_t but_pin;
    bool flag;
    int8_t delta;
};

#define RS  PC15
#define EN  PA0
#define D4  PB11
#define D5  PB10
#define D6  PB1
#define D7  PB0
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

//*****************************************************************************
// VARIABLES
//*****************************************************************************

int16_t set = 30;         // variable for setted temparature
char buff_1[17];          // buffer for second line of LCD
char buff_2[17];          // buffer for first line of LCD
bool upd_screen = false;  // flag for updating screen
uint8_t period_for_sensor = 250;
uint16_t temp_raw = 0;    // raw value of temperature
uint16_t temp = 0;        // filtered value of temperature
uint8_t cnt = 0;          // counter for dimmer
uint16_t val = 50;         // value for comparing to cnt
uint8_t btn_cnt = 0;
uint8_t btn_delay = 0;
bool enc_enable = false;
uint8_t power_koef = 0;
uint8_t enc_delay = 0;

void setup() {

    pinMode(HEATER_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);

    pinMode(ZERO_DETECT, INPUT);
    attachInterrupt(ZERO_DETECT, EXT_INT0, RISING);

    pinMode(ENC_SW, INPUT);
    attachInterrupt(ENC_SW, EXT_INT1, RISING);

    enc = (encoder_t*)malloc(sizeof(enc));
    enc_init(enc);
    enc->a_pin = ENC_A;
    enc->b_pin = ENC_B;

    Timer2.pause();
    Timer2.setPeriod(1000);
    Timer2.attachInterrupt(TIMER_UPDATE_INTERRUPT, TIMER2_COMPA);
    Timer2.refresh();
    Timer2.resume();

    Timer3.pause();
    Timer3.setPeriod(100);
    Timer3.attachInterrupt(TIMER_UPDATE_INTERRUPT, TIMER3_COMPA);
    Timer3.refresh();
    Timer3.resume();

    lcd.begin(16, 2);
    lcd.print("ushani privet");

    pinMode(CS, OUTPUT);
    digitalWrite(CS, HIGH);
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV16);

}

void loop() {

    if (enc_check_state(enc)) {
        int8_t tmp_delta = enc_get_delta(enc);
        set = constrain(set+tmp_delta, 30, 700);      
        upd_screen = true;
        enc_delay = ENC_TIMER;
    }

    if (!period_for_sensor) {
      
        temp_raw = MAX6675_read();
        temp = (1 - 0.2) * (double)temp + 0.2 * (double)temp_raw;
        val = computePID(temp, set, PID_P, PID_I, PID_D, 0.25, MIN_VAL, MAX_VAL);
        upd_screen = true;
        period_for_sensor = SENSOR_PERIOD;
    }


    if (upd_screen) {
        sprintf(buff_1, "temp=%d/%dCdeg", temp, set);

        power_koef = (4 == val) ? 0 : (val*100) / (MAX_VAL + MIN_VAL);
        
        sprintf(buff_2, "PowKoef=%d", power_koef);
        
        Timer2.pause();
        lcd.clear();
        lcd.home();
        lcd.print(buff_1);
        lcd.setCursor(0, 1);
        lcd.print(buff_2);

        lcd.setCursor(15, 1);
        if (enc_enable)
            lcd.print("E");
        else
            lcd.print(" ");
        
        Timer2.resume();
        upd_screen = false;
    }
}

//*****************************************************************************
// TIMER WITH PERIOD 1 MILISECOND
//*****************************************************************************

void TIMER2_COMPA(void) 
{
    if(enc_enable)
        enc_read(enc);

    if(enc_delay != 0)
        enc_delay--;
    else if(enc_enable)
        enc_enable = false;

    if(btn_delay != 0)
      btn_delay--;

    if(0 == btn_delay && btn_cnt > 0)
      btn_cnt = 0;

    if (period_for_sensor > 0)
      period_for_sensor--;
}

//*****************************************************************************
// TIMER WITH PERIOD 100 MICROSECONDS
//*****************************************************************************

void TIMER3_COMPA()
{
    if(cnt != 0) {
        cnt--;
    } 
    else if(!digitalRead(HEATER_PIN) && val != 4) {
            digitalWrite(HEATER_PIN, HIGH);
    }    
}

//*****************************************************************************
// EXTERNAL INTERUPT FOR ZERO DETECTOR
//*****************************************************************************

void EXT_INT0(void)
{
    digitalWrite(HEATER_PIN, LOW);

    if(val != 4)
        cnt = (MAX_VAL + MIN_VAL+1) - val;
}

//*****************************************************************************
// EXTERNAL INTERUPT FOR ENCODET SWITCH
//*****************************************************************************

void EXT_INT1(void)
{
    if (!enc_enable) {
      btn_cnt++;
      btn_delay = BUTTON_TIMER;
  
      if(3 == btn_cnt) {
        enc_enable = true;
        enc_delay = ENC_TIMER;
      }      
    }
}

//-----------------------------------------------------------------------------

uint16_t MAX6675_read()
{
  uint8_t a1 = 0;
  uint8_t a2 = 0;
  uint16_t res = 0;

  digitalWrite(CS, LOW);
  a1 = SPI.transfer(0x00);
  a2 = SPI.transfer(0x00);
  digitalWrite(CS, HIGH);
  res = (a1 << 8) | a2;

  return res >> 5;
}

//-----------------------------------------------------------------------------

void enc_init(struct encoder_t *enc)
{
  pinMode(enc->a_pin, INPUT);
  pinMode(enc->b_pin, INPUT);

  enc->flag = 0;
  enc->delta = 0;
}

//-----------------------------------------------------------------------------

void enc_read(struct encoder_t *enc)
{
  static uint8_t up_state = 0;
  static uint8_t down_state = 0;
  static uint8_t old_state = 0;

  uint8_t new_state = 0;
  new_state |= (digitalRead(enc->a_pin));
  new_state |= (digitalRead(enc->b_pin) << 1);

  if (new_state != old_state) {

    switch (old_state) {
      case 0:
        if (new_state == 2) down_state++;
        if (new_state == 1) up_state++;
        break;

      case 1:
        if (new_state == 0) down_state++;
        if (new_state == 3) up_state++;
        break;

      case 2:
        if (new_state == 3) down_state++;
        if (new_state == 0) up_state++;
        break;

      case 3:
        if (new_state == 1) down_state++;
        if (new_state == 2) up_state++;
        break;
    }


    old_state = new_state;

    if (up_state >= 4) {
      enc->delta++;
      up_state = 0;
      enc->flag = 1;
    }

    if (down_state >= 4) {
      enc->delta--;
      down_state = 0;
      enc->flag = 1;
    }


  }
  else {
    enc->flag = 0;
  }
}

//-----------------------------------------------------------------------------

bool enc_check_state(struct encoder_t *enc)
{
  return enc->flag;
}

//-----------------------------------------------------------------------------

int8_t enc_get_delta(struct encoder_t *enc)
{
  int8_t tmp = 0;

  if (true == enc->flag) {
    enc->flag = 0;
    tmp = enc->delta;
    enc->delta = 0;
  }

  return tmp;
}

//-----------------------------------------------------------------------------

int computePID(uint16_t input, uint16_t setpoint, float kp, float ki, float kd, float dt, uint16_t minOut, uint16_t maxOut) 
{

    float err = (float)setpoint - (float)input;
    static float integral = 0, prevErr = 0;

    integral = constrain((integral + (float)err * dt * ki), (int16_t)minOut, (int16_t)maxOut);

    float Diff = (err - prevErr) / dt;
    prevErr = err;
    return constrain(err * kp + integral + Diff * kd, minOut, maxOut);
}
