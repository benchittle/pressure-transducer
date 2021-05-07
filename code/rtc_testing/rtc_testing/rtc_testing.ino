#include <avr/sleep.h>
#include <avr/wdt.h>
#include "RTClib.h"

RTC_DS3231 rtc;

volatile bool flag = false;
bool state = 1;

void setup() {
  Serial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);

  // put your main code here, to run repeatedly:
  if (rtc.begin()) {
    Serial.println("Found");
    
  } else {
    Serial.println("Not found");
  }

  if (rtc.lostPower()) {
    Serial.println("Lost power");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  Serial.flush();
  enableSampling();
}

void enableSampling() {
  // Switching to an asynchronous clock source as described in section 18.9 
  // (Asynchronous Operation of Timer/Counter2) of the datasheet
  TIMSK2 = 0;
  
  rtc.enable32K();
  ASSR |= _BV(EXCLK) | _BV(AS2);
  
  TCCR2A = 0;
  // Set the prescaler value to clk/128. This will result in the timer overflowing once per second.
  TCCR2B = _BV(CS22) | _BV(CS20);
  // Set the timer to 0.
  TCNT2 = 0;
  // Wait for the above changes to be made.
  while (ASSR & (_BV(TCN2UB) | _BV(TCR2AUB) | _BV(TCR2BUB)));
  // Clear the interrupt flags.
  TIFR2 = _BV(OCF2B) | _BV(OCF2A) | _BV(TOV2);

  // Set the timer to create an interrupt whenever it overflows.
  TIMSK2 = _BV(TOIE2);
}

ISR(TIMER2_OVF_vect) {
  flag = true;
}

void lightSleep() {
  // Save previous state of ADC registry.
  byte adcsra = ADCSRA;

  // We have to make sure we don't re-enter power-save mode too quickly to
  // avoid unwanted behaviour.
  OCR2A = 0;
  while (ASSR & _BV(OCR2AUB));
  
  set_sleep_mode(SLEEP_MODE_PWR_SAVE);
  // Disable ADC while sleeping.
  ADCSRA = 0;
  sleep_enable();

  // Prevent interrupts while getting ready to sleep. If an interrupt were to occur
  // before sleeping, it would detatch and no longer run to wake up the device.
  noInterrupts();
  wdt_disable();
  sleep_bod_disable();
  interrupts();
  sleep_cpu();
  
  sleep_disable();

  //wdt_enable(WDTO_1S);
  ADCSRA = adcsra;
}



void loop() {
  char timestr[32] = "MMM DD, YYYY @ hh:mm:ss";
  
  //digitalWrite(LED_BUILTIN, HIGH);
  //delay(3000);
  //digitalWrite(LED_BUILTIN, LOW);
  //delay(3000);
  if (flag) {
    Serial.println(rtc.now().toString(timestr));
    Serial.flush();
    digitalWrite(LED_BUILTIN, state ? HIGH : LOW);
    state = !state;
    flag = false;
  }
  lightSleep();
  TIMSK2 = _BV(TOIE2);
}
