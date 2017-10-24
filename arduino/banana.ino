#include <Wire.h>

#define SLAVE_ADDRESS 0x04
#define MASTER_ADDRESS 0x03

int number = 0;
int state = 0;

// constants won't change. They're used here to set pin numbers:
const int buttonPin = 2;    // the number of the pushbutton pin
const int ledPin = 13;      // the number of the LED pin
const int analogOutPin = 5; // Analog output pin that the LED is attached to
const int powerOutPin = 6;  // Controls the power supply (0 = on)
const int powerOutPlusPin = 7;  // Controls the power supply (1 = on)
const int interruptPin = 10;  // 

#define NO_TIMEOUT ~0L
unsigned long requestTime = 0;
unsigned long timeout = NO_TIMEOUT;
byte interrupting = 0;

enum State {
  STATE_OFF, STATE_BOOTING, STATE_ON, STATE_TERMINATING, STATE_RESETTING, STATE_INVALID
};
byte nextState = STATE_INVALID;
unsigned buttonSendRequest = false;

enum Effect {
  OFF,
  ON,
  MID,
  RISE,
  FADE,
  FAST_BLINK,
  SLOW_BLINK,
  HEARTBEAT
};

  
byte effectLoop(byte effect) {
  byte outputValue = 0;
  unsigned long count = millis() >> 2;
  
  switch (effect) {
    case OFF:
      outputValue = 0;
      break;
    case ON:
      outputValue = 255;
      break;
    case MID:
      outputValue = 150;
      break;
    case RISE:
      outputValue = count;
      break;
    case FADE:
      outputValue = ~count;
      break;
    case SLOW_BLINK:
      outputValue = bitRead(count, 7) ? 255 : 0;
      break;
    case FAST_BLINK:
      outputValue = bitRead(count, 5) ? 255 : 0;
      break;
    case HEARTBEAT:
      outputValue = bitRead(count, 8) ? ~count : count;
      break;
    default:
      effect = OFF;
      break;
  }
  
  return outputValue;
}

enum ButtonAction {
  NONE,
  MAYBE_SHORT_PRESS,
  SHORT_PRESS,
  MAYBE_LONG_PRESS,
  SURE_LONG_PRESS,
  LONG_PRESS
};

bool buttonPressed() {
  static byte counter = 0;
  static byte pressed = 0;
  
  int reading = digitalRead(buttonPin);
  //return reading;
  if (reading) {
    if (counter < 10) {
      counter++;
    } else {
      pressed = 1;
    }
  } else {   // !reading
    if (counter > 0) {
      counter--;
    } else {
      pressed = 0;
    }
  }
  return pressed;
}

byte buttonLoop() {
  static ButtonAction buttonState = NONE;
  static unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
  
  bool pressed = buttonPressed();
  //return pressed;
  
  switch (buttonState) {
    case NONE:
      if (pressed) {
        buttonState = MAYBE_SHORT_PRESS;
        lastDebounceTime = millis();
      }
      break;
    case MAYBE_SHORT_PRESS:
      if ((millis() - lastDebounceTime) > 100) {
        if (pressed) {
          buttonState = MAYBE_LONG_PRESS;
        } else {
          buttonState = SHORT_PRESS;
        }
      } else if (!pressed) {
        buttonState = NONE;
      }
      break;
    case MAYBE_LONG_PRESS:
      if (!pressed) {
        buttonState = SHORT_PRESS;
      } else if ((millis() - lastDebounceTime) > 5000) {
        buttonState = SURE_LONG_PRESS;
      }
      break;
    case SURE_LONG_PRESS:
      if (!pressed) {
        buttonState = LONG_PRESS;
      }
      break;
    default:
      buttonState = NONE;
      break;
  }
  return buttonState;
}

byte stateLoop(byte buttonState) {
  static byte state = STATE_OFF;
  
  switch (state) {
    case STATE_OFF:
      if (buttonState == SHORT_PRESS) {
        state = STATE_BOOTING;
      } else if (/* temporary */ buttonState == LONG_PRESS) {
        state = STATE_RESETTING;
      }
      break;
    case STATE_RESETTING:
    case STATE_BOOTING:
      if (/*temporary*/ buttonState == SHORT_PRESS) {
        state = STATE_ON;
      } else if (buttonState == LONG_PRESS) {
        state = STATE_OFF;
      }
      break;
    case STATE_ON:
      if (buttonState == SHORT_PRESS) {
        state = STATE_TERMINATING;
      } else if (buttonState == LONG_PRESS) {
        state = STATE_OFF;
      }
      break;
    case STATE_TERMINATING:
      if (buttonState == LONG_PRESS || buttonState == SHORT_PRESS) {
        state = STATE_OFF;
      }
      break;
  }
  
  if (timeout != NO_TIMEOUT && millis() - requestTime >= timeout) {
    if (nextState != STATE_INVALID) {
      Serial.print("Changing state to: ");
      Serial.println(nextState);
      state = nextState;
      nextState = STATE_INVALID;
    }
    timeout = NO_TIMEOUT;
  }
  
  return state;
}

void outputLoop(byte state, byte buttonState) {
  static Effect effect = OFF;
  
  digitalWrite(powerOutPin, state == STATE_OFF);
  digitalWrite(powerOutPlusPin, state != STATE_OFF);
  digitalWrite(ledPin, state != STATE_OFF);
  
  // we must be extra coutious never to send logical 1 to interrupt pin - it is directly connected to Banana and 5V would fry it.
  // instead we entier send output 0 to ground it, or we set input and disconnect pull-up. 
  // The pin is effectively disconnected, but there is a pull-up in Banana, so logical 1 is read.
  digitalWrite(interruptPin, 0);
  pinMode(interruptPin, interrupting ? OUTPUT : INPUT);
  digitalWrite(interruptPin, 0);
  
  switch (state) {
    case STATE_OFF:
      effect = OFF;
      break;
    case STATE_BOOTING:
      effect = RISE;
      break;
    case STATE_ON:
      effect = MID;
      break;
    case STATE_TERMINATING:
      effect = FADE;
      break;
    case STATE_RESETTING:
      effect = HEARTBEAT;
      break;
  }
  
  switch (buttonState) {
    case MAYBE_SHORT_PRESS:
    case MAYBE_LONG_PRESS:
      effect = ON;
      break;
    case SURE_LONG_PRESS:
      effect = FAST_BLINK;
      break;
  }
  
  analogWrite(analogOutPin, effectLoop(effect));
}

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);
  
  pinMode(buttonPin, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(powerOutPin, OUTPUT);
  pinMode(interruptPin, INPUT);
  digitalWrite(interruptPin, 0);
  
  
  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  Serial.println("Ready!");
}

void loop() {
  byte buttonState = buttonLoop();
  
  if (buttonState == SHORT_PRESS) {
    interrupting = interrupting ? 0 : 'b';  // TODO
  }
    
  byte state = stateLoop(buttonState);
  outputLoop(state, buttonState);
}

// callback for received data
void receiveData(int byteCount){
  int i = 0;
  char arr[11];
  for (i =0; i < 10 && i < byteCount && Wire.available(); i++) {
    arr[i] = Wire.read();
  }
  arr[i] = 0;
  
  /*Serial.print("Received ");
  Serial.print(byteCount);
  Serial.print(" bytes: ");
  Serial.print(arr);
  */
  byte command = arr[0];
  byte arg = arr[1];
  
  /*
  Serial.print("Command ");
  Serial.print(command);
  Serial.print(", arg ");
  Serial.print(arg);
  Serial.println();*/
  
  if (command != 0) {
    Serial.print("Command: ");
    Serial.println(command);
  }
  
  switch (command) {
    case 'f': nextState = STATE_OFF; break;
    case 'b': nextState = STATE_BOOTING; break;
    case 'n': nextState = STATE_ON; break;
    case 't': nextState = STATE_TERMINATING; break;
    case 'r': nextState = STATE_RESETTING; break;
    case 'x': nextState = STATE_INVALID; break;
    case 0: break;
    default: 
      command = 0; 
      Serial.print("Invalid command: ");
      Serial.println(command);
      break;
  }
  
  if (command != 0) {
    Serial.print("Next state: ");
    Serial.println(nextState);
    
    timeout = (arg-'0') * 1000;
    
    Serial.print("Timeout: ");
    Serial.println(timeout);
    
    requestTime = millis();
  }
}

// callback for sending data
void sendData(){
  Wire.write(interrupting);
}
