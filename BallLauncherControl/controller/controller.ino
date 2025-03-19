#define CONTROL_PIN 13
#define FEEDBACK_PIN 4
#define STATE_0 '0'
#define STATE_1 '1'
char state = STATE_0;

void startRelay();
bool throwBall();
void stopRelay();
void runSequence();


void setup() {
  // put your setup code here, to run once:
  digitalWrite(CONTROL_PIN, HIGH);
  digitalWrite(FEEDBACK_PIN, LOW);
  pinMode(CONTROL_PIN, OUTPUT);
  digitalWrite(CONTROL_PIN, HIGH);
  pinMode(FEEDBACK_PIN, INPUT_PULLUP);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0 and state == STATE_0) {
        // read the incoming byte:
        char command = Serial.read();

        if (command == STATE_1){
          state = STATE_1;
          runSequence();
          state = STATE_0;

        } else if (command == STATE_0){
          stopRelay();
        }
      }
}



void startRelay(){
  // Serial.write("S");
  digitalWrite(CONTROL_PIN, LOW);
  while (true){
    if (digitalRead(FEEDBACK_PIN)){
      delay(100);
      break;
    }
  }
}

bool throwBall() // returns true when the buttonpress is detected.  
{
  startRelay();
  while(true){
    if (!digitalRead(FEEDBACK_PIN)){
      // Serial.write("B");
      break;
    }
  }
  return true;
}

void stopRelay(){
  digitalWrite(CONTROL_PIN, HIGH);
  Serial.write("D");
  
}

void runSequence(){
  throwBall();
  stopRelay();
}
