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
  Serial.begin(9600);
  pinMode(CONTROL_PIN, OUTPUT);
  pinMode(FEEDBACK_PIN, INPUT_PULLUP);

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
  Serial.println("Starting Relay");
  digitalWrite(CONTROL_PIN, LOW);
  while (true){
    if (digitalRead(FEEDBACK_PIN)){
      break;
    }
  }
}

bool throwBall() // returns true when the buttonpress is detected.  
{
  startRelay();
  while(true){
    if (!digitalRead(FEEDBACK_PIN)){
      Serial.println("Button Pressed");
      break;
    }
  }
  return true;
}

void stopRelay(){
  digitalWrite(CONTROL_PIN, HIGH);
  Serial.println("Stopping Relay");
  
}

void runSequence(){
  throwBall();
  stopRelay();
}
