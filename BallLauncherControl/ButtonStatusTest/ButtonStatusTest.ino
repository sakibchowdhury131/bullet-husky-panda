#define CONTROL_PIN 13
#define FEEDBACK_PIN 4
#define STATE_0 '0'
#define STATE_1 '1'
char state = STATE_0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(CONTROL_PIN, OUTPUT);
  pinMode(FEEDBACK_PIN, INPUT_PULLUP);
  digitalWrite(CONTROL_PIN, HIGH);

}

void loop() {
  Serial.println(digitalRead(FEEDBACK_PIN));

}
