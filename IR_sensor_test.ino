// IR avoidance test
#define far 8
#define near 9
void setup() {
  // put your setup code here, to run once:
  pinMode(far, INPUT);
  pinMode(near, INPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  int far_out = digitalRead(far);
  int near_out = digitalRead(near);
  Serial.print("far : ");
  if (far_out == 0) Serial.print("detected!");
  else Serial.print("clear!");

  Serial.print(" | ");
  Serial.print("near : ");
  if (near_out == 0) Serial.print("detected!");
  else Serial.print("clear!");
  Serial.print("\n");
}
