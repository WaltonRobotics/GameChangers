void setup()
{
  Serial.begin(9600);
  analogWrite(2 , 255);

  analogWrite(3 , 0);

  analogWrite(4 , 0);

}

void loop()
{
  Serial.print("hello world");
  Serial.println();
}
