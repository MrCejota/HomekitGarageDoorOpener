/*
 * garage_door_opener_func.h
 *
 *  Created on: 2022-10-06
 *      Author: MrCejota (Carlos Sola)
 */

float temperatureCalculation(int tempSensorPin){
  float temperature;
  float pinVoltage;
  float voltage;
  
  //voltage = (pinVoltage * (1.0/ 1023.0))*1000.0;
  pinVoltage = analogRead(tempSensorPin);
  voltage = pinVoltage;  
  temperature = (voltage - 424)/6.25;
  Serial.print("Pin Value: ");
  Serial.print(pinVoltage);
  Serial.print("\n");
  Serial.print("Calculated Voltage");
  Serial.print(voltage);
  Serial.print("\n");
  Serial.print("Temperature value: ");
  Serial.print(temperature);
  Serial.print("\n");
  return temperature;
}

float ultrasonicDistance(int trigPin, int echoPin, float tempAir){
    //Value of speed of sound in air per temperature
    float Cair;
    Cair = (331.3 + 0.606 * tempAir) * 39.37;
    
    //Trigger the ultrasonic sensor
    float duration;
    digitalWrite(trigPin,LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin,HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin,LOW);
    //Time that it takes the sound wave to leave AND return to the sensor
    duration = pulseIn(echoPin,HIGH);

    //Calculate the distance (in mm) based on the speed of sound.
    float distance;
    distance = 25.4 * (Cair * duration / (1000000)) / 2.0;
    Serial.print("The distance on Pin ");
    Serial.print(echoPin);
    Serial.print(" is: ");
    Serial.print(distance);
    Serial.print("\n");

    return distance;
}

bool sensorTriggered(float distance, float triggerDistance, float tolerance){
  bool sensorTrig;
  if((distance >= triggerDistance - tolerance) && (distance <= triggerDistance + tolerance)){
    sensorTrig = TRUE;
  }
  else{
    sensorTrig = FALSE;
  }
  return sensorTrig;

}
