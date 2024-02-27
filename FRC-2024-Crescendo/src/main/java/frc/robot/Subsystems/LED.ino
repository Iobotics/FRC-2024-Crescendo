#include <FastLED.h>


#define DATA_PIN1 6       // Connect the data pin of the swiffer LED strip to digital pin 6
#define DATA_PIN2 5       // Connect the data pin of the shooter LED strip to digital pin 5
#define SENSOR_PIN 2      // Connect the optical sensor to digital pin 2
#define LED_TYPE WS2812B  // Adjust based on your LED strip type
#define COLOR_ORDER GRB    // Adjust based on your LED strip color order
#define SWIFFER_LEDS 30    // Adjust based on the number of LEDs in your swiffer strip
#define SHOOTER_LEDS 50    // Adjust based on the number of LEDs in your shooter strip


CRGB swiffer[SWIFFER_LEDS];
CRGB shooter[SHOOTER_LEDS];
bool hasBlinkedShooter = false;


void setup() {
  FastLED.addLeds<LED_TYPE, DATA_PIN1, COLOR_ORDER>(swiffer, SWIFFER_LEDS);
  FastLED.addLeds<LED_TYPE, DATA_PIN2, COLOR_ORDER>(shooter, SHOOTER_LEDS);
  pinMode(SENSOR_PIN, INPUT);
}


void loop() {
  // Check the status of the optical sensor
  int sensorValue = digitalRead(SENSOR_PIN);


  if (sensorValue == HIGH) {
    fill_solid(swiffer, SWIFFER_LEDS, CRGB(255, 0, 0));
    fill_solid(shooter, SHOOTER_LEDS, CRGB(255, 50, 0));
    FastLED.show();
    hasBlinkedShooter = false;  // Reset the flag when the sensor is tripped
  } else {
    fill_solid(swiffer, SWIFFER_LEDS, CRGB(0, 255, 255)); // Set color to constant orange


    // Sensor is not tripped, flash the shooter LED strip orange
    if (!hasBlinkedShooter) {
      for (int i = 0; i < 5; i++) {
        fill_solid(shooter, SHOOTER_LEDS, CRGB(0, 255, 0));
        FastLED.show();
        delay(100); // Wait for 100 milliseconds (adjust as needed)
        fill_solid(shooter, SHOOTER_LEDS, CRGB(0, 0, 0));
        FastLED.show();
        delay(100); // Wait for 100 milliseconds
      }


      fill_solid(shooter, SHOOTER_LEDS, CRGB(255, 50, 0)); // Set color back to constant orange
      FastLED.show();


      hasBlinkedShooter = true;  // Set the flag after flashing
    }
  }
}



