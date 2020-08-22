#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <ros.h>
#include <std_msgs/Int8.h>
#include <Adafruit_NeoPixel.h>

ros::NodeHandle nh;
#define NEOPIXPIN 7
#define NUMPIX 31

Adafruit_NeoPixel pixels(NUMPIX, NEOPIXPIN, NEO_GRB + NEO_KHZ800);
#define DELAYVAL 200

uint8_t state = 2;
uint8_t animationState = 0;
uint8_t intensity = 0;
int animationTime = 15;
long prvTime = 0;
uint8_t on = 0;

void state_cb(const std_msgs::Int8 &cmd_msg)
{

    // READ STATE
    if (cmd_msg.data == 1)
    {
        state = 1;
    }
    else if (cmd_msg.data == 0)
    {
        state = 0;
	on = 0;
    }
}

ros::Subscriber<std_msgs::Int8> sub("/odrive/state", state_cb);

void setup()
{
    Serial.begin(9600);
    nh.getHardware()->setBaud(9600);
    nh.initNode();
    nh.subscribe(sub);

    pixels.begin();

    for (int i = 0; i < NUMPIX; i++)
    { // For each pixel...
        pixels.setPixelColor(i, pixels.Color(150, 150, 0));
    }
    pixels.show(); // Send the updated pixel colors to the hardware.
}

void loop()
{

    pixels.clear();

    switch (state)
    {
    case 0: // Yellow
        if(millis()-prvTime>animationTime){
            prvTime = millis();
            for (int i = 0; i < NUMPIX; i++)
            {
                pixels.setPixelColor(i, pixels.Color(intensity,intensity, 0));
            }

            if(animationState == 0){
                intensity++;
                if(intensity > 200){
                    intensity = 200;
                    animationState = 1; 
                }
            }

            if(animationState == 1){
                intensity--;
                if(intensity < 1){
                    intensity = 1;
                    animationState = 0; 
                }
            }
	    pixels.show();
        }
        
        break;
    
    case 1:
        
	if(!on){

	    for (int i = 0; i < NUMPIX; i++)
            {
                pixels.setPixelColor(i, pixels.Color(150,150,150));
            }
            pixels.show();
            on = 1;
        }

        break;
    
    default:
        break;
    }

    nh.spinOnce();
    delay(1);
}
