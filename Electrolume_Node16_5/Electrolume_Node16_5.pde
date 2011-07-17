//     my app       |  ICNAPP   | ICNlibrary  |
//                  |           |             |
//    write code    |  modify   | don't touch |
//       here       | as needed | bits & c.   |

#define RUNTIME
#include <DmxSimple.h>

void setup() 
{
#ifdef RUNTIME
  netstart(); // KEEP THIS LINE
#else
  Serial.begin(9600);
#endif
  DmxSimple.usePin(3);

  DmxSimple.maxChannel(4);
}

/* OUTPUT
 - 6 combi
 - 6 combiEnergy
 - 6 combiChange
 - 4 ligths
 - combiTotal
 
 TODO
 x langsommere combiEnergy
 - mere lys
 x hurtigere combiChange
 
 */
#define range 254
long time = 0;

float combi[6];
float combiOld[6];
float combiEnergy[6];
float combiTotal =0;
float combiChange[6];

float ligths[4];
float ligths_dmx[4];

float maxSlider =3015;
float cutoffSlider =     100;
float factorLightSlider = 1.7f;
float changeSLider = 1.3f;
float energySlider = 0.3;

/*
  control sliders:
  maxSlider
  cutoffSlider
  factorLight 
  changeSlider
  energySlider

  

*/
void loop()
{


  /// RESET

  for(int i = 0;i<4;i++)
  {
    ligths[i] = 0;
  }
  for(int i = 0;i<6;i++)
  {
    combiOld[i] = combi[i];
  }


  /// READ DATA
  int index = 0;
  int lightTotal = 0;
  for(int i = 0; i < 3 ;i ++)
  {
    for(int b = i+1; b < 4; b++)
    {
      // RAW READ
      combi[index] = mapIt(touchRead(i,b) - cutoffSlider, maxSlider);


      ligths[i] = ligths[i] + combi[index];
      ligths[b] = ligths[b] + combi[index];
      lightTotal = lightTotal + combi[index];
      index++;

    }

  }

  lightTotal = lightTotal / 6.0f;

  // map sums


  for(int i=0;i < 4;i++)
  {
    ligths[i] = ligths[i] / 3.0f;

  }

  /// PROCESS DATA

  combiTotal = combiTotal *0.7 + lightTotal * 0.2;
  for(int i = 0; i < 6 ;i ++)
  {
    combiChange[i] = combiChange[i] * 0.92f + (makeEven(combi[i] - combiOld[i]))/changeSLider; 
    combiChange[i] = mapIt(combiChange[i],range);
  }
  for(int i = 0; i < 6 ;i ++)
  {
    if(combi[i] > 40)
      combiEnergy[i] = combiEnergy[i] + combi[i]*energySlider;
    else
      combiEnergy[i] = combiEnergy[i] * 0.6;

    combiEnergy[i] =  mapIt(combiEnergy[i],range);
  }


  /// SEND DATA
  index = 0;

  // SEND DATA

  /* OUTPUT
   - 6 combi
   - 6 combiEnergy
   - 6 combiChange
   - 4 ligths
   - combiTotal
   
   
   */

  for(int i = 0; i < 6; i++)
  {
    sendData(index++,combi[i]);

  }
  for(int i = 0; i < 6; i++)
  {
    sendData(index++,combiEnergy[i]);

  }
  for(int i = 0; i < 6;i++)
  {
    sendData(index++,combiChange[i]);

  }

  for(int i = 0; i < 4;i++)
  {
    sendData(index++,ligths[i]);

  }
  sendData(index++,combiTotal);
  sendDataEnd();

  //add noise
  for(int i = 0; i < 4;i++)
  {
    ligths_dmx[i] *= 0.99;
   
    if(ligths[i] < 0.5 && random(0.0f,1000.0f) > 999.9f)
    {
      ligths_dmx[i] = random(800,1000);
    }
    
    ligths_dmx[i] =  max(ligths_dmx[i],min(255,ligths[i]*255.0f/range*factorLightSlider)); 
   
    
   
  }
  
  // DMX 
  if(time < millis())
  {

    time = millis() + 20;
    for(int i=0;i<4;i++)
    {
      DmxSimple.write(i+1, min(ligths_dmx[i],255));
    }

  } 
  /// DMX 


}

float makeEven(float value)
{
  if (value >0)
  {
    return value;
  }
  return value * -1; 

}
void sendData(int index, float value)
{

#ifdef RUNTIME
  writeReturnInt(index,(int)value);
#else
  Serial.print((int)value);
  Serial.print("  ");
#endif
}

void sendDataEnd()
{
#ifndef RUNTIME
  Serial.println("");
#endif
}


float mapIt(int value, int maximum)
{
  return max(0.0f,min(range,((float)value/(float)maximum * range)));

}

int touchRead(int pin1, int pin2) // Brug analoge tal
{
  int sum=0; // summerings int
  
  for(int i=0; i < 10; i++) // 10 samples - ændre efter behov.
  {
    pinMode(pin1+14, OUTPUT); // pin1 til output


    digitalWrite(pin1 + 14,HIGH); 
    delayMicroseconds(10);
    int high = analogRead(pin2);

    digitalWrite(pin1 + 14,LOW);
    delayMicroseconds(10);
    int low = analogRead(pin2);


    pinMode(pin1+14, INPUT);
    digitalWrite(pin1 + 14,LOW);
    sum = sum +high-low;
  }

  return (int)((float)sum/4.0f);

}

/*

if(touchRead(0,1)>1000) // analogue 0,1
{
    
  
}


*/

