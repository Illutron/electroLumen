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
 - langsommere combiEnergy
 - mere lys
 - hurtigere combiChange
 
 */
#define range 254
long time = 0;

float combi[6];
float combiOld[6];
float combiEnergy[6];
float combiTotal =0;
float combiChange[6];
float ligths[4];
int maxSlider = 3057;
int cutoffSlider = 100;
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
    combiChange[i] = combiChange[i] * 0.8f + (makeEven(combi[i] - combiOld[i]))/2.0f; 
    combiChange[i] = mapIt(combiChange[i],range);
  }
  for(int i = 0; i < 6 ;i ++)
  {
    if(combi[i] > 20)
      combiEnergy[i] = combiEnergy[i] + combi[i]*0.08;
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

  // DMX 
  if(time < millis())
  {

    time = millis() + 100;
    for(int i=0;i<4;i++)
    {
      DmxSimple.write(i+1, ligths[i]*255.0f/range);
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

int touchRead(int pin1, int pin2)
{
  int sum=0;
  for(int i=0; i < 10; i++)
  {
    pinMode(pin1+14, OUTPUT);


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

