

unsigned long rpmstart;
int rpm;
float rpm_cur;
int rev = 0;

int debugcounter = 0;

//ignition
float ignition_rpm[10] = { 0,1000,2000,3000,4000,5000,6000,7000,8000,9000 };
float ignition_advance[10] = {10000.0, 5000.0, 2500.0, 1666.667, 1250.0, 1000.0, 833.333, 714.286, 625.0, 555.556};
const int ignilistlen = sizeof(ignition_rpm) / sizeof(ignition_rpm[0]);
const byte ignition_pin = 8;

float ignition_cap[9];
float advance_cap[9];
float time_ign;

//inject
double inject_time;
double X[] = { 0, 500, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000 };           //rpm x axis
double Y[] = { 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100 };                        //inject dur y axis
const int Xcount = sizeof(X) / sizeof(X[0]);
const int Ycount = sizeof(Y) / sizeof(Y[0]);
const byte injector_pin = 7;

int map_sensor;


bool trig = false;

//test data
//rpm   |
//      |____>map

/*double Z[Xcount][Ycount] = {                                        
  {70, 71, 72, 73, 74, 75, 76, 77},
  {60, 61, 62, 63, 64, 65, 66, 67},
  {50, 51, 52, 53, 54, 55, 56, 57},
  {40, 41, 42, 43, 44, 45, 46, 47},
  {30, 31, 32, 33, 34, 35, 36, 37},
  {20, 21, 22, 23, 24, 25, 26, 27},
  {10, 11, 12, 13, 14, 15, 16, 17},
  {00, 01, 02, 03, 04, 05, 06, 07}
};*/

double Z[Xcount][Ycount] = {
  {0.0,359.9,719.7,1079.3,1438.7,1797.9,2157.0,2516.0,2874.7,3233.3,3591.7},
  {0.0,359.0,717.9,1076.5,1435.0,1793.4,2151.6,2509.6,2867.4,3225.1,3582.6},
  {0.0,358.1,716.0,1073.8,1431.4,1788.8,2146.1,2503.2,2860.2,3216.9,3573.6},
  {0.0,357.2,714.2,1071.1,1427.8,1784.3,2140.7,2496.9,2852.9,3208.8,3564.5},
  {0.0,356.3,712.4,1068.3,1424.1,1779.8,2135.2,2490.5,2845.6,3200.6,3555.4},
  {0.0,355.4,710.6,1065.6,1420.5,1775.2,2129.8,2484.1,2838.3,3192.4,3546.3},
  {0.0,354.5,708.8,1062.9,1416.9,1770.7,2124.3,2477.8,2831.1,3184.2,3537.2},
  {0.0,353.6,706.9,1060.2,1413.2,1766.1,2118.8,2471.4,2823.8,3176.0,3528.1},
  {0.0,352.6,705.1,1057.4,1409.6,1761.6,2113.4,2465.0,2816.5,3167.9,3519.0},
  {0.0,351.7,703.3,1054.7,1406.0,1757.0,2107.9,2458.7,2809.3,3159.7,3509.9},
  {0.0,350.8,701.5,1052.0,1402.3,1752.5,2102.5,2452.3,2802.0,3151.5,3500.8}
};

void alustus_ignition_cap()
{
  for (int i = 1; i < ignilistlen; i++)
  {
    ignition_cap[i - 1] = ignition_rpm[i] - ignition_rpm[i - 1];
    advance_cap[i - 1] = ignition_advance[i] - ignition_advance[i - 1];
  }
}

void ignition_interpolate(float rpmm)
{
  for (int i = 0; i < ignilistlen; i++)
  {
    if (rpmm < ignition_rpm[i])
    {
      int z = i - 1;
      time_ign = (((rpmm - ignition_rpm[z]) / ignition_cap[z]) * advance_cap[z]) + ignition_advance[z];
      break;
    }
    else  if (rpmm == ignition_rpm[i])
    {
      time_ign = ignition_advance[i];
      break;
    }
  }
}


double bilinearXY(int x, int y)
{
  int xIndex, yIndex;

  if ((x < X[0]) || (x > X[Xcount - 1])) {
    return 3500; // arbitrary... outta range hommat
  }

  if ((y < Y[0]) || (y > Y[Ycount - 1])) {
    return 3500; // arbitrary...
  }

  for (int i = Xcount - 2; i >= 0; --i)
    if (x >= X[i]) {
      xIndex = i;
      break;
    }

  for (int i = Ycount - 2; i >= 0; --i)
    if (y >= Y[i]) {
      yIndex = i;
      break;
    }

  
  /*Serial.print(xIndex);
  Serial.print("  ");
  Serial.println(yIndex);*/


  // Q11 = (x1, y1), Q12 = (x1, y2), Q21 = (x2, y1), and Q22 = (x2, y2)

  double x1, y1, x2, y2;
  double fQ11, fQ12, fQ21, fQ22;
  double fxy1, fxy2, fxy;

  x1 = X[xIndex];
  x2 = X[xIndex + 1];
  y1 = Y[yIndex];
  y2 = Y[yIndex + 1];


  // naa on ne mita pitaa saataa lambdalla
  fQ11 = Z[xIndex][yIndex];
  fQ12 = Z[xIndex][yIndex + 1];
  fQ21 = Z[xIndex + 1][yIndex];
  fQ22 = Z[xIndex + 1][yIndex + 1];

  fxy1 = ((x2 - x) / (x2 - x1)) * fQ11 + ((x - x1) / (x2 - x1)) * fQ21;
  fxy2 = ((x2 - x) / (x2 - x1)) * fQ12 + ((x - x1) / (x2 - x1)) * fQ22;

  fxy = ((y2 - y) / (y2 - y1)) * fxy1 + ((y - y1) / (y2 - y1)) * fxy2;

  return fxy;

}

void fire() {
  rev ++;
}

void setup()
{
  Serial.begin(9600);
  alustus_ignition_cap();
  pinMode(2, INPUT_PULLUP);
  /*//skeidaa
  TCCR1B &= ~(1 << CS12);
  TCCR1B |= (1 << CS11);
  TCCR1B &= ~(1 << CS10);
  //more skeida
  TCCR1B &= ~(1 << WGM13);
  TCCR1B |= (1 << WGM12);
  TCCR1A |= (1 << WGM11);
  TCCR1A |= (1 << WGM10);*/

  /*DDRB |= (1 << PB1) | (1 << PB2);
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);
  ICR1 = 0xffff;*/
  
  
  
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(2), fire, RISING);
}

void loop()
{
  if(rev >= 2)
  {
    //unsigned long StartTime = micros();
    rev = 0;
    rpm_cur = float(1000000.00/(micros() - rpmstart))*60.0*2;
    rpmstart = micros();
    //rpm = rpm_cur;
    
    
    ignition_interpolate(rpm_cur);


    delayMicroseconds(time_ign);
    PORTB = PORTB | B00000001;
    delayMicroseconds(50);
    PORTB = PORTB & B11111110;

    

    
    map_sensor = analogRead(A0);
    inject_time = bilinearXY(rpm_cur, 95);

    //analogWrite(9, int(inject_time));
    //OCR1A = uint16_t(inject_time*65000);

    
    PORTD = PORTD | B10000000;
    delayMicroseconds(inject_time);
    PORTD = PORTD & B00000111;
    

    /*
    PORTD = PORTD & B00000111;
    delayMicroseconds(int(inject_time));
    PORTD = PORTD | B10000000;*/
    
    
    //unsigned long ElapsedTime = micros() - StartTime;
    

    if (debugcounter >= 100)
    {
      debugcounter = 0;
      Serial.print("rpm: ");
      Serial.println(rpm_cur);

      Serial.print("Ennakko aika: ");
      Serial.println(time_ign);

      Serial.print("Ruiskutus aika: ");
      Serial.println(inject_time);
    
      //Serial.print("loop micros: ");
      //Serial.println(ElapsedTime);
    }
    debugcounter++;
  }
}
