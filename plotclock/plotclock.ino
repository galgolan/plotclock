// Plotclock
// cc - by Johannes Heberlein 2014
// v 1.04
// thingiverse.com/joo   wiki.fablab-nuernberg.de
// units: mm; microseconds; radians
// origin: bottom left of drawing surface
// time library see http://playground.arduino.cc/Code/time 
// RTC  library see http://playground.arduino.cc/Code/time 
//               or http://www.pjrc.com/teensy/td_libs_DS1307RTC.html


// comment out one of the two following lines
#define CALIBRATION         // enable calibration mode
//#define REALTIMECLOCK     // enable real time clock

// When in calibration mode, adjust the following factor until the servos move exactly 90 degrees
#define SERVOFAKTORLEFT   650
#define SERVOFAKTORRIGHT  680

// Zero-position of left and right servo
// When in calibration mode, adjust the NULL-values so that the servo arms are at all times parallel
// either to the X or Y axis
#define SERVOLEFTNULL   1650
#define SERVORIGHTNULL  500

// some servos operate in reverse direction than others (CW vs CCW)
// if your servos are running in the reverse direction (for example: you get a mirror image on the drawing surface),
// uncomment any of the 2 following lines to fix this.
#define SERVOREVERSELEFT
#define SERVOREVERSERIGHT

// defines the pins where the servo motors are connected
#define SERVOPINLIFT  2
#define SERVOPINLEFT  3
#define SERVOPINRIGHT 4

// lift positions of lifting servo
#define LIFT_DRAWING      950         // on drawing surface
#define LIFT_BETWEEN      1100        // between numbers
#define LIFT_TAKE_SWEEPER 1320        // going towards sweeper

#define DIGITS_SCALE	  0.9
#define DRAWING_HEIGHT  25

// speed of liftimg arm, higher is slower
#define LIFTSPEED 1000

// length of arms
#define L1 35
#define L2 57.1
#define L3 13.2

// origin points of left and right servo 
#define O1X 21
#define O1Y -25
#define O2X 48
#define O2Y -25

// 
#define PARKX -7
#define PARKY 15
#define ERASEMAXX 60    // erase motion width ?

#include <Time.h> // see http://playground.arduino.cc/Code/time 
#include <Servo.h>

#ifdef REALTIMECLOCK
// for instructions on how to hook up a real time clock,
// see here -> http://www.pjrc.com/teensy/td_libs_DS1307RTC.html
// DS1307RTC works with the DS1307, DS1337 and DS3231 real time clock chips.
// Please run the SetTime example to initialize the time on new RTC chips and begin running.

	#include <Wire.h>
	#include <DS1307RTC.h> // see http://playground.arduino.cc/Code/time    
#endif

// state variables
int servoLift = 1500;	// current pos of lift servo

Servo servo_lift;
Servo servo_left;
Servo servo_right;

volatile double lastX = 75;
volatile double lastY = PARKY;

volatile int inc = 1;

int last_min = 0;
#ifdef REALTIMECLOCK
void setup_rtc()
{
	//while (!Serial) { ; } // wait for serial port to connect. Needed for Leonardo only

	// Set current time only the first to values, hh,mm are needed  
	tmElements_t tm;
	if (RTC.read(tm)) 
	{
		setTime(tm.Hour,tm.Minute,tm.Second,tm.Day,tm.Month,tm.Year);
		Serial.println("DS1307 time is set OK.");
	} 
	else 
	{
		if (RTC.chipPresent())
			Serial.println("DS1307 is stopped.  Please run the SetTime example to initialize the time and begin running.");
		else 
			Serial.println("DS1307 read error!  Please check the circuitry.");

		// Set current time only the first to values, hh,mm are needed
		setTime(15,05,0,0,0,0);
  }
}
#else
void setup_calibration()
{
	// Set current time only the first to values, hh,mm are needed
	setTime(15,35,0,0,0,0);
}
#endif
void setup() 
{ 
	Serial.begin(9600);
	#ifdef REALTIMECLOCK
		setup_rtc();
	#else  
		setup_calibration();
	#endif

	moveTo(PARKX, PARKY);
	lift_drawing();
	servo_lift.attach(SERVOPINLIFT);
	servo_left.attach(SERVOPINLEFT);
	servo_right.attach(SERVOPINRIGHT);
	delay(1000);
}
#ifdef REALTIMECLOCK
int get_tens_digit(int num)
{
	for(int i=0; (i+1)*10 <= num; ++i);
}

int get_units_digit(int tens_digit, int num)
{
	return num - tens_digit*10;
}
void loop_rtc()
{
	int i = 0;
	// wait for minute to change so we can start drawing
	if (last_min != minute())
	{	
		int hour1 = get_tens_digit(hour());
		int hour2 = get_units_digit(hour1, hour());
		int minute1 = get_tens_digit(minute());
		int minute2 = get_units_digit(minute1, minute());
		
		// erase 2x
		lift_take_sweeper();
		erase();
		erase();
		
		draw_number(5, DRAWING_HEIGHT, hour1, DIGITS_SCALE);
		draw_number(19, DRAWING_HEIGHT, hour2, DIGITS_SCALE);
		draw_colon(28, DRAWING_HEIGHT, DIGITS_SCALE);
		draw_number(34, DRAWING_HEIGHT, minute1, DIGITS_SCALE);
		draw_number(48, DRAWING_HEIGHT, minute2, DIGITS_SCALE);
		
		lift_between();
		moveTo(PARKX, PARKY);		
		last_min = minute();
	}
}
#else
volatile int targetLift = LIFT_BETWEEN;
void loop_calibration()
{
	if (Serial.available() > 0)
	{
		int inByte = Serial.read();
		switch (inByte)
		{
		  case 'w':
			lastY += inc;
		  break;
		  case 'a':
			lastX -= inc;
		  break;
		  case 's':
			lastY -= inc;
		  break;
		  case 'd':
			lastX += inc;
		  break;
		  case 'r':
			lift_drawing();
			lastX = PARKX;
			lastY = PARKY;
		  break;
		  case 'z':
			targetLift = LIFT_TAKE_SWEEPER;
			break;
		  case 'x':
			targetLift = LIFT_BETWEEN;
			break;
		  case 'c':
			targetLift = LIFT_DRAWING;
			break;		
      case 'i':
      targetLift += 10;
      break;
      case 'k':
      targetLift -= 10;
      break;
      case ':':
      draw_colon(lastX,lastY,DIGITS_SCALE);
      break;
      case 'e':
      erase();
      break;
		}
		if((inByte >= '0') && (inByte <= '9'))
		{
		  draw_number(lastX, lastY, inByte - '0', 1);
		}

		Serial.print(lastX);
		Serial.print(",");
		Serial.print(lastY);
    Serial.print(",");
    Serial.println(targetLift);
	}
	  
	set_XY(lastX, lastY);
  lift(targetLift);
	delay(10);
	return;
	
	// Servohorns will have 90° between movements, parallel to x and y axis
  //moveTo(-3, 29.2);
  //delay(500);
  //moveTo(74.1, 28);
  //delay(500);
}
#endif


void loop() 
{ 
#ifdef CALIBRATION
	loop_calibration();
#else 
	loop_rtc();
#endif
} 

// Writing numeral with bx by being the bottom left originpoint. Scale 1 equals a 20 mm high font.
// The structure follows this principle: move to first startpoint of the numeral, lift down, draw numeral, lift up
void draw_number(float bx, float by, int num, float scale)
{
  switch (num)
  {
  case 0:
    moveTo(bx + 12 * scale, by + 6 * scale);
    lift_drawing();
    bogenGZS(bx + 7 * scale, by + 10 * scale, 10 * scale, -0.8, 6.7, 0.5);
    lift_between();
    break;
  case 1:
    moveTo(bx + 3 * scale, by + 15 * scale);
    lift_drawing();
    moveTo(bx + 10 * scale, by + 20 * scale);
    moveTo(bx + 10 * scale, by + 0 * scale);
    lift_between();
    break;
  case 2:
    moveTo(bx + 2 * scale, by + 12 * scale);
    lift_drawing();
    bogenUZS(bx + 8 * scale, by + 14 * scale, 6 * scale, 3, -0.8, 1);
    moveTo(bx + 1 * scale, by + 0 * scale);
    moveTo(bx + 12 * scale, by + 0 * scale);
    lift_between();
    break;
  case 3:
    moveTo(bx + 2 * scale, by + 17 * scale);
    lift_drawing();
    bogenUZS(bx + 5 * scale, by + 15 * scale, 5 * scale, 3, -2, 1);
    bogenUZS(bx + 5 * scale, by + 5 * scale, 5 * scale, 1.57, -3, 1);
    lift_between();
    break;
  case 4:
    moveTo(bx + 10 * scale, by + 0 * scale);
    lift_drawing();
    moveTo(bx + 10 * scale, by + 20 * scale);
    moveTo(bx + 2 * scale, by + 6 * scale);
    moveTo(bx + 12 * scale, by + 6 * scale);
    lift_between();
    break;
  case 5:
    moveTo(bx + 2 * scale, by + 5 * scale);
    lift_drawing();
    bogenGZS(bx + 5 * scale, by + 6 * scale, 6 * scale, -2.5, 2, 1);
    moveTo(bx + 5 * scale, by + 20 * scale);
    moveTo(bx + 12 * scale, by + 20 * scale);
    lift_between();
    break;
  case 6:
    moveTo(bx + 2 * scale, by + 10 * scale);
    lift_drawing();
    bogenUZS(bx + 7 * scale, by + 6 * scale, 6 * scale, 2, -4.4, 1);
    moveTo(bx + 11 * scale, by + 20 * scale);
    lift_between();
    break;
  case 7:
    moveTo(bx + 2 * scale, by + 20 * scale);
    lift_drawing();
    moveTo(bx + 12 * scale, by + 20 * scale);
    moveTo(bx + 2 * scale, by + 0);
    lift_between();
    break;
  case 8:
    moveTo(bx + 5 * scale, by + 10 * scale);
    lift_drawing();
    bogenUZS(bx + 5 * scale, by + 15 * scale, 5 * scale, 4.7, -1.6, 1);
    bogenGZS(bx + 5 * scale, by + 5 * scale, 5 * scale, -4.7, 2, 1);
    lift_between();
    break;
  case 9:
    moveTo(bx + 9 * scale, by + 11 * scale);
    lift_drawing();
    bogenUZS(bx + 7 * scale, by + 15 * scale, 5 * scale, 4, -0.5, 1);
    moveTo(bx + 5 * scale, by + 0);
    lift_between();
    break;
  }
}

void erase()
{
	  lift_drawing();
    moveTo(70, 46);
    moveTo(ERASEMAXX, 43);

    moveTo(ERASEMAXX, 49);
    moveTo(5, 49);
    moveTo(5, 45);
    moveTo(ERASEMAXX, 45);
    moveTo(ERASEMAXX, 40);

    moveTo(5, 40);
    moveTo(5, 35);
    moveTo(ERASEMAXX, 35);
    moveTo(ERASEMAXX, 30);

    moveTo(5, 30);
    moveTo(5, 25);
    moveTo(ERASEMAXX, 25);
    moveTo(ERASEMAXX, 20);

    moveTo(5, 20);
    moveTo(60, 44);

    moveTo(PARKX, PARKY);
    lift_take_sweeper();
}

void draw_colon(float bx, float by, float scale)
{
	moveTo(bx + 5 * scale, by + 15 * scale);
    lift_drawing();
    bogenGZS(bx + 5 * scale, by + 15 * scale, 0.1 * scale, 1, -1, 1);
    lift_between();
    moveTo(bx + 5 * scale, by + 5 * scale);
    lift_drawing();
    bogenGZS(bx + 5 * scale, by + 5 * scale, 0.1 * scale, 1, -1, 1);
    lift_between();
}

void lift_drawing()
{
	lift(LIFT_DRAWING);
}

void lift_between()
{
	lift(LIFT_BETWEEN);
}

void lift_take_sweeper()
{
	lift(LIFT_TAKE_SWEEPER);
}

void lift(int lift)
{
	int increment;
	if (servoLift > lift)
		increment = -1;
	else
		increment = 1;
	
	while(servoLift != lift)
	{
		servoLift += increment;
		servo_lift.writeMicroseconds(servoLift);
    delayMicroseconds(LIFTSPEED);
	}	
}

void bogenUZS(float bx, float by, float radius, int start, int ende, float sqee) {
  float inkr = -0.05;
  float count = 0;

  do {
    moveTo(sqee * radius * cos(start + count) + bx,
    radius * sin(start + count) + by);
    count += inkr;
  } 
  while ((start + count) > ende);

}

void bogenGZS(float bx, float by, float radius, int start, int ende, float sqee) {
  float inkr = 0.05;
  float count = 0;

  do {
    moveTo(sqee * radius * cos(start + count) + bx,
    radius * sin(start + count) + by);
    count += inkr;
  } 
  while ((start + count) <= ende);
}

// moves the pen along linear line from (lastX,lastY) to (pX,pY)
void moveTo(double pX, double pY) {
  double dx, dy, c;
  int i;

  // dx dy of new point
  dx = pX - lastX;
  dy = pY - lastY;
  //path lenght in mm, times 4 equals 4 steps per mm
  c = floor(4 * sqrt(dx * dx + dy * dy));

  if (c < 1) c = 1;

  for (i = 0; i <= c; i++) {
    // draw line point by point
    set_XY(lastX + (i * dx / c), lastY + (i * dy / c));

  }

  lastX = pX;
  lastY = pY;
}

double return_angle(double a, double b, double c) {
  // cosine rule for angle between c and a
  return acos((a * a + c * c - b * b) / (2 * a * c));
}

float reverse_servo(float microseconds)
{
  return map(microseconds, 0, 3000, 3000, 0);
}

// moves the pen to the coordinate (Tx,Ty)
void set_XY(double Tx, double Ty) 
{
  delay(1);
  double dx, dy, c, a1, a2, Hx, Hy;

  // calculate triangle between pen, servoLeft and arm joint
  // cartesian dx/dy
  dx = Tx - O1X;
  dy = Ty - O1Y;

  // polar length (c) and angle (a1)
  c = sqrt(dx * dx + dy * dy); // 
  a1 = atan2(dy, dx); //
  a2 = return_angle(L1, L2, c);

  double left_ms = floor(((a2 + a1 - M_PI) * SERVOFAKTORLEFT) + SERVOLEFTNULL);
#ifdef SERVOREVERSELEFT
  left_ms = reverse_servo(left_ms);
#endif
  servo_left.writeMicroseconds(left_ms);

  // calculate joint arm point for triangle of the right servo arm
  a2 = return_angle(L2, L1, c);
  Hx = Tx + L3 * cos((a1 - a2 + 0.621) + M_PI); //36,5°
  Hy = Ty + L3 * sin((a1 - a2 + 0.621) + M_PI);

  // calculate triangle between pen joint, servoRight and arm joint
  dx = Hx - O2X;
  dy = Hy - O2Y;

  c = sqrt(dx * dx + dy * dy);
  a1 = atan2(dy, dx);
  a2 = return_angle(L1, (L2 - L3), c);

  double right_ms = floor(((a1 - a2) * SERVOFAKTORRIGHT) + SERVORIGHTNULL);
#ifdef SERVOREVERSERIGHT
  right_ms = reverse_servo(right_ms);
#endif
  servo_right.writeMicroseconds(right_ms);
}
