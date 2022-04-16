#include "TransmissionMath_lib.hpp"

double Point::getLength(Point p)
{
  return sqrt(pow(p.x - x, 2) + pow(p.y - y, 2));
}

std::string Point::toString()
{
   std::string s = "(" + std::to_string(x) + "," + std::to_string(y) + ")";
   return s;
}

/**************************************************************************************
*  Find abslute heading angle in degree from this point to the destination point p
*  return angle 0 <= angle < 360
**************************************************************************************/
double Point::getHeadingToPoint_Degree(Point p)
{
  double angle = 0;
  double tmpX = p.getX() - x;
  double tmpY = p.getY() - y;
  if(tmpX == 0)
  {
    if(tmpY >= 0)
    {
      angle = 90;
    }
    else
    {
      angle = 270;
    }
  } else {
    angle = radiansToDegrees(atan(tmpY/tmpX));
    if(tmpX > 0 && tmpY >= 0){ //in the first quadrant
      angle = angle;
    } else if(tmpX < 0 && tmpY >= 0){ //in the second quadrant
      angle = angle + 180;
    } else if(tmpX < 0 && tmpY < 0){ //in the third quadrant
      angle = angle + 180;
    } else if(tmpX > 0 && tmpY < 0){ //in the forth quadrant
      angle = angle + 360;
    }
  }
  return angle;
}


Line::Line(){
  Point p1(0, 0);
  Point p2(1, 1);
  A = p2.y - p1.y;
	B = (p2.x - p1.x) * -1;
	C = (p2.y - p1.y)*p1.x + (p2.x - p1.x)*p1.y;
	if (A < 0) {
		A = 0 - A;
		B = 0 - B;
		C = 0 - C;
	}
}


Line::Line(Point p1, double slope){
//  y - p1.y = slope * (x - p1.x);
//  y - p1.y = slope*x - slope*p1.x;
//  slope*x - y -slope*p1.x + p1.y = 0;
  if (slope == INFINITY){
    A = p1.x;
    B = 0;
    C = 0;
  }
  else{
    A = slope;
    B = 1;
    C = p1.y - (p1.x * slope);
  }
}

Line::Line(Point p1, Point p2){
	A = p2.y - p1.y;
	B = (p2.x - p1.x) * -1;
	C = (p2.y - p1.y)*p1.x + (p2.x - p1.x)*p1.y;
	if (A < 0) {
		A = 0 - A;
		B = 0 - B;
		C = 0 - C;
	}
}


double Line::getSlope(){
  if(B == 0){
    return INFINITY;
  }
  return -A/B;
}

double Point::getAngle(Point p)
{
  double angle;
  if (p.x - x == 0) {
    if (y <= p.y)
    {
      angle = 3.1415926 / 2;  //90
    }
    else
    {
      angle = 3.1415926 / 2 * 3;  //270
    }
  } else {
    angle = atan((p.y - y) / (p.x - x));
    if(p.x < x){
      angle = angle + 3.1415926;
    }
  }
  return angle;
}

/*
Point Line::getIntercection(Line line){
  double xx;
  double yy;
  if (getSlope() == line.getSlope()){
    Point tmp = Point(INFINITY, INFINITY);
//    return Point(INFINITY, INFINITY);
    return tmp;
  }
  else {
    xx = (line.C - C) / (A - line.A);
    yy = -1 * A * xx - C;
  }
  return Point(xx, yy);
}
*/

Directed_Line_Segment::Directed_Line_Segment(){
  initial = Point(0, 0);
  terminal = Point(1, 0);
  line = Line(initial, terminal);
}

Directed_Line_Segment::Directed_Line_Segment(Point initialPoint, Point terminalPoint){
  initial = initialPoint;
  terminal = terminalPoint;
  line = Line(initial, terminal);
}

/******************************************************************************************
* return angle between 0 and 360 degree, starting from the point "initial"
*        to the point "terminal"
******************************************************************************************/
double Directed_Line_Segment::get_Normalized_Angle(){
  double angle = 0;
  double X_diff = terminal.x - initial.x;
  double Y_diff = terminal.y - initial.y;

  if(X_diff == 0){
    if(Y_diff >= 0){
      angle = 90;
    } else {
      angle = 270;
    }
  } else {
    angle = radiansToDegrees(atan(Y_diff/X_diff));
    if(X_diff > 0 && Y_diff >= 0){ //in the first quadrant
      angle = angle;
    } else if(X_diff < 0 && Y_diff >= 0){ //in the second quadrant
      angle = angle + 180;
    } else if(X_diff < 0 && Y_diff < 0){ //in the third quadrant
      angle = angle + 180;
    } else if(X_diff > 0 && Y_diff < 0){ //in the forth quadrant
      angle = angle + 360;
    }
  }

  return angle;
}

double Directed_Line_Segment::get_Length(){
  return sqrt(pow(terminal.x - initial.x, 2) + pow(terminal.y - initial.y, 2));
}

Line Directed_Line_Segment::get_Line(){
  return line;
}


//***************************************************************************
// int getIntersectionTwoLines(Line L1, Line L2, Point *p){
//     get intersection p of two lines L1 and L2.
//   Line L1: one line
//   Line L2: second line
//
// return  Point p: the intersection of two lines
//        p = (INFINITY, INFINITY): means there are infinity number of
//            intersections of the two lines
//***************************************************************************/
Point getIntersectionTwoLines(Line L1, Line L2){
  Point p(0, 0);
  float determinant = L1.A * L2.B - L2.A * L1.B;
  if(determinant == 0) {
  	p.x = INFINITY;
  	p.y = INFINITY;
  	return p;
  }

  p.x = (L1.C * L2.B - L2.C * L1.B)/determinant;
  p.y = (L1.A * L2.C - L2.A * L1.C)/determinant;
	return p;
}

double getSign(double x){
  if (x > 0){
    return 1;
  }
  if (x < 0){
    return -1;
  }
  return 0;
}

double abs_double(double value){
  if (value < 0){
    return (-1 * value);
  }
  return value;
}

//cut a specified number in a range (limit)
//positive or negative limit does not affect result
double truncateNumber(double a, double limit){
  double sign = getSign(a);
  a = std::abs(a);
  limit = std::abs(limit);
  if(a > limit){
      return limit * sign;
  } else { //limit < 0
      return a * sign;
  }
}

double max_abs_4_numbers (double a, double b, double c, double d){
  a = std::abs(a);
  b = std::abs(b);
  c = std::abs(c);
  d = std::abs(d);

  if(a < b){
    a = b;
  }
  if(a < c){
    a = c;
  }
  if(a < d){
    a = d;
  }
  return a;
}

double max_abs_4_numbers (DriveSpeedConfig con){
  double a = std::abs(con.leftBackMotorSpeed);
  double b = std::abs(con.leftFrontMotorSpeed);
  double c = std::abs(con.rightBackMotorSpeed);
  double d = std::abs(con.rightFrontMotorSpeed);

  if(a < b){
    a = b;
  }
  if(a < c){
    a = c;
  }
  if(a < d){
    a = d;
  }
  return a;
}

DriveSpeedConfig assignDriveSpeed(double leftFront, double rightFront, double leftBack, double rightBack){
  DriveSpeedConfig con;
  con.leftFrontMotorSpeed = leftFront;
  con.leftBackMotorSpeed = leftBack;
  con.rightFrontMotorSpeed = rightFront;
  con.rightBackMotorSpeed = rightBack;
  return con;
}

DriveSpeedConfig add_two_speed_config(DriveSpeedConfig config_1, DriveSpeedConfig config_2){
  DriveSpeedConfig con;
  con.leftBackMotorSpeed = config_1.leftBackMotorSpeed + config_2.leftBackMotorSpeed;
  con.leftFrontMotorSpeed = config_1.leftFrontMotorSpeed + config_2.leftFrontMotorSpeed;
  con.rightBackMotorSpeed = config_1.rightBackMotorSpeed + config_2.rightBackMotorSpeed;
  con.rightFrontMotorSpeed = config_1.rightFrontMotorSpeed + config_2.rightFrontMotorSpeed;
  return con;
}

DriveSpeedConfig add_three_speed_config(DriveSpeedConfig con1, DriveSpeedConfig con2, DriveSpeedConfig con3){
  DriveSpeedConfig con;
  con.leftBackMotorSpeed = con1.leftBackMotorSpeed + con2.leftBackMotorSpeed + con3.leftBackMotorSpeed;
  con.leftFrontMotorSpeed = con1.leftFrontMotorSpeed + con2.leftFrontMotorSpeed + con3.leftFrontMotorSpeed;
  con.rightBackMotorSpeed = con1.rightBackMotorSpeed + con2.rightBackMotorSpeed + con3.rightBackMotorSpeed;
  con.rightFrontMotorSpeed = con1.rightFrontMotorSpeed + con2.rightFrontMotorSpeed + con3.rightFrontMotorSpeed;
  return con;
}


DriveSpeedConfig proportional_enlarge_speed_config(DriveSpeedConfig config, double speedLimit){
  DriveSpeedConfig con = {0,0,0,0};
  double maxSpeed = max_abs_4_numbers(config);
  speedLimit = std::abs(speedLimit);

  if(speedLimit == 0){
    return con;
  }

  if(maxSpeed == 0){
    return con;
  }

  if(maxSpeed <= speedLimit){
    con.leftBackMotorSpeed = config.leftBackMotorSpeed/maxSpeed*speedLimit;
    con.leftFrontMotorSpeed = config.leftFrontMotorSpeed/maxSpeed*speedLimit;
    con.rightBackMotorSpeed = config.rightBackMotorSpeed/maxSpeed*speedLimit;
    con.rightFrontMotorSpeed = config.rightFrontMotorSpeed/maxSpeed*speedLimit;
  } else if(maxSpeed > speedLimit){
    con = proportional_projection_one_speed_config(config, speedLimit);
  }
  return con;
}


DriveSpeedConfig proportional_projection_one_speed_config(DriveSpeedConfig config, double speedLimit){
  DriveSpeedConfig con = {0,0,0,0};
  double maxSpeed = max_abs_4_numbers(config);
  speedLimit = std::abs(speedLimit);

  if(speedLimit == 0){
    return con;
  }

  if(maxSpeed == 0){
    return con;
  } else if(maxSpeed <= speedLimit){
    con.leftBackMotorSpeed = config.leftBackMotorSpeed;
    con.leftFrontMotorSpeed = config.leftFrontMotorSpeed;
    con.rightBackMotorSpeed = config.rightBackMotorSpeed;
    con.rightFrontMotorSpeed = config.rightFrontMotorSpeed;
  } else {
    con.leftBackMotorSpeed = speedLimit * config.leftBackMotorSpeed / maxSpeed;
    con.leftFrontMotorSpeed = speedLimit * config.leftFrontMotorSpeed / maxSpeed;
    con.rightBackMotorSpeed = speedLimit * config.rightBackMotorSpeed / maxSpeed;
    con.rightFrontMotorSpeed = speedLimit * config.rightFrontMotorSpeed / maxSpeed;
  }
  return con;
}

DriveSpeedConfig proportional_projection_two_speed_config(DriveSpeedConfig config_1,
                                                          DriveSpeedConfig config_2, double maxSpeed){
  DriveSpeedConfig con1;
  DriveSpeedConfig con2;
  DriveSpeedConfig con;
  maxSpeed = std::abs(maxSpeed);
  con1 = proportional_projection_one_speed_config(config_1, maxSpeed);
  con2 = proportional_projection_one_speed_config(config_2, maxSpeed);
  con = add_two_speed_config(con1, con2);
  con = proportional_projection_one_speed_config(con, maxSpeed);
  return con;
}


DriveSpeedConfig proportional_projection_three_speed_config(DriveSpeedConfig config_1,
                                                            DriveSpeedConfig config_2,
                                                            DriveSpeedConfig config_3, double maxSpeed){
  DriveSpeedConfig con1;
  DriveSpeedConfig con2;
  DriveSpeedConfig con3;
  DriveSpeedConfig con;
  maxSpeed = std::abs(maxSpeed);
  con1 = proportional_projection_one_speed_config(config_1, maxSpeed);
  con2 = proportional_projection_one_speed_config(config_2, maxSpeed);
  con3 = proportional_projection_one_speed_config(config_3, maxSpeed);
  con = add_three_speed_config(con1, con2, con3);
  con = proportional_projection_one_speed_config(con, maxSpeed);
  return con;
}


double degreesToRadians(double degree){
  return (degree * 3.1415926/180);
}

double radiansToDegrees(double radians){
  return (radians * 180 / 3.1415926);
}

double distanceToDegrees(double distance_cm, double wheel_diameter_inch){
  return 360 * distance_cm / (3.1415926 * wheel_diameter_inch * 2.54);
}

double degreesToDistanceCm(double degree, double wheel_diameter_inch){
  return degree / 360 * (3.1415926 * wheel_diameter_inch * 2.54);
}


/***************************************************************************
* double mod(double a, double b)
*   return positive reminder of a/b less than b
*
*   double a:  numerator
*   double b:  positive denominator
****************************************************************************/
double mod_lib(double a, double b){
	if(b == 0) {
		return -100;
	}
	b = std::abs(b);
	double tmp = (double)((long)(a/b));
	tmp = a - tmp * b;
	if(tmp < 0) {
		tmp = tmp + b;
	}
	return tmp;
}

/***************************************************************************************************************************************
* double getNormalizedAngle(double angle_lib)
*   return value related to angle_lib which is mapped into 0 and 360 degree.
*
*   float angle_lib:
*   for example: getNormalizedAngle(370)  will return 10
*                getNormalizedAngle(-350) will return 10
***************************************************************************************************************************************/
double getNormalizedAngle(double angle_lib){
	return mod_lib(angle_lib, 360);
}
