/**********************************************************************************************
* The classes and functions in this library are implemented in math_lib.cpp
**********************************************************************************************/
#include "main.h"

struct DriveSpeedConfig
{
  double leftFrontMotorSpeed;
  double rightFrontMotorSpeed;
  double leftBackMotorSpeed;
  double rightBackMotorSpeed;
};

class Point
{
    public:
        double x;
        double y;

        Point() {
          x = 0;
          y = 0;
        };

        Point(double xx, double yy) {
          x = xx;
          y = yy;
        };

        Point(Point& p){
          x = p.getX();
          y = p.getY();
        };

        double getX() {return x;};

        double getY() {return y;};

        void setX(double xx) {x = xx;};

        void setY(double yy) {y = yy;};

        double getLength(Point p);
        double getHeadingToPoint_Degree(Point p);
        double getAngle(Point p);
        std::string toString();
};

class Line
{
    public:
        //using Ax + By + C = 0
        double A;
        double B;
        double C;

        Line();
        Line(Point p1, double slope);
        Line(Point p1, Point p2);
        double getSlope();
        Point getIntercection(Line line);
};


class Directed_Line_Segment
{
    public:
        Point initial;
        Point terminal;
        Line line;

        Directed_Line_Segment();
        Directed_Line_Segment(Point initialPoint, Point terminalPoint);
        double get_Normalized_Angle();
        double get_Length();
        Line get_Line();
};


/***************************************************************************
* double mod(double a, double b)
*   return positive reminder of a/b less than b
*
*   double a:  numerator
*   double b:  positive denominator
****************************************************************************/
double mod_lib(double a, double b);

/***************************************************************************************************************************************
* double getNormalizedAngle(double angle_lib)
*   return value related to angle_lib which is mapped into 0 and 360 degree.
*
*   float angle_lib:
*   for example: getNormalizedAngle(370)  will return 10
*                getNormalizedAngle(-350) will return 10
***************************************************************************************************************************************/
double getNormalizedAngle(double angle_lib);

double degreesToRadians(double degree);
double radiansToDegrees(double degree);

double distanceToDegrees(double distance_cm, double wheel_diameter_inch);
double degreesToDistanceCm(double degree, double wheel_diameter_inch);


double getSign(double x);

double abs_double(double value);

double truncateNumber(double a, double limit);

double max_abs_4_numbers (DriveSpeedConfig con);

double max_abs_4_numbers (double a, double b, double c, double d);

DriveSpeedConfig add_two_speed_config(DriveSpeedConfig config_1, DriveSpeedConfig config_2);

DriveSpeedConfig add_three_speed_config(DriveSpeedConfig config_1, DriveSpeedConfig config_2, DriveSpeedConfig config_3);

DriveSpeedConfig proportional_enlarge_speed_config(DriveSpeedConfig config, double speedLimit);

DriveSpeedConfig proportional_projection_one_speed_config(DriveSpeedConfig config, double speedLimit);

DriveSpeedConfig proportional_projection_two_speed_config(DriveSpeedConfig config_1,
                                                          DriveSpeedConfig config_2, double maxSpeed);


DriveSpeedConfig proportional_projection_three_speed_config(DriveSpeedConfig config_1,
                                                            DriveSpeedConfig config_2,
                                                            DriveSpeedConfig config_3, double maxSpeed);

DriveSpeedConfig assignDriveSpeed(double leftFront, double rightFront, double leftBack, double rightBack);

DriveSpeedConfig get_diagonal_moving_config(double movingDirection, double currentHeading, double maxSpeed);

double convert_target_to_relative_angle_lib(double currentHeading, double targetHeading);

double convert_to_relative_angle_by_turn_direction_lib(double currentHeading, double targetHeading, int turnDirection);
