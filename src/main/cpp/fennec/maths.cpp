#include "maths.h"

#include <math.h>

v2 rotate(v2 vector, float angle)
{
    return v2
    {
        (float)(cos(-angle) * vector.x - sin(-angle) * vector.y),
        (float)(sin(-angle) * vector.x + cos(-angle) * vector.y)
    };
}

float dot(v2 a, v2 b)
{
    return a.x * b.x + a.y * b.y;
}

v2 leftPerpendicular(v2 vector)
{
    return v2 { -vector.y, vector.x };
}

v2 rightPerpendicular(v2 vector)
{
    return v2 { vector.y, -vector.x };
}

float sign(float x)
{
    if (x < 0) return -1;
    if (x > 0) return 1;
    return 0;
}

float length(v2 v) {
    return sqrtf(v.x * v.x + v.y * v.y);
}

float lengthSq(v2 v) {
    return v.x*v.x +v.y*v.y;
}

float degToRad(float degrees)
{
    return degrees*PI/180.0;
}

float radToDeg(float radians)
{
    return radians/(PI/180.0);
}




bool solveIK(v2 solutions[], float arm1_length, float arm2_length, v2 target_point)
{

    if (length(target_point) > (arm1_length + arm2_length))
        return false;

    if (length(target_point) < (arm1_length - arm2_length))
        return false;

    v2 origin_to_target_direction   = normalize(target_point);
    v2 origin_to_target_tangent     = rightPerpendicular(origin_to_target_direction);

    float arm1_sq = arm1_length*arm1_length;
    float arm2_sq = arm2_length*arm2_length;


    float distance_along_direction = 
        (arm2_sq- arm1_sq - lengthSq(target_point)) 
        /
        (-2 * length(target_point));

    float distance_along_tangent =
        sqrtf(arm1_sq - distance_along_direction*distance_along_direction);

    if (solutions)
    {
        solutions[0] = origin_to_target_direction * distance_along_direction
                        + origin_to_target_tangent   * distance_along_tangent;

        solutions[1] = origin_to_target_direction * distance_along_direction
                        - origin_to_target_tangent   * distance_along_tangent;
    }
    return true;
}



v2 transform(mat2x2 mat, v2 vec)
{
	v2 result {
		mat.m[0] * vec.x + mat.m[1] * vec.y,
		mat.m[2] * vec.x + mat.m[3] * vec.y,
	};
	return result;
}


float angleBetween(v2 from, v2 to)
{
	from = normalize(from);
	to = normalize(to);

    

	float angle = acos(CLAMP(dot(from, to), -1, 1));
  if (dot(to, rightPerpendicular(from)) < 0) {
		angle = -angle;
  }
  return angle;
}


float leastAngularError(float current_value, float target_value )
{
    float error_1 = target_value - current_value;
    float error_2 = target_value - (current_value + 2 * M_PI) ;
    float error_3 = target_value - (current_value - 2 * M_PI) ;
    float error = (fabs(error_1) < fabs(error_2)) ? error_1: error_2;
    error = (fabs(error) < fabs(error_3)) ? error: error_3;

    return error;
}
