#pragma once

// Note:
// *** DONT RENAME THIS FILE TO MATH.H!! ***
// math.h is taken so renaming by the C standard library,
// which will make this one will cause conflicts



// @Todo inline all of these

#include "math.h"

#define PI 3.14159265358979
#define INCH_TO_METER (1.0 / 39.37)

#define MIN(x, y) (((x)<(y))?(x):(y))
#define MAX(x, y) (((x)>(y))?(x):(y))
#define CLAMP(x, lower, upper) MAX(lower, MIN(upper, x))

float sign(float x);

float degToRad(float degrees);
float radToDeg(float radians);







struct v2
{
	float x;
  float y;    
};

// @Todo inline all of these

// Preforms a clockwise rotation in radians on the 2d vector
v2 rotate(v2 vector, float angle);

v2 leftPerpendicular(v2 vector);
v2 rightPerpendicular(v2 vector);


inline float mix(float a, float b, float t) { return a * (1 - t) + b * t; }
inline v2 mix(v2 a, v2 b, float t) {
	return v2 {
		mix(a.x, b.x, t),
		mix(a.y, b.y, t),
	};
}

float dot(v2 a, v2 b);
float length(v2 v);
float lengthSq(v2 v);

inline v2 operator-(v2 a) { return v2 { -a.x, -a.y }; }

inline v2 operator+(v2 a, v2 b) { return v2 { a.x + b.x, a.y + b.y }; }
inline v2 operator-(v2 a, v2 b) { return v2 { a.x - b.x, a.y - b.y }; }
inline v2 operator*(v2 a, v2 b) { return v2 { a.x * b.x, a.y * b.y }; }
inline v2 operator/(v2 a, v2 b) { return v2 { a.x / b.x, a.y / b.y }; }

inline v2 operator*(v2 a, float b) { return v2 { a.x * b, a.y * b }; }
inline v2 operator/(v2 a, float b) { return v2 { a.x / b, a.y / b }; }


inline v2 normalize(v2 v) { float l = length(v); if (l == 0) l = 1; return v / v2 { l, l }; }



struct v3
{
  float x;
  float y;
	float z;    
};

inline float length(v3 v) { return sqrtf(v.x*v.x+v.y*v.y+v.z*v.z); }

inline v3 operator+(v3 a, v3 b) { return v3 { a.x + b.x, a.y + b.y, a.z + b.z }; }
inline v3 operator/(v3 a, v3 b) { return v3 { a.x / b.x, a.y / b.y, a.z / b.z }; }

inline v3 normalize(v3 v) { float l = length(v); if (l == 0) l = 1; return v / v3 { l, l, l }; }




struct Pose
{
  v2    position;
  float rotation;
};

struct mat2x2
{
	float m[2 * 2];
};

v2 transform(mat2x2 mat, v2 vec);

inline mat2x2 inverse(mat2x2 mat)
{
	mat2x2 output = 
	{
		mat.m[3], -mat.m[1], 
		-mat.m[2], mat.m[0]
	};

	float inv_det = 1/(mat.m[0] * mat.m[3] - mat.m[1] * mat.m[2]);
	
	for (int i=0; i<4; ++i) 
		output.m[i] *= inv_det;

	return output;
}

bool solveIK(v2 solutions[], float arm1_length, float arm2_length, v2 target_point);


float angleBetween(v2 from, v2 to);

float leastAngularError(float current_value, float target_value );
