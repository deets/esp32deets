#include "madgwick.hh"

#include <math.h>

//---------------------------------------------------------------------------------------------------
// Definitions

#define betaDef		0.1f		// 2 * proportional gain
namespace {

#pragma GCC diagnostic ignored "-Wstrict-aliasing"
float inv_sqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
#pragma GCC diagnostic pop

} // namespace


MadgwickAHRS::MadgwickAHRS(float sample_freq)
  :  beta(betaDef)
  , q0(1.0f)
  , q1(0.0f)
  , q2(0.0f)
  , q3(0.0f)
  , sample_freq(sample_freq)
{
}



void MadgwickAHRS::update_imu(float gx, float gy, float gz, float ax, float ay, float az)
{
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-this->q1 * gx - this->q2 * gy - this->q3 * gz);
  qDot2 = 0.5f * (this->q0 * gx + this->q2 * gz - this->q3 * gy);
  qDot3 = 0.5f * (this->q0 * gy - this->q1 * gz + this->q3 * gx);
  qDot4 = 0.5f * (this->q0 * gz + this->q1 * gy - this->q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * this->q0;
    _2q1 = 2.0f * this->q1;
    _2q2 = 2.0f * this->q2;
    _2q3 = 2.0f * this->q3;
    _4q0 = 4.0f * this->q0;
    _4q1 = 4.0f * this->q1;
    _4q2 = 4.0f * this->q2;
    _8q1 = 8.0f * this->q1;
    _8q2 = 8.0f * this->q2;
    q0q0 = this->q0 * this->q0;
    q1q1 = this->q1 * this->q1;
    q2q2 = this->q2 * this->q2;
    q3q3 = this->q3 * this->q3;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * this->q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * this->q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * this->q3 - _2q1 * ax + 4.0f * q2q2 * this->q3 - _2q2 * ay;
    recipNorm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= this->beta * s0;
    qDot2 -= this->beta * s1;
    qDot3 -= this->beta * s2;
    qDot4 -= this->beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  this->q0 += qDot1 * (1.0f / this->sample_freq);
  this->q1 += qDot2 * (1.0f / this->sample_freq);
  this->q2 += qDot3 * (1.0f / this->sample_freq);
  this->q3 += qDot4 * (1.0f / this->sample_freq);

  // Normalise quaternion
  recipNorm = inv_sqrt(this->q0 * this->q0 + this->q1 * this->q1 + this->q2 * this->q2 + this->q3 * this->q3);
  this->q0 *= recipNorm;
  this->q1 *= recipNorm;
  this->q2 *= recipNorm;
  this->q3 *= recipNorm;
}


void MadgwickAHRS::compute_angles(float& roll, float& pitch, float& yaw)
{
  roll = atan2f(this->q0*this->q1 + this->q2*this->q3, 0.5f - this->q1*this->q1 - this->q2*this->q2) *  57.29578f;
  pitch = asinf(-2.0f * (this->q1*this->q3 - this->q0*this->q2)) * 57.29578f;;
  yaw = atan2f(this->q1*this->q2 + this->q0*this->q3, 0.5f - this->q2*this->q2 - this->q3*this->q3) * 57.29578f + 180.0f;
}
