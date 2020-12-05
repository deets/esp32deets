// Copyright: 2020, Diez B. Roggisch, Berlin, all rights reserved
#pragma once

// see http://www.cs.ndsu.nodak.edu/~siludwig/Publish/papers/SPIE20181.pdf

struct MadgwickAHRS
{
  float beta;				// algorithm gain
  float q0, q1, q2, q3;
  float sample_freq;

  MadgwickAHRS(float sample_freq);
  void update_imu(float gx, float gy, float gz, float ax, float ay, float az);
  void compute_angles(float& roll, float& pitch, float& yaw);

};
