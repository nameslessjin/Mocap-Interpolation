#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <iostream>
#include "motion.h"
#include "interpolator.h"
#include "types.h"

Interpolator::Interpolator()
{
  //Set default interpolation type
  m_InterpolationType = LINEAR;

  //set default angle representation to use for interpolation
  m_AngleRepresentation = EULER;
}

Interpolator::~Interpolator()
{
}

//Create interpolated motion
void Interpolator::Interpolate(Motion * pInputMotion, Motion ** pOutputMotion, int N) 
{
  //Allocate new motion
  *pOutputMotion = new Motion(pInputMotion->GetNumFrames(), pInputMotion->GetSkeleton()); 

  //Perform the interpolation
  if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == EULER))
    LinearInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == QUATERNION))
    LinearInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == EULER))
    BezierInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == QUATERNION))
    BezierInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else
  {
    printf("Error: unknown interpolation / angle representation type.\n");
    exit(1);
  }
}

int bIndex = 0;
int startF = 0;
int endF = 0;
int axis = 0;
bool debug = false;

void Interpolator::LinearInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;

  if (debug) std::cout << "frame count: " << inputLength << " method: Linear Euler" << std::endl;

  // Skip N frames
  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
        interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1-t) + endPosture->bone_rotation[bone] * t;

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::Rotation2Euler(double R[9], double angles[3])
{
  double cy = sqrt(R[0]*R[0] + R[3]*R[3]);

  if (cy > 16*DBL_EPSILON) 
  {
    angles[0] = atan2(R[7], R[8]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = atan2(R[3], R[0]);
  } 
  else 
  {
    angles[0] = atan2(-R[5], R[4]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = 0;
  }

  for(int i=0; i<3; i++)
    angles[i] *= 180 / M_PI;
}

void Interpolator::Euler2Rotation(double angles[3], double R[9])
{
  double y = angles[0] * M_PI / 180.0f; // gamma
  double b = angles[1] * M_PI / 180.0f; // beta
  double a = angles[2] * M_PI / 180.0f; // alpha

  // RzRyRx
  R[0] = cos(a) * cos(b);
  R[1] = cos(a) * sin(b) * sin(y) - sin(a) * cos(y);
  R[2] = cos(a) * sin(b) * cos(y) + sin(a) * sin(y);
  R[3] = sin(a) * cos(b);
  R[4] = sin(a) * sin(b) * sin(y) + cos(a) * cos(y);
  R[5] = sin(a) * sin(b) * cos(y) - cos(a) * sin(y);
  R[6] = -sin(b);
  R[7] = cos(b) * sin(y);
  R[8] = cos(b) * cos(y);
}

void Interpolator::BezierInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this
}

void Interpolator::LinearInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this
}

void Interpolator::BezierInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this
}

void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double> & q) 
{
  // given euler angles, build rotation matrix, convert matrix to quaternion
  double R[9];
  Euler2Rotation(angles, R);
  q = q.Matrix2Quaternion(R);
  
}

void Interpolator::Quaternion2Euler(Quaternion<double> & q, double angles[3]) 
{
  // given quaternion, build rotation matrix, extract angles from matrix
  double R[9];
  q.Quaternion2Matrix(R);
  Rotation2Euler(R, angles);
}

Quaternion<double> Interpolator::Slerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd_)
{
  Quaternion<double> result;

  qStart.Normalize();
  qEnd_.Normalize();

  double cosTheta = qStart.dot(qEnd_);

  // handle quaternion double cover
  // quaternion handle the same rotation in two ways, q and -q are the same rotation
  // To ensure the shortest path is taken during interpolation, if cosTheta < 0, negate q2 and cosThea
  if (cosTheta < 0.0f)
  {
    cosTheta = -cosTheta;
    qStart = qStart * -1;
  }

  double theta = acos(cosTheta);

  result = (sin((1 - t) * theta) * qStart + sin(t * theta) * qEnd_) / sin(theta);

  // Handle Near zero dot product
  // This happens when cosTheta is close to 1, theta is close to 0
  // Tt means the quaternions are nearly parallel
  if (isnan(result.Gets()))
    result = Lerp(t, qStart, qEnd_);

  return result;
}

Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q)
{
  Quaternion<double> result;
  
  double cosTheta = p.dot(q);
  result = 2 * (cosTheta) * q - p;
  return result;
}

vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3)
{
  vector result;

  // Linear interpolation between control points
  vector q0 = Lerp(t, p0, p1);
  vector q1 = Lerp(t, p1, p2);
  vector q2 = Lerp(t, p2, p3);

  // Lienar interpolation on the new points
  vector r0 = Lerp(t, q0, q1);
  vector r1 = Lerp(t, q1, q2);

  // repeat the interpolation again
  result = Lerp(t, r0, r1);

  return result;
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3)
{

  // Similar to Euler, instead of LERP, we use SLERP
  Quaternion<double> result, q0, q1, q2, q3, r0, r1;

  q0 = Slerp(t, p0, p1);
  q1 = Slerp(t, p1, p2);
  q2 = Slerp(t, p2, p3);
  r0 = Slerp(t, q0, q1);
  r1 = Slerp(t, q1, q2);
  result = Slerp(t, r0, r1);

  return result;
}

Quaternion<double> Interpolator::Lerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd_)
{
  return (1 - t) * qStart + t * qEnd_;
}

vector Interpolator::Lerp(double t, vector start, vector end)
{
  return  start * (1 - t) + end * t;
}