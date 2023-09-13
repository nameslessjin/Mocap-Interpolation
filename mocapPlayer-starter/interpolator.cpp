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


void Interpolator::LinearInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;

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
  int inputLength = pInputMotion->GetNumFrames();

  int startKeyframe = 0;


  // Skip N frames
  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // control points for bezier
    int prevStartKeyframe = startKeyframe - N - 1;
    int nextEndKeyframe = endKeyframe + N + 1;
    Posture *prevStartPosture = nullptr, *nextEndPosture = nullptr;
    Posture postureA, postureB;

    if (startKeyframe == 0)
    {
      // startKeyFrame is p1, endKeyframe is p2, nextEndKeyframe is p3
      nextEndPosture = pInputMotion->GetPosture(nextEndKeyframe);

      // special case, need a1 and bn
      // a1 = lerp(q1, lerp(q3, q2, 2.0), 1.0f / 3) for root
      postureA.root_pos = Lerp(1.0f / 3, startPosture->root_pos, Lerp(2.0, nextEndPosture->root_pos, endPosture->root_pos));

      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; ++bone)
      {
        postureA.bone_rotation[bone] = Lerp(1.0f / 3, startPosture->bone_rotation[bone], Lerp(2.0, nextEndPosture->bone_rotation[bone], endPosture->bone_rotation[bone]));
      }

      // an_- = lerp(lerp(qn-1, qn, 2.0), qn+1, 0.5)
      // bn = lerp(qn, an_, -1.0 /3)
      vector an_hat = Lerp(0.5, Lerp(2.0, startPosture->root_pos, endPosture->root_pos), nextEndPosture->root_pos);
      postureB.root_pos = Lerp(-1.0f / 3, endPosture->root_pos, an_hat);
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; ++bone)
      {
        an_hat = Lerp(0.5, Lerp(2.0, startPosture->bone_rotation[bone], endPosture->bone_rotation[bone]), nextEndPosture->bone_rotation[bone]);
        postureB.bone_rotation[bone] = Lerp(-1.0f / 3, endPosture->bone_rotation[bone], an_hat);
      }
    }
    else if (nextEndKeyframe >= inputLength)
    {
      //  prevStartPosture is pn-1, startPosture is pn, endPosture is pn+1
      prevStartPosture = pInputMotion->GetPosture(prevStartKeyframe);

      // an_- = lerp(lerp(qn-1, qn, 2.0), qn+1, 0.5)
      // an = lerp(qn, an_, 1.0 / 3)
      vector an_hat = Lerp(0.5, Lerp(2.0, prevStartPosture->root_pos, startPosture->root_pos), endPosture->root_pos);
      postureA.root_pos = Lerp(1.0f / 3, startPosture->root_pos, an_hat);
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; ++bone)
      {
        an_hat = Lerp(0.5, Lerp(2.0, prevStartPosture->bone_rotation[bone], startPosture->bone_rotation[bone]), endPosture->bone_rotation[bone]);
        postureA.bone_rotation[bone] = Lerp(1.0f / 3, startPosture->bone_rotation[bone], an_hat);
      }

      // special case
      // bn = lerp(qn, lerp(qn-2, qn-1, 2.0), 1.0f / 3) for getting the last one.  since we have 1 offset so:
      // bn+1 = lerp(qn+1, lerp(qn-1, qn, 2.0), 1.0f / 3) 
      postureB.root_pos = Lerp(1.0f / 3, endPosture->root_pos, Lerp(2.0, prevStartPosture->root_pos, startPosture->root_pos));
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; ++bone)
      {
        postureB.bone_rotation[bone] = Lerp(1.0f / 3, endPosture->bone_rotation[bone], Lerp(2.0, prevStartPosture->bone_rotation[bone], startPosture->bone_rotation[bone]));
      }
    }
    else
    {
      // prevStartPosture is pn-1, startKeyFrame is pn, endKeyframe is pn+1, nextEndKeyframe is pn+2
      prevStartPosture = pInputMotion->GetPosture(prevStartKeyframe);
      nextEndPosture = pInputMotion->GetPosture(nextEndKeyframe);

      // an_- = lerp(lerp(qn-1, qn, 2.0), qn+1, 0.5)
      // an = lerp(qn, an_, 1.0 / 3)
      vector an_hat = Lerp(0.5, Lerp(2.0, prevStartPosture->root_pos, startPosture->root_pos), endPosture->root_pos);
      postureA.root_pos = Lerp(1.0f / 3, startPosture->root_pos, an_hat);
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; ++bone)
      {
        an_hat = Lerp(0.5, Lerp(2.0, prevStartPosture->bone_rotation[bone], startPosture->bone_rotation[bone]), endPosture->bone_rotation[bone]);
        postureA.bone_rotation[bone] = Lerp(1.0f / 3, startPosture->bone_rotation[bone], an_hat);
      }

      // bn+1 = lerp(qn+1, an+1_, -1.0 /3)
      an_hat = Lerp(0.5, Lerp(2.0, startPosture->root_pos, endPosture->root_pos), nextEndPosture->root_pos);
      postureB.root_pos = Lerp(-1.0f / 3, endPosture->root_pos, an_hat);
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; ++bone)
      {
        an_hat = Lerp(0.5, Lerp(2.0, startPosture->bone_rotation[bone], endPosture->bone_rotation[bone]), nextEndPosture->bone_rotation[bone]);
        postureB.bone_rotation[bone] = Lerp(-1.0f / 3, endPosture->bone_rotation[bone], an_hat);
      }
    }

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = DeCasteljauEuler(t, startPosture->root_pos, postureA.root_pos, postureB.root_pos, endPosture->root_pos);

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
        interpolatedPosture.bone_rotation[bone] = DeCasteljauEuler(t, startPosture->bone_rotation[bone], postureA.bone_rotation[bone], postureB.bone_rotation[bone], endPosture->bone_rotation[bone]);

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));

}

void Interpolator::LinearInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;

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
      interpolatedPosture.root_pos = Lerp(t, startPosture->root_pos, endPosture->root_pos);

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
      {
        // get bone rotation angles, build quaternion with them
        double startAngles[3], endAngles[3], intermediateAngles[3];
        Quaternion<double> startQ, endQ;
        startPosture->bone_rotation[bone].getValue(startAngles);
        endPosture->bone_rotation[bone].getValue(endAngles);
        Euler2Quaternion(startAngles, startQ);
        Euler2Quaternion(endAngles, endQ);

        // calculate the interpolation with slerp and convert back to euler
        Quaternion<double> intermediateQ = Slerp(t, startQ, endQ);
        Quaternion2Euler(intermediateQ, intermediateAngles);
        
        interpolatedPosture.bone_rotation[bone] = vector(intermediateAngles);
      }

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::BezierInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int inputLength = pInputMotion->GetNumFrames();

  int startKeyframe = 0;

  // Skip N frames
  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // control points for bezier
    int prevStartKeyframe = startKeyframe - N - 1;
    int nextEndKeyframe = endKeyframe + N + 1;
    Posture *prevStartPosture = nullptr, *nextEndPosture = nullptr;
    Posture postureA, postureB;

    // for bone rotation angles and bone quaternion
    double startAngles[3], endAngles[3], prevStartAngles[3], nextEndAngles[3], intermediateAngles[3], aAngles[3], bAngles[3];
    Quaternion<double> startQ, endQ, prevStartQ, nextEndQ, tmpQ, intermediateQ, an_hat_q, qA, qB;

    if (startKeyframe == 0)
    {
      // startKeyFrame is q1/qn-1, endKeyframe is q2/qn, nextEndKeyframe is q3/qn+1
      nextEndPosture = pInputMotion->GetPosture(nextEndKeyframe);

      // special case, need a1 and bn
      // a1 = lerp(q1, slerp(q3, q2, 2.0), 1.0f / 3) for root
      postureA.root_pos = Lerp(1.0f / 3, startPosture->root_pos, Lerp(2.0, nextEndPosture->root_pos, endPosture->root_pos));

      // an_- = slerp(slerp(qn-1, qn, 2.0), qn+1, 0.5)
      // bn = slerp(qn, an_, -1.0 /3)
      vector an_hat = Lerp(0.5, Lerp(2.0, startPosture->root_pos, endPosture->root_pos), nextEndPosture->root_pos);
      postureB.root_pos = Lerp(-1.0f / 3, endPosture->root_pos, an_hat);

      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; ++bone)
      {
        // get bone rotation angles, build quaternion with them
        startPosture->bone_rotation[bone].getValue(startAngles);
        endPosture->bone_rotation[bone].getValue(endAngles);
        nextEndPosture->bone_rotation[bone].getValue(nextEndAngles);
        Euler2Quaternion(startAngles, startQ);
        Euler2Quaternion(endAngles, endQ);
        Euler2Quaternion(nextEndAngles, nextEndQ);

        // calculate the interpolation with slerp and convert back to euler
        // for a1
        tmpQ = Slerp(2.0, nextEndQ, endQ);
        intermediateQ = Slerp(1.0f / 3, startQ, tmpQ);
        Quaternion2Euler(intermediateQ, intermediateAngles);
        postureA.bone_rotation[bone] = vector(intermediateAngles);

        // for b2
        tmpQ = Slerp(2.0, startQ, endQ);
        an_hat_q = Slerp(0.5, tmpQ, nextEndQ);
        intermediateQ = Slerp(-1.0f / 3, endQ, an_hat_q);
        Quaternion2Euler(intermediateQ, intermediateAngles);
        postureB.bone_rotation[bone] = vector(intermediateAngles);

      }
    }
    else if (nextEndKeyframe >= inputLength)
    {
      //  prevStartPosture is qn-1, startPosture is qn, endPosture is qn+1
      prevStartPosture = pInputMotion->GetPosture(prevStartKeyframe);

      // an_- = lerp(lerp(qn-1, qn, 2.0), qn+1, 0.5)
      // an = lerp(qn, an_, 1.0 / 3)
      vector an_hat = Lerp(0.5, Lerp(2.0, prevStartPosture->root_pos, startPosture->root_pos), endPosture->root_pos);
      postureA.root_pos = Lerp(1.0f / 3, startPosture->root_pos, an_hat);

      // special case
      // bn = lerp(qn, lerp(qn-2, qn-1, 2.0), 1.0f / 3) for getting the last one.  since we have 1 offset so:
      // bn+1 = lerp(qn+1, lerp(qn-1, qn, 2.0), 1.0f / 3) 
      postureB.root_pos = Lerp(1.0f / 3, endPosture->root_pos, Lerp(2.0, prevStartPosture->root_pos, startPosture->root_pos));
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; ++bone)
      {
        // get bone rotation angles, build intermediate quaternion with them
        prevStartPosture->bone_rotation[bone].getValue(prevStartAngles);
        startPosture->bone_rotation[bone].getValue(startAngles);
        endPosture->bone_rotation[bone].getValue(endAngles);
        Euler2Quaternion(startAngles, startQ);
        Euler2Quaternion(endAngles, endQ);
        Euler2Quaternion(prevStartAngles, prevStartQ);

        // for general an
        tmpQ = Slerp(2.0, prevStartQ, startQ);
        an_hat_q = Slerp(0.5, tmpQ, endQ);
        intermediateQ = Slerp(1.0f / 3, startQ, an_hat_q);
        Quaternion2Euler(intermediateQ, intermediateAngles);
        postureA.bone_rotation[bone] = vector(intermediateAngles);

        // for bn+1
        tmpQ = Slerp(2.0, prevStartQ, startQ);
        intermediateQ = Slerp(1.0f / 3, endQ, tmpQ);
        Quaternion2Euler(intermediateQ, intermediateAngles);
        postureB.bone_rotation[bone] = vector(intermediateAngles);
      }

    }
    else
    {
      // prevStartPosture is pn-1, startKeyFrame is pn, endKeyframe is pn+1, nextEndKeyframe is pn+2
      prevStartPosture = pInputMotion->GetPosture(prevStartKeyframe);
      nextEndPosture = pInputMotion->GetPosture(nextEndKeyframe);

      // an_- = lerp(lerp(qn-1, qn, 2.0), qn+1, 0.5)
      // an = lerp(qn, an_, 1.0 / 3)
      vector an_hat = Lerp(0.5, Lerp(2.0, prevStartPosture->root_pos, startPosture->root_pos), endPosture->root_pos);
      postureA.root_pos = Lerp(1.0f / 3, startPosture->root_pos, an_hat);

      // bn+1 = lerp(qn+1, an+1_, -1.0 /3)
      an_hat = Lerp(0.5, Lerp(2.0, startPosture->root_pos, endPosture->root_pos), nextEndPosture->root_pos);
      postureB.root_pos = Lerp(-1.0f / 3, endPosture->root_pos, an_hat);
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; ++bone)
      {
        // get bone rotation angles, build intermediate quaternion with them
        prevStartPosture->bone_rotation[bone].getValue(prevStartAngles);
        startPosture->bone_rotation[bone].getValue(startAngles);
        endPosture->bone_rotation[bone].getValue(endAngles);
        nextEndPosture->bone_rotation[bone].getValue(nextEndAngles);
        Euler2Quaternion(startAngles, startQ);
        Euler2Quaternion(endAngles, endQ);
        Euler2Quaternion(prevStartAngles, prevStartQ);
        Euler2Quaternion(nextEndAngles, nextEndQ);

        // for general an
        tmpQ = Slerp(2.0, prevStartQ, startQ);
        an_hat_q = Slerp(0.5, tmpQ, endQ);
        intermediateQ = Slerp(1.0f / 3, startQ, an_hat_q);
        Quaternion2Euler(intermediateQ, intermediateAngles);
        postureA.bone_rotation[bone] = vector(intermediateAngles);

        // for b2
        tmpQ = Slerp(2.0, startQ, endQ);
        an_hat_q = Slerp(0.5, tmpQ, nextEndQ);
        intermediateQ = Slerp(-1.0f / 3, endQ, an_hat_q);
        Quaternion2Euler(intermediateQ, intermediateAngles);
        postureB.bone_rotation[bone] = vector(intermediateAngles);

      }
    }

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = DeCasteljauEuler(t, startPosture->root_pos, postureA.root_pos, postureB.root_pos, endPosture->root_pos);

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
      {
        startPosture->bone_rotation[bone].getValue(startAngles);
        endPosture->bone_rotation[bone].getValue(endAngles);
        Euler2Quaternion(startAngles, startQ);
        Euler2Quaternion(endAngles, endQ);
        postureA.bone_rotation[bone].getValue(aAngles);
        postureB.bone_rotation[bone].getValue(bAngles);
        Euler2Quaternion(aAngles, qA);
        Euler2Quaternion(bAngles, qB);

        intermediateQ = DeCasteljauQuaternion(t, startQ, qA, qB, endQ);
        Quaternion2Euler(intermediateQ, intermediateAngles);

        interpolatedPosture.bone_rotation[bone] = vector(intermediateAngles);
      }

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));

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
  return Slerp(2.0, p, q);
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