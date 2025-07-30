/********************************************************************
* Description: pm_vector.h
*  Vector structs and functions (generalizes PmCartesian)
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author: Robert W. Ellenberg
* License: GPL Version 2
* System: Linux
*
* Copyright (c) 2015 All rights reserved.
********************************************************************/
#ifndef PM_VECTOR_H
#define PM_VECTOR_H
#include "posemath.h"
#include "rtapi_bool.h"
#include "pm_vector_types.h"

#ifdef __cplusplus
extern "C" {
#endif

extern const PmVector PmVector_zero;

PmVector VecCopyFrom(PmVector const * const from);

PmVector VecVecAdd(PmVector v1, PmVector const * const v2);
int VecVecAddEq(PmVector * const v, PmVector const * const v2);

PmVector VecVecSub(PmVector v1, PmVector const * const v2);
int VecVecSubEq(PmVector * const v, PmVector const * const v2);
PmVector VecScalMult(PmVector v1, double s);
int VecScalMultEq(PmVector * const v, double s);

double VecVecDot(PmVector const * const v1, PmVector const * const v2);
double VecMag(PmVector const * const v1);
double VecMagSq(PmVector const * const v1);
double VecVecDisp(PmVector const * const v1, PmVector const * const v2);

int VecUnit(PmVector const * const v1, PmVector * const out);
int VecUnitEq(PmVector * const v1);
int VecAbs(PmVector const * const v1, PmVector * const out);

int VecVecDirection(
    PmVector const * const to,
    PmVector const * const from,
    PmVector * const u);

double VecMin(PmVector const * const v1);
double VecMax(PmVector const * const v1);
double VecAbsMax(PmVector const * const v1);

int CartToVec(
    PmCartesian const * const p_xyz,
    PmCartesian const * const p_abc,
    PmCartesian const * const p_uvw,
    PmVector * const out);

int VecSetXYZ(
    PmCartesian const * const p_xyz,
    PmVector * const out);

int VecSetABC(
    PmCartesian const * const p_abc,
    PmVector * const out);

int VecToCart(
    PmVector const * const vec,
    PmCartesian * p_xyz,
    PmCartesian * p_abc,
    PmCartesian * p_uvw);

int VecVecOrthonormal(
    PmVector const * const a,
    PmVector const * const b,
    PmVector * const u,
    PmVector * const v);

PmVector VecVecHarmonicAddSq(PmVector u,
    PmVector const * const v);

int VecVecUnitParallel(PmVector const * const u,
    PmVector const * const v,
    double threshold_sq);

int VecVecUnitAntiParallel(
    PmVector const * const u,
    PmVector const * const v,
    double threshold);

bool VecHasNAN(PmVector const * const v);
double VecVLimit(PmVector const * const uVec, double v_target, double v_limit_linear, double v_limit_angular);

#ifdef __cplusplus
}				/* matches extern "C" for C++ */
#endif

#endif
