#ifndef _MAGLEV_PARAMETERS_H_
#define _MAGLEV_PARAMETERS_H_

//#include <time.h>
//#include <sys/time.h>
//#include "rdtsc.h"

// Constants for use only with the Maglev
#ifndef PI
#define PI 3.141592
#endif /* PI */

#define DATA_SIZE 6
#define ABS_MIN_POS -0.005
#define TIME_MAX 10000

// Constants related only to the Maglev
const char maglev_server_name[] = "192.168.0.2";
// const char force_save_file_name[] = "../data_00_0_00x/force_saved.txt"; 
const double position_boundary = 0.012;
const double rotation_boundary = PI * 8.0 / 180.0;
const double maglev_boundary_left[6] = {-position_boundary, -position_boundary, -position_boundary, -rotation_boundary, -rotation_boundary, -rotation_boundary};
const double maglev_boundary_right[6] = {position_boundary, position_boundary, position_boundary, rotation_boundary, rotation_boundary, rotation_boundary};

const bool default_force_control_directions[6] = {true, true, true, false, false, false};

/******************************************************************************/
// Gain Matrix Section
//
// This section contains several Gain Matrices.  Each Gain Matrix has the
// following characteristics.  The rows correspond to directions (translational
// or rotational.  The columns correspond to the gain types.  This is organized
// as follows:
// 
//        P-gain | I-gain | D/V-gain | FF-gain
// --------------+--------+----------+--------
// x-dir         |        |          |        
// --------------+--------+----------+--------
// y-dir         |        |          |        
// --------------+--------+----------+--------
// z-dir         |        |          |        
// --------------+--------+----------+--------
// x-rot         |        |          |        
// --------------+--------+----------+--------
// y-rot         |        |          |        
// --------------+--------+----------+--------
// z-rot         |        |          |        
// --------------+--------+----------+--------
// 

// PIDF gains used for the internal position controller
// These are loaded using the ml_SetGainVecAxes function
const double default_internal_gains[][4] = {
                        { 2400.0, 0.0, 15.00, 0.0},
                        { 2400.0, 0.0, 15.00, 0.0},
                        {10000.0, 0.0, 15.00, 3.5},
                        {   50.0, 0.0,  0.15, 0.0},
                        {   50.0, 0.0,  0.15, 0.0},
                        {   10.0, 0.0,  0.15, 0.0}};

// PIVF gains used for the "new" position controller
// This is our controller
const double default_position_gains[][4] = {
                        { 1600.0, 0.0, 15.00, 0.000000},
                        { 1600.0, 0.0, 15.00, 0.000000},
                        { 7000.0, 0.0, 15.00, 5.673596},
                        {   30.0,20.0,  0.15, 0.000000},
                        {   30.0,20.0,  0.15, 0.000000},
                        {   10.0, 0.0,  0.15, 0.000000}};

// PIV force controller gains
// This is our controller
const double default_force_gains[][4] = {
                        { 0.5,20.0, 5.0, 0.0},
                        { 0.5,20.0, 5.0, 0.0},
                        { 0.5,40.0,20.0, 0.0},
                        { 0.0, 0.0, 0.0, 0.0},
                        { 0.0, 0.0, 0.0, 0.0},
                        { 0.0, 0.0, 0.0, 0.0}};

// PIDF gains used for the other internal controllers (boundary, locked,
//      constrained).  These are loaded using the ml_SetGainVecAxes function.
//      The z-direction feed-forward gain is only used for the locked axis.
const double default_gains_other[][4] = {
                        { 4800.0, 0.0, 15.00, 0.0},
                        { 4800.0, 0.0, 15.00, 0.0},
                        {15000.0, 0.0, 15.00, 3.5},
                        {   50.0, 0.0,  0.15, 0.0},
                        {   50.0, 0.0,  0.15, 0.0},
                        {   15.0, 0.0,  0.15, 0.0}};

// End of Gain Matrix Section
/******************************************************************************/

#endif


