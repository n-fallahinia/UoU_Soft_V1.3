#ifndef MAGLEVCONTROL_H
#define MAGLEVCONTROL_H

#include "ml_api.h"
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "ForceSensor.h"
#include "maglev_parameters.h"

#define SAVE_FORCE_COLS 67
#define SAVE_FORCE_DATA 40000
#define NUM_TRANSITIONS 2000
#define MAGLEV_FREQUENCY 10.0
#define CONTROL_FREQUENCY 1000.0
#define OPERATING_FREQUENCY 1500.0

class MaglevControl
{
public:
        // Arrays of current gains to use in controllers
	double current_gains_force[6][4];
	double current_gains_position[6][4];
	double current_gains_internal[6][4];
	
	// Array of directions for use in forc control
	bool force_control_directions[6];
	
        // Define position and force variables
        ml_position_t current_position;
        ml_position_t desired_position;
	double desired_force[6];
        //double desired_position[6];
        
        // Define "flag" variable to track whether the temperature (0),
        //      current (1) or forces (2) exerted by the Maglev
        unsigned short int update_variable;
        
        // Flags to indicate whether the maglev is on and whether the force
        //      controller is on
	bool flag_maglev_start;
	bool flag_force_control_start;
	
	// Constructor/destructor
	MaglevControl();
	~MaglevControl();
	
	void maglevConnect (  );
	void maglevClearError (  );
	void maglevTakeOff ( bool thumb );
	void maglevLand (  );
	void maglevTurnOff ( );
	
	void maglevReadPositionGain(double [6][4]);
	void maglevGetInternalGains();
	void maglevSetInternalGains();
	//void maglevSetForceGain( double * gains);
	float maglevGetFrequency( );
	void maglevSetFrequency( int f );
	void maglevSetForce();
	void maglevSetSaveFileName( char folder_name[18] );
	
	void maglevStartForceController();
	void maglevStopForceController();
	bool maglevController ( Reading curr_force);
	//Tom This is what the above line used to say
	//void maglevController (  double * , double * );
	void maglevSaveGains();
	
	void maglevSaveForce();
	void maglevStopSaveForce();
	void maglevPrintGainMatrices();
	void maglevGetPosition();
	void maglevPrintOtherGains();
	void maglevGetTemperature();
	void maglevGetForce();
	void maglevGetCurrent();
	void maglevSetDesiredPosition();
private:
	// Arrays of gains to achieve after transitions
	double desired_gains_force[6][4];
	double desired_gains_position[6][4];
	double desired_gains_internal[6][4];
	
	// Arrays of gains to start with for the transition
	double starting_gains_internal[6][4];
        double starting_gains_position[6][4];
        double starting_gains_force[6][4];
        
	// Arrays of gains read from file
	double file_gains_force[6][4];
	double file_gains_position[6][4];
	
	// Stores the starting location of the transition
        int end_counter;
        
	// Values to keep track of controller parameters
	double time_old;
	double position_old[6];
	int controller_counter;
	double integral[6];
        ml_forces_t pid_force;
        double gravity_vector[3];
        double velocity[6];
        double rotation_matrix[3][3];
	
	// Maglev force controller frequency
	double control_freq;
	
	// Variables used to track Maglev properties
        ml_currents_t maglev_currents;
        ml_temps_t current_temperatures;
        ml_forces_t maglev_forces;
	ml_fault_t current_fault;
	ml_device_handle_t maglev_handle;
	ml_gainset_type_t mp_gain_type;
	ml_gain_vec_t gain_vec;
        
    // Name of file containing the force gains
	char force_gain_file_name[];
	char force_save_file_name[40];

	// Parameters for the low-pass velocity filter
	double alpha;
	double tau;
	double cutoff_frequency;
	
	//long long int *save_force_data;
	double save_force_data[SAVE_FORCE_DATA][SAVE_FORCE_COLS];
	bool flag_save_force;
	int save_force_counter;
	FILE *save_force_file;
	
	void maglevInitGains();
	//int tick_callback_handler( ml_device_handle_t, ml_position_t * );	
	bool maglevMove2Position ( double * new_position );
	bool maglevOutsideBoundary();
	inline double minValue(double my_array[6]);
	inline float minValue(float my_array[6]);
	void maglevCompensateForGravity();
	void maglevGetGravityVector(double gravity[DATA_SIZE]);
	void maglevZeroInternalGains();
	void maglevResetInternalGains();
};

#endif
