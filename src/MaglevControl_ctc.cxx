#include "MaglevControl.h"
#include "TimeHandler.h"

#define MAX_MAGIDX 31
#define MAX_POWIDX 1
#define FORCE_START_INDEX 0
#define FORCE_STOP_INDEX 6
#define MAX_FORCE_INTEGRAL 10.0
#define MAX_POSITION_INTEGRAL 1.0
#define USE_CTC 0
#define ROTATE_COG 0

using namespace std;

extern int tick_callback_handler( ml_device_handle_t maglev_handle,  ml_position_t *maglev_position );

MaglevControl::MaglevControl()
{
	mp_gain_type = ML_GAINSET_TYPE_NORMAL;
	gain_scale = 100.0;
	
	for (int i = 0; i < DATA_SIZE; i++)
	{
	        integral[i] = 0.0;
	        desired_force[i] = 0.0;
                desired_position[i] = 0.0;
	        raw_force_signal[i] = 0.0;
	        temperature[i] = 0.0;
                currents[i] = 0.0;
                force_control_directions[i] = default_force_control_directions[i];
        }
        desired_force[2] = -1.0;
        
	click_per_sec = clickPerSecond();
        
	sprintf ( force_gain_file_name, "force_gains.txt" );
	
	// Set up the velocity filter parameters
	double delta_time = 0.001;
	cutoff_frequency = 200.0; // Hz
	tau = 1.0 / (2.0 * PI * cutoff_frequency);
	alpha = tau / (tau + delta_time);
	
	flag_save_force = 0;
	flag_maglev_start = false;
}//MaglevControl

MaglevControl::~MaglevControl()
{

}//~MaglevControl

void MaglevControl::maglevGetInternalGains()
{
	if (ml_GetGainVecAxes ( maglev_handle, mp_gain_type, &gain_vec) != 0)
	{
		cout << "Fatal ERROR -- Can not get the gain from the maglev!\n";
		exit(0);
	}
	
	for (int i=0; i<6; i++)
	{		
		if ( i<3)
		{
			//current_internal_gains[i][0] = gain_vec.values[i].p/gain_scale;
			current_internal_gains[i][0] = gain_vec.values[i].p;
			//gains[i][0] = position_gains[i][0]/gain_scale;
		} 
		else
		{
			current_internal_gains[i][0] = gain_vec.values[i].p;
			//gains[i][0] = position_gains[i][0];
		}
		current_internal_gains[i][1] = gain_vec.values[i].i;
		current_internal_gains[i][2] = gain_vec.values[i].d;
		current_internal_gains[i][3] = gain_vec.values[i].ff;
		//gains[i][1] = position_gains[i][1];
		//gains[i][2] = position_gains[i][2];
				
	}
	
}//maglevGetInternalGains

void
MaglevControl::maglevSetInternalGains()
{
        // Iterate to assign each direction's gain values
	for (int i=0; i<6; i++)
	{
	        // In the translational directions, scale the proportional gains
		if ( i<3 )
		{
		  	//gain_vec.values[i].p  = current_internal_gains[i][0]*gain_scale;
		  	gain_vec.values[i].p  = current_internal_gains[i][0];
		}
		else
		{
			gain_vec.values[i].p  = current_internal_gains[i][0];
		}
		  	
	  	gain_vec.values[i].i  = current_internal_gains[i][1];
		gain_vec.values[i].d  = current_internal_gains[i][2];
		gain_vec.values[i].ff = current_internal_gains[i][3];
	}
	ml_SetGainVecAxes ( maglev_handle, mp_gain_type, gain_vec );
}//maglevSetPositionGain

//not being called
/*
void
MaglevControl::maglevSetForceGain( double * gains)
{
	for ( int i=0; i<3; i++)
	{
		force_gains[0][i] = gains[i];
	}
}
*/
//Call when the controller starts.  The gains should have been saved in a txt file
void
MaglevControl::maglevInitGains()
//void
//MaglevControl::maglevInitForceGains()
{
	//Read in the gains from a file predefined
	FILE *pFile;
	pFile = fopen( force_gain_file_name, "r" );
	
	// Find the current values of the internal gains
	maglevGetInternalGains();
	
	// Set the current values for the arrays that will hold the "transition" gains
	for (int i = 0; i < 6; i++)
	{
		for (int j=0 ; j < 3; j++)
		{
			current_position_gains[i][j] = 0.0;
			current_force_gains[i][j] = 0.0;
		}
		current_position_gains[i][3] = 0.0;
	}
	
	// If we couldn't read the gain file, then just use the defaults in the maglev_parameters.h file
	if (pFile==NULL)
  	{
    	        printf("Cannot locate the initial force gain file %s, using hard-coded defaults\n", force_gain_file_name);
                fclose (pFile);
	        for (int i = 0; i < 6; i++)
	        {
		        for (int j=0 ; j < 3; j++)
		        {
		                desired_internal_gains[i][j] = 0.0;
			        desired_position_gains[i][j] = default_position_gains[i][j];
			        desired_force_gains[i][j] = default_force_gains[i][j];
		        }
	                desired_internal_gains[i][3] = 0.0;
		        desired_position_gains[i][3] = default_position_gains[i][3];
	        }
	}
	else
	{
	        // Read the force gains from the file
	        printf("Reading Force Gains\n================================\n");
	        for (int i = 0; i < 6; i++)
	        {
        	        float a;
		        printf("Direction %d: ||", i);
		        for (int j=0 ; j < 4; j++)
		        {
		                desired_internal_gains[i][j] = 0.0;
			        fscanf(pFile, "%f", &a);
			        desired_position_gains[i][j] = a;
			        printf("%7.2f |", a);
		        }
		        for (int j=0; j < 3; j++)
		        {
			        fscanf(pFile, "%f", &a);
			        desired_force_gains[i][j] = a;
			        printf("| %4.1f ", a);
		        }
		        printf("\n");
	        }
	        
	        // Read the line containing the "directions to control" information
	        printf("Controlling force in (");
	        for (int i = 0; i < DATA_SIZE; i++)
	        {
	                int a;
	                fscanf(pFile, "%d", &a);
	                if (a == 0)
	                {
	                        force_control_directions[i] = false;
                        }
                        else
                        {
	                        force_control_directions[i] = true;
	                        printf("%d", i);
                        }
	        }
                printf(")\n");
		
		// Close the file and announce completion
	        fclose (pFile);
	        printf("Done -- The force gains have been read in!\n");
        }
}//maglevInitGains

void
MaglevControl::maglevSaveGains()
{
	//Read in the gains from a file predefined
	FILE *pFile;
	pFile = fopen( force_gain_file_name, "w" );
	for (int i = 0; i < 6; i++)
	{
	        // Write the current direction's position gains
		for (int j=0 ; j < 4; j++)
		{
			fprintf(pFile, "%f ", current_position_gains[i][j]);
		}
		
		// Write the current direction's force gains
		for (int j=0; j < 3; j++)
		{
			fprintf(pFile, "%f ", current_force_gains[i][j]);
		}
		fprintf(pFile, "\n");
	}
	for (int i = 0; i< 6; i++)
	{
	        if (force_control_directions[i])
	        {
	                fprintf(pFile, "1 ");
                }
                else
                {
                        fprintf(pFile, "0 ");
                }
	}
	fclose (pFile);
	std::cout << "Done -- The current controller gains of the Maglev have been changed\n";
}//maglevSaveGains



void
MaglevControl::maglevSetFrequency( int f )
{
        // If the Maglev is off, do not run
	if ( !flag_maglev_start )
	{
                return;
        }
        
 	if ( ml_SetServoFrequency ( maglev_handle, (float) f ) )
 	{
 		cout << "Fatal ERROR -- Cannot set the freqency of the maglev!";
		exit(0);
	}
	control_freq = f;
	
	// Recalculate the filter proportionality constant
	double delta_time = 1.0 / control_freq;
	alpha = tau / (tau + delta_time);
	
	printf("Current frequency is %d.\n", f);
	printf("New velocity filter proportionality constant is %5.3f\n", alpha);
}//maglevSetFrequency

float
MaglevControl::maglevGetFrequency( )
{
	float a;
	if ( ml_GetServoFrequency ( maglev_handle, &a ) )
	{
 		cout << "Fatal ERROR -- Cannot get the freqency of the maglev!";
		exit(0);
	}
	control_freq = a;
	return ( a );
}//maglevGetFrequency


void
MaglevControl::maglevConnect (  )
{
 	ml_fault_t fault;
	std::cout << "Trying to connect to the maglev\n";
	
  	if (ml_Connect ( &maglev_handle, maglev_server_name ) != ML_STATUS_OK)
	{
    		printf("\nFailed connecting to %s. Wrong server address? Server down?\n",maglev_server_name);
    		exit( -1 );
	}

	flag_maglev_start = true;
	std::cout << "Done -- The maglev is connected\n0\t";
}//maglevConnect

void
MaglevControl::maglevClearError (  )
{
	ml_ResetFault (	maglev_handle );
	std::cout << "Done -- Errors are cleared\n";
}//maglevClearError

void
MaglevControl::maglevTakeOff ( bool thumb )
{
        // Attempt to take off until possible
        maglevGetPosition();
        double minPos = minValue(current_position);
        printf("(Pos=%7.3f, minPos=%7.3f)!\n", minPos, ABS_MIN_POS);
        while(minPos < ABS_MIN_POS)
        {
                current_fault.value = 1;
                while(current_fault.value != 0)
                {
                        ml_GetFault( maglev_handle, &current_fault );
                        ml_ResetFault(maglev_handle);
                        usleep(500000);
                        printf("\n\tCannot take off, sensor out of range or some other fault: %d, trying again...\n", current_fault.value);
                }
                printf("\nAttempting to take off...\n");
                ml_Takeoff(maglev_handle);
                maglevGetPosition();
                minPos = minValue(current_position);
                if(minPos < ABS_MIN_POS)
                {
                        printf("Failed (Pos=%7.3f, minPos=%7.3f)!\n", minPos, ABS_MIN_POS);
                }
                else
                {
                        printf("Success (Pos=%7.3f, minPos=%7.3f)!\n", minPos, ABS_MIN_POS);
                }
        }
        
        
	double startPosition[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double zPos, myTime;
	
	// Initialize the gains
	maglevInitGains();
	
	// Move the Maglev to the appropriate starting position
	float zStart;
	if(thumb)
	{
		zStart = 0.005;
		startPosition[0] = 0.001;
		for(zPos = startPosition[0]; zPos < zStart; zPos += 0.001)
		{
			startPosition[0] = zPos;
			maglevMove2Position(startPosition);
			myTime = currTimeMS(click_per_sec);
			while(currTimeMS(click_per_sec) < myTime + 200);
		}
	}
	else
	{
		zStart = -0.005;
		startPosition[2] = -0.001;
		for(zPos = startPosition[2]; zPos > zStart; zPos -= 0.001)
		{
			startPosition[2] = zPos;
			maglevMove2Position(startPosition);
			myTime = currTimeMS(click_per_sec);
			while(currTimeMS(click_per_sec) < myTime + 200);
		}
	}
	
        //maglevPrintGainMatrices();
        //double gravity[DATA_SIZE];
        //maglevGetGravityVector(gravity);
        //for (int dirIdx = 0; dirIdx < DATA_SIZE; dirIdx++)
        //{
        //        printf("%d | %5.2f\n", dirIdx, gravity[dirIdx]);
        //}
        //maglevCompensateForGravity();
        //maglevPrintGainMatrices();
}//maglevTakeOff

inline double MaglevControl::minValue(double my_array[6])
{
        double current_minimum = my_array[0];
        for(int i=1; i < 6; i++)
        {
                if(my_array[i] < current_minimum)
                {
                        current_minimum = my_array[i];
                }
        }
        return current_minimum;
} // minValue

void
MaglevControl::maglevLand (  )
{
	std::cout << "Landing the device\n";
	ml_Land( maglev_handle );
	std::cout << "Done -- The device has landed\n";
}//maglevLand


void
MaglevControl::maglevTurnOff ( )
{
	//Unregister the callback function from the server
	printf("Turning off the force controller\n");
	ml_UnregisterCallbackTick( maglev_handle );
	// Land the maglev
	printf("Landing the maglev\n");
	ml_Land( maglev_handle );
  	// disconnect from server
	printf("Turning off the maglev\n");
  	ml_Disconnect( maglev_handle );
  	
  	// Officially declare the Maglev off
  	flag_maglev_start = false;
  	std::cout << "Done -- The maglev is disconnected\n";
}//maglevTurnOff

// Gets velocity information from the Maglev
void
MaglevControl::maglevGetSpeed ( double * speed )
{
	double propConst = 0.95;
	ml_velocities_t read_speed;
	
	ml_GetVelocity ( maglev_handle, &read_speed );
	
	for ( int i=0; i<6; i++)
	{
		// Retrieves speed from the Maglev
		speed[i] = speed[i] * propConst + read_speed.values[i] * (1-propConst);
		
		// Calculates speed using backward difference derivative
/*		speed[i] = current_position[i] - position_old[i];
		position_old[i] = current_position[i];//*/
	}
}//maglevGetSpeed


void
MaglevControl::maglevStartForceController()
{
        // Declare variables
        double starting_internal_gains[6][4];
        double starting_position_gains[6][4];
        double starting_force_gains[6][3];
        bool gains_changed = false;
        
        // Pause for a second
        sleep(1);
        
	// Try to start saving the force before we start the controller:
	maglevSaveForce();
	
	// Unregister any existing callback function from the server
	ml_UnregisterCallbackTick( maglev_handle );
	
	// Register the callback function to the server
	ml_RegisterCallbackTick ( maglev_handle, tick_callback_handler );
	flag_force_control_start = true;
	std::cout << "Done -- The force controller starts\n";
	
	for(int i=0; i<6; i++)
	{
	        // Clear the force integral
	        integral[i] = 0.0;
	        
	        // Set the starting gain values for each type
		for(int j=0; j<3; j++)
		{
			starting_internal_gains[i][j] = current_internal_gains[i][j];
			starting_position_gains[i][j] = current_position_gains[i][j];
			starting_force_gains[i][j] = current_force_gains[i][j];
		}
		starting_internal_gains[i][3] = current_internal_gains[i][3];
		starting_position_gains[i][3] = current_position_gains[i][3];
	}
	
	// I added the following lines so the force should be saved as soon as the force controller is turned on.
/*	flag_save_force = 1;
	save_force_counter = 0;//*/
	// That didn't work so I tried this one:
	//maglevSaveForce();
	
	// Slowly progress to new gains
	double interpolation_constant;
	while(save_force_counter <= NUM_TRANSITIONS)
	{
	        if (gains_changed)
	        {
	                if ((save_force_counter % 10) == 0)
	                {
	                        gains_changed = false;
	                }
	        }
	        else
	        {
	                interpolation_constant = (double)save_force_counter / NUM_TRANSITIONS;
		        for(int i=FORCE_START_INDEX; i<FORCE_STOP_INDEX; i++)
		        {
			        for(int j=0; j<3; j++)
			        {
				        current_internal_gains[i][j] = starting_internal_gains[i][j] + (desired_internal_gains[i][j]-starting_internal_gains[i][j]) * interpolation_constant;
				        current_position_gains[i][j] = starting_position_gains[i][j] + (desired_position_gains[i][j]-starting_position_gains[i][j]) * interpolation_constant;
				        current_force_gains[i][j] = starting_force_gains[i][j] + (desired_force_gains[i][j]-starting_force_gains[i][j]) * interpolation_constant;
			        }
			        current_internal_gains[i][3] = starting_internal_gains[i][3] + (desired_internal_gains[i][3]-starting_internal_gains[i][3]) * interpolation_constant;
			        current_position_gains[i][3] = starting_position_gains[i][3] + (desired_position_gains[i][3]-starting_position_gains[i][3]) * interpolation_constant;
		        }
		        maglevSetInternalGains();
		        gains_changed = true;
	        }
		//printf("%03d | %7.2f\n", save_force_counter, current_internal_gains[0][0]);
	}
	//maglevPrintGainMatrices();
}//maglevStartForceController

void
MaglevControl::maglevStopForceController()
{
        // Declare needed variables
	double interpolation_constant;
	static int start_counter = save_force_counter;
        double starting_internal_gains[6][4];
        double starting_position_gains[6][4];
        double starting_force_gains[6][3];
        bool gains_changed = false;
	
        // Set the starting gains for interpolation
	for(int i=0; i<6; i++)
	{
	        // Set the gain values for each type
		for(int j=0; j<3; j++)
		{
		        // Starting gain values for transition gains
			starting_internal_gains[i][j] = current_internal_gains[i][j];
			starting_position_gains[i][j] = current_position_gains[i][j];
			starting_force_gains[i][j] = current_force_gains[i][j];
			
			// Desired gain values for transition gains
			desired_internal_gains[i][j] = default_internal_gains[i][j];
			desired_position_gains[i][j] = 0.0;
			desired_force_gains[i][j] = 0.0;
		}
		// Starting gain values (feed-forward)
		starting_internal_gains[i][3] = current_internal_gains[i][3];
		starting_position_gains[i][3] = current_position_gains[i][3];
		
		// Desired gain values feed-forward)
		desired_internal_gains[i][3] = default_internal_gains[i][3];
		desired_position_gains[i][3] = 0.0;
	}
	
	// Slowly progress to new gains
	printf("Stepping down the force controller...");
	while((save_force_counter - start_counter) <= NUM_TRANSITIONS)
	{
	        if (gains_changed)
	        {
	                if (((save_force_counter - start_counter) % 10) == 0)
	                {
	                        gains_changed = false;
	                }
	        }
	        else
	        {
	                interpolation_constant = (double)(save_force_counter - start_counter) / NUM_TRANSITIONS;
		        for(int i=FORCE_START_INDEX; i<FORCE_STOP_INDEX; i++)
		        {
			        for(int j=0; j<3; j++)
			        {
				        current_internal_gains[i][j] = starting_internal_gains[i][j] + (desired_internal_gains[i][j]-starting_internal_gains[i][j]) * interpolation_constant;
				        current_position_gains[i][j] = starting_position_gains[i][j] + (desired_position_gains[i][j]-starting_position_gains[i][j]) * interpolation_constant;
				        current_force_gains[i][j] = starting_force_gains[i][j] + (desired_force_gains[i][j]-starting_force_gains[i][j]) * interpolation_constant;
			        }
			        current_internal_gains[i][3] = starting_internal_gains[i][3] + (desired_internal_gains[i][3]-starting_internal_gains[i][3]) * interpolation_constant;
			        current_position_gains[i][3] = starting_position_gains[i][3] + (desired_position_gains[i][3]-starting_position_gains[i][3]) * interpolation_constant;
		        }
		        maglevSetInternalGains();
		        //printf("%03d | %7.2f\n", save_force_counter, current_internal_gains[0][0]);
		        gains_changed = true;
	        }
	}
	printf("Done!\n");
	//maglevPrintGainMatrices();
	
	// Completely stop the force controller
	ml_UnregisterCallbackTick( maglev_handle );
	flag_force_control_start = false;
	std::cout << "Done -- The force controller stops\n";
}//maglevStopForceController

/*
	const double gains[][6] = {{ 2400, 0.0,   100, -0.0, -200.0, 0.0},
			   {  750, 0.0,    30, -0.0, -100.0, 0.0},
                           {10000, 0.0,   100, -0.0, -200.0,  0.0},
                           { 50.0, 0.0, 0.150, 0.0, 0.0, 0.0},
                           { 50.0, 0.0, 0.150, 0.0, 0.0, 0.0},
                           { 10.0, 0.0, 0.150, 0.0, 0.0, 0.0}};
*/

void
MaglevControl::maglevController ( Reading curr_force)
{
        // Variables needed for the controller
        double time_step, K_P, K_V, K_I;
        double mass = 0.555;
        double r_x = 0.000614;
        double r_y = 0.001225;
        double r_z = -0.031669;
        double gravity = 9.81;
        double cg_x, cg_y, cg_z;
        double velocity[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double proportional;
        double pid_output[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double maglev_input[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        //double mass_vector[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        //double nonlinear_vector[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double gravity_vector[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        
        // Calculate information that depends on previous loop
        if (save_force_counter == 0)
        {
                // Assume zero velocity, zero time step from previous counter
                time_step = 0.0;
        }
        else
        {
                // Calculate time step
                time_step = curr_force.time - save_force_data[save_force_counter-1][0];
        }
        
        // Store the current position of the Maglev
        maglevGetPosition();
        
        // For debugging purposes, store the force, currents and coil temperatures
        //maglevGetForce();
        //maglevGetCurrent();
        //maglevGetTemperature();
        
	// Implement the controller in each direction
	for (int dirIdx=FORCE_START_INDEX; dirIdx < FORCE_STOP_INDEX; dirIdx++)
	{
                // Calculate information that depends on previous loop
                if (save_force_counter == 0)
                {
                        // Assume zero velocity since we have no prior data
                        velocity[dirIdx] = 0.0;
                }
                else
                {
                        // Calculate velocity using first-order backwards difference derivative
                        velocity[dirIdx] = (current_position[dirIdx] - save_force_data[save_force_counter-1][dirIdx+19]) / time_step;
                        
                        // Filter the velocity using a first-order low-pass filter
                        velocity[dirIdx] = alpha * velocity[dirIdx] + (1.0 - alpha) * save_force_data[save_force_counter-1][dirIdx+37];
                }
                
                // Switch depending on force control direction
                if (force_control_directions[dirIdx])
                {
                        // Extract force gains from matrix
                        K_P = current_force_gains[dirIdx][0];        
                        K_I = current_force_gains[dirIdx][1];
                        K_V = current_force_gains[dirIdx][2];
                        
                        // Calculate proportional error
                        proportional = desired_force[dirIdx] - curr_force.force[dirIdx];
                        
                        // Calculate integral term
                        if (K_I == 0.0)
                        {
                                integral[dirIdx] = 0.0;
                        }
                        else
                        {
                                if (fabs(integral[dirIdx]) <= MAX_FORCE_INTEGRAL)
                                {
                                        integral[dirIdx] += proportional * time_step;
                                }
                        }
                        
                        // Calculate controller term
                        /*if (USE_CTC)
                        {*/
                                pid_output[dirIdx] = -(K_P * proportional + K_I * integral[dirIdx] + desired_force[dirIdx]) - K_V * velocity[dirIdx];
                        /*}
                        else
                        {
                                pid_output[dirIdx] = K_P * proportional + K_I * integral[dirIdx] - K_V * velocity[dirIdx] + desired_force[dirIdx];
                        }*/
                }
                else
                {
                        // Extract position gains from matrix
                        K_P = current_position_gains[dirIdx][0];        
                        K_I = current_position_gains[dirIdx][1];
                        K_V = current_position_gains[dirIdx][2];
                        
                        // Calculate proportional error
                        //proportional[dirIdx] = desired_position[dirIdx] - current_position[dirIdx];
                        proportional = desired_position[dirIdx] - current_position[dirIdx];
                        
                        // Calculate integral term
                        if (K_I == 0.0)
                        {
                                integral[dirIdx] = 0.0;
                        }
                        else
                        {
                                if (fabs(integral[dirIdx]) <= MAX_POSITION_INTEGRAL)
                                {
                                        integral[dirIdx] += proportional * time_step;
                                }
                        }
                        
                        // Calculate controller term
                        pid_output[dirIdx] = K_P * proportional + K_I * integral[dirIdx] - K_V * velocity[dirIdx];
                }
        }//*/
        
        if (ROTATE_COG)
        {
                // Do not rotate the center of gravity vector
                cg_x = r_x;
                cg_y = r_y;
                cg_z = r_z;
        }
        else
        {
                // Calculate rotated center of gravity vector
                /*double sin3 = current_position[3];
                double sin4 = current_position[4];
                double sin5 = current_position[5];
                double cos3 = 1.0-0.5*current_position[3]*current_position[3];
                double cos4 = 1.0-0.5*current_position[4]*current_position[4];
                double cos5 = 1.0-0.5*current_position[5]*current_position[5];
                cg_x = (cos4*cos5 + sin3*sin4*sin5)*r_x + (sin3*sin4*cos5 - cos4*sin5)*r_y + (cos3*sin5)*r_z;
                cg_y = (cos3*sin5)*r_x + (cos3*cos5)*r_y - (sin3)*r_z;
                cg_z = (sin3*cos4*sin5 - cos4*cos5)*r_x + (cos3*cos4*cos5 + sin4*sin5)*r_y + (cos3*cos5)*r_z;//*/
                cg_x = ((1.0-0.5*current_position[4]*current_position[4])*(1.0-0.5*current_position[5]*current_position[5]) + (current_position[3])*(current_position[4])*(current_position[5]))*r_x + ((current_position[3])*(current_position[4])*(1.0-0.5*current_position[5]*current_position[5]) - (1.0-0.5*current_position[4]*current_position[4])*(current_position[5]))*r_y + ((1.0-0.5*current_position[3]*current_position[3])*(current_position[5]))*r_z;
                cg_y = ((1.0-0.5*current_position[3]*current_position[3])*(current_position[5]))*r_x + ((1.0-0.5*current_position[3]*current_position[3])*(1.0-0.5*current_position[5]*current_position[5]))*r_y - ((current_position[3]))*r_z;
                cg_z = ((current_position[3])*(1.0-0.5*current_position[4]*current_position[4])*(current_position[5]) - (1.0-0.5*current_position[4]*current_position[4])*(1.0-0.5*current_position[5]*current_position[5]))*r_x + ((1.0-0.5*current_position[3]*current_position[3])*(1.0-0.5*current_position[4]*current_position[4])*(1.0-0.5*current_position[5]*current_position[5]) + (current_position[4])*(current_position[5]))*r_y + ((1.0-0.5*current_position[3]*current_position[3])*(1.0-0.5*current_position[5]*current_position[5]))*r_z;
        }
        
        // Calculate terms in the gravity vector
        gravity_vector[2] = mass*gravity;
        gravity_vector[3] = -mass*gravity*cg_y;
        gravity_vector[4] = mass*gravity*cg_x;
        
        if (USE_CTC)
        {
                // Computed Torque Control
                // Declare flotor parameter estimates for computed torque control
                double inertia_xx = 0.001686;
                double inertia_xy = 0.000038;
                double inertia_xz = 0.000000;
                double inertia_yy = 0.001736;
                double inertia_yz = 0.000013;
                double inertia_zz = 0.000870;
                
                // Calculate terms in the M*u vector
                mass_vector[0] = mass*(pid_output[0] + cg_z*pid_output[4] - cg_y*pid_output[5]);
                mass_vector[1] = mass*(pid_output[1] + cg_x*pid_output[5] - cg_z*pid_output[3]);
                mass_vector[2] = mass*(pid_output[2] + cg_y*pid_output[3] - cg_x*pid_output[4]);
                mass_vector[3] = mass*(cg_y*pid_output[2] - cg_z*pid_output[1]) + (inertia_xx*pid_output[3] + inertia_xy*pid_output[4] + inertia_xz*pid_output[5]);
                mass_vector[4] = mass*(cg_z*pid_output[0] - cg_x*pid_output[2]) + (inertia_xy*pid_output[3] + inertia_yy*pid_output[4] + inertia_yz*pid_output[5]);
                mass_vector[5] = mass*(cg_x*pid_output[1] - cg_y*pid_output[0]) + (inertia_xz*pid_output[3] + inertia_yz*pid_output[4] + inertia_zz*pid_output[5]);
                
                // Calculate terms in the nonlinear vector
                nonlinear_vector[0] = mass*(cg_x*(velocity[4]*velocity[4] + velocity[5]*velocity[5]) - cg_y*velocity[3]*velocity[4] - cg_z*velocity[3]*velocity[5]);
                nonlinear_vector[1] = mass*(cg_y*(velocity[3]*velocity[3] + velocity[5]*velocity[5]) - cg_x*velocity[3]*velocity[4] - cg_z*velocity[4]*velocity[5]);
                nonlinear_vector[2] = mass*(cg_z*(velocity[3]*velocity[3] + velocity[4]*velocity[4]) - cg_x*velocity[3]*velocity[5] - cg_y*velocity[4]*velocity[5]);
                nonlinear_vector[3] = inertia_yz*(velocity[4]*velocity[4]-velocity[5]*velocity[5]) + velocity[4]*velocity[5]*(inertia_zz-inertia_yy) + velocity[3]*(inertia_xz*velocity[4] - inertia_xy*velocity[5]);
                nonlinear_vector[4] = inertia_xz*(velocity[5]*velocity[5]-velocity[3]*velocity[3]) + velocity[3]*velocity[5]*(inertia_xx-inertia_zz) + velocity[4]*(inertia_xy*velocity[5] - inertia_yz*velocity[3]);
                nonlinear_vector[5] = inertia_xy*(velocity[3]*velocity[3]-velocity[4]*velocity[4]) + velocity[3]*velocity[4]*(inertia_yy-inertia_xx) + velocity[5]*(inertia_yz*velocity[3] - inertia_xz*velocity[4]);
                
                // Calculate computed torque control inputs to Maglev
                maglev_input[0] = mass_vector[0] - nonlinear_vector[0];
                maglev_input[1] = mass_vector[1] - nonlinear_vector[1];
                maglev_input[2] = mass_vector[2] - nonlinear_vector[2] + gravity_vector[2];
                maglev_input[3] = mass_vector[3] - nonlinear_vector[3] + gravity_vector[3];
                maglev_input[4] = mass_vector[4] - nonlinear_vector[4] + gravity_vector[4];
                maglev_input[5] = mass_vector[5] - nonlinear_vector[5];
        }
        else
        {
                // PID control (static feed-forward gain to prevent instability)
                /*maglev_input[0] = pid_output[0] + current_position_gains[0][3];
                maglev_input[1] = pid_output[1] + current_position_gains[1][3];
                maglev_input[2] = pid_output[2] + current_position_gains[2][3];
                maglev_input[3] = pid_output[3] + current_position_gains[3][3];
                maglev_input[4] = pid_output[4] + current_position_gains[4][3];
                maglev_input[5] = pid_output[5] + current_position_gains[5][3];//*/
                
                // PID control (with Gravity compensation feedback)
                maglev_input[0] = pid_output[0];
                maglev_input[1] = pid_output[1];
                maglev_input[2] = pid_output[2] + gravity_vector[2];
                maglev_input[3] = pid_output[3] + gravity_vector[3];
                maglev_input[4] = pid_output[4] + gravity_vector[4];
                maglev_input[5] = pid_output[5];
        }
        
	// Save the data in each direction
        // Store data in array
        if (save_force_counter < SAVE_FORCE_DATA)
        {
                // Save the time
                save_force_data[save_force_counter][0] = curr_force.time;
	        for (int dirIdx=FORCE_START_INDEX; dirIdx < FORCE_STOP_INDEX; dirIdx++)
	        {
                        save_force_data[save_force_counter][dirIdx+1] = desired_force[dirIdx];
                        save_force_data[save_force_counter][dirIdx+7] = curr_force.force[dirIdx];
                        save_force_data[save_force_counter][dirIdx+13] = desired_position[dirIdx];
                        save_force_data[save_force_counter][dirIdx+19] = current_position[dirIdx];
                        save_force_data[save_force_counter][dirIdx+25] = maglev_input[dirIdx];
                        save_force_data[save_force_counter][dirIdx+31] = nonlinear_vector[dirIdx];
                        save_force_data[save_force_counter][dirIdx+37] = velocity[dirIdx];
                        save_force_data[save_force_counter][dirIdx+43] = curr_force.raw_force_signal[dirIdx];
                        save_force_data[save_force_counter][dirIdx+49] = integral[dirIdx];
                        save_force_data[save_force_counter][dirIdx+55] = currents[dirIdx];
                        save_force_data[save_force_counter][dirIdx+61] = pid_output[dirIdx];
        	}
	        save_force_counter++;
        }
        
        // Set the force on the Maglev to follow the controller input
        maglevSetForce(maglev_input);
}//maglevController

//Get the position of maglev
void MaglevControl::maglevGetPosition()
{
	ml_position_t current_pos;
	ml_GetActualPosition( maglev_handle, &current_pos );
	int i;
	for ( i = 0; i < 6; i++ )
	{
		current_position[i] = current_pos.values[i];
	}
}//maglevGetPosition

//Set the position of maglev
int
MaglevControl::maglevMove2Position ( double * new_position )
{
	ml_position_t des_pos;
	
	int i;
	for ( i = 0; i < 6; i++ )
	{
		//First, check if the position and orientation are valid
		if ( new_position[i] > maglev_boundary_left[i] && new_position[i] < maglev_boundary_right[i] )
		{
			std::cout << "Moving to " << new_position[i] << " which is in the boundary\n";
			des_pos.values[i] = new_position[i];
		}
		else
		{
			if ( new_position[i] < maglev_boundary_left[i] ) 
				des_pos.values[i] = maglev_boundary_left[i];
			else
				des_pos.values[i] = maglev_boundary_right[i];
				
			std::cout << "The new position " << i << " is out of boundary and ";
			std::cout << "the new position is " << new_position[i] << "\n";
			return 0;
		}
	}
	//std::cout << "HERE is new position " << new_position[2] << "\n";
	ml_SetDesiredPosition( maglev_handle, des_pos );
	return 1;
}//maglevMove2Position

//Set the force of maglev
void
MaglevControl::maglevSetForce ( double * new_force )
{
	ml_forces_t des_force;
	int i;
	for ( i = 0; i < 6; i++ )
	{
		des_force.values[i] = new_force[i];
	}
	ml_SetForces( maglev_handle, des_force );
}//maglevSetForce

//Get the current set of forces and torques of maglev
void MaglevControl::maglevGetForce ()
{
	ml_forces_t current_forces;
	ml_GetForces( maglev_handle, &current_forces);
	int i;
	for ( i = 0; i < 6; i++ )
	{
		maglev_force[i] = current_forces.values[i];
	}
}//maglevGetForce

//Get the current set of coil currents of maglev
void MaglevControl::maglevGetCurrent ()
{
	ml_currents_t current_currents;
	ml_GetForces( maglev_handle, &current_currents);
	int i;
	for ( i = 0; i < 6; i++ )
	{
		currents[i] = current_currents.values[i];
	}
}//maglevGetForce

//Get the current set of coil temperatures of maglev
void MaglevControl::maglevGetTemperature ()
{
	ml_temps_t current_temperatures;
	ml_GetForces( maglev_handle, &current_temperatures);
	int i;
	for ( i = 0; i < 6; i++ )
	{
		temperature[i] = current_temperatures.values[i];
	}
}//maglevGetForce

void 
MaglevControl::maglevSaveForce ( )
{
	//Save 5 second of force
	//
	save_force_counter = 0;
/*	
	if (save_force_data[0] != NULL)
	{
		for ( int i = 0; i<SF_LEN; i++)
		{
			printf("Deleting old data number %d\n", i);
			delete []save_force_data[i];
		}

	}
	//save_force_data = new long long int[SAVE_FORCE_MAX];
	
	for ( int i = 0; i< SF_LEN; i++ )
	{
		printf("Creating new data number %d\n", i);
		save_force_data[i] = new double[SAVE_FORCE_MAX];
	}*/
	
	flag_save_force = true;
	std::cout << "Start to save ... \n";
}//maglevSaveForce

void 
MaglevControl::maglevStopSaveForce ()
{
	//Open the file
	std::cout << "Saving the data into a file\n";
	save_force_file = fopen( "force_saved.txt", "w" ); 
	std::cout << "Saving " << save_force_counter << " of a possible " << SAVE_FORCE_DATA << " points of force data ";
	if (save_force_counter > SAVE_FORCE_DATA)
	{
	        save_force_counter = SAVE_FORCE_DATA;
        }
	for ( int i=0; i < save_force_counter; i++)
	{
		fprintf(save_force_file, "%lf ", save_force_data[i][0]);
		for ( int j = 1; j<SAVE_FORCE_COLS; j++)
		{
			fprintf(save_force_file, "%f ", save_force_data[i][j]);
		}
		fprintf(save_force_file, "\n");
	}
	fclose( save_force_file );
	std::cout << "\nDone\n";
	flag_save_force = 0;
/*	for ( int i = 0; i<SF_LEN; i++)
	{
		delete []save_force_data[i];
	}*/
	save_force_counter = 0;
}//maglevStopSaveForce

void MaglevControl::maglevPrintGainMatrices()
{
        // Make sure the current_internal_gains matrix is up-to-date
        maglevGetInternalGains();
        
        // Print a header for the gain table
        printf("Dir||  Int Kp  |  Int Ki  |  Int Kv  |  Int Kff ||  F2P Kp  |  F2P Ki  |  F2P Kv  |  F2P Kff || F2F Kp| F2F Ki| F2F Kv|\n");
        printf("---++----------+----------+----------+----------++----------+----------+----------+----------++-------+-------+-------+\n");
        
        // Iterate through each direction and print each gain matrix
        for (int dirIdx=0; dirIdx<DATA_SIZE; dirIdx++)
        {
                // Print the direction
                printf(" %d ||", dirIdx);
                
                // Print the internal position controller gains
                for (int gainIdx=0; gainIdx<4; gainIdx++)
                {
                        printf(" %8.2f |", current_internal_gains[dirIdx][gainIdx]);
                }
                
                // Print the external position controller gains
                for (int gainIdx=0; gainIdx<4; gainIdx++)
                {
                        printf("| %8.2f ", current_position_gains[dirIdx][gainIdx]);
                }
                
                // Print the force controller gains
                printf("|");
                for (int gainIdx=0; gainIdx<3; gainIdx++)
                {
                        printf("| %5.2f ", current_force_gains[dirIdx][gainIdx]);
                }
                printf("|\n");
        }
} // maglevPrintGainMatrices

void MaglevControl::maglevCompensateForGravity()
{
        // Gravity vector object
        ml_forces_t gravity;
        
        // Get the gravity vector currently being used by the Maglev
        ml_GetGravity(maglev_handle, &gravity);
        printf("Current MLHD Gravity Vector:\n");
        for (int dirIdx = 0; dirIdx < DATA_SIZE; dirIdx++)
        {
                printf("%d | %5.2f\n", dirIdx, gravity.values[dirIdx]);
        }
        
        // Find the orientation of the gravity vector in the current frame
        ml_FindGravity(maglev_handle, &gravity);
        printf("Actual Gravity Vector:\n");
        for (int dirIdx = 0; dirIdx < DATA_SIZE; dirIdx++)
        {
                printf("%d | %5.2f\n", dirIdx, gravity.values[dirIdx]);
        }
        
        // Set the gravity vector
        ml_SetGravity(maglev_handle, gravity);
        printf("Gravity vector set\n");
        
        // Tell the MLHD to use the gravity vector
        ml_DefyGravity(maglev_handle);
        printf("Maglev is now using the gravity vector\n");
}

void MaglevControl::maglevGetGravityVector(double gravity_vector[DATA_SIZE])
{
        // Gravity vector object
        ml_forces_t gravity;
        
        // Get the gravity vector currently being used by the Maglev
        ml_GetGravity(maglev_handle, &gravity);
        for (int dirIdx = 0; dirIdx < DATA_SIZE; dirIdx++)
        {
                gravity_vector[dirIdx] = gravity.values[dirIdx];
        }
}

void MaglevControl::maglevPrintOtherGains()
{
        // Extract the LOCKED gains
	if (ml_GetGainVecAxes ( maglev_handle, ML_GAINSET_TYPE_LOCKED, &gain_vec) != 0)
	{
		cout << "Fatal ERROR -- Can not get the LOCKED gains from the maglev!\n";
		exit(0);
	}
	
        // Print a header for the gain table
        printf("=================  LOCKED GAINS  =================\n");
        printf("Dir||  Int Kp  |  Int Ki  |  Int Kv  |  Int Kff ||\n");
        printf("---++----------+----------+----------+----------++\n");
        
        // Iterate through each direction and print each gain matrix
        for (int dirIdx=0; dirIdx<DATA_SIZE; dirIdx++)
        {
                // Print the direction
                printf(" %d ||", dirIdx);
                
        	// Print the LOCKED gains
                printf(" %8.2f |", gain_vec.values[dirIdx].p);
                printf(" %8.2f |", gain_vec.values[dirIdx].i);
                printf(" %8.2f |", gain_vec.values[dirIdx].d);
                printf(" %8.2f |", gain_vec.values[dirIdx].ff);
                
                // Print an end-of-line marker
                printf("|\n");
        }
        
        // Extract the LOCKED gains
	if (ml_GetGainVecAxes ( maglev_handle, ML_GAINSET_TYPE_CONSTRAINED, &gain_vec) != 0)
	{
		cout << "Fatal ERROR -- Can not get the CONSTRAINED gains from the maglev!\n";
		exit(0);
	}
	
        // Print a header for the gain table
        printf("===============  CONSTRAINED GAINS  ==============\n");
        printf("Dir||  Int Kp  |  Int Ki  |  Int Kv  |  Int Kff ||\n");
        printf("---++----------+----------+----------+----------++\n");
        
        // Iterate through each direction and print each gain matrix
        for (int dirIdx=0; dirIdx<DATA_SIZE; dirIdx++)
        {
                // Print the direction
                printf(" %d ||", dirIdx);
                
        	// Print the LOCKED gains
                printf(" %8.2f |", gain_vec.values[dirIdx].p);
                printf(" %8.2f |", gain_vec.values[dirIdx].i);
                printf(" %8.2f |", gain_vec.values[dirIdx].d);
                printf(" %8.2f |", gain_vec.values[dirIdx].ff);
                
                // Print an end-of-line marker
                printf("|\n");
        }
        
        // Extract the LOCKED gains
	if (ml_GetGainVecAxes ( maglev_handle, ML_GAINSET_TYPE_BOUNDARY, &gain_vec) != 0)
	{
		cout << "Fatal ERROR -- Can not get the BOUNDARY gains from the maglev!\n";
		exit(0);
	}
	
        // Print a header for the gain table
        printf("==================  BOUNDARY GAINS  ==============\n");
        printf("Dir||  Int Kp  |  Int Ki  |  Int Kv  |  Int Kff ||\n");
        printf("---++----------+----------+----------+----------++\n");
        
        // Iterate through each direction and print each gain matrix
        for (int dirIdx=0; dirIdx<DATA_SIZE; dirIdx++)
        {
                // Print the direction
                printf(" %d ||", dirIdx);
                
        	// Print the LOCKED gains
                printf(" %8.2f |", gain_vec.values[dirIdx].p);
                printf(" %8.2f |", gain_vec.values[dirIdx].i);
                printf(" %8.2f |", gain_vec.values[dirIdx].d);
                printf(" %8.2f |", gain_vec.values[dirIdx].ff);
                
                // Print an end-of-line marker
                printf("|\n");
        }
}//maglevPrintOtherGains

