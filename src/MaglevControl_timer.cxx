#include "MaglevControl.h"
#include "TimeHandler.h"

#define MAX_MAGIDX 31
#define MAX_POWIDX 1
#define FORCE_START_INDEX 0
#define FORCE_STOP_INDEX 6
#define MAX_FORCE_INTEGRAL 10.0
#define MAX_POSITION_INTEGRAL 1.0

using namespace std;

////////////////////////////////////////////////////////////////////////////////
// Callback Handler declarations
//      Set up here as external functions since the Maglev API cannot handle
//              them as part of the class (or at least I can't figure out how to
//              make it accept them as part of the class).
////////////////////////////////////////////////////////////////////////////////
extern int tick_callback_handler( ml_device_handle_t maglev_handle,  ml_position_t *maglev_position );
extern int fault_callback_handler( ml_device_handle_t maglev_handle,  ml_fault_t maglev_fault );
extern int temp_callback_handler( ml_device_handle_t maglev_handle, ml_temps_t *maglev_temps );
extern int boundary_callback_handler( ml_device_handle_t maglev_handle, ml_boundary_violation_t *maglev_boundary );

// Constructor for the Maglev
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
	
	control_freq = 1000.0;
	
	flag_save_force = 0;
	flag_maglev_start = false;
}//MaglevControl

MaglevControl::~MaglevControl()
{

}//~MaglevControl

// Retrieve the Maglev internal position controller gains
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

// Set the internal position controller gains
void MaglevControl::maglevSetInternalGains()
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
}//maglevSetInternalGains

// Read the Maglev gains from a data file
void MaglevControl::maglevInitGains()
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
        maglevPrintOtherGains();
}//maglevInitGains

// Save the current gains to a data file
void MaglevControl::maglevSaveGains()
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

// Set the force controller frequency
void
MaglevControl::maglevSetFrequency( int f )
{
        // If the Maglev is off, do not run
	if ( !flag_maglev_start )
	{
	        // Do not echo anything because this function is called by moving the slider bar
	        return;
        }
        
        // Set the maglev frequency, if within limits
        if ((f > 100) && (f < 10000))
        {
                control_freq = (double)f;
        }
        else
        {
                printf("Fatal ERROR -- Desired frequency is outside range!\n");
                exit(0);
        }
        
	// Recalculate the filter proportionality constant
	//double delta_time = 1.0 / control_freq;
	//alpha = tau / (tau + delta_time);
	
	printf("Current frequency is %d.\n", f);
	printf("New velocity filter proportionality constant is %5.3f\n", alpha);
}//maglevSetFrequency

// Get the force controller frequency
float
MaglevControl::maglevGetFrequency( )
{
	/*float a;
	if ( ml_GetServoFrequency ( maglev_handle, &a ) )
	{
 		cout << "Fatal ERROR -- Cannot get the freqency of the maglev!";
		exit(0);
	}
	control_freq = a;*/
	return ( control_freq );
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
	
	// Register the Tick Callback handler
	if (ml_RegisterCallbackTick(maglev_handle, tick_callback_handler) != ML_STATUS_OK)
	{
	        printf("\n\tFailed to properly register the Tick Callback handler!\n\n");
	}
	
	// Register the Fault Callback handler
	if (ml_RegisterCallbackFault(maglev_handle, fault_callback_handler) != ML_STATUS_OK)
	{
	        printf("\n\tFailed to properly register the Fault Callback handler!\n\n");
	}
	
	// Register the "OverTemp" Callback handler
	if (ml_RegisterCallbackOvertemp(maglev_handle, temp_callback_handler) != ML_STATUS_OK)
	{
	        printf("\n\tFailed to properly register the Overtemp Callback handler!\n\n");
	}
	
	// Register the "Boundary Violation" Callback handler
	if (ml_RegisterCallbackFlotorBoundaryViolation(maglev_handle, boundary_callback_handler) != ML_STATUS_OK)
	{
	        printf("\n\tFailed to properly register the Boundary Violation Callback handler!\n\n");
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

void MaglevControl::maglevTakeOff ( bool thumb )
{
        // Attempt to take off until possible
        ml_GetActualPosition( maglev_handle, &current_pos );
        float minPos = minValue(current_pos.values);
        printf("(Pos=%7.3f, minPos=%7.3f)!\n", minPos, ABS_MIN_POS);
        while(minPos < ABS_MIN_POS)
        {
                ml_GetFault( maglev_handle, &current_fault );
                ml_ResetFault(maglev_handle);
                usleep(500000);
                while(current_fault.value != ML_FAULT_TYPE_CLEAR)
                {
                        printf("\tCannot take off (");
                        fault_callback_handler(maglev_handle, current_fault);
                        ml_GetFault( maglev_handle, &current_fault );
                        ml_ResetFault(maglev_handle);
                        usleep(500000);
                }
                printf("\nAttempting to take off...\n");
                ml_Takeoff(maglev_handle);
                ml_GetActualPosition( maglev_handle, &current_pos );
                minPos = minValue(current_pos.values);
                if(minPos < ABS_MIN_POS)
                {
                        printf("Failed (Pos=%7.3f, minPos=%7.3f)!\n", minPos, ABS_MIN_POS);
                }
                else
                {
                        printf("Success (Pos=%7.3f, minPos=%7.3f)!\n", minPos, ABS_MIN_POS);
                }
        }
        
	// Initialize the gains
	maglevInitGains();
	
	// Set the Maglev operating frequency
	if ( ml_SetServoFrequency ( maglev_handle, OPERATING_FREQUENCY ) )
 	{
 		cout << "Fatal ERROR -- Cannot set the operating freqency of the Maglev!";
		exit(0);
	}
        
	float a;
	if ( ml_GetServoFrequency ( maglev_handle, &a ) )
	{
 		cout << "Fatal ERROR -- Cannot get the freqency of the maglev!";
		exit(0);
	}
	else
	{
	        printf("\tMaglev Servo Frequency is %5.2f\n", a);
        }
	
	// Move the Maglev to the appropriate starting position
	/*double startPosition[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double zPos, myTime;
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
	}//*/
	
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

inline float MaglevControl::minValue(float my_array[6])
{
        float current_minimum = my_array[0];
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
	// Turn off the force controller, if needed
	if (flag_force_control_start)
	{
        	maglevStopForceController();
        	flag_force_control_start = false;
	}
        maglevPrintGainMatrices();
        
        // Mark the maglev as turned off
  	flag_maglev_start = false;
	
        // Unregister the Fault Callback handler
	if (ml_UnregisterCallbackFault(maglev_handle) != ML_STATUS_OK)
	{
	        printf("\n\tFailed to properly unregister the Fault Callback handler!\n\n");
	}
	else
	{
	        printf("\tUnregistered the Fault Callback handler\n");
	}
	
        // Unregister the "Overtemp" Callback handler
	if (ml_UnregisterCallbackOvertemp(maglev_handle) != ML_STATUS_OK)
	{
	        printf("\n\tFailed to properly unregister the Overtemp Callback handler!\n\n");
	}
	else
	{
	        printf("\tUnregistered the Overtemp Callback handler\n");
	}
	
        // Unregister the Boundary Violation Callback handler
	if (ml_UnregisterCallbackFlotorBoundaryViolation(maglev_handle) != ML_STATUS_OK)
	{
	        printf("\n\tFailed to properly unregister the Boundary Violation Callback handler!\n\n");
	}
	else
	{
	        printf("\tUnregistered the Boundary Violation Callback handler\n");
	}
	
        // Unregister the Tick Callback handler
	if (ml_UnregisterCallbackTick(maglev_handle) != ML_STATUS_OK)
	{
	        printf("\n\tFailed to properly unregister the Tick Callback handler!\n\n");
	}
	else
	{
	        printf("\tUnregistered the Tick Callback handler\n");
	}
	
	float a;
	if ( ml_GetServoFrequency ( maglev_handle, &a ) )
	{
 		cout << "Fatal ERROR -- Cannot get the freqency of the maglev!";
		exit(0);
	}
	else
	{
	        printf("\tMaglev Servo Frequency is %5.2f\n", a);
        }
	
	// Land the maglev, if needed
        if (!maglevOutsideBoundary())
        {
	        printf("Landing the Maglev\n");
	        ml_Land( maglev_handle );
        }
        
        // Save the data, if it's still being recorded
        if (flag_save_force)
        {
                maglevStopSaveForce();
        }
        
  	// Disconnect from the Maglev server
	printf("Disconnecting from the Maglev server\n");
  	ml_Disconnect( maglev_handle );
  	
  	// Officially declare the Maglev off
  	printf("Done -- The maglev is disconnected\n");
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
        // Reset the controller counter.  Also declare that the force gains have
        //      not been changed.  This will start the process of ramping the
        //      the gains from the default internal gains to the desired
        //      position/force gains.
        printf("Starting the force controller\n");
        controller_counter = 0;
        end_counter = 0;
        
        // Set the flag that indicates that the force controller has started
        flag_force_control_start = true;
        
        // Set the starting gain values for each gain type and clear the force integral
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
        printf("\tInitial gains set\n");
        
        float a;
	if ( ml_GetServoFrequency ( maglev_handle, &a ) )
	{
 		cout << "Fatal ERROR -- Cannot get the freqency of the maglev!";
		exit(0);
	}
	else
	{
	        printf("\tMaglev Servo Frequency is %5.2f\n", a);
        }
}//maglevStartForceController

void
MaglevControl::maglevStopForceController()
{
        // Reset the controller counter.  Also declare that the force gains have
        //      not been changed.  This will start the process of ramping the
        //      the gains from the default internal gains to the desired
        //      position/force gains.
        printf("Stepping down the force controller...");
        end_counter = controller_counter;
        
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
        
        float a;
	if ( ml_GetServoFrequency ( maglev_handle, &a ) )
	{
 		cout << "Fatal ERROR -- Cannot get the freqency of the maglev!";
		exit(0);
	}
	else
	{
	        printf("\tMaglev Servo Frequency is %5.2f\n", a);
        }
}//maglevStopForceController

void
MaglevControl::maglevController ( Reading curr_force)
{
        // Variables needed for the controller
        double time_step, K_P, K_V, K_I, measured_maglev, desired_maglev;
        double mass = 0.555;
        double r_x = 0.000614;
        double r_y = 0.001225;
        double r_z = -0.031669;
        double gravity = 9.81;
        double velocity[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double proportional;
        double rotation_matrix[][3] = {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};
        
	////////////////////////////////////////////////////////////////////////
	// This part of the code checks to see whether we are in the transition
	//      phase of the force controller (i.e., right after the "Start" or
	//      "Stop" command has been given).  If so, it uses the variable
	//      controller_counter to perform linear interpolation on the gains
	//      and smoothly transition between the default internal controller
	//      gains and the gains specified in the data file.
	////////////////////////////////////////////////////////////////////////
	if ((controller_counter-end_counter) <= NUM_TRANSITIONS)
	{
                if ((controller_counter-end_counter) % 10 == 0)
                {
                        double interpolation_constant = (double)(controller_counter-end_counter) / NUM_TRANSITIONS;
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
                        printf("\t\t%03d | %03d | %5.2f | i%7.2f | p%7.2f | f%5.2f\n", controller_counter, end_counter, interpolation_constant, current_internal_gains[0][0], current_position_gains[0][0], current_force_gains[0][0]);
                }
/*                if (gains_changed)
                {
                        if ((controller_counter-end_counter) % 10 == 0)
                        {
                                gains_changed = false;
                        }
                }
                else
                {
                        interpolation_constant = (double)(controller_counter-end_counter) / NUM_TRANSITIONS;
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
                        printf("\t\t%03d | %03d | %5.2f | i%7.2f | p%7.2f | f%5.2f\n", controller_counter, end_counter, interpolation_constant, current_internal_gains[0][0], current_position_gains[0][0], current_force_gains[0][0]);
                }*/
                
                // Conditions to completely stop the force controller
                if (((controller_counter - end_counter) == NUM_TRANSITIONS) && (end_counter != 0))
                {
                        // Completely stop the force controller
                        flag_force_control_start = false;
                        printf("**********************************************\n");
                        printf("The force controller has stopped!\n");
                        printf("**********************************************\n");
                }
        }
	
        // Calculate time step
        if (controller_counter == 0)
        {
                // Assume zero time step from previous counter
                time_step = 0.0;
        }
        else
        {
                // Calculate time step
                time_step = curr_force.time - time_old;
                //time_step = 0.001;
        }
        time_old = curr_force.time;
        //printf("\t\tTime step: %5.2f\n", time_step);
        
        // Store the current position of the Maglev
        // maglevGetPosition();
	ml_GetActualPosition( maglev_handle, &current_pos );
	//int i;
	//for ( i = 0; i < 6; i++ )
	//{
        //        current_position[i] = current_pos.values[i];
	//}
        
        // Define rotation matrix sine/cosine values
        //double sin3 = current_pos.values[3];
        //double sin4 = current_pos.values[4];
        //double sin5 = current_pos.values[5];
        //double cos3 = 1.0-0.5*current_pos.values[3]*current_pos.values[3];
        //double cos4 = 1.0-0.5*current_pos.values[4]*current_pos.values[4];
        //double cos5 = 1.0-0.5*current_pos.values[5]*current_pos.values[5];
        
        // Calculate rotation matrix
        //rotation_matrix[0][0] = cos4*cos5 + sin3*sin4*sin5;
        //rotation_matrix[0][1] = sin3*sin4*cos5 - cos4*sin5;
        //rotation_matrix[0][2] = cos3*sin5;
        //rotation_matrix[1][0] = cos3*sin5;
        //rotation_matrix[1][1] = cos3*cos5;
        //rotation_matrix[1][2] = sin3;
        //rotation_matrix[2][0] = sin3*cos4*sin5 - cos4*cos5;
        //rotation_matrix[2][1] = cos3*cos4*cos5 + sin4*sin5;
        //rotation_matrix[2][2] = cos3*cos5;
        
        // Calculate rotation matrix
        rotation_matrix[0][0] = (1.0-0.5*current_pos.values[4]*current_pos.values[4])*(1.0-0.5*current_pos.values[5]*current_pos.values[5]);
        rotation_matrix[0][1] = (current_pos.values[3])*(current_pos.values[4])*(1.0-0.5*current_pos.values[5]*current_pos.values[5]) - (1.0-0.5*current_pos.values[3]*current_pos.values[3])*(current_pos.values[5]);
        rotation_matrix[0][2] = (1.0-0.5*current_pos.values[3]*current_pos.values[3])*(current_pos.values[4])*(1.0-0.5*current_pos.values[5]*current_pos.values[5]) + (current_pos.values[3])*(current_pos.values[5]);
        rotation_matrix[1][0] = (1.0-0.5*current_pos.values[4]*current_pos.values[4])*(current_pos.values[5]);
        rotation_matrix[1][1] = (current_pos.values[3])*(current_pos.values[4])*(current_pos.values[5]) + (1.0-0.5*current_pos.values[3]*current_pos.values[3])*(1.0-0.5*current_pos.values[5]*current_pos.values[5]);
        rotation_matrix[1][2] = (1.0-0.5*current_pos.values[3]*current_pos.values[3])*(current_pos.values[4])*(current_pos.values[5]) - (current_pos.values[3])*(1.0-0.5*current_pos.values[5]*current_pos.values[5]);
        rotation_matrix[2][0] = -(current_pos.values[4]);
        rotation_matrix[2][1] = (current_pos.values[3])*(1.0-0.5*current_pos.values[4]*current_pos.values[4]);
        rotation_matrix[2][2] = (1.0-0.5*current_pos.values[3]*current_pos.values[3])*(1.0-0.5*current_pos.values[4]*current_pos.values[4]);
        
        // Rotate the forces from the sensor frame into the Maglev frame
        //for (int dirIdx = 0; dirIdx < 3; dirIdx++)
        //{
        //        for (int altDir = 0; altDir < 3; altDir++)
        //        {
        //                measured_maglev[dirIdx] = measured_maglev[dirIdx] + rotation_matrix[dirIdx][altDir] * curr_force.force[dirIdx];
        //                desired_maglev[dirIdx] = desired_maglev[dirIdx] + rotation_matrix[dirIdx][altDir] * desired_force[dirIdx];
        //        }
        //}
        
        // Implement the controller in each direction
	for (int dirIdx=FORCE_START_INDEX; dirIdx < FORCE_STOP_INDEX; dirIdx++)
	{
                // Calculate information that depends on previous loop
                if (controller_counter == 0)
                {
                        // Assume zero velocity since we have no prior data
                        velocity[dirIdx] = 0.0;
                }
                else
                {
                        // Two steps in one line:
                        //      (1) Calculate velocity using first-order backwards difference derivative (portion of formula multiplied by alpha)
                        //      (2) Filter the velocity using a first-order low-pass filter
                        velocity[dirIdx] = alpha * ((current_pos.values[dirIdx] - position_old[dirIdx]) / time_step) + (1.0 - alpha) * velocity[dirIdx];
                }
                
                // Store the old position
                position_old[dirIdx] = current_pos.values[dirIdx];
                
                // Switch depending on force control direction
                if (force_control_directions[dirIdx])
                {
	                // Rotate the forces from the sensor frame into the Maglev frame
	                measured_maglev = 0.0;
	                desired_maglev = 0.0;
                        for (int altDir = 0; altDir < 3; altDir++)
                        {
                                measured_maglev = measured_maglev + rotation_matrix[dirIdx][altDir] * curr_force.force[altDir];
                                desired_maglev = desired_maglev + rotation_matrix[dirIdx][altDir] * desired_force[altDir];
                                //measured_maglev = measured_maglev + rotation_matrix[altDir][dirIdx] * curr_force.force[altDir];
                                //desired_maglev = desired_maglev + rotation_matrix[altDir][dirIdx] * desired_force[altDir];
                        }
                        
                        // Extract force gains from matrix
                        K_P = current_force_gains[dirIdx][0];        
                        K_I = current_force_gains[dirIdx][1];
                        K_V = current_force_gains[dirIdx][2];
                        
                        // Calculate proportional error
                        //proportional = desired_force[dirIdx] - curr_force.force[dirIdx];
                        proportional = desired_maglev - measured_maglev;
                        
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
                        
                        // Calculate controller term -- negative sign in front
                        //      of Proportional, Integral and Desired is to
                        //      correct for our sign convention
                        //des_force.values[dirIdx] = -(K_P * proportional + K_I * integral[dirIdx] + desired_maglev) - K_V * velocity[dirIdx];
                        des_force.values[dirIdx] = -(K_P * proportional + K_I * integral[dirIdx] + desired_maglev) - K_V * velocity[dirIdx];
                }
                else
                {
                        // Extract position gains from matrix
                        K_P = current_position_gains[dirIdx][0];        
                        K_I = current_position_gains[dirIdx][1];
                        K_V = current_position_gains[dirIdx][2];
                        
                        // Zero the desired force in this direction
                        desired_force[dirIdx] = 0.0;
                        
                        // Calculate proportional error
                        //proportional[dirIdx] = desired_position[dirIdx] - current_pos.values[dirIdx];
                        proportional = desired_position[dirIdx] - current_pos.values[dirIdx];
                        
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
                        des_force.values[dirIdx] = K_P * proportional + K_I * integral[dirIdx] - K_V * velocity[dirIdx];
                }
        }
        
        // Add the gravity correction terms
        des_force.values[2] = des_force.values[2] + mass*gravity;
        des_force.values[3] = des_force.values[3] - mass*gravity*r_y;
        des_force.values[4] = des_force.values[4] + mass*gravity*r_x;
        
        // Set the force of the Maglev to the output of the PID controller
        //maglevSetForce(pid_output);
	//int i;
	//for ( i = 0; i < 6; i++ )
	//{
        //        des_force.values[i] = new_force[i];
	//}
	ml_SetForces( maglev_handle, des_force );
	
        // Store data in array
	if (flag_save_force)
	{
                if (save_force_counter < SAVE_FORCE_DATA)
                {
                        // Save the time
                        save_force_data[save_force_counter][0] = curr_force.time;
	                for (int dirIdx=FORCE_START_INDEX; dirIdx < FORCE_STOP_INDEX; dirIdx++)
	                {
                                save_force_data[save_force_counter][dirIdx+1] = desired_force[dirIdx];
                                save_force_data[save_force_counter][dirIdx+7] = curr_force.force[dirIdx];
                                save_force_data[save_force_counter][dirIdx+13] = desired_position[dirIdx];
                                save_force_data[save_force_counter][dirIdx+19] = current_pos.values[dirIdx];
                                save_force_data[save_force_counter][dirIdx+37] = velocity[dirIdx];
                                //save_force_data[save_force_counter][dirIdx+43] = curr_force.raw_force_signal[dirIdx];
                                save_force_data[save_force_counter][dirIdx+49] = integral[dirIdx];
                                save_force_data[save_force_counter][dirIdx+61] = des_force.values[dirIdx];
                	}
                	save_force_counter++;
                }//*/
        }
        controller_counter++;
}//maglevController

//Get the position of maglev
void MaglevControl::maglevGetPosition()
{
	ml_GetActualPosition( maglev_handle, &current_pos );
	int i;
	for ( i = 0; i < 6; i++ )
	{
		current_position[i] = current_pos.values[i];
	}
}//maglevGetPosition

// Determine whether the Maglev flotor is currently outside the boundary
bool MaglevControl::maglevOutsideBoundary ()
{
        bool outside_boundary = false;
        
        // Detect whether a boundary-violation fault has occurred
        ml_GetFault( maglev_handle, &current_fault );
        usleep(500000);
        if(current_fault.value != ML_FAULT_TYPE_CLEAR)
        {
                // If a fault has occurred, determine whether one of the
                //      components is an out-of-range fault
                if (current_fault.value >= ML_FAULT_TYPE_FLOTOR_OVERSPEED)
                {
                        // Flotor moving too fast
                        current_fault.value -= ML_FAULT_TYPE_FLOTOR_OVERSPEED;
                }
                if (current_fault.value >= ML_FAULT_TYPE_COIL_OVERCURRENT)
                {
                        // Coil current too high
                        current_fault.value -= ML_FAULT_TYPE_COIL_OVERCURRENT;
                }
                if (current_fault.value >= ML_FAULT_TYPE_COIL_OVERTEMP)
                {
                        // Coil too hot
                        current_fault.value -= ML_FAULT_TYPE_COIL_OVERTEMP;
                }
                if (current_fault.value >= ML_FAULT_TYPE_SENSOR_OUT_OF_RANGE)
                {
                        // Sensor out of range - Exactly what we're looking for
                        outside_boundary = true;
                }
        }
        else
        {
                // Record the Maglev's actual position
                maglevGetPosition();
                
                // Loop to detect whether the actual position is outside the specified boundary
                int i = 0;
	        while ((!outside_boundary) && (i < 6))
	        {
		        // Check if the position and orientation are valid
		        if ((current_pos.values[i] < maglev_boundary_left[i]) || (current_pos.values[i] > maglev_boundary_right[i]))
		        {
			        outside_boundary = true;
		        }
		        
	                // Increment the counter
	                i++;
	        }
        }
        
	return (outside_boundary);
} // maglevOutsideBoundary

// Display the fault condition on the Maglev
/*int MaglevControl::maglevDisplayFault (ml_fault_type_t the_fault_value)
{
        // Detect all fault types currently registered
        if(the_fault_value != ML_FAULT_TYPE_CLEAR)
        {
                printf("Faults currently registered: (");
                // If a fault has occurred, determine whether one of the
                //      components is an out-of-range fault
                if (the_fault_value >= ML_FAULT_TYPE_FLOTOR_OVERSPEED)
                {
                        // Flotor moving too fast
                        the_fault_value -= ML_FAULT_TYPE_FLOTOR_OVERSPEED;
                        printf("Flotor_Speed");
                }
                if (the_fault_value >= ML_FAULT_TYPE_COIL_OVERCURRENT)
                {
                        // Coil current too high
                        current_fault.value -= ML_FAULT_TYPE_COIL_OVERCURRENT;
                        printf("Coil_Current ");
                }
                if (the_fault_value >= ML_FAULT_TYPE_COIL_OVERTEMP)
                {
                        // Coil too hot
                        current_fault.value -= ML_FAULT_TYPE_COIL_OVERTEMP;
                        printf("Coil_Temp ");
                }
                if (the_fault_value >= ML_FAULT_TYPE_SENSOR_OUT_OF_RANGE)
                {
                        // Sensor out of range
                        current_fault.value -= ML_FAULT_SENSOR_OUT_OF_RANGE;
                        printf("Sensor_Range");
                }
                if (the_fault_value != ML_FAULT_TYPE_CLEAR)
                {
                        printf(" ERROR - Unknown fault detected!");
                        return 1;
                }
                printf(")\n");
        }
        return 0;
} // maglevDisplayFault
*/
//Set the position of maglev
bool
MaglevControl::maglevMove2Position ( double * new_position )
{
	ml_position_t des_pos;
	bool outside_boundary = false;
	
	// Loop to verify that the new (desired) position is inside the boundary
	int i;
	printf("Moving to [");
	while ((!outside_boundary) && (i < 6))
	{
	        // Print the coordinate
		printf("%6.3f ", new_position[i]);
		
		// Check if the position and orientation are valid
		if ( new_position[i] > maglev_boundary_left[i] && new_position[i] < maglev_boundary_right[i] )
		{
			des_pos.values[i] = new_position[i];
		}
		else
		{
			if ( new_position[i] < maglev_boundary_left[i] )
			{
				des_pos.values[i] = maglev_boundary_left[i];
                        }
			else
			{
                                des_pos.values[i] = maglev_boundary_right[i];
                        }
			outside_boundary = true;
		}
		
	        // Increment the counter
	        i++;
	}
        if (outside_boundary)
        {
                // Do not set the desired position
        	printf("] which is outside the boundary\n");
        }
        else
        {
                // Set the desired position
        	printf("] which is in the boundary\n");
                ml_SetDesiredPosition( maglev_handle, des_pos );
        }
        
        return(!outside_boundary);
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

