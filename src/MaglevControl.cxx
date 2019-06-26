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
    // Assume that, for the internal controller, we'll always use NORMAL
    // control (as opposed to BOUNDARY, LOCKED or CONSTRAINED).
	mp_gain_type = ML_GAINSET_TYPE_NORMAL;
	
	// Assign the array parameters in a loop
	for (int i = 0; i < DATA_SIZE; i++)
	{
        // Assign the variables representing controller parameters
        integral[i] = 0.0;
        velocity[i] = 0.0;
        desired_force[i] = 0.0;
        desired_position.values[i] = 0.0;
        force_control_directions[i] = default_force_control_directions[i];
        pid_force.values[i] = 0.0;
            
        // Assign the variables representing Maglev parameters
        current_temperatures.values[i] = 0.0;
        current_position.values[i] = 0.0;
        maglev_currents.values[i] = 0.0;
        maglev_forces.values[i] = 0.0;
    }
        
    // Define the desired force in the z-direction as -1 to prevent a zero
    //      desired force
    desired_force[2] = -1.0;
    
    // Define the name of the file containing the force gains
	// sprintf (force_gain_file_name, "force_gains.txt");

	// Define the force controller frequency (in Hz)
	control_freq = 1000.0;

	// Set up the velocity filter parameters
	cutoff_frequency = 200.0; // Hz
	tau = 1.0 / (2.0 * PI * cutoff_frequency);
	alpha = tau / (tau + (1.0/control_freq));

	// Calculate the gravity-compensation vector
    double mass = 0.555;
    double r_x = 0.000614;
    double r_y = 0.001225;
    double r_z = -0.031669;
    double gravity = 9.81;
    gravity_vector[0] = mass*gravity;
    gravity_vector[1] = -mass*gravity*r_y;
    gravity_vector[2] = mass*gravity*r_x;
    
	// Determines whether the force is currently being saved
	flag_save_force = false;
	
	// Determines whether the system is connected to the Maglev server
	flag_maglev_start = false;
	
	// Define the initial value of the "flag" that indicates which variable
	//      to update: (0) temperatures, (1) currents or (2) forces.
	update_variable = 0;

	// sprintf ( force_save_file_name, "../data_00_0_00x/force_saved.txt" );

}//MaglevControl

MaglevControl::~MaglevControl()
{
        if (flag_save_force)
        {
                maglevStopSaveForce();
        }
        if (flag_maglev_start)
        {
                maglevTurnOff();
        }
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
		current_gains_internal[i][0] = gain_vec.values[i].p;
		current_gains_internal[i][1] = gain_vec.values[i].i;
		current_gains_internal[i][2] = gain_vec.values[i].d;
		current_gains_internal[i][3] = gain_vec.values[i].ff;
	}
	//printf("X are found as:%f, %f, %f\n",current_gains_internal[0][0], current_gains_internal[0][1], current_gains_internal[0][2]);
}//maglevGetInternalGains

// Set the Maglev's internal controller gains to zero
void MaglevControl::maglevZeroInternalGains()
{
        // Iterate over each of the approved gain types
        for (int gain_type = 1; gain_type <= 3; gain_type++)
        {
                // Iterate to assign each direction's gain values
	        for (int i=0; i<6; i++)
	        {
	                // Assign Proportional, Integral, Derivative and Feed-Forward
	                //      gains in this direction
                        gain_vec.values[i].p  = 0.0;
                        gain_vec.values[i].i  = 0.0;
                        gain_vec.values[i].d  = 0.0;
                        gain_vec.values[i].ff = 0.0;
	        }
                
	        // Send the gains to the Maglev
	        ml_SetGainVecAxes(maglev_handle, (ml_gainset_type_t)gain_type, gain_vec);
        }
} // maglevZeroInternalGains

// Set the Maglev's internal controller gains to default values
void MaglevControl::maglevResetInternalGains()
{
        // Iterate over each of the approved gain types
        for (int gain_type = 1; gain_type <= 3; gain_type++)
        {
                // Iterate to assign each direction's gain values
	        for (int i=0; i<6; i++)
	        {
	                // Assign Proportional, Integral, and Derivative gains
	                //      in this direction
                        gain_vec.values[i].p  = default_gains_other[i][0];
                        gain_vec.values[i].i  = default_gains_other[i][1];
                        gain_vec.values[i].d  = default_gains_other[i][2];
                        if (gain_type == ML_GAINSET_TYPE_LOCKED)
                        {
                                // Set feed-forward gains for the Locked gains
                                gain_vec.values[i].ff = default_gains_other[i][3];
                        }
                        else
                        {
                                // Do not set feed-forward gains for the Constrained/Boundary gains
                                gain_vec.values[i].ff = 0.0;
                        }
                }
                
	        // Send the gains to the Maglev
	        ml_SetGainVecAxes(maglev_handle, (ml_gainset_type_t)gain_type, gain_vec);
        }
} // maglevResetInternalGains

// Set the internal position controller gains
void MaglevControl::maglevSetInternalGains()
{
    // Iterate to assign each direction's gain values
	for (int i=0; i<6; i++)
	{
	        // Assign Proportional, Integral, Derivative and Feed-Forward
	        //      gains in this direction
                gain_vec.values[i].p  = current_gains_internal[i][0];
                gain_vec.values[i].i  = current_gains_internal[i][1];
                gain_vec.values[i].d  = current_gains_internal[i][2];
                gain_vec.values[i].ff = current_gains_internal[i][3];
	}
	
	// Send the gains to the Maglev
	ml_SetGainVecAxes ( maglev_handle, mp_gain_type, gain_vec );
}//maglevSetInternalGains

// Read the Maglev gains from a data file
void MaglevControl::maglevInitGains()
{
	//Read in the gains from a file predefined
	FILE *pFile;
	pFile = fopen("../force_gains/force_gains.txt", "r" );

	// Find the current values of the internal gains
	maglevGetInternalGains();
	
	// Set the current values for the arrays that will hold the "transition" gains
	for (int i = 0; i < 6; i++)
	{
		for (int j=0 ; j < 3; j++)
		{
			current_gains_position[i][j] = 0.0;
			current_gains_force[i][j] = 0.0;
		}
		current_gains_position[i][3] = 0.0;
	}
	
	// If we couldn't read the gain file, then just use the defaults in the maglev_parameters.h file
	if (pFile==NULL)
  	{
    	    printf("Cannot locate the initial force gain file, using hard-coded defaults\n");
            // fclose (pFile);
	        for (int i = 0; i < 6; i++)
	        {
		        for (int j=0 ; j < 3; j++)
		        {
			        file_gains_position[i][j] = default_position_gains[i][j];
			        file_gains_force[i][j] = default_force_gains[i][j];
		        }
		        file_gains_position[i][3] = default_position_gains[i][3];
	        }
	        
	        printf("read position gians are: \n");
	    	for (int i = 0; i < 6; ++i)
			{
				for (int j = 0; j < 4; ++j)
		 		{
		 			printf("%f  ",file_gains_position[i][j]);
		 		}
				printf("\n"); 
			}
	        
	        // Get the default force control directions
	        for (int i = 0; i < DATA_SIZE; i++)
	        {
                force_control_directions[i] = default_force_control_directions[i];
	        }
   	    printf("Done -- The force gains have been read in!\n");
    
	}
	else
	{
	    // Read the force gains from the file
	    for (int i = 0; i < 6; i++)
	    {
        	float a;
		    for (int j=0 ; j < 4; j++)
		    {
			    fscanf(pFile, "%f", &a);
			    file_gains_position[i][j] = a;
		    }
		    for (int j=0; j < 3; j++)
		    {
			    fscanf(pFile, "%f", &a);
			    file_gains_force[i][j] = a;
		    }
	    }
	 //    printf("read position gians are: \n");
	 //    for (int i = 0; i < 6; ++i)
		// {
		// 	for (int j = 0; j < 4; ++j)
		//  	{
		//  		printf("%f  ",file_gains_position[i][j]);
		//  	}
		// 	printf("\n"); 
		// }
		// printf("read force gains are :\n");
		// for (int i = 0; i < 6; ++i)
		// {
		// 	for (int j = 0; j < 4; ++j)
		// 	{
		// 		printf("%f ",file_gains_force[i][j]);
		// 	}
		// 	printf("\n");
		// } 
	    // Read the line containing the "directions to control" information
	    for (int i = 0; i < DATA_SIZE; i++)
	    {
	        force_control_directions[i] = default_force_control_directions[i];
	  
	             // int a;
	             // fscanf(pFile, "%d", &a);
	             // if (a == 0)
	             // {
	             //     force_control_directions[i] = false;
                 // }
                 // else
                 // {
	             //     force_control_directions[i] = true;
                 // }
	    }
	 //    for (int i = 0; i < 6; ++i)
	 //    {
	 //    	printf("%d  ",default_force_control_directions[i]);
	 //    }
		// printf("\n");

		// Close the file and announce completion
	    fclose (pFile);
	    printf("Done -- The force gains have been read in!\n");
    }
}//maglevInitGains

// Save the current gains to a data file
void MaglevControl::maglevSaveGains()
{
	//Read in the gains from a file predefined
	FILE *pFile;
	pFile = fopen("../force_gains/force_gains.txt", "w" );
	for (int i = 0; i < 6; i++)
	{
	        // Write the current direction's position gains
		for (int j=0 ; j < 4; j++)
		{
			fprintf(pFile, "%f ", current_gains_position[i][j]);
		}
		
		// Write the current direction's force gains
		for (int j=0; j < 3; j++)
		{
			fprintf(pFile, "%f ", current_gains_force[i][j]);
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
void MaglevControl::maglevSetFrequency( int f )
{
    // If the Maglev is off, do not run
	if ( !flag_maglev_start )
	{
	        // Do not echo anything because this function is called by moving the slider bar
	        return;
        }
        
        // Set the maglev frequency, if within limits
        if ((f > ML_SERVO_INTERVAL_MIN) && (f < OPERATING_FREQUENCY))
        {
                control_freq = (double)f;
        }
        else
        {
                printf("Fatal ERROR -- Desired frequency is outside range!\n");
                exit(0);
        }
        
	// Recalculate the filter proportionality constant
	alpha = tau / (tau + (1.0 / control_freq));
	
	printf("Current frequency is %d.\n", f);
	printf("New velocity filter proportionality constant is %5.3f\n", alpha);
}//maglevSetFrequency

// Get the force controller frequency
float MaglevControl::maglevGetFrequency( )
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

void MaglevControl::maglevConnect (  )
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
	
	// Get the Maglev boundary sphere radius
	float radius;
	if (ml_GetBoundaryRadius(maglev_handle, &radius) != ML_STATUS_OK)
	{
	        printf("\n\tFailed to properly retrieve the Maglev's boundary sphere radius!\n\n");
	}
	else
	{
	        printf("\n\tMaglev boundary sphere radius is %9.6f\n", radius);
	}
	
	flag_maglev_start = true;
	std::cout << "Done -- The maglev is connected\n0\t";
}//maglevConnect

void MaglevControl::maglevClearError (  )
{
	ml_ResetFault (	maglev_handle );
	std::cout << "Done -- Errors are cleared\n";
}//maglevClearError

void MaglevControl::maglevTakeOff ( bool thumb )
{
    // Attempt to take off until possible
        ml_GetActualPosition( maglev_handle, &current_position );
        float minPos = minValue(current_position.values);
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
                ml_GetActualPosition( maglev_handle, &current_position );
                minPos = minValue(current_position.values);
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

void MaglevControl::maglevLand (  )
{
	std::cout << "Landing the device\n";
	ml_Land( maglev_handle );
	std::cout << "Done -- The device has landed\n";
}//maglevLand

// Disconnect from the Maglev server.  If necessary, stop the force controller,
//      shut down all interrupts, and land the flotor.  Save the data, if it's
//      still being recorded.
void MaglevControl::maglevTurnOff ( )
{
	// Turn off the force controller, if needed
	if (flag_force_control_start)
	{
        	maglevStopForceController();
        	flag_force_control_start = false;
	}
        
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
        
        // Mark the maglev as turned off
  	flag_maglev_start = false;
	
  	// Disconnect from the Maglev server
	printf("Disconnecting from the Maglev server\n");
  	ml_Disconnect( maglev_handle );
  	
  	// Officially declare the Maglev off
  	printf("Done -- The Maglev is disconnected\n");
} // maglevTurnOff

// Start the force controller.  This initializes the controller counter to zero
//      and starts the ramping-up process from the Maglev's internal position
//      controller to the hybrid force-position controller.
void MaglevControl::maglevStartForceController()
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
                        // Set the starting PID gain values for each type
                        starting_gains_internal[i][j] = current_gains_internal[i][j];
                        starting_gains_position[i][j] = current_gains_position[i][j];
                        starting_gains_force[i][j] = current_gains_force[i][j];
                        
                        // Set the desired PID gain values for each type
                        desired_gains_internal[i][j] = 0.0;
                        desired_gains_position[i][j] = file_gains_position[i][j];
                        desired_gains_force[i][j] = file_gains_force[i][j];
                }
                
                // Set the starting feed-forward gain values
                starting_gains_internal[i][3] = current_gains_internal[i][3];
                starting_gains_position[i][3] = current_gains_position[i][3];
                
                // Set the desired feed-forward gain values
                desired_gains_internal[i][3] = 0.0;
                desired_gains_position[i][3] = file_gains_position[i][3];
        }
        
        // Turn off the boundary, locked and constrained gains
        maglevZeroInternalGains();
        
        // Echo a message to the user
        printf("\tInitial gains set\n");
}//maglevStartForceController

// Stop the force controller.  This starts the ramping-down process from the 
//      hybrid force-position controller to the Maglev's internal position
//      controller.
void MaglevControl::maglevStopForceController()
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
                        // Set the starting PID gain values for each type
                        starting_gains_internal[i][j] = current_gains_internal[i][j];
                        starting_gains_position[i][j] = current_gains_position[i][j];
                        starting_gains_force[i][j] = current_gains_force[i][j];
                        
                        // Set the desired PID gain values for each type
                        desired_gains_internal[i][j] = default_internal_gains[i][j];
                        desired_gains_position[i][j] = 0.0;
                        desired_gains_force[i][j] = 0.0;
                }
                // Set the starting feed-forward gain values
                starting_gains_internal[i][3] = current_gains_internal[i][3];
                starting_gains_position[i][3] = current_gains_position[i][3];
                
                // Set the desired feed-forward gain values
                desired_gains_internal[i][3] = default_internal_gains[i][3];
                desired_gains_position[i][3] = 0.0;
        }
        
        // Turn on the boundary, locked and constrained gains
        maglevResetInternalGains();
}//maglevStopForceController

// Run the hybrid force-position controller.
bool MaglevControl::maglevController ( Reading curr_force)
{
    // Variables needed for the controller
        double time_step;
        double proportional, K_P, K_V, K_I, measured_maglev, desired_maglev;
        bool return_value = false;
    // printf("Force Controller Started!!!\n");    
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
                //printf("Start Transision\n");
                if ((controller_counter-end_counter) % 10 == 0)
                {
                        double interpolation_constant = (double)(controller_counter-end_counter) / NUM_TRANSITIONS;
                        for(int i=FORCE_START_INDEX; i<FORCE_STOP_INDEX; i++)
                        {
                                for(int j=0; j<3; j++)
                                {
                                        current_gains_internal[i][j] = starting_gains_internal[i][j] + (desired_gains_internal[i][j]-starting_gains_internal[i][j]) * interpolation_constant;
                                        current_gains_position[i][j] = starting_gains_position[i][j] + (desired_gains_position[i][j]-starting_gains_position[i][j]) * interpolation_constant;
                                        current_gains_force[i][j] = starting_gains_force[i][j] + (desired_gains_force[i][j]-starting_gains_force[i][j]) * interpolation_constant;
                                }
                                current_gains_internal[i][3] = starting_gains_internal[i][3] + (desired_gains_internal[i][3]-starting_gains_internal[i][3]) * interpolation_constant;
                                current_gains_position[i][3] = starting_gains_position[i][3] + (desired_gains_position[i][3]-starting_gains_position[i][3]) * interpolation_constant;
                        }
                        maglevSetInternalGains();
                        printf("\t\t%03d | %03d | %5.2f | i%7.2f | p%7.2f | f%5.2f\n", controller_counter, end_counter, interpolation_constant, current_gains_internal[0][0], current_gains_position[2][0], current_gains_force[2][0]);
                }
                
                // Condition to stop ramping and update the gains
                if ((controller_counter - end_counter) == NUM_TRANSITIONS)
                {
                        // Change the return value to update the gains
                        return_value = true;
                        
                        // Completely start/stop the force controller
                        if (end_counter == 0)
                        {
                                // Message to the user
                                printf("**********************************************\n");
                                printf("The force controller completely engaged!\n");
                                printf("**********************************************\n");
                        }
                        else
                        {
                                // Completely stop the force controller
                                flag_force_control_start = false;
                                printf("**********************************************\n");
                                printf("The force controller has stopped!\n");
                                printf("**********************************************\n");
                        }
                }
    }
	
	////////////////////////////////////////////////////////////////////////
	// This part of the code implements the actual controller.  It performs
	//      the following steps:
	//      (1) Calculate the time step (used in the integral and velocity
	//          calculations).
	//      (2) Update the current position reading.
	//      (3) Calculate the rotation matrix values (used to rotate the
	//          desired and measured forces).
	//      (4) Determine the Hybrid Controller values in each direction.
	//          This involves the following:
	//              (a) Estimate the velocity in the current direction
	//              (b) If performing force control, rotate the desired and
	//                  measured forces into the Maglev frame and calculate
	//                  the force error.
	//              (c) If performing position control, calculate the
	//                  position error.
	//              (d) Estimate the integral term.
	//              (e) Calculate the PID controller output.
	//      (5) Add gravity compensation.
	//      (6) Send the controller output to the Maglev.
	////////////////////////////////////////////////////////////////////////
	
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
        
        // Store the current position of the Maglev
        // maglevGetPosition();
		ml_GetActualPosition( maglev_handle, &current_position );
        
        // Calculate rotation matrix
        rotation_matrix[0][0] = cos(current_position.values[4])*cos(current_position.values[5]);
        rotation_matrix[0][1] = sin(current_position.values[3])*sin(current_position.values[4])*cos(current_position.values[5]) - cos(current_position.values[3])*sin(current_position.values[5]);
        rotation_matrix[0][2] = cos(current_position.values[3])*sin(current_position.values[4])*cos(current_position.values[5]) + sin(current_position.values[3])*sin(current_position.values[5]);
        rotation_matrix[1][0] = cos(current_position.values[4])*sin(current_position.values[5]);
        rotation_matrix[1][1] = sin(current_position.values[3])*sin(current_position.values[4])*sin(current_position.values[5]) + cos(current_position.values[3])*cos(current_position.values[5]);
        rotation_matrix[1][2] = cos(current_position.values[3])*sin(current_position.values[4])*sin(current_position.values[5]) - sin(current_position.values[3])*cos(current_position.values[5]);
        rotation_matrix[2][0] = -sin(current_position.values[4]);
        rotation_matrix[2][1] = sin(current_position.values[3])*cos(current_position.values[4]);
        rotation_matrix[2][2] = cos(current_position.values[3])*cos(current_position.values[4]);
        
        // Implement the controller in each direction
	for (int dirIdx=FORCE_START_INDEX; dirIdx < FORCE_STOP_INDEX; dirIdx++)
	{
                // Estimate the velocity in this direction
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
                        velocity[dirIdx] = alpha * ((current_position.values[dirIdx] - position_old[dirIdx]) / time_step) + (1.0 - alpha) * velocity[dirIdx];
                }
                
                // Store the old position
                position_old[dirIdx] = current_position.values[dirIdx];
                
                // Switch depending on force control direction
                if (force_control_directions[dirIdx])
                {
	                // Rotate the forces from the sensor frame into the Maglev frame
	                measured_maglev = 0.0;
	                desired_maglev = 0.0;
                        for (int altDir = 0; altDir < 3; altDir++)
                        {
                                // Assume the rotation matrix is correct as calculated
                                measured_maglev = measured_maglev + rotation_matrix[dirIdx][altDir] * curr_force.force[altDir];
                                desired_maglev = desired_maglev + rotation_matrix[dirIdx][altDir] * desired_force[altDir];
                                
                                // Assume we've calculated the transpose of the rotation matrix
                                //measured_maglev = measured_maglev + rotation_matrix[altDir][dirIdx] * curr_force.force[altDir];
                                //desired_maglev = desired_maglev + rotation_matrix[altDir][dirIdx] * desired_force[altDir];
                        }
                        
                        // Extract force gains from matrix
                        K_P = current_gains_force[dirIdx][0];        
                        K_I = current_gains_force[dirIdx][1];
                        K_V = current_gains_force[dirIdx][2];
                        
                        // Calculate proportional error
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
                        pid_force.values[dirIdx] = -(K_P * proportional + K_I * integral[dirIdx] + desired_maglev) - K_V * velocity[dirIdx];
                }
                else
                {
                        // Extract position gains from matrix
                        K_P = current_gains_position[dirIdx][0];        
                        K_I = current_gains_position[dirIdx][1];
                        K_V = current_gains_position[dirIdx][2];
                        
                        // Zero the desired force in this direction
                        desired_force[dirIdx] = 0.0;
                        
                        // Calculate proportional error
                        proportional = desired_position.values[dirIdx] - current_position.values[dirIdx];
                        
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
                        pid_force.values[dirIdx] = K_P * proportional + K_I * integral[dirIdx] - K_V * velocity[dirIdx];
                }
     }
        
        /*// Increase force steps
        if (controller_counter < 5000.0)
        {
                double my_interp = (double)controller_counter / 5000.0;
                pid_force.values[0] = -12.0*my_interp;
                pid_force.values[1] = 12.0*my_interp;
                pid_force.values[2] = 15.0*my_interp;
        }
        else
        {
                pid_force.values[0] = -12.0;
                pid_force.values[1] = 12.0;
                pid_force.values[2] = 15.0;
        }*/
        
        // Add the gravity correction terms
        pid_force.values[2] = pid_force.values[2] + gravity_vector[0];
        pid_force.values[3] = pid_force.values[3] + gravity_vector[1];
        pid_force.values[4] = pid_force.values[4] + gravity_vector[2];
        
        // Set the force of the Maglev to the output of the PID controller
        //maglevSetForce();
	ml_SetForces( maglev_handle, pid_force );
	
	////////////////////////////////////////////////////////////////////////
	// This part of the code saves the data to an array and updates the
	//      different counters.
	////////////////////////////////////////////////////////////////////////
        
        // Store data in array
	if ((flag_save_force) && (save_force_counter < SAVE_FORCE_DATA))
    {
       // Save the time
            save_force_data[save_force_counter][0] = curr_force.time;
            for (int dirIdx=FORCE_START_INDEX; dirIdx < FORCE_STOP_INDEX; dirIdx++)
            {
                save_force_data[save_force_counter][dirIdx+1] = desired_force[dirIdx];
                save_force_data[save_force_counter][dirIdx+7] = curr_force.force[dirIdx];
                save_force_data[save_force_counter][dirIdx+13] = desired_position.values[dirIdx];
                save_force_data[save_force_counter][dirIdx+19] = current_position.values[dirIdx];
                save_force_data[save_force_counter][dirIdx+25] = current_temperatures.values[dirIdx];
                save_force_data[save_force_counter][dirIdx+31] = maglev_forces.values[dirIdx];
                save_force_data[save_force_counter][dirIdx+37] = velocity[dirIdx];
                save_force_data[save_force_counter][dirIdx+43] = curr_force.raw_force_signal[dirIdx];
                save_force_data[save_force_counter][dirIdx+49] = integral[dirIdx];
                save_force_data[save_force_counter][dirIdx+55] = maglev_currents.values[dirIdx];
                save_force_data[save_force_counter][dirIdx+61] = pid_force.values[dirIdx];
        	}
        	save_force_counter++;
    }
        controller_counter++;
        return(return_value);
} // maglevController

// Get the current position of maglev
void MaglevControl::maglevGetPosition()
{
	ml_GetActualPosition( maglev_handle, &current_position );
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
		        if ((current_position.values[i] < maglev_boundary_left[i]) || (current_position.values[i] > maglev_boundary_right[i]))
		        {
			        outside_boundary = true;
		        }
		        
	                // Increment the counter
	                i++;
	        }
        }  
	return (outside_boundary);
} 
// maglevOutsideBoundary

// Tell the Maglev to move to the desired position
void MaglevControl::maglevSetDesiredPosition()
{
        ml_SetDesiredPosition(maglev_handle, desired_position);
} // maglevSetDesiredPosition

// Set the position of maglev
bool MaglevControl::maglevMove2Position ( double * new_position )
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

// Set the (open-loop) force the Maglev will exert
void MaglevControl::maglevSetForce ()
{
        ml_SetForces( maglev_handle, pid_force );
} // maglevSetForce

// Get the current set of forces and torques of maglev
void MaglevControl::maglevGetForce ()
{
	ml_GetForces( maglev_handle, &maglev_forces);
} // maglevGetForce

// Get the current set of coil currents of maglev
void MaglevControl::maglevGetCurrent ()
{
	ml_GetCurrents( maglev_handle, &maglev_currents);
} // maglevGetCurrent

// Get the current set of coil temperatures of maglev
void MaglevControl::maglevGetTemperature ()
{
	ml_GetTemp( maglev_handle, &current_temperatures);
} // maglevGetTemperature

// Begin saving force controller data
void MaglevControl::maglevSaveForce ( )
{
	// Reset the counter
	save_force_counter = 0;
	
	// Mark the flag that the force is being saved
	flag_save_force = true;
	
	// Tell the user
	printf("Starting to save force controller data\n");
} // maglevSaveForce

// Stop saving force controller data
void MaglevControl::maglevStopSaveForce ()
{
	// Open the file
	save_force_file = fopen(force_save_file_name, "w" );
	printf("Saving the force controller data into the %s\n", force_save_file_name); 
	printf("Saving %d of a possible %d points of force data\n", save_force_counter, SAVE_FORCE_DATA);
	
	// Round down to the size of the array
	if (save_force_counter > SAVE_FORCE_DATA)
	{
	    save_force_counter = SAVE_FORCE_DATA;
    }
        
    // Iterate to save all recorded data
	for ( int i=0; i < save_force_counter; i++)
	{	
		fprintf(save_force_file, "%f ", save_force_data[i][0]);
		for ( int j = 1; j<SAVE_FORCE_COLS; j++)
		{
			fprintf(save_force_file, "%f ", save_force_data[i][j]);
		}
		fprintf(save_force_file, "\n");
	}
	fclose( save_force_file );
	printf("\tDone\n");
	
	// Set the flag that the force is no longer being saved
	flag_save_force = 0;
	
	// Reset the counter
	save_force_counter = 0;
}//maglevStopSaveForce

// Set the name of the file in which to save the force data
void MaglevControl::maglevSetSaveFileName( char folder_name[18] )
{
        sprintf(force_save_file_name, "%sforce_saved.txt", folder_name);
        printf("Saving data to %s\n", force_save_file_name);
} // maglevSetSaveFileName


// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void MaglevControl::maglevPrintGainMatrices()
{
        // Make sure the current_gains_internal matrix is up-to-date
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
                        printf(" %8.2f |", current_gains_internal[dirIdx][gainIdx]);
                }
                
                // Print the external position controller gains
                for (int gainIdx=0; gainIdx<4; gainIdx++)
                {
                        printf("| %8.2f ", current_gains_position[dirIdx][gainIdx]);
                }
                
                // Print the force controller gains
                printf("|");
                for (int gainIdx=0; gainIdx<3; gainIdx++)
                {
                        printf("| %5.2f ", current_gains_force[dirIdx][gainIdx]);
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

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

