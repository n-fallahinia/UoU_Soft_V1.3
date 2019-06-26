#include "ForceSensor.h"
#include "TimeHandler.h"
#include <cstdlib>

// Define the force sensor that is attached to the MLHD
#define FT9555

// Determine the force sensor in use and load the appropriate calibration and gain matrices.
#ifdef FT9556
        // The FT9556 force sensor is in use, load its calibration and gain matrices
        double calib[6][6] = {{  0.29617,  0.93174, -1.53677, 36.17997,  1.64337,-35.16264},
                              {  3.01534,-45.45889, -1.51281, 23.35465, -1.15435, 20.04677},
                              { 21.78121, -0.11022, 22.77882, -0.62224, 21.23569, -1.02077},
                              {  1.30844,  0.27816, 39.11265, -1.26415,-37.56760,  1.67656},
                              {-43.87937,  1.20351, 24.61545, -0.97497, 20.70109, -0.71850},
                              {  1.21434,-18.54361,  1.09023,-20.78799,  1.21599,-21.16204}};
        double gain[6] = {21.7710100343547, 21.7710100343547, 12.3565192086878, 3.88204409546003, 3.88204409546003, 2.94818173962909};
#else
       
#ifdef FT9555
        // The FT7151 force sensor is in use; load its calibration and gain matrices
        double calib[6][6] = {{   0.06105,  -0.19672,  -1.79433,  37.70342,   2.06223, -37.52904},
                              {   2.02210, -44.83887,  -1.40611,  21.12528,  -1.40373,  22.29920},
                              { -22.17321,  -0.29397, -22.09094,  -0.75540, -22.46264,  -0.83756},
                              {   0.86343,   0.62380, -37.57811,  -1.73684,  39.28231,   1.42668},
                              {  43.45049,   0.61411, -21.70741,  -0.41397, -23.31255,  -1.47309},
                              {   0.44082, -22.53919,   1.48699, -21.63087,   1.43589, -22.34803}};
        double gain[6] = {22.7300084098357, 22.7300084098357, 11.7868333238604, 3.64585766892664, 3.64585766892664, 3.1484034675519};
        #else
                // These are the default "dummy" calibration and gain matrices to use when a bad force sensor has been specified.
                double calib[6][6] = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
                double gain[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        #endif
#endif

ForceSensor::ForceSensor()
{
        // Channels 8-13 are where the force sensor is currently connected
        channel_start = 0;
        channel_end = 5;
        
        // What are these two variables for?
        max_range = 0;
        counter = 0;
        
        // Indicates that the force sensor is turned off when the sensor is initialized
        flag_force_sensor = false;
        
        // Determine the number of internal ticks per second
        click_sec = clickPerSecond();
        printf("There are %lld clicks per second.\n", click_sec);
        
        // Define the name of the device
        sprintf(device_name,"/dev/comedi0");
}//ForceSensor

void ForceSensor::startForceSensor()
{
        // If the force sensor is already on, don't try to start it again
        if (flag_force_sensor)
        {
                printf("The force sensor has already been started\n");
                return;
        }
        
        // Start the DAC
        device = comedi_open ( device_name );
        if ( !device )
        {
                // If the device did not start, give an error message and quit
                comedi_perror ( device_name );
                exit ( -1 );
        }
        printf("Success!\n");
        
        // Determine the force sensor offset
        getOffset ();
        
        // Determine the starting time
        time_start = currTimeMS(click_sec);
        printf("Starting Time = %1.0f\n", time_start);
        
        // Determine the sub-device numbers for the analog input and analog output
        analog_input = comedi_find_subdevice_by_type(device, COMEDI_SUBD_AI, 0);
        analog_output = comedi_find_subdevice_by_type(device, COMEDI_SUBD_AO, 0);
        num_inputs = comedi_get_n_channels(device, analog_input);
        num_outputs = comedi_get_n_channels(device, analog_output);
        
        // Determine properties of the analog output channel used for light control
        light_channel = 0;
        light_range = 0;
        light_rng = comedi_get_range(device, analog_output, light_channel, light_range);
        light_max = comedi_get_maxdata(device, analog_output, light_channel);
        
        // Report DAC Properties
        printf("Data Acquisition Card Properties:\n");
        printf("\tAnalog Input Subdevice=%d\tNumber of Channels=%d\n", analog_input, num_inputs);
        printf("\tAnalog Output Subdevice=%d\tNumber of Channels=%d\n", analog_output, num_outputs);
        
        // Mark that the force sensor is now capable of recording data
        flag_force_sensor = true;
}//startForceSensor

void ForceSensor::readNewData()
{
	// Determine the current time
	curr_force.time = (currTimeMS(click_sec) - time_start) / 1000.0;
	
	// Read the voltage from the DAC
	//printf("volt | ");
	readVoltage ( curr_force.raw_force_signal );
	
	//Convert the voltage to force reading
	//printf("convert ");
	voltage2Force ( curr_force.raw_force_signal, curr_force.force );
	//printf("(%5.2f->%5.2f) | ", curr_force.raw_force_signal[1], curr_force.force[1]);
	
	// Subtract the force offset
	//printf("offset | ");
	forceOffset ( curr_force.force );
	
	// Print some information on each iteration
	//printf("%lld\t%5.2f\t%5.2f\t%5.2f\n", curr_force.time, curr_force.force[0], curr_force.force[1], curr_force.force[2]);
}//readNewData

void ForceSensor::readVoltage ( double voltage[] )
{
        // Declare variables
        int n_chans,chanIdx, i;
        long int max_data;
        comedi_range *rgn;
        lsampl_t data;
        
        //Open the device
        n_chans = comedi_get_n_channels ( device, analog_input );
        if ( channel_end > n_chans ) 
        {
                printf ( "The channel range is wrong -- %d \n", n_chans );
                exit ( -1 );
        }
        
        // Read the force sensor channels
        for(chanIdx = channel_start; chanIdx <= channel_end; ++chanIdx)
        {
                // Calculate the zero-based index of the current channel
                i = chanIdx - channel_start;
                
                // I am not sure why we need this parameter
                int range = 1;
                
                // Determine the maximum data value of the channel -- Do we need to do this on every sampling?
                max_data = comedi_get_maxdata ( device, analog_input, chanIdx );
                
                // Determine the range of the channel -- Do we need to do this on every sampling?
                rgn = comedi_get_range(device, analog_input, chanIdx, range);
                
                // Read the data from the current channel
                comedi_data_read ( device, analog_input, chanIdx, range, AREF_GROUND, &data );
                
                // If at the maximum data level, subtract one (Why?)
                if ( data == max_data )
                {
                        data = data -1;
                }
                
                // If at the minimum data level, add one (Why?)
                if ( data ==0 )
                {
                        data = 1;
                }
                
                // Convert the data value to a voltage value
                voltage[i] = comedi_to_phys ( data, rgn, max_data );
                //printf("| %6.3f", voltage[i]);
        }
}//readVoltage

void 
ForceSensor::voltage2Force ( double * voltage, double * force)
{
    int i, j;
    //std::cout << "Force reading is ";
    for(i=0; i<6; i++)
    {
	force[i] = 0;
	for(j=0; j<6; j++)
	{
	   force[i] += calib[i][j] * voltage[j];
	}
	// Added negative sign to account for force sensor sign convention
	force[i] /= gain[i];	// Force/Torque data in direction {Fx,Fy,Fz,Tx,Ty,Tz}
//	force[i] /= (gain[i]);	// Force/Torque data in direction {Fx,Fy,Fz,Tx,Ty,Tz}
	//std::cout << force[i] <<" : ";
    }
//	std::cout << "\n";
	
}//voltage2Force

void
ForceSensor::getOffset ( )
{
    int i, j;
    int force_zero_average_num = 10;
    force_offset[0] = force_offset[1] = force_offset[2] = force_offset[3] = force_offset[4] = force_offset[5] = 0.0;
    //printf("Got here!\n");
    for ( i=0; i<force_zero_average_num; i++)
    {
		double force_zero_tmp[6];
		//printf("Iteration %d of %d!\n", i, force_zero_average_num);
		readVoltage ( curr_force.raw_force_signal );	
		//Convert the voltage to force reading
		voltage2Force ( curr_force.raw_force_signal, force_zero_tmp );
		// Store offsets for each channel
		for ( j=0; j<6; j++ )
		{
		   force_offset[j] += force_zero_tmp[j]/((double) force_zero_average_num);
		}
    }
    //Display something to show the offset is taken
}//getOffset


void
ForceSensor::forceOffset ( double *force_curr )

{
    for ( int i=0; i<6; i++ )
    {	
		force_curr[i] -= force_offset[i];
	//std::cout << force_curr[i] <<" : ";
    }
}//forceOffset

void ForceSensor::write_voltage_to_light(double voltage)
{
        // Declare variables required 
        int result;
        lsampl_t data;
        
        // Convert the voltage value to a data value
        data = comedi_from_phys(voltage, light_rng, light_max);
        
        // Write the data value to the Analog Output channel
        result = comedi_data_write(device,  analog_output, light_channel, light_range, AREF_GROUND, data);
}
