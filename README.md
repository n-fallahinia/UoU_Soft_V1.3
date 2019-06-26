# UoU_soft_v1.3
********************************************************************************
UofU Calibration Software
  
The Linux Control and Calibration Software for Maglev Device
  
University of Utah
  
Biorobotics Lab
	
Navid Fallahinia <n.fallahinia@utah.edu>
********************************************************************************
* About gui_v1.3:

This is intended to be a GUI for use with the Magnetic Levitation Haptic
Device (MLHD or Maglev), with classes to facilitate interaction with all of the
various components.

The intent is that this program may be used for diagnostic purposes, as well as
a base for development of new applications when future research requires it.
Copy this entire folder to a new working folder in order to modify it for your
individual needs. gui_v1.3 is a modified version of generic GUI of the maglev Device 
which has libraries to control the folotor position as well as the applied force to 
the fingertip. Also it has libraries to interact with the force sensors and record the
data from Flea2 and Flea3 cameras.

!!!IMPORTANT!!!

USE THE FOLLOWING ORDER TO START THE CAMERA:

	1- connect
	2- lift up
	3- camera
	4- controller
	5- traj files
	6- training

********************************************************************************
Please do not modify the contents of this folder for your individual research.
Only modify this folder if you intend to change the general GUI template!
********************************************************************************

*Description of contents:

	1.BarPlot - Class files giving the ability to show real-time bar plots of data.
    	Generally intended as a way to display instantaneous forces and torques
    	during experiments.

	2.CameraControl - Class files for interacting with the Point Grey Research FLEA
	    camera through the IEEE 1394 port.  Uses the dc1394 library.

	3.ForceSensor - Class files for interacting with the Sensoray 626 card and,
	    through it, the ATI Nano17 force/torque sensor.

	4.ImageDisplay - Class files for displaying images in an extended Fl_Box.

	5.Images - Class files for handling images.

	6.MaglevControl - Class files for interacting with the MLHD (Magnetic Levitation
	    Haptic Device).

	7.TimeHandler - Class files for tracking time.

	8.TimePlot - Class files giving the ability to show real-time xy scatter plots of
	    data.  Generally intended as a way to display data over time (i.e., the last
	    5 seconds of force or position data).

	9.TrajectoryData - Class files for tracking a trajectory. Make sure to put the 
		trajectories in the "/trajGen/large_trajectory/".

	10.UserInterface - Class files for the GUI (Graphical User Interface).  These were
	    originally generated using FLUID and take advantage of FLTK, but have been
	    heavily modified from the file generated by FLUID.

********************************************************************************
* Installation:
********************************************************************************

Compile using 'make'.  If this doesn't work, make sure you have the
basic tools installed to compile. If you can successfully compile
other things, consult the author, as he has probably made a mistake. 

This gui needs "haptics" software from Butterfly Haptits be installed!!
All other required libraries are located in the ~/UUSoftware/Libraries.
Make sure to install them all before compiling the Makefile. There is also 
a shell script called "install_libs.sh" to install all those libraries on my github 

********************************************************************************
* Mailing List:
********************************************************************************
f you are interested in any of my code, find bugs, or want additional features, 
please contact me. Not all of it is actively maintained, but if there is something 
you want to use, I am happy to pick up development on it again.

********************************************************************************
* More Information:
********************************************************************************
The Git repository can be cloned locally using:
	
	git clone https://github.com/n-fallahinia/UoU_soft_v1.3.git

gui_v1.3 may be freely distibuted and modified in accordance with
the MIT License.
