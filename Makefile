CXX = g++

# Include location for the header files
IDIR = include
MAGLEV_INC = include/maglev
OPENCV_INC = include/opencv
CXXFLAGS = $(patsubst %,-I%,$(IDIR)) $(patsubst %,-isystem%,$(MAGLEV_INC)) $(patsubst %,-isystem%,$(OPENCV_INC)) -Wall -ansi

# Define different directories
SDIR = src
ODIR = obj
LDIR = lib
BDIR = bin

# Include libraries for the Maglev driver
LIBS = -L$(LDIR) -lmlhi_api_linux
#LIBS += -ldc1394_control -lraw1394 -pthread
LIBS += -ldc1394 -lraw1394 -pthread
# Include libraries for graphics
LIBS += -Lopencv -lopencv_core -lopencv_highgui -lopencv_cvv -lopencv_ximgproc -lopencv_imgproc
# Include libraries for FLTK
LIBS += `fltk-config --ldflags --cxxflags --use-images --use-gl` -s -lcomedi -lm -lc -lrt
# Include libraries for OpenGL
LIBS += -lGLU

# Define dependensies
_DEPS = CubeView Plot2DView BitOperating Graph3DAxis CameraControl Images Training ImageDisplay
_DEPS += MaglevControl ForceSensor maglev_parameters config main
DEPS = $(patsubst %,$(IDIR)/%.h,$(_DEPS))

# Define object files
_OBJ = CubeView Plot2DView BitOperating Graph3DAxis CameraControl Images Training ImageDisplay
_OBJ += MaglevControl ForceSensor main
OBJ = $(patsubst %,$(ODIR)/%.o,$(_OBJ))

# Define target output file
TARGET := $(BDIR)/main

all: $(TARGET)

$(info ==================== University of Utah ====================)
$(info ===================== Bio-Robotics Lab =====================)
$(info ============== Calibration Software GUI_ctc V1.2 ===========)
$(info )

# Rule to build regular .o files
$(ODIR)/%.o: $(SDIR)/%.cxx $(DEPS)
	@echo Building obj files
	$(CXX) -c -o $@ $< -w $(CXXFLAGS)

# Rule to build the target output file
$(TARGET): $(OBJ)
	@echo Building target output file:
	$(CXX) -o $@ $^ $(CXXFLAGS) $(LIBS)

.PHONY: clean
clean: 
	@echo cleaning objs
	rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~ 
	rm -f $(TARGET)