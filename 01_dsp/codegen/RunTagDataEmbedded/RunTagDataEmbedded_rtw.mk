###########################################################################
## Makefile generated for component 'RunTagDataEmbedded'. 
## 
## Makefile     : RunTagDataEmbedded_rtw.mk
## Generated on : Fri Mar 28 10:34:25 2025
## Final product: ./RunTagDataEmbedded.lib
## Product type : static-library
## 
###########################################################################

###########################################################################
## MACROS
###########################################################################

# Macro Descriptions:
# PRODUCT_NAME            Name of the system to build
# MAKEFILE                Name of this makefile
# MODELLIB                Static library target

PRODUCT_NAME              = RunTagDataEmbedded
MAKEFILE                  = RunTagDataEmbedded_rtw.mk
MATLAB_ROOT               = .
MATLAB_BIN                = .
MATLAB_ARCH_BIN           = .
START_DIR                 = /home/
TGT_FCN_LIB               = ISO_C
SOLVER_OBJ                = 
CLASSIC_INTERFACE         = 0
MODEL_HAS_DYNAMICALLY_LOADED_SFCNS = 
RELATIVE_PATH_TO_ANCHOR   = ../../..
C_STANDARD_OPTS           = 
CPP_STANDARD_OPTS         = 
MODELLIB                  = RunTagDataEmbedded.lib

###########################################################################
## TOOLCHAIN SPECIFICATIONS
###########################################################################

# Toolchain Name:          GNU GCC BeagleBone
# Supported Version(s):    
# ToolchainInfo Version:   2024b
# Specification Revision:  1.0
# 
#-------------------------------------------
# Macros assumed to be defined elsewhere
#-------------------------------------------

# LINUX_TGT_LIBS

#-----------
# MACROS
#-----------

CCOUTPUTFLAG   = --output_file=
LDOUTPUTFLAG   = --output_file=

TOOLCHAIN_SRCS = 
TOOLCHAIN_INCS = 
TOOLCHAIN_LIBS = -lm -lrt -lpthread -ldl -lasound -lSDL -lm -lstdc++ -lrt -lpthread -ldl -lasound -lSDL

#------------------------
# BUILD TOOL COMMANDS
#------------------------

# Assembler: GNU GCC BeagleBone Assembler
AS = as

# C Compiler: GNU GCC BeagleBone C Compiler
CC = gcc

# Linker: GNU GCC BeagleBone Linker
LD = gcc

# C++ Compiler: GNU GCC BeagleBone C++ Compiler
CPP = g++

# C++ Linker: GNU GCC BeagleBone C++ Linker
CPP_LD = g++

# Archiver: GNU GCC BeagleBone Archiver
AR = ar

# MEX Tool: MEX Tool
MEX_PATH = $(MATLAB_ARCH_BIN)
MEX = "$(MEX_PATH)/mex"

# Download: Download
DOWNLOAD =

# Execute: Execute
EXECUTE = $(PRODUCT)

# Builder: Make Tool
MAKE = make


#-------------------------
# Directives/Utilities
#-------------------------

ASDEBUG             = -g
AS_OUTPUT_FLAG      = -o
CDEBUG              = -g
C_OUTPUT_FLAG       = -o
LDDEBUG             = -g
OUTPUT_FLAG         = -o
CPPDEBUG            = -g
CPP_OUTPUT_FLAG     = -o
CPPLDDEBUG          = -g
OUTPUT_FLAG         = -o
ARDEBUG             =
STATICLIB_OUTPUT_FLAG =
MEX_DEBUG           = -g
RM                  =
ECHO                = echo
MV                  =
RUN                 =

#--------------------------------------
# "Faster Runs" Build Configuration
#--------------------------------------

ARFLAGS              = -r
ASFLAGS              = -c \
                       $(ASFLAGS_ADDITIONAL) \
                       $(INCLUDES)
CFLAGS               = -c \
                       -MMD -MP -MF"$(@:%.o=%.dep)" -MT"$@"  \
                       -O2
CPPFLAGS             = -c \
                       -MMD -MP -MF"$(@:%.o=%.dep)" -MT"$@"  \
                       -O2
CPP_LDFLAGS          =
CPP_SHAREDLIB_LDFLAGS  = -shared 
DOWNLOAD_FLAGS       =
EXECUTE_FLAGS        =
LDFLAGS              =
MEX_CPPFLAGS         =
MEX_CPPLDFLAGS       =
MEX_CFLAGS           =
MEX_LDFLAGS          =
MAKE_FLAGS           = -f $(MAKEFILE)
SHAREDLIB_LDFLAGS    = -shared 



###########################################################################
## OUTPUT INFO
###########################################################################

PRODUCT = ./RunTagDataEmbedded.lib
PRODUCT_TYPE = "static-library"
BUILD_TYPE = "Static Library"

###########################################################################
## INCLUDE PATHS
###########################################################################

INCLUDES_BUILDINFO = -I./

INCLUDES = $(INCLUDES_BUILDINFO)

###########################################################################
## DEFINES
###########################################################################

DEFINES_ = -D__MW_TARGET_USE_HARDWARE_RESOURCES_H__
DEFINES_CUSTOM = 
DEFINES_SKIPFORSIL = -DARM_PROJECT -D_USE_TARGET_UDP_ -D_RUNONTARGETHARDWARE_BUILD_ -DSTACK_SIZE=200000
DEFINES_STANDARD = -DMODEL=RunTagDataEmbedded

DEFINES = $(DEFINES_) $(DEFINES_CUSTOM) $(DEFINES_SKIPFORSIL) $(DEFINES_STANDARD)

###########################################################################
## SOURCE FILES
###########################################################################

SRCS = coder_fileops.c coder_platform.c RunTagDataEmbedded_data.c rt_nonfinite.c rtGetNaN.c rtGetInf.c RunTagDataEmbedded_initialize.c RunTagDataEmbedded_terminate.c RunTagDataEmbedded.c pwd1.c strcat.c fullfile.c hasIRIPrefix.c fileManager.c fread.c coderFread.c ifWhileCond.c mrdivide_helper.c mtimes.c minOrMax.c NoveldaChipParams.c string1.c NoveldaDDC.c linspace.c mean.c conv.c fft.c FFTImplementationCallback.c abs.c findpeaks.c eml_setop.c norm.c colon.c strcmp.c sin.c cos.c smoothdata.c circshift.c round.c ProcessFrames.c RunTagDataEmbedded_emxutil.c

ALL_SRCS = $(SRCS)

###########################################################################
## OBJECTS
###########################################################################

OBJS = coder_fileops.c.o coder_platform.c.o RunTagDataEmbedded_data.c.o rt_nonfinite.c.o rtGetNaN.c.o rtGetInf.c.o RunTagDataEmbedded_initialize.c.o RunTagDataEmbedded_terminate.c.o RunTagDataEmbedded.c.o pwd1.c.o strcat.c.o fullfile.c.o hasIRIPrefix.c.o fileManager.c.o fread.c.o coderFread.c.o ifWhileCond.c.o mrdivide_helper.c.o mtimes.c.o minOrMax.c.o NoveldaChipParams.c.o string1.c.o NoveldaDDC.c.o linspace.c.o mean.c.o conv.c.o fft.c.o FFTImplementationCallback.c.o abs.c.o findpeaks.c.o eml_setop.c.o norm.c.o colon.c.o strcmp.c.o sin.c.o cos.c.o smoothdata.c.o circshift.c.o round.c.o ProcessFrames.c.o RunTagDataEmbedded_emxutil.c.o

ALL_OBJS = $(OBJS)

###########################################################################
## PREBUILT OBJECT FILES
###########################################################################

PREBUILT_OBJS = 

###########################################################################
## LIBRARIES
###########################################################################

LIBS = 

###########################################################################
## SYSTEM LIBRARIES
###########################################################################

SYSTEM_LIBS = 

###########################################################################
## ADDITIONAL TOOLCHAIN FLAGS
###########################################################################

#---------------
# C Compiler
#---------------

CFLAGS_BASIC = $(DEFINES) $(INCLUDES)

CFLAGS += $(CFLAGS_BASIC)

#-----------------
# C++ Compiler
#-----------------

CPPFLAGS_BASIC = $(DEFINES) $(INCLUDES)

CPPFLAGS += $(CPPFLAGS_BASIC)

###########################################################################
## INLINED COMMANDS
###########################################################################


DERIVED_SRCS = $(subst .o,.dep,$(OBJS))

build:

%.dep:



-include codertarget_assembly_flags.mk
-include *.dep


###########################################################################
## PHONY TARGETS
###########################################################################

.PHONY : all build clean info prebuild download execute


all : build
	echo "### Successfully generated all binary outputs."


build : prebuild $(PRODUCT)


prebuild : 


download : $(PRODUCT)


execute : download


###########################################################################
## FINAL TARGET
###########################################################################

#---------------------------------
# Create a static library         
#---------------------------------

$(PRODUCT) : $(OBJS) $(PREBUILT_OBJS)
	echo "### Creating static library "$(PRODUCT)" ..."
	$(AR) $(ARFLAGS)  $(PRODUCT) $(OBJS)
	echo "### Created: $(PRODUCT)"


###########################################################################
## INTERMEDIATE TARGETS
###########################################################################

#---------------------
# SOURCE-TO-OBJECT
#---------------------

%.c.o : %.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.s.o : %.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.cpp.o : %.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.c.o : %.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.s.o : %.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.cpp.o : %.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.c.o : %.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.s.o : %.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.cpp.o : %.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.c.o : %.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.s.o : %.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.cpp.o : %.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


coder_fileops.c.o : coder_fileops.c
	$(CC) $(CFLAGS) -o "$@" "$<"


coder_platform.c.o : coder_platform.c
	$(CC) $(CFLAGS) -o "$@" "$<"


RunTagDataEmbedded_data.c.o : RunTagDataEmbedded_data.c
	$(CC) $(CFLAGS) -o "$@" "$<"


rt_nonfinite.c.o : rt_nonfinite.c
	$(CC) $(CFLAGS) -o "$@" "$<"


rtGetNaN.c.o : rtGetNaN.c
	$(CC) $(CFLAGS) -o "$@" "$<"


rtGetInf.c.o : rtGetInf.c
	$(CC) $(CFLAGS) -o "$@" "$<"


RunTagDataEmbedded_initialize.c.o : RunTagDataEmbedded_initialize.c
	$(CC) $(CFLAGS) -o "$@" "$<"


RunTagDataEmbedded_terminate.c.o : RunTagDataEmbedded_terminate.c
	$(CC) $(CFLAGS) -o "$@" "$<"


RunTagDataEmbedded.c.o : RunTagDataEmbedded.c
	$(CC) $(CFLAGS) -o "$@" "$<"


pwd1.c.o : pwd1.c
	$(CC) $(CFLAGS) -o "$@" "$<"


strcat.c.o : strcat.c
	$(CC) $(CFLAGS) -o "$@" "$<"


fullfile.c.o : fullfile.c
	$(CC) $(CFLAGS) -o "$@" "$<"


hasIRIPrefix.c.o : hasIRIPrefix.c
	$(CC) $(CFLAGS) -o "$@" "$<"


fileManager.c.o : fileManager.c
	$(CC) $(CFLAGS) -o "$@" "$<"


fread.c.o : fread.c
	$(CC) $(CFLAGS) -o "$@" "$<"


coderFread.c.o : coderFread.c
	$(CC) $(CFLAGS) -o "$@" "$<"


ifWhileCond.c.o : ifWhileCond.c
	$(CC) $(CFLAGS) -o "$@" "$<"


mrdivide_helper.c.o : mrdivide_helper.c
	$(CC) $(CFLAGS) -o "$@" "$<"


mtimes.c.o : mtimes.c
	$(CC) $(CFLAGS) -o "$@" "$<"


minOrMax.c.o : minOrMax.c
	$(CC) $(CFLAGS) -o "$@" "$<"


NoveldaChipParams.c.o : NoveldaChipParams.c
	$(CC) $(CFLAGS) -o "$@" "$<"


string1.c.o : string1.c
	$(CC) $(CFLAGS) -o "$@" "$<"


NoveldaDDC.c.o : NoveldaDDC.c
	$(CC) $(CFLAGS) -o "$@" "$<"


linspace.c.o : linspace.c
	$(CC) $(CFLAGS) -o "$@" "$<"


mean.c.o : mean.c
	$(CC) $(CFLAGS) -o "$@" "$<"


conv.c.o : conv.c
	$(CC) $(CFLAGS) -o "$@" "$<"


fft.c.o : fft.c
	$(CC) $(CFLAGS) -o "$@" "$<"


FFTImplementationCallback.c.o : FFTImplementationCallback.c
	$(CC) $(CFLAGS) -o "$@" "$<"


abs.c.o : abs.c
	$(CC) $(CFLAGS) -o "$@" "$<"


findpeaks.c.o : findpeaks.c
	$(CC) $(CFLAGS) -o "$@" "$<"


eml_setop.c.o : eml_setop.c
	$(CC) $(CFLAGS) -o "$@" "$<"


norm.c.o : norm.c
	$(CC) $(CFLAGS) -o "$@" "$<"


colon.c.o : colon.c
	$(CC) $(CFLAGS) -o "$@" "$<"


strcmp.c.o : strcmp.c
	$(CC) $(CFLAGS) -o "$@" "$<"


sin.c.o : sin.c
	$(CC) $(CFLAGS) -o "$@" "$<"


cos.c.o : cos.c
	$(CC) $(CFLAGS) -o "$@" "$<"


smoothdata.c.o : smoothdata.c
	$(CC) $(CFLAGS) -o "$@" "$<"


circshift.c.o : circshift.c
	$(CC) $(CFLAGS) -o "$@" "$<"


round.c.o : round.c
	$(CC) $(CFLAGS) -o "$@" "$<"


ProcessFrames.c.o : ProcessFrames.c
	$(CC) $(CFLAGS) -o "$@" "$<"


RunTagDataEmbedded_emxutil.c.o : RunTagDataEmbedded_emxutil.c
	$(CC) $(CFLAGS) -o "$@" "$<"


###########################################################################
## DEPENDENCIES
###########################################################################

$(ALL_OBJS) : rtw_proj.tmw $(MAKEFILE)


###########################################################################
## MISCELLANEOUS TARGETS
###########################################################################

info : 
	echo "### PRODUCT = $(PRODUCT)"
	echo "### PRODUCT_TYPE = $(PRODUCT_TYPE)"
	echo "### BUILD_TYPE = $(BUILD_TYPE)"
	echo "### INCLUDES = $(INCLUDES)"
	echo "### DEFINES = $(DEFINES)"
	echo "### ALL_SRCS = $(ALL_SRCS)"
	echo "### ALL_OBJS = $(ALL_OBJS)"
	echo "### LIBS = $(LIBS)"
	echo "### MODELREF_LIBS = $(MODELREF_LIBS)"
	echo "### SYSTEM_LIBS = $(SYSTEM_LIBS)"
	echo "### TOOLCHAIN_LIBS = $(TOOLCHAIN_LIBS)"
	echo "### ASFLAGS = $(ASFLAGS)"
	echo "### CFLAGS = $(CFLAGS)"
	echo "### LDFLAGS = $(LDFLAGS)"
	echo "### SHAREDLIB_LDFLAGS = $(SHAREDLIB_LDFLAGS)"
	echo "### CPPFLAGS = $(CPPFLAGS)"
	echo "### CPP_LDFLAGS = $(CPP_LDFLAGS)"
	echo "### CPP_SHAREDLIB_LDFLAGS = $(CPP_SHAREDLIB_LDFLAGS)"
	echo "### ARFLAGS = $(ARFLAGS)"
	echo "### MEX_CFLAGS = $(MEX_CFLAGS)"
	echo "### MEX_CPPFLAGS = $(MEX_CPPFLAGS)"
	echo "### MEX_LDFLAGS = $(MEX_LDFLAGS)"
	echo "### MEX_CPPLDFLAGS = $(MEX_CPPLDFLAGS)"
	echo "### DOWNLOAD_FLAGS = $(DOWNLOAD_FLAGS)"
	echo "### EXECUTE_FLAGS = $(EXECUTE_FLAGS)"
	echo "### MAKE_FLAGS = $(MAKE_FLAGS)"


clean : 
	$(ECHO) "### Deleting all derived files ..."
	$(RM) $(PRODUCT)
	$(RM) $(ALL_OBJS)
	$(RM) *.c.dep
	$(RM) *.cpp.dep
	$(ECHO) "### Deleted all derived files."


