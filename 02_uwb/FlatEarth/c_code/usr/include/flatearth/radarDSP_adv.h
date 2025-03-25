/**
   @file radarDSP_adv.h

   The RadarDSP library provides a number of very common yet useful functions that in many cases are trivial to implement, but clog up the code with "for" loops that make the code more difficult to follow.

   This library also provides libraries for generating an assortment of window functions, signal statistics, and filtering.

   @copyright 2015 by FlatEarth, Inc

   @par Environment
   Environment Independent

   @par Compiler
   Compiler Independent

   @author Raymond Weber
   @author Justin Hadella
*/

#ifndef RADARDSP_ADV_h
#define RADARDSP_ADV_h


#ifdef __cplusplus
extern "C"
{
#endif





/** FFT using NE10 library (ARM only)
  @param real Real components of the input and output
  @param im Imaginary components of the input and output
  @param length Length of the FFT
  @return SUCCESS or error code
  @ingroup radarDSP_advanced_fft_ne10
*/
int radarDSP_FFT_ne10(int *real, int *im, int length);


/** Hilbert transform using NE10 library (ARM only)
  @param real Real components of the input and output
  @param im Imaginary components of the input and output
  @param length Length of the transform
  @return SUCCESS or error code
  @ingroup radarDSP_advanced_fft_ne10
*/
int radarDSP_Hilbert_ne10(int *real, int *im, int length);


/** IFFT using NE10 library (ARM only)
  @param real Real components of the input and output
  @param im Imaginary components of the input and output
  @param length Length of the IFFT
  @return SUCCESS or error code
  @warning The FFT is floats and simply casts to doubles so the accuracy will be lower then a true double FFT
  @ingroup radarDSP_advanced_fft_ne10
*/
int radarDSP_IFFT_ne10(int *real, int *im, int length);


/** FFT using NE10 library (ARM only)
  @param real Real components of the input and output
  @param im Imaginary components of the input and output
  @param length Length of the FFT
  @return SUCCESS or error code
  @warning The FFT is floats and simply casts to doubles so the accuracy will be lower then a true double FFT
  @ingroup radarDSP_advanced_fft_ne10
*/
int radarDSP_sp_FFT_ne10(float *real, float *im, int length);


/** Hilbert transform using NE10 library (ARM only)
  @param real Real components of the input and output
  @param im Imaginary components of the input and output
  @param length Length of the transform
  @return SUCCESS or error code
  @ingroup radarDSP_advanced_fft_ne10
*/
int radarDSP_sp_Hilbert_ne10(float *real, float *im, int length);


/** IFFT using NE10 library (ARM only)
  @param real Real components of the input and output
  @param im Imaginary components of the input and output
  @param length Length of the IFFT
  @return SUCCESS or error code
  @ingroup radarDSP_advanced_fft_ne10
*/
int radarDSP_sp_IFFT_ne10(float *real, float *im, int length);


/** FFT using NE10 library (ARM only)
  @param data Interlaced Re-Im array
  @param length Length of the FFT
  @return SUCCESS or error code
  @ingroup radarDSP_advanced_fft_ne10
*/
int radarDSP_spz_FFT_ooura(float *data, int length);


/** IFFT using NE10 library (ARM only)
  @param data Interlaced Re-Im array
  @param length Length of the IFFT
  @return SUCCESS or error code
  @ingroup radarDSP_advanced_fft_ne10
*/
int radarDSP_spz_IFFT_ooura(float *data, int length);


/** Hilbert using NE10 library (ARM only)
  @param data Interlaced Re-Im array
  @param length Length of the transform
  @return SUCCESS or error code
  @ingroup radarDSP_advanced_fft_ne10
*/
int radarDSP_spz_Hilbert_ooura(float *data, int length);


/** FFT using NE10 library (ARM only)
  @param real Real components of the input and output
  @param im Imaginary components of the input and output
  @param length Length of the FFT
  @return SUCCESS or error code
  @warning The FFT is floats and simply casts to doubles so the accuracy will be lower then a true double FFT
  @ingroup radarDSP_advanced_fft_ne10
*/
int radarDSP_dp_FFT_ne10(double *real, double *im, int length);


/** Hilbert transform using NE10 library (ARM only)
  @param real Real components of the input and output
  @param im Imaginary components of the input and output
  @param length Length of the transform
  @return SUCCESS or error code
  @warning The FFT is floats and simply casts to doubles so the accuracy will be lower then a true double FFT
  @ingroup radarDSP_advanced_fft_ne10
*/
int radarDSP_dp_Hilbert_ne10(double *real, double *im, int length);


/** IFFT using NE10 library (ARM only)
  @param real Real components of the input and output
  @param im Imaginary components of the input and output
  @param length Length of the IFFT
  @return SUCCESS or error code
  @warning The FFT is floats and simply casts to doubles so the accuracy will be lower then a true double FFT
  @ingroup radarDSP_advanced_fft_ne10
*/
int radarDSP_dp_IFFT_ne10(double *real, double *im, int length);

/** FFT using NE10 library (ARM only)
  @param data Input/Output complex interlaced data
  @param length Length of the FFT
  @return SUCCESS or error code
  @warning The FFT is floats and simply casts to doubles so the accuracy will be lower then a true double FFT
  @ingroup radarDSP_advanced_fft_ne10
*/
int radarDSP_dpz_FFT_ne10(double *data, int length);


/** Hilbert transform using NE10 library (ARM only)
  @param data Input/Output complex interlaced data
  @param length Length of the transform
  @return SUCCESS or error code
  @warning The FFT is floats and simply casts to doubles so the accuracy will be lower then a true double FFT
  @ingroup radarDSP_advanced_fft_ne10
*/
int radarDSP_dpz_Hilbert_ne10(double *data, int length);


/** IFFT using NE10 library (ARM only)
  @param data Input/Output complex interlaced data
  @param length Length of the IFFT
  @return SUCCESS or error code
  @warning The FFT is floats and simply casts to doubles so the accuracy will be lower then a true double FFT
  @ingroup radarDSP_advanced_fft_ne10
*/
int radarDSP_dpz_IFFT_ne10(double *data, int length);

/** FFT using NE10 library (ARM only)
  @param data Interlaced Re-Im array
  @param length Length of the FFT
  @return SUCCESS or error code
  @warning The FFT is floats and simply casts to doubles so the accuracy will be lower then a true double FFT
  @ingroup radarDSP_advanced_fft_ne10
*/
int radarDSP_dpz_FFT_ooura(double *data, int length);


/** IFFT using NE10 library (ARM only)
  @param data Interlaced Re-Im array
  @param length Length of the IFFT
  @return SUCCESS or error code
  @warning The FFT is floats and simply casts to doubles so the accuracy will be lower then a true double FFT
  @ingroup radarDSP_advanced_fft_ne10
*/
int radarDSP_dpz_IFFT_ooura(double *data, int length);


/** Hilbert using NE10 library (ARM only)
  @param data Interlaced Re-Im array
  @param length Length of the transform
  @return SUCCESS or error code
  @warning The FFT is floats and simply casts to doubles so the accuracy will be lower then a true double FFT
  @ingroup radarDSP_advanced_fft_ne10
*/
int radarDSP_dpz_Hilbert_ooura(double *data, int length);







/** Do an integer precision FFT using the double precision ooura FFT library, in place
  @param real Real input and output matrix
  @param im Imaginary input and output matrix
  @param length Length of the FFT to perform
  @return SUCCESS or error code
  @ingroup radarDSP_advanced_fft_ooura
*/
int radarDSP_FFT_ooura(int *real, int *im, int length);


/** Do an integer precision IFFT using the double precision ooura FFT library, in place
  @param real Real input and output matrix
  @param im Imaginary input and output matrix
  @param length Length of the FFT to perform
  @return SUCCESS or error code
  @ingroup radarDSP_advanced_fft_ooura
*/
int radarDSP_IFFT_ooura(int *real, int *im, int length);







/** Do an integer precision Hilbert Transform using the double precision ooura FFT library, in place
  @param real Real input and output matrix
  @param im Imaginary input and output matrix
  @param length Length of the FFT to perform
  @return SUCCESS or error code
  @ingroup radarDSP_advanced_fft_ooura
*/
int radarDSP_Hilbert_ooura(int *real, int *im, int length);


/** Do a float precision FFT using the double precision ooura FFT library, in place
  @param real Real input and output matrix
  @param im Imaginary input and output matrix
  @param length Length of the FFT to perform
  @return SUCCESS or error code
  @ingroup radarDSP_advanced_fft_ooura
*/
int radarDSP_sp_FFT_ooura(float *real, float *im, int length);


/** Do a float precision IFFT using the double precision ooura FFT library, in place
  @param real Real input and output matrix
  @param im Imaginary input and output matrix
  @param length Length of the FFT to perform
  @return SUCCESS or error code
  @ingroup radarDSP_advanced_fft_ooura
*/
int radarDSP_sp_IFFT_ooura(float *real, float *im, int length);


/** Do a float precision Hilbert using the double precision ooura FFT library, in place
  @param real Real input and output matrix
  @param im Imaginary input and output matrix
  @param length Length of the FFT to perform
  @return SUCCESS or error code
  @ingroup radarDSP_advanced_fft_ooura
*/
int radarDSP_sp_Hilbert_ooura(float *real, float *im, int length);


/** Do a double precision FFT using the double precision ooura FFT library, in place
  @param real Real input and output matrix
  @param im Imaginary input and output matrix
  @param length Length of the FFT to perform
  @return SUCCESS or error code
  @ingroup radarDSP_advanced_fft_ooura
*/
int radarDSP_dp_FFT_ooura(double *real, double *im, int length);


/** Do a double precision IFFT using the double precision ooura FFT library, in place
  @param real Real input and output matrix
  @param im Imaginary input and output matrix
  @param length Length of the FFT to perform
  @return SUCCESS or error code
  @ingroup radarDSP_advanced_fft_ooura
*/


int radarDSP_dp_IFFT_ooura(double *real, double *im, int length);
/** Do a double precision Hilbert using the double precision ooura FFT library, in place
  @param real Real input and output matrix
  @param im Imaginary input and output matrix
  @param length Length of the FFT to perform
  @return SUCCESS or error code
  @ingroup radarDSP_advanced_fft_ooura
*/
int radarDSP_dp_Hilbert_ooura(double *real, double *im, int length);


/** Do a integer precision FFT using the float precision KISSFFT library, in place
  @param real Real input and output matrix
  @param im Imaginary input and output matrix
  @param length Length of the FFT to perform
  @return SUCCESS or error code
  @ingroup radarDSP_advanced_fft_kissfft
*/
int radarDSP_FFT_kissfft(int *real, int *im, int length);


/** Do a integer precision IFFT using the float precision KISSFFT library, in place
  @param real Real input and output matrix
  @param im Imaginary input and output matrix
  @param length Length of the FFT to perform
  @return SUCCESS or error code
  @ingroup radarDSP_advanced_fft_kissfft
*/
int radarDSP_IFFT_kissfft(int *real, int *im, int length);


/** Do a integer precision Hilbert using the float precision KISSFFT library, in place
  @param real Real input and output matrix
  @param im Imaginary input and output matrix
  @param length Length of the FFT to perform
  @return SUCCESS or error code
  @ingroup radarDSP_advanced_fft_kissfft
*/
int radarDSP_Hilbert_kissfft(int *real, int *im, int length);


/** Do a float precision FFT using the float precision KISSFFT library, in place
  @param real Real input and output matrix
  @param im Imaginary input and output matrix
  @param length Length of the FFT to perform
  @return SUCCESS or error code
  @ingroup radarDSP_advanced_fft_kissfft
*/
int radarDSP_sp_FFT_kissfft(float *real, float *im, int length);


/** Do a float precision IFFT using the float precision KISSFFT library, in place
  @param real Real input and output matrix
  @param im Imaginary input and output matrix
  @param length Length of the data to perform
  @return SUCCESS or error code
  @ingroup radarDSP_advanced_fft_kissfft
*/
int radarDSP_sp_IFFT_kissfft(float *real, float *im, int length);


/** Do a float precision Hilbert using the float precision KISSFFT library, in place
  @param real Real input and output matrix
  @param im Imaginary input and output matrix
  @param length Length of the data to perform
  @return SUCCESS or error code
  @ingroup radarDSP_advanced_fft_kissfft
*/
int radarDSP_sp_Hilbert_kissfft(float *real, float *im, int length);


/** Do a double precision FFT using the float precision KISSFFT library, in place
  @param real Real input and output matrix
  @param im Imaginary input and output matrix
  @param length Length of the data to perform
  @return SUCCESS or error code
  @ingroup radarDSP_advanced_fft_kissfft
*/
int radarDSP_dp_FFT_kissfft(double *real, double *im, int length);


/** Do a double precision IFFT using the float precision KISSFFT library, in place
  @param real Real input and output matrix
  @param im Imaginary input and output matrix
  @param length Length of the data to perform
  @return SUCCESS or error code
  @ingroup radarDSP_advanced_fft_kissfft
*/
int radarDSP_dp_IFFT_kissfft(double *real, double *im, int length);


/** Do a double precision Hilbert using the float precision KISSFFT library, in place
  @param real Real input and output matrix
  @param im Imaginary input and output matrix
  @param length Length of the data to perform
  @return SUCCESS or error code
  @ingroup radarDSP_advanced_fft_kissfft
*/
int radarDSP_dp_Hilbert_kissfft(double *real, double *im, int length);






/** Perform an FFT using the pffft library
  @param real Real part of the input/output matricies
  @param im Imaginary part of the input/output matricies
  @param length Length of the data vector
  @warning length must be a multiple of 4 and >16?
  @warning Internally this is a float fft and rounding may result on other datatypes
  @ingroup radarDSP_advanced_fft_pffft
*/
int radarDSP_FFT_pffft(int *real, int *im, int length);

/** Perform an IFFT using the pffft library
  @param real Real part of the input/output matricies
  @param im Imaginary part of the input/output matricies
  @param length Length of the data vector
  @warning length must be a multiple of 4 and >16?
  @warning Internally this is a float fft and rounding may result on other datatypes
  @ingroup radarDSP_advanced_fft_pffft
*/
int radarDSP_IFFT_pffft(int *real, int *im, int length);

/** Perform an Hilbert Transform using the pffft library
  @param real Real part of the input/output matricies
  @param im Imaginary part of the input/output matricies
  @param length Length of the data vector
  @warning length must be a multiple of 4 and >16?
  @warning Internally this is a float fft and rounding may result on other datatypes
  @ingroup radarDSP_advanced_fft_pffft
*/
int radarDSP_Hilbert_pffft(int *real, int *im, int length);

/** Perform an FFT using the pffft library
  @param data Interlaced complex input/output matricies
  @param length Length of the data vector
  @warning length must be a multiple of 4 and >16?
  @warning Internally this is a float fft and rounding may result on other datatypes
  @ingroup radarDSP_advanced_fft_pffft
*/
int radarDSP_z_FFT_pffft(int *data, int length);

/** Perform an IFFT using the pffft library
  @param data Interlaced complex input/output matricies
  @param length Length of the data vector
  @warning length must be a multiple of 4 and >16?
  @warning Internally this is a float fft and rounding may result on other datatypes
  @ingroup radarDSP_advanced_fft_pffft
*/
int radarDSP_z_IFFT_pffft(int *data, int length);

/** Perform a Hilbert Transform using the pffft library
  @param data Interlaced complex input/output matricies
  @param length Length of the data vector
  @warning length must be a multiple of 4 and >16?
  @warning Internally this is a float fft and rounding may result on other datatypes
  @ingroup radarDSP_advanced_fft_pffft
*/
int radarDSP_z_Hilbert_pffft(int *data, int length);



/** Perform an FFT using the pffft library
  @param real Real part of the input/output matricies
  @param im Imaginary part of the input/output matricies
  @param length Length of the data vector
  @warning length must be a multiple of 4 and >16?
  @ingroup radarDSP_advanced_fft_pffft
*/
int radarDSP_sp_FFT_pffft(float *real, float *im, int length);

/** Perform an IFFT using the pffft library
  @param real Real part of the input/output matricies
  @param im Imaginary part of the input/output matricies
  @param length Length of the data vector
  @warning length must be a multiple of 4 and >16?
  @ingroup radarDSP_advanced_fft_pffft
*/
int radarDSP_sp_IFFT_pffft(float *real, float *im, int length);

/** Perform an Hilbert Transform using the pffft library
  @param real Real part of the input/output matricies
  @param im Imaginary part of the input/output matricies
  @param length Length of the data vector
  @warning length must be a multiple of 4 and >16?
  @ingroup radarDSP_advanced_fft_pffft
*/
int radarDSP_sp_Hilbert_pffft(float *real, float *im, int length);

/** Perform an FFT using the pffft library
  @param data Interlaced complex input/output matricies
  @param length Length of the data vector
  @warning length must be a multiple of 4 and >16?
  @ingroup radarDSP_advanced_fft_pffft
*/
int radarDSP_spz_FFT_pffft(float *data, int length);

/** Perform an IFFT using the pffft library
  @param data Interlaced complex input/output matricies
  @param length Length of the data vector
  @warning length must be a multiple of 4 and >16?
  @ingroup radarDSP_advanced_fft_pffft
*/
int radarDSP_spz_IFFT_pffft(float *data, int length);

/** Perform a Hilbert Transform using the pffft library
  @param data Interlaced complex input/output matricies
  @param length Length of the data vector
  @warning length must be a multiple of 4 and >16?
  @ingroup radarDSP_advanced_fft_pffft
*/
int radarDSP_spz_Hilbert_pffft(float *data, int length);



/** Perform an FFT using the pffft library
  @param real Real part of the input/output matricies
  @param im Imaginary part of the input/output matricies
  @param length Length of the data vector
  @warning length must be a multiple of 4 and >16?
  @warning Internally this is a float fft and rounding may result on other datatypes
  @ingroup radarDSP_advanced_fft_pffft
*/
int radarDSP_dp_FFT_pffft(double *real, double *im, int length);

/** Perform an IFFT using the pffft library
  @param real Real part of the input/output matricies
  @param im Imaginary part of the input/output matricies
  @param length Length of the data vector
  @warning length must be a multiple of 4 and >16?
  @warning Internally this is a float fft and rounding may result on other datatypes
  @ingroup radarDSP_advanced_fft_pffft
*/
int radarDSP_dp_IFFT_pffft(double *real, double *im, int length);

/** Perform an Hilbert Transform using the pffft library
  @param real Real part of the input/output matricies
  @param im Imaginary part of the input/output matricies
  @param length Length of the data vector
  @warning length must be a multiple of 4 and >16?
  @warning Internally this is a float fft and rounding may result on other datatypes
  @ingroup radarDSP_advanced_fft_pffft
*/
int radarDSP_dp_Hilbert_pffft(double *real, double *im, int length);

/** Perform an FFT using the pffft library
  @param data Interlaced complex input/output matricies
  @param length Length of the data vector
  @warning length must be a multiple of 4 and >16?
  @warning Internally this is a float fft and rounding may result on other datatypes
  @ingroup radarDSP_advanced_fft_pffft
*/
int radarDSP_dpz_FFT_pffft(double *data, int length);

/** Perform an IFFT using the pffft library
  @param data Interlaced complex input/output matricies
  @param length Length of the data vector
  @warning length must be a multiple of 4 and >16?
  @warning Internally this is a float fft and rounding may result on other datatypes
  @ingroup radarDSP_advanced_fft_pffft
*/
int radarDSP_dpz_IFFT_pffft(double *data, int length);

/** Perform a Hilbert Transform using the pffft library
  @param data Interlaced complex input/output matricies
  @param length Length of the data vector
  @warning length must be a multiple of 4 and >16?
  @warning Internally this is a float fft and rounding may result on other datatypes
  @ingroup radarDSP_advanced_fft_pffft
*/
int radarDSP_dpz_Hilbert_pffft(double *data, int length);













/**
  @defgroup radarDSP_advanced Advanced Math Functions
  Lower level functions where the exact library and method can be selected for an operation and may have fewer bound checks.
  @warning May have fewer safety checks and may be more suseptable to crashes.
  @warning These functions are not recommended for most users and may change more over time then the standard API.
  @warning These functions may simply return error codes if they are not compatible or built for spacific architectures.  The standard API will always use a good library for the architecture.

  @ingroup radarDSP
*/


/**
  @defgroup radarDSP_advanced_fft FFT Functions
  Lower level functions where the exact library and method can be selected for an operation and may have fewer bound checks.

  @ingroup radarDSP_advanced
*/



/**
  @defgroup radarDSP_advanced_fft_ne10 NE10 FFTs
  Lower level functions where the exact library and method can be selected for an operation and may have fewer bound checks.

  @ingroup radarDSP_advanced_fft
*/

/**
  @defgroup radarDSP_advanced_fft_kissfft KISSFFT FFTs
  Lower level functions where the exact library and method can be selected for an operation and may have fewer bound checks.

  @ingroup radarDSP_advanced_fft
*/

/**
  @defgroup radarDSP_advanced_fft_ooura Ooura FFTs
  Lower level functions where the exact library and method can be selected for an operation and may have fewer bound checks.

  @ingroup radarDSP_advanced_fft
*/

/**
  @defgroup radarDSP_advanced_fft_pffft PFFFT FFTs
  Lower level functions where the exact library and method can be selected for an operation and may have fewer bound checks.

  @ingroup radarDSP_advanced_fft
*/


#ifdef __cplusplus
}
#endif


#endif
