/**
   @file radarDSP.h

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

#ifndef RADARDSP_h
#define RADARDSP_h

#include <stdio.h>
#include <string.h>
#include <float.h>
#include <math.h>

#if defined(__GNUC__)
#define DEPRECATED __attribute__((deprecated)) /**< mark a function as deprecated in gcc. */
#endif

#ifdef __cplusplus
extern "C"
{
#endif


/** complex integer datatype (C99 Compatible) */
typedef struct
{
  int r;    /**< Real part */
  int i;    /**< Imaginary part */
} salsa_complex_int;

/** complex float datatype (C99 Compatible) */
typedef struct
{
  float r;  /**< Real part */
  float i;  /**< Imaginary part */
} salsa_complex_float;

/** complex double datatype (C99 Compatible) */
typedef struct
{
  double r;  /**< Real part */
  double i;  /**< Imaginary part */
} salsa_complex_double;


/**
  Structure to hold all the variables needed to create an integer IIR filter
  @ingroup radarDSP_filters
*/
typedef struct int_filter_conf
{
  int *a;           //!< Numerator coefficients of the transfer function
  int *b;           //!< Denominator coefficients of the transfer function
  int an;           //!< Number of Numerator Coefficients
  int bn;           //!< Number of Denominator Coefficients
  int *ataps;       //!< Internal memory for the filter
  int *btaps;       //!< Internal memory for the filter
} int_filter_conf;

/** Structures for linked list based datatypes
*/
typedef struct fenode
{
  void *data;               //!< Pointer to the data for the node in the linked list
  struct fenode *next;      //!< Pointer to the next element in the linked list
  struct fenode *prev;      //!< Pointer to the previous element in the linked list
  int length;               //!< Number of elements in the linked list
} fenode_t;

/** Structure to make a circular buffer with a linked list
*/
typedef struct circBuffer
{
  fenode_t *currNodeR;      //!< Current node in the circular buffer
  fenode_t *currNode;       //!< Current node in the circular buffer
  int items;                //!< Number of elements in the circular buffer
} circBuffer_t;

/** Structures a simple data buffer
*/
typedef struct circBuffer_simple
{
  void *data;               //!< Pointer to the data for the node in the linked list
  int bufferLength;         //!< Size of the buffer in bytes
  int currentElement;       //!< Start point of the buffer
  int elements;             //!< Number of elements in the buffer
} circBuffer_simple_t;

/**
  Structure to hold all the variables needed to create a single precision IIR filter
  @ingroup radarDSP_filters
*/
typedef struct sp_filter_conf
{
  float *a;         //!< Numerator coefficients of the transfer function
  float *b;         //!< Denominator coefficients of the transfer function
  int an;           //!< Number of Numerator Coefficients
  float *taps;      //!< Internal memory for the filter
} sp_filter_conf;

/**
  Structure to hold all the variables needed to create a double precision FIR filter
  @ingroup radarDSP_filters
*/
typedef struct dp_filter_conf
{
  double *a;        //!< Numerator coefficients of the transfer function
  double *b;        //!< Denominator coefficients of the transfer function
  int an;           //!< Number of Numerator Coefficients
  double *taps;     //!< Internal memory for the filter
} dp_filter_conf;

/** Rectangular Window Function */
#define SALSA_RECTANGULARWINDOW 0
/** Bartlett Window Function */
#define SALSA_BARTLETTWINDOW 1
/** Welch Window Function */
#define SALSA_WELCHWINDOW 2
/** Hann Window Function */
#define SALSA_HANNWINDOW 3
/** Hamming Window Function */
#define SALSA_HAMMINGWINDOW 4
/** Bohman Window Function */
#define SALSA_BOHMANWINDOW 5
/** Blackman Window Function */
#define SALSA_BLACKMANWINDOW 6
/** Blackman Harris Window Function */
#define SALSA_BLACKMANHARRISWINDOW 7
/** Blackman Nuttall Window Function */
#define SALSA_BLACKMANNUTTALLWINDOW 8
/** Gaussian Window Function */
#define SALSA_GAUSSIANWINDOW 9
/** Flat-Top Window Function */
#define SALSA_FLATTOPWINDOW 10


// Might want these to live somewhere else?
/** Convert an int array to a double array
  @param A An array of integers (input)
  @param B An array of doubles (output array)
  @param An Length of the array
  @ingroup radarDSP_conversions
*/
int radarDSP_convertIntArray(int *A, double *B, int An);

/** Convert an unsigned int array to a double array
  @param A An array of integers (input)
  @param B An array of doubles (output array)
  @param An Length of the array
  @ingroup radarDSP_conversions
*/
int radarDSP_convertUIntArray(unsigned int *A, double *B, int An);

// Might want these to live somewhere else?
/** Convert an int array to a double array
  @param A An array of integers (input)
  @param B An array of doubles (output array)
  @param An Length of the array
  @ingroup radarDSP_conversions
*/
int radarDSP_convertIntArrayToFloat(int *A, float *B, int An);

/** Convert a unsigned int array to a double array
  @param A An array of integers (input)
  @param B An array of doubles (output array)
  @param An Length of the array
  @ingroup radarDSP_conversions
*/
int radarDSP_convertUIntArrayToFloat(unsigned int *A, float *B, int An);

/** Generic FIR filter implementation
  @param  *B Array of B coefficients
  @param  *Bn Number of B coefficients
  @param  *A Array of A coefficients
  @param  *An Number of A coefficients
  @param  *X Input data array
  @param  *Xn Length of input array
  @param  *Xp Pass by reference return of the filtered signal
  @note These functions are not recommended for future use. The radarDSP_dp_IIR_Filter series of functions is more stable and faster.  This function may be deprecated in the future.
  @ingroup radarDSP_filters
  @note Using radarDSP_dp_fir_init(), radarDSP_dp_fir(), and radarDSP_dp_fir_free() is preferred when using the filter more then once and result in higher performant code.
*/
int radarDSP_dp_Filter(double *B, int Bn, double *A, int An, double *X, int Xn, double *Xp);

/** Generic FIR filter implementation
  @param  *B Array of B coefficients
  @param  *Bn Number of B coefficients
  @param  *A Array of A coefficients
  @param  *An Number of A coefficients
  @param  *X Input data array
  @param  *Xn Length of input array
  @param  *Xp Pass by reference return of the filtered signal
  @note Using radarDSP_sp_fir_init(), radarDSP_sp_fir(), and radarDSP_sp_fir_free() is preferred when using the filter more then once and result in higher performant code.
  @ingroup radarDSP_filters
  @deprecated
*/
DEPRECATED int radarDSP_sp_Filter(float *B, int Bn, float *A, int An, float *X, int Xn, float *Xp);

/** Generic FIR filter implementation
  @param  *B Array of B coefficients
  @param  *Bn Number of B coefficients
  @param  *A Array of A coefficients
  @param  *An Number of A coefficients
  @param  *X Input data array
  @param  *Xn Length of input array
  @param  *Xp Pass by reference return of the filtered signal
  @note Using radarDSP_int_fir_init(), radarDSP_int_fir(), and radarDSP_int_fir_free() is preferred when using the filter more then once and result in higher performant code.
  @ingroup radarDSP_filters
  @deprecated
*/
DEPRECATED int radarDSP_Filter(double *B, int Bn, double *A, int An, int *X, int Xn, int *Xp);

/** Generic FIR filter implementation (Same input size as output size)
  @param *B  Array of B coefficients

  @param Bn  Bn Number of B coefficients
  @param *X  Input data array
  @param Xn  Input data array size
  @param *Xp  Ouput Data Array
  @ingroup radarDSP_filters
*/
int radarDSP_FIR_Filter(int *B, int Bn, int *X, int Xn, int *Xp);

/** Generic FIR filter implementation (Same input size as output size)
  @param *B  Array of B coefficients
  @param Bn  Bn Number of B coefficients
  @param *X  Input data array
  @param Xn  Input data array size
  @param *Xp  Ouput Data Array
  @ingroup radarDSP_filters
*/
int radarDSP_sp_FIR_Filter(float *B, int Bn, float *X, int Xn, float *Xp);

/** Generic FIR filter implementation (Same input size as output size)
  @param *B  Array of B coefficients
  @param Bn  Bn Number of B coefficients
  @param *X  Input data array
  @param Xn  Input data array size
  @param *Xp  Ouput Data Array
  @ingroup radarDSP_filters
*/
int radarDSP_dp_FIR_Filter(double *B, int Bn, double *X, int Xn, double *Xp);

/** Find the maximum value and index in the array
  @param *X    Input data array
  @param Xn    Input data array size
  @param *maxValue  Pass by reference double with the max value in the array
  @param *maxInd  Pass by reference integer with the max index in the array
  @ingroup radarDSP_statistics
*/
int radarDSP_Max(int *X, int Xn, int *maxValue, int *maxInd);

/** Find the maximum value and index in the array
  @param *X    Input data array
  @param Xn    Input data array size
  @param *maxValue  Pass by reference double with the max value in the array
  @param *maxInd  Pass by reference integer with the max index in the array
  @ingroup radarDSP_statistics
*/
int radarDSP_sp_Max(float *X, int Xn, float *maxValue, int *maxInd);

/** Find the maximum value and index in the array
  @param *X    Input data array
  @param Xn    Input data array size
  @param *maxValue  Pass by reference double with the max value in the array
  @param *maxInd  Pass by reference integer with the max index in the array
  @ingroup radarDSP_statistics
*/
int radarDSP_dp_Max(double *X, int Xn, double *maxValue, int *maxInd);

//----------------------

/** Find the minimum value and index in the array
  @param *X    Input data array
  @param Xn    Input data array size
  @param *minValue  Pass by reference double with the min value in the array
  @param *minInd  Pass by reference integer with the min index in the array
  @ingroup radarDSP_statistics
*/
int radarDSP_Min(int *X, int Xn, int *minValue, int *minInd);

/** Find the minimum value and index in the array
  @param *X    Input data array
  @param Xn    Input data array size
  @param *minValue  Pass by reference double with the min value in the array
  @param *minInd  Pass by reference integer with the min index in the arrays
  @ingroup radarDSP_statistics
*/
int radarDSP_sp_Min(float *X, int Xn, float *minValue, int *minInd);

/** Find the minimum value and index in the array
  @param *X    Input data array
  @param Xn    Input data array size
  @param *minValue  Pass by reference double with the min value in the array
  @param *minInd  Pass by reference integer with the min index in the array
  @ingroup radarDSP_statistics
*/
int radarDSP_dp_Min(double *X, int Xn, double *minValue, int *minInd);

//----------------------

/** Sum up the array
  @param *X    Input data array
  @param Xn    Input data array size
  @param *sum  The sum of the array
  @ingroup radarDSP_statistics
*/
int radarDSP_Sum(int *X, int Xn, int *sum);

/** Sum up the array
  @param *X    Input data array
  @param Xn    Input data array size
  @param *sum  The sum of the array
  @ingroup radarDSP_statistics
*/
int radarDSP_sp_Sum(float *X, int Xn, float *sum);

/** Sum up the array
  @param *X    Input data array
  @param Xn    Input data array size
  @param *sum  The sum of the array
  @ingroup radarDSP_statistics
*/
int radarDSP_dp_Sum(double *X, int Xn, double *sum);


/** Sum up the array
  @param *X    Input data array
  @param Xn    Input data array size
  @param *sum  The sum of the array
  @ingroup radarDSP_statistics
*/
int radarDSP_z_Sum(salsa_complex_int *X, int Xn, salsa_complex_int *sum);

/** Sum up the array
  @param *X    Input data array
  @param Xn    Input data array size
  @param *sum  The sum of the array
  @ingroup radarDSP_statistics
*/
int radarDSP_spz_Sum(salsa_complex_float *X, int Xn, salsa_complex_float *sum);

/** Sum up the array
  @param *X    Input data array
  @param Xn    Input data array size
  @param *sum  The sum of the array
  @ingroup radarDSP_statistics
*/
int radarDSP_dpz_Sum(salsa_complex_double *X, int Xn, salsa_complex_double *sum);


//----------------

/** Run a Digital-Down-Conversion on a real signal to extract I/Q channels
  @param X    Real double signal
  @param Xn    Length of the signal
  @param chip  Radar chip used (0=X1 Low, 1=X1 Mid, 2=X2)
  @param pgen  Pulse Generator number to use (X2 only: 0-10)
  @param sampler The Samper to use (X1 only: 0=4mm, 1=8mm, 2=4cm range bins)
  @param image    I Component of the DDC signal
  @param quadrature    Q component of the DDC signal

  @ingroup radarDSP_ddc
*/
int radarDSP_DDC(double *X, int Xn, int chip, int sampler, int pgen, double *image, double *quadrature);


/** Run a Digital-Down-Conversion on a real signal to extract I/Q channels
  @param X    Real signal
  @param Xn    Length of the signal
  @param chip  Radar chip used (0=X1 Low, 1=X1 Mid, 2=X2)
  @param pgen  Pulse Generator number to use (X2 only: 0-10)
  @param sampler The Samper to use (X1 only: 0=4mm, 1=8mm, 2=4cm range bins)
  @param image    I Component of the DDC signal
  @param quadrature    Q component of the DDC signal

  @ingroup radarDSP_ddc
*/
int radarDSP_sp_DDC(float *X, int Xn, int chip, int sampler, int pgen, float *image, float *quadrature);



/**
  Compute an IFFT on the signal
  @param real The real component of the input and output signals
  @param im The imaginary components of the input and output signals
  @param length The length of the signal, MUST BE BASE 2

  @note The transform is performed in-place and will overwrite the input signals
  @ingroup radarDSP_fft
*/
int radarDSP_FFT(int *real, int *im, int length);

/**
  Compute an IFFT on the signal
  @param real The real component of the input and output signals
  @param im The imaginary components of the input and output signals
  @param length The length of the signal, MUST BE BASE 2

  @note The transform is performed in-place and will overwrite the input signals
  @ingroup radarDSP_fft
*/
int radarDSP_sp_FFT(float *real, float *im, int length);

/**
  Compute an IFFT on the signal
  @param real The real component of the input and output signals
  @param im The imaginary components of the input and output signals
  @param length The length of the signal, MUST BE BASE 2

  @note The transform is performed in-place and will overwrite the input signals
  @ingroup radarDSP_fft
*/
int radarDSP_dp_FFT(double *real, double *im, int length);

//-----------

/**
  Compute an IFFT on the signal
  @param real The real component of the input and output signals
  @param im The imaginary components of the input and output signals
  @param length The length of the signal, MUST BE BASE 2

  @note The transform is performed in-place and will overwrite the input signals
  @ingroup radarDSP_fft
*/
int radarDSP_IFFT(int *real, int *im, int length);

/**
  Compute an IFFT on the signal
  @param real The real component of the input and output signals
  @param im The imaginary components of the input and output signals
  @param length The length of the signal, MUST BE BASE 2

  @note The transform is performed in-place and will overwrite the input signals
  @ingroup radarDSP_fft
*/
int radarDSP_sp_IFFT(float *real, float *im, int length);

/**
  Compute an IFFT on the signal
  @param real The real component of the input and output signals
  @param im The imaginary components of the input and output signals
  @param length The length of the signal, MUST BE BASE 2

  @note The transform is performed in-place and will overwrite the input signals
  @ingroup radarDSP_fft
*/
int radarDSP_dp_IFFT(double *real, double *im, int length);

//------------

/**
  Computes the Hilbert transform using the FFT method
  @param real Real component of the input and output
  @param im Imaginary component of the input and output
  @param length Length of the time vector

  @note Performance could be improved by not moving data back and forth from vec structures between the FFT and the IFFT
  @ingroup radarDSP_fft
*/
int radarDSP_Hilbert(int *real, int *im, int length);

/**
  Computes the Hilbert transform using the FFT method
  @param real Real component of the input and output
  @param im Imaginary component of the input and output
  @param length Length of the time vector

  @note Performance could be improved by not moving data back and forth from vec structures between the FFT and the IFFT
  @ingroup radarDSP_fft
*/
int radarDSP_sp_Hilbert(float *real, float *im, int length);

/**
  Computes the Hilbert transform using the FFT method
  @param real Real component of the input and output
  @param im Imaginary component of the input and output
  @param length Length of the time vector

  @note Performance could be improved by not moving data back and forth from vec structures between the FFT and the IFFT
  @ingroup radarDSP_fft
*/
int radarDSP_dp_Hilbert(double *real, double *im, int length);

//-----------------

/**
  Computes the Envelope of given input signal
  @param [in] input   The input signal
  @param [in] output  The envelope of the input signal
  @param [in] length  The length of the signal

  @warning This function will change datatypes in a future release
  @deprecated This function will be refactored to int in a future revision, use radarDSP_dp_Envelope() for doubles instead
  @ingroup radarDSP_ddc
*/
DEPRECATED int radarDSP_Envelope(double *input, double *output, int length);

/**
  Computes the Envelope of given input signal
  @param [in] input   The input signal
  @param [in] output  The envelope of the input signal
  @param [in] length  The length of the signal
  @ingroup radarDSP_ddc
*/
int radarDSP_sp_Envelope(float *input, float *output, int length);

/**
  Computes the Envelope of given input signal
  @param [in] input   The input signal
  @param [in] output  The envelope of the input signal
  @param [in] length  The length of the signal
  @ingroup radarDSP_ddc
*/
int radarDSP_dp_Envelope(double *input, double *output, int length);

/**
  Prunes input signal by user determined dB amount

  The maximum signal amplitude is determined and values 'x' dB below this max
  are set to 0.
  @param [in] input   The input signal
  @param [in] output  The pruned output signal
  @param [in] length  The length of the signal
  @param [in] prune   Pruning threshold (in dB)
  @ingroup radarDSP_thresholds
*/
int radarDSP_Prune(int *input, int *output, unsigned int length, int prune);

/**
  Prunes input signal by user determined dB amount

  The maximum signal amplitude is determined and values 'x' dB below this max
  are set to 0.
  @param [in] input   The input signal
  @param [in] output  The pruned output signal
  @param [in] length  The length of the signal

  @param [in] prune   Pruning threshold (in dB)
  @ingroup radarDSP_thresholds
*/
int radarDSP_sp_Prune(float *input, float *output, unsigned int length, float prune);

/**
  Prunes input signal by user determined dB amount

  The maximum signal amplitude is determined and values 'x' dB below this max
  are set to 0.
  @param [in] input   The input signal
  @param [in] output  The pruned output signal
  @param [in] length  The length of the signal

  @param [in] prune   Pruning threshold (in dB)
  @ingroup radarDSP_thresholds
*/
int radarDSP_dp_Prune(double *input, double *output, unsigned int length, double prune);


//----------------

/**
  Compute the mean of the data
  @param dataset The dataset to compute the mean of
  @param length The length of the dataset
  @ingroup radarDSP_statistics
*/
int radarDSP_mean(int *dataset, unsigned int length);

/**
  Compute the mean of the data
  @param dataset The dataset to compute the mean of
  @param length The length of the dataset
  @ingroup radarDSP_statistics
*/
float radarDSP_sp_mean(float *dataset, unsigned int length);

/**
  Compute the mean of the data
  @param dataset The dataset to compute the mean of
  @param length The length of the dataset
  @ingroup radarDSP_statistics
*/
double radarDSP_dp_mean(double *dataset, unsigned int length);

//---------------------

/**
  Compute the standard deviation of the dataset
  @param dataset The dataset to compute the standard deviation of
  @param length The length of the dataset

  @note This computes the SAMPLE std like MATLAB (divided by n-1 instead of n like it would be for population)
  @ingroup radarDSP_statistics
*/
int radarDSP_std(int *dataset, unsigned int length);

/**
  Compute the standard deviation of the dataset
  @param dataset The dataset to compute the standard deviation of
  @param length The length of the dataset

  @note This computes the SAMPLE std like MATLAB (divided by n-1 instead of n like it would be for population)
  @ingroup radarDSP_statistics
*/
float radarDSP_sp_std(float *dataset, unsigned int length);

/**
  Compute the standard deviation of the dataset
  @param dataset The dataset to compute the standard deviation of
  @param length The length of the dataset

  @note This computes the SAMPLE std like MATLAB (divided by n-1 instead of n like it would be for population)
  @ingroup radarDSP_statistics
*/
double radarDSP_dp_std(double *dataset, unsigned int length);

//--------------------------

/**
  Compute the root mean square of the dataset
  @param dataset The dataset to compute the RMS of
  @param length The length of the dataset
  @ingroup radarDSP_statistics
*/
int radarDSP_RMS(int *dataset, unsigned int length);

/**
  Compute the root mean square of the dataset
  @param dataset The dataset to compute the RMS of
  @param length The length of the dataset
  @ingroup radarDSP_statistics
*/
float radarDSP_sp_RMS(float *dataset, unsigned int length);

/**
  Compute the root mean square of the dataset
  @param dataset The dataset to compute the RMS of
  @param length The length of the dataset
  @ingroup radarDSP_statistics
*/
double radarDSP_dp_RMS(double *dataset, unsigned int length);

//---------------------------

/**
  Compute the Variance of the data
  @param dataset The dataset to compute the variance of
  @param length The length of the dataset

  @note This computes the SAMPLE variance like MATLAB (divided by n-1 instead of n like it would be for population variance)
  @note This function uses 32bit integers and overflows are not handled.  Overflows are likely with swings in the area of 50k counts....  This cannot be resolved as the result of this is greater then the INT_MAX return of this function.
  @ingroup radarDSP_statistics
*/
int radarDSP_var(int *dataset, unsigned int length);

/**
  Compute the Variance of the data
  @param dataset The dataset to compute the variance of
  @param length The length of the dataset

  @note This computes the SAMPLE variance like MATLAB (divided by n-1 instead of n like it would be for population variance)
  @ingroup radarDSP_statistics
*/
float radarDSP_sp_var(float *dataset, unsigned int length);

/**
  Compute the Variance of the data
  @param dataset The dataset to compute the variance of
  @param length The length of the dataset

  @note This computes the SAMPLE variance like MATLAB (divided by n-1 instead of n like it would be for population variance)
  @ingroup radarDSP_statistics
*/
double radarDSP_dp_var(double *dataset, unsigned int length);

//--------------------------

/**
  Compute and remove the mean from all elements of a vector
  @param sequence The input dataset
  @param returnSequence return the data in this dataset
  @param length The length of the dataset
  @ingroup radarDSP_vec
*/
int radarDSP_removeMean(int *sequence, int *returnSequence, unsigned int length);


/**
  Compute and remove the min from all elements of a vector
  @param sequence The input dataset
  @param returnSequence return the data in this dataset
  @param length The length of the dataset
  @ingroup radarDSP_vec
*/
int radarDSP_sp_removeMean(float *sequence, float *returnSequence, unsigned int length);

/**
  Compute and remove the mean from all elements of a vector
  @param sequence The input dataset
  @param returnSequence return the data in this dataset
  @param length The length of the dataset
  @ingroup radarDSP_vec
*/
int radarDSP_dp_removeMean(double *sequence, double *returnSequence, unsigned int length);

//--------------------------

/**
  Compute and remove the min from all elements of a vector
  @param sequence The input dataset
  @param returnSequence return the data in this dataset
  @param length The length of the dataset
  @ingroup radarDSP_vec
*/
int radarDSP_removeMin(int *sequence, int *returnSequence, unsigned int length);


/**
  Compute and remove the mean from all elements of a vector
  @param sequence The input dataset
  @param returnSequence return the data in this dataset
  @param length The length of the dataset
  @ingroup radarDSP_vec
*/
int radarDSP_sp_removeMin(float *sequence, float *returnSequence, unsigned int length);

/**
  Compute and remove the min from all elements of a vector
  @param sequence The input dataset
  @param returnSequence return the data in this dataset
  @param length The length of the dataset
  @ingroup radarDSP_vec
*/
int radarDSP_dp_removeMin(double *sequence, double *returnSequence, unsigned int length);

//--------------------------

/** Create a window function of the type specified

  @param windowType    Type of window function requested "WindowType_enum")
  @param W        Pointer to a double array to put the window in
  @param Wn        Length of the window to create
  @ingroup radarDSP_windowFns
*/
int radarDSP_Window(int windowType, double *W, int Wn);

/** Create a window function of the type specified

  @param windowType    Type of window function requested "WindowType_enum")
  @param W        Pointer to a double array to put the window in
  @param Wn        Length of the window to create
  @ingroup radarDSP_windowFns
*/
int radarDSP_sp_Window(int windowType, float *W, int Wn);

/** Create a window function of the type specified

  @param windowType    Type of window function requested "WindowType_enum")
  @param W        Pointer to a double array to put the window in
  @param Wn        Length of the window to create
  @ingroup radarDSP_windowFns
*/
int radarDSP_dp_Window(int windowType, double *W, int Wn);


//--------------------------

/**
  Compute the abs of all elements of an integer array
  @param sequence Pointer to an integer array (pass by reference containing input and output)
  @param length Length of the array

  @returns int RadarDSPReturnCodes
  @ingroup radarDSP_vec
*/
int radarDSP_absVec(int *sequence, unsigned int length);

/**
  Compute the complex abs of all elements of an double array
  @param re_sequence Pointer to an integer array (pass by reference containing input and output)
  @param im_sequence Pointer to an integer array
  @param length Length of the array

  @returns int RadarDSPReturnCodes
  @ingroup radarDSP_vec
*/
int radarDSP_dz_absVec(double *re_sequence, double *im_sequence, unsigned int length);

/**
  Compute the abs of all elements of an integer array
  @param sequence Pointer to an integer array (pass by reference containing input and output)
  @param length Length of the array

  @returns int RadarDSPReturnCodes
  @ingroup radarDSP_vec
*/
int radarDSP_sp_absVec(float *sequence, unsigned int length);

/**
  Compute the abs of all elements of an integer array
  @param sequence Pointer to an integer array (pass by reference containing input and output)
  @param length Length of the array

  @returns int RadarDSPReturnCodes
  @ingroup radarDSP_vec
*/
int radarDSP_dp_absVec(double *sequence, unsigned int length);

//-------------------

/**
  Compute the abs of all elements of an double array
  @param sequence Pointer to an double array (pass by reference containing input and output)
  @param length Length of the array

  @returns int RadarDSPReturnCodes
  @ingroup radarDSP_vec
*/
int radarDSP_fabsVec(double *sequence, unsigned int length);


//------------------

/** Sort a vector of ints
  @param sequence The sequence to sort
  @param length How many elements are in the sequence
  @ingroup radarDSP_sort
*/
int radarDSP_sort(int *sequence, int length);

/** Sort a vector of floats
  @param sequence The sequence to sort
  @param length How many elements are in the sequence
  @ingroup radarDSP_sort
*/
int radarDSP_sp_sort(float *sequence, int length);

/** Sort a vector of doubles
  @param sequence The sequence to sort
  @param length How many elements are in the sequence
  @ingroup radarDSP_sort
*/
int radarDSP_dp_sort(double *sequence, int length);

/** Sort a vector of complex ints
  @param sequence The sequence to sort
  @param length How many elements are in the sequence
  @ingroup radarDSP_sort
*/
int radarDSP_z_sort(int *sequence, int length);

/** Sort a vector of complex floats
  @param sequence The sequence to sort
  @param length How many elements are in the sequence
  @ingroup radarDSP_sort
*/
int radarDSP_spz_sort(float *sequence, int length);

/** Sort a vector of complex doubles
  @param sequence The sequence to sort
  @param length How many elements are in the sequence
  @ingroup radarDSP_sort
*/
int radarDSP_dpz_sort(double *sequence, int length);



//----------------------


/** Find the points where the waveform is above a threshold and return in indices
  @param sequence The input waveform
  @param threshold A threshold value for each samples
  @param returnSequence The thresholder result
  @param length The length of the vector to operate on
  @ingroup radarDSP_thresholds
*/
int radarDSP_thresholdIndex(int *sequence, int *threshold, int *returnSequence, int length);

/** Find the points where the waveform is above a threshold and return in indices
  @param sequence The input waveform
  @param threshold A threshold value for each samples
  @param returnSequence The thresholder result
  @param length The length of the vector to operate on
  @ingroup radarDSP_thresholds
*/
int radarDSP_sp_thresholdIndex(float *sequence, float *threshold, int *returnSequence, int length);

/** Find the points where the waveform is above a threshold and return in indices
  @param sequence The input waveform
  @param threshold A threshold value for each samples
  @param returnSequence The thresholder result
  @param length The length of the vector to operate on
  @ingroup radarDSP_thresholds
*/
int radarDSP_dp_thresholdIndex(double *sequence, double *threshold, int *returnSequence, int length);

//------------------------


/** Find the points where the waveform is above a threshold and filter out the low samples in the return sequence
  @param sequence The input waveform
  @param threshold A threshold value for each samples
  @param returnSequence The thresholder result, sequence can be the same array
  @param length The length of the vector to operate on
  @ingroup radarDSP_thresholds
*/
int radarDSP_thresholdValues(int *sequence, int *threshold, int *returnSequence, int length);

/** Find the points where the waveform is above a threshold and filter out the low samples in the return sequence
  @param sequence The input waveform
  @param threshold A threshold value for each samples
  @param returnSequence The thresholder result, sequence can be the same array
  @param length The length of the vector to operate on
  @ingroup radarDSP_thresholds
*/
int radarDSP_sp_thresholdValues(float *sequence, float *threshold, float *returnSequence, int length);

/** Find the points where the waveform is above a threshold and filter out the low samples in the return sequence
  @param sequence The input waveform
  @param threshold A threshold value for each samples
  @param returnSequence The thresholder result, sequence can be the same array
  @param length The length of the vector to operate on
  @ingroup radarDSP_thresholds
*/
int radarDSP_dp_thresholdValues(double *sequence, double *threshold, double *returnSequence, int length);

/** Interpolate a peak in the signal to get higher accuracy.  With good signal this can yield sub mm accuracy.
  @param [in] sequence double sequence to operate on containing the peak
  @param [in] peakIndex Nearest integer bin where the peak is located
  @param [out] interpPeakIndex Pass by reference double to hold the new peak estimate
  @return Status code
  @ingroup radarDSP_interpolation
*/
int radarDSP_dp_gaussianInterpolatePeakIndex(double *sequence, int peakIndex, double *interpPeakIndex);

/** This is an internal function to compute variance with integer speed, but scale the result such that the variance is result*scalingFactor.  This makes standard deviation with large values possible without giving up integer performance.
  @param dataset Dataset to work with
  @param length Length of the dataset
  @param result
  @param scalingFactor
  @return A status code

  @note This function may or may not be present in future API's....
  @ingroup radarDSP_statistics
*/
int radarDSP_var_wOverflowDetection(int *dataset, unsigned int length, int *result, int *scalingFactor);

/** Function to compute the median of an integer array
  @param list 1d list to find the median of
  @param elements Number of elements in the array
  @ingroup radarDSP_statistics
*/
int radarDSP_median(int *list, int elements);

/** Function to compute the median of an single array
  @param list 1d list to find the median of
  @param elements Number of elements in the array
  @ingroup radarDSP_statistics
*/
float radarDSP_sp_median(float *list, int elements);

/** Function to compute the median of an double array
  @param list 1d list to find the median of
  @param elements Number of elements in the array
  @ingroup radarDSP_statistics
*/
double radarDSP_dp_median(double *list, int elements);

/** Function to initialize a single precision iir filter
  @param config pointer to a iir_sp_filter_conf to initialize
  @param a feedback coefficients
  @param b feedforward coefficients
  @param an number of coefficients (a & b must match)
  @returns SUCCESS or FAILURE
  @ingroup radarDSP_filters
*/
int radardsp_sp_iir_init(sp_filter_conf *config, float *a, int an, float *b);

/** Single Precision IIR Filter Function
  @param config pointer to an initialized iir filter configuration
  @param x input matrix
  @param xn length of the input matrix
  @param xp output matrix
  @note X & Xp can be the same matrix
  @pre iir_sp_filter_conf must be initialized
  @ingroup radarDSP_filters
*/
int radardsp_sp_iir(sp_filter_conf *config, float *x, int xn, float *xp);

/** Free single precision iir filter memory
  @param config Pointer to the iir filter to free
  @ingroup radarDSP_filters
*/
int radardsp_sp_iir_free(sp_filter_conf *config);

/** Function to initialize a double precision iir filter
  @param config pointer to a iir_dp_filter_conf to initialize
  @param a feedback coefficients
  @param b feedforward coefficients
  @param an number of coefficients (a & b must match)
  @returns SUCCESS or FAILURE
  @ingroup radarDSP_filters
*/
int radardsp_dp_iir_init(dp_filter_conf *config, double *a, int an, double *b);

/** Double Precision IIR Filter Function
  @param config pointer to an initialized iir filter configuration
  @param x input matrix
  @param xn length of the input matrix
  @param xp output matrix
  @note X & Xp can be the same matrix
  @pre iir_sp_filter_conf must be initialized
  @ingroup radarDSP_filters
*/
int radardsp_dp_iir(dp_filter_conf *config, double *x, int xn, double *xp);

/** Free double precision iir filter memory
  @param config Pointer to the iir filter to free
  @ingroup radarDSP_filters
*/
int radardsp_dp_iir_free(dp_filter_conf *config);

/** Function to initialize a single precision fir filter
  @param config pointer to a sp_filter_conf to initialize
  @param b feedforward coefficients
  @param bn number of coefficients
  @returns SUCCESS or FAILURE
  @ingroup radarDSP_filters
*/
int radarDSP_sp_fir_init(sp_filter_conf *config, float *b, int bn);

/** Single Precision FIR Filter Function
  @param config pointer to an initialized fir filter configuration
  @param x input matrix
  @param xn length of the input matrix
  @param xp output matrix
  @note X & Xp can be the same matrix
  @pre sp_filter_conf must be initialized
  @ingroup radarDSP_filters
*/
int radarDSP_sp_fir(sp_filter_conf *config, float *x, int xn, float *xp);

/** Free single precision fir filter memory
  @param config Pointer to the fir filter to free
  @ingroup radarDSP_filters
*/
int radarDSP_sp_fir_free(sp_filter_conf *config);

/** Function to initialize a double precision fir filter
  @param config pointer to a dp_filter_conf to initialize
  @param b feedforward coefficients
  @param bn number of coefficients
  @returns SUCCESS or FAILURE
  @ingroup radarDSP_filters
*/
int radarDSP_dp_fir_init(dp_filter_conf *config, double *b, int bn);

/** Double Precision FIR Filter Function
  @param config pointer to an initialized fir filter configuration
  @param x input matrix
  @param xn length of the input matrix
  @param xp output matrix
  @note X & Xp can be the same matrix
  @pre dp_filter_conf must be initialized
  @ingroup radarDSP_filters
*/
int radarDSP_dp_fir(dp_filter_conf *config, double *x, int xn, double *xp);

/** Free double precision fir filter memory
  @param config Pointer to the fir filter to free
  @ingroup radarDSP_filters
*/
int radarDSP_dp_fir_free(dp_filter_conf *config);

/** Function to initialize an int precision fir filter
  @param config pointer to an int_filter_conf to initialize
  @param b feedforward coefficients
  @param bn number of coefficients
  @returns SUCCESS or FAILURE
  @ingroup radarDSP_filters
*/
int radarDSP_int_fir_init(int_filter_conf *config, int *b, int bn);

/** Integer Precision FIR Filter Function
  @param config pointer to an initialized fir filter configuration
  @param x input matrix
  @param xn length of the input matrix
  @param xp output matrix
  @note X & Xp can be the same matrix
  @pre dp_filter_conf must be initialized
  @ingroup radarDSP_filters
*/
int radarDSP_int_fir(int_filter_conf *config, int *x, int xn, int *xp);

/** Free integer precision fir filter memory
  @param config Pointer to the fir filter to free
  @ingroup radarDSP_filters
*/
int radarDSP_int_fir_free(int_filter_conf *config);

/** Function to initialize an int precision iir filter
  @param config pointer to an int_filter_conf to initialize
  @param a feedback coefficients
  @param b feedforward coefficients
  @param an number of coefficients (a & b must match)
  @param bn number of coefficients (a & b must match)
  @returns SUCCESS or FAILURE
  @ingroup radarDSP_filters
*/
int radardsp_int_iir_init(int_filter_conf *config, int *b, int bn, int *a, int an);

/** Integer Precision IIR Filter Function
  @param config pointer to an initialized fir filter configuration
  @param x input matrix
  @param xn length of the input matrix
  @param xp output matrix
  @note X & Xp can be the same matrix
  @pre int_filter_conf must be initialized
  @ingroup radarDSP_filters
*/
int radardsp_int_iir(int_filter_conf *config, int *x, int xn, int *xp);

/** Free integer iir filter memory
  @param config Pointer to the iir filter to free
  @ingroup radarDSP_filters
*/
int radardsp_int_iir_free(int_filter_conf *config);

/**
  Initialize a bandpass filter function for a spacific chip and PGEN
  @param moduleType (Chipotle = 0, Cayenne = 1, Ancho =2)
  @param pgen
  @return SUCCESS/FAILURE
  @ingroup radarDSP_bpf
*/
int radarDSP_FrameBandpassFilterInit(int moduleType, int pgen);

/**
  Run the initialized bandpass filtering function on the counters
  @param input The radar counters as float array
  @param n Length of the input and output arrays
  @param output The filtered output
  @returns SUCCESS/FAILURE
  @note This function supports being run in-place on the input and output matrix
  @note The filter must first have been initialized with radarDSP_FilterBandpassFilterInit() before running this function
  @ingroup radarDSP_bpf
*/
int radarDSP_FrameBandpassFilter(float *input, int n, float *ouput);

/**
  Frees the memory allocations made to perform the filtering functions
  @returns SUCCESS/FAILURE
  @ingroup radarDSP_bpf
*/
int radarDSP_FrameBandpassFilterFree();


/** Perform a vector convolution on vectors a & b , Integer version
  This should be functionally equivalent to c = conv(a,b) in MATLAB, anything otherwise is a bug

  @param a First input signal vector
  @param b Second input signal vector
  @param lenA Length of the "a" vector
  @param lenB Length of the "b" vector
  @param c Output vector which will be of size (lenA+lenB-1)

  @note Passthrough operation is allowed, but it is on the user to insure that the output matrix is large enough

  @ingroup matrix
*/
int radarDSP_conv(int *a, int *b, int lenA, int lenB, int *c);

/** Perform a vector convolution on vectors a & b , Float version
  This should be functionally equivalent to c = conv(a,b) in MATLAB, anything otherwise is a bug

  @param a First input signal vector
  @param b Second input signal vector
  @param lenA Length of the "a" vector
  @param lenB Length of the "b" vector
  @param c Output vector which will be of size (lenA+lenB-1)

  @note Passthrough operation is allowed, but it is on the user to insure that the output matrix is large enough

  @ingroup matrix
*/
int radarDSP_sp_conv(float *a, float *b, int lenA, int lenB, float *c);

/** Perform a vector convolution on vectors a & b , Double version
  This should be functionally equivalent to c = conv(a,b) in MATLAB, anything otherwise is a bug

  @param a First input signal vector
  @param b Second input signal vector
  @param lenA Length of the "a" vector
  @param lenB Length of the "b" vector
  @param c Output vector which will be of size (lenA+lenB-1)

  @note Passthrough operation is allowed, but it is on the user to insure that the output matrix is large enough

  @ingroup matrix
*/
int radarDSP_dp_conv(double *a, double *b, int lenA, int lenB, double *c);

/** Perform a vector convolution on vectors a & b , Integer version
  This should be functionally equivalent to c = conv(a,b,'same') in MATLAB, anything otherwise is a bug

  @param a First input signal vector
  @param b Second input signal vector
  @param lenA Length of the "a" vector
  @param lenB Length of the "b" vector
  @param c Output vector which will be of size (lenA)

  @note Passthrough operation is allowed, but it is on the user to insure that the output matrix is large enough

  @ingroup matrix
*/
int radarDSP_conv_same(int *a, int *b, int lenA, int lenB, int *c);

/** Perform a vector convolution on vectors a & b , Float version
  This should be functionally equivalent to c = conv(a,b,'same') in MATLAB, anything otherwise is a bug

  @param a First input signal vector
  @param b Second input signal vector
  @param lenA Length of the "a" vector
  @param lenB Length of the "b" vector
  @param c Output vector which will be of size (lenA)

  @note Passthrough operation is allowed, but it is on the user to insure that the output matrix is large enough

  @ingroup matrix
*/
int radarDSP_sp_conv_same(float *a, float *b, int lenA, int lenB, float *c);

/** Perform a vector convolution on vectors a & b , Double version
  This should be functionally equivalent to c = conv(a,b,'same') in MATLAB, anything otherwise is a bug

  @param a First input signal vector
  @param b Second input signal vector
  @param lenA Length of the "a" vector
  @param lenB Length of the "b" vector
  @param c Output vector which will be of size (lenA)

  @note Passthrough operation is allowed, but it is on the user to insure that the output matrix is large enough

  @ingroup matrix
*/
int radarDSP_dp_conv_same(double *a, double *b, int lenA, int lenB, double *c);

/** Perform a vector convolution on vectors a & b , Integer version
  This should be functionally equivalent to c = conv(a,b,'valid') in MATLAB, anything otherwise is a bug

  @param a First input signal vector
  @param b Second input signal vector
  @param lenA Length of the "a" vector
  @param lenB Length of the "b" vector
  @param c Output vector which will be of size (lenA-len+1)

  @note Passthrough operation is allowed, but it is on the user to insure that the output matrix is large enough

  @ingroup matrix
*/
int radarDSP_conv_valid(int *a, int *b, int lenA, int lenB, int *c);

/** Perform a vector convolution on vectors a & b , Float version
  This should be functionally equivalent to c = conv(a,b,'valid') in MATLAB, anything otherwise is a bug

  @param a First input signal vector
  @param b Second input signal vector
  @param lenA Length of the "a" vector
  @param lenB Length of the "b" vector
  @param c Output vector which will be of size (lenA-len+1)

  @note Passthrough operation is allowed, but it is on the user to insure that the output matrix is large enough

  @ingroup matrix
*/
int radarDSP_sp_conv_valid(float *a, float *b, int lenA, int lenB, float *c);

/** Perform a vector convolution on vectors a & b , Double version
  This should be functionally equivalent to c = conv(a,b,'valid') in MATLAB, anything otherwise is a bug

  @param a First input signal vector
  @param b Second input signal vector
  @param lenA Length of the "a" vector
  @param lenB Length of the "b" vector
  @param c Output vector which will be of size (lenA-len+1)

  @note Passthrough operation is allowed, but it is on the user to insure that the output matrix is large enough

  @ingroup matrix
*/
int radarDSP_dp_conv_valid(double *a, double *b, int lenA, int lenB, double *c);


// -------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------
// BEGIN Alpha/Beta functions, likely to change names and prototypes in the future, and some may become private
// -------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------

int gauss_jordan(int *A, int rows);
int sp_gauss_jordan(float *A, int rows);
int dp_gauss_jordan(double *A, int rows);
int SVD(int *A, int *U, int *S, int *V, int rows);
int sp_SVD(float *A, float *U, float *S, float *V, int rows);
int dp_SVD(double *A, double *U, double *S, double *V, int rows);
int EIG(int *A, int *V, int *D, int *W, int rows, int cols);
int sp_EIG(float *A, float *V, float *D, float *W, int rows, int cols);
int matrix_dp_eig(double *A, int rows, double *V, double *D);
int INV(int *A, int rows);
int matrix_sp_inv(float *A, int rows, float *invA);


/** Compute a convolution of two signals using FFT's
  @param a First input vector of size lenA
  @param b Second input vector of size lenB
  @param lenA Number of elements in vector a
  @param lenB Number of elements in vector b
  @param c Output vector of size lenA+lenB-1
  @returns SUCCESS or error code
  @note Supports passthrough operations
  @ingroup radardsp_vec
*/
int radarDSP_conv_fft(int *a, int *b, int lenA, int lenB, int *c);

/** Compute a convolution of two signals using FFT's
  @param a First input vector of size lenA
  @param b Second input vector of size lenB
  @param lenA Number of elements in vector a
  @param lenB Number of elements in vector b
  @param c Output vector of size lenA+lenB-1
  @returns SUCCESS or error code
  @note Supports passthrough operations
  @ingroup radardsp_vec
*/
int radarDSP_sp_conv_fft(float *a, float *b, int lenA, int lenB, float *c);

/** Compute a convolution of two signals using FFT's
  @param a First input vector of size lenA
  @param b Second input vector of size lenB
  @param lenA Number of elements in vector a
  @param lenB Number of elements in vector b
  @param c Output vector of size lenA+lenB-1
  @returns SUCCESS or error code
  @note Supports passthrough operations
  @ingroup radardsp_vec
*/
int radarDSP_dp_conv_fft(double *a, double *b, int lenA, int lenB, double *c);

/** Compute a convolution of two signals using FFT's matching the length of the first vector
  @param a First input vector of size lenA
  @param b Second input vector of size lenB
  @param lenA Number of elements in vector a
  @param lenB Number of elements in vector b
  @param c Output vector of size lenA
  @returns SUCCESS or error code
  @note Supports passthrough operations
  @ingroup radardsp_vec
*/
int radarDSP_conv_fft_same(int *a, int *b, int lenA, int lenB, int *c);

/** Compute a convolution of two signals using FFT's matching the length of the first vector
  @param a First input vector of size lenA
  @param b Second input vector of size lenB
  @param lenA Number of elements in vector a
  @param lenB Number of elements in vector b
  @param c Output vector of size lenA
  @returns SUCCESS or error code
  @note Supports passthrough operations
  @ingroup radardsp_vec
*/
int radarDSP_sp_conv_fft_same(float *a, float *b, int lenA, int lenB, float *c);

/** Compute a convolution of two signals using FFT's matching the length of the first vector
  @param a First input vector of size lenA
  @param b Second input vector of size lenB
  @param lenA Number of elements in vector a
  @param lenB Number of elements in vector b
  @param c Output vector of size lenA
  @returns SUCCESS or error code
  @note Supports passthrough operations
  @ingroup radardsp_vec
*/
int radarDSP_dp_conv_fft_same(double *a, double *b, int lenA, int lenB, double *c);

/** Compute a convolution of two signals using FFT's returning only the non-zero padded region
  @param a First input vector of size lenA
  @param b Second input vector of size lenB
  @param lenA Number of elements in vector a
  @param lenB Number of elements in vector b
  @param c Output vector of size lenA-lenB+1
  @returns SUCCESS or error code
  @note Supports passthrough operations
  @ingroup radardsp_vec
*/
int radarDSP_conv_fft_valid(int *a, int *b, int lenA, int lenB, int *c);

/** Compute a convolution of two signals using FFT's returning only the non-zero padded region
  @param a First input vector of size lenA
  @param b Second input vector of size lenB
  @param lenA Number of elements in vector a
  @param lenB Number of elements in vector b
  @param c Output vector of size lenA-lenB+1
  @returns SUCCESS or error code
  @note Supports passthrough operations
  @ingroup radardsp_vec
*/
int radarDSP_sp_conv_fft_valid(float *a, float *b, int lenA, int lenB, float *c);

/** Compute a convolution of two signals using FFT's returning only the non-zero padded region
  @param a First input vector of size lenA
  @param b Second input vector of size lenB
  @param lenA Number of elements in vector a
  @param lenB Number of elements in vector b
  @param c Output vector of size lenA-lenB+1
  @returns SUCCESS or error code
  @note Supports passthrough operations
  @ingroup radardsp_vec
*/
int radarDSP_dp_conv_fft_valid(double *a, double *b, int lenA, int lenB, double *c);

/** Compute the cross correlation of two vectors using FFT's
  @param a First input vector of size lenA
  @param b Second input vector of size lenB
  @param lenA Number of elements in vector a
  @param lenB Number of elements in vector b
  @param c Output vector of size lenA+lenB-1
  @returns SUCCESS or error code
  @note Supports passthrough operations
  @ingroup radardsp_vec
*/
int radarDSP_crosscorr_fft(int *a, int *b, int lenA, int lenB, int *c);

/** Compute the cross correlation of two vectors using FFT's
  @param a First input vector of size lenA
  @param b Second input vector of size lenB
  @param lenA Number of elements in vector a
  @param lenB Number of elements in vector b
  @param c Output vector of size lenA+lenB-1
  @returns SUCCESS or error code
  @note Supports passthrough operations
  @ingroup radardsp_vec
*/
int radarDSP_sp_crosscorr_fft(float *a, float *b, int lenA, int lenB, float *c);

/** Compute the cross correlation of two vectors using FFT's
  @param a First input vector of size lenA
  @param b Second input vector of size lenB
  @param lenA Number of elements in vector a
  @param lenB Number of elements in vector b
  @param c Output vector of size lenA+lenB-1
  @returns SUCCESS or error code
  @note Supports passthrough operations
  @ingroup radardsp_vec
*/
int radarDSP_dp_crosscorr_fft(double *a, double *b, int lenA, int lenB, double *c);


/** Perform a row reduction on a matrix without partial pivots
  This is slightly faster but also slightly less stable then the version with partial pivots
  @param A Flattened input matrix of size (rows x cols)
  @param rows Number of rows in the input and output matricies
  @param cols Number of columns in the input and output matricies
  @param B Flatened output matrix of size (rows x cols)
  @returns SUCCESS or error code
  @note Supports passthrough operations
  @ingroup matrix
*/
int matrixAdv_sp_rref_no_pp(float *A, int rows, int cols, float *B);


/** Perform a row reduction on a matrix without partial pivots
  This is slightly faster but also slightly less stable then the version with partial pivots
  @param A Flattened input matrix of size (rows x cols)
  @param rows Number of rows in the input and output matricies
  @param cols Number of columns in the input and output matricies
  @param B Flatened output matrix of size (rows x cols)
  @returns SUCCESS or error code
  @note Supports passthrough operations
  @ingroup matrix
*/
int matrixAdv_dp_rref_no_pp(double *A, int rows, int cols, double *B);


/** Perform a row reduction on a matrix without partial pivots
  This is slightly faster but also slightly less stable then the version with partial pivots
  @param A Flattened input matrix of size (rows x cols)
  @param rows Number of rows in the input and output matricies
  @param cols Number of columns in the input and output matricies
  @param B Flatened output matrix of size (rows x cols)
  @returns SUCCESS or error code
  @note Supports passthrough operations
  @ingroup matrix
*/
int matrix_sp_rref_no_pp(float *A, int rows, int cols, float *B);


/** Perform a row reduction on a matrix without partial pivots
  This is slightly faster but also slightly less stable then the version with partial pivots
  @param A Flattened input matrix of size (rows x cols)
  @param rows Number of rows in the input and output matricies
  @param cols Number of columns in the input and output matricies
  @param B Flatened output matrix of size (rows x cols)
  @returns SUCCESS or error code
  @note Supports passthrough operations
  @ingroup matrix
*/
int matrix_dp_rref_no_pp(double *A, int rows, int cols, double *B);


/** Copy separate real and complex int arrays into a interlaced complex int array
  @param R Input int matrix of real components
  @param I Input int matrix of complex components
  @param length Length of the array to convert
  @param C Complex int datatype array output
  @return SUCCESS or Error Code
  @ingroup radarDSP_conversions
*/
int radarDSP_convert_RI_to_C(int *R, int *I, int length, int *C);

/** Copy separate real and complex float arrays into a interlaced complex float array
  @param R Input float matrix of real components
  @param I Input float matrix of complex components
  @param length Length of the array to convert
  @param C Complex float datatype array output
  @return SUCCESS or Error Code
  @ingroup radarDSP_conversions
*/
int radarDSP_sp_convert_RI_to_C(float *R, float *I, int length, float *C);

/** Copy separate real and complex double arrays into a interlaced complex double array
  @param R Input double matrix of real components
  @param I Input double matrix of complex components
  @param length Length of the array to convert
  @param C Complex double datatype array output
  @return SUCCESS or Error Code
  @ingroup radarDSP_conversions
*/
int radarDSP_dp_convert_RI_to_C(double *R, double *I, int length, double *C);

/** Copy a interlaced complex int array to separate real and complex int arrays
  @param C Complex int datatype array input
  @param length Length of the array to convert
  @param R Ouput int matrix to hold the real components
  @param I Ouput int matrix to hold the complex components
  @return SUCCESS or Error Code
  @ingroup radarDSP_conversions
*/
int radarDSP_convert_C_to_RI(int *C,  int length, int *R, int *I);

/** Copy a interlaced complex float array to separate real and complex float arrays
  @param C Complex float datatype array input
  @param length Length of the array to convert
  @param R Ouput float matrix to hold the real components
  @param I Ouput float matrix to hold the complex components
  @return SUCCESS or Error Code

  @ingroup radarDSP_conversions
*/
int radarDSP_sp_convert_C_to_RI(float *C, int length, float *R, float *I);

/** Copy a interlaced complex double array to separate real and complex double arrays
  @param C Complex double datatype array input
  @param length Length of the array to convert
  @param R Ouput double matrix to hold the real components
  @param I Ouput double matrix to hold the complex components
  @return SUCCESS or Error Code

  @ingroup radarDSP_conversions
*/
int radarDSP_dp_convert_C_to_RI(double *C, int length, double *R, double *I);

/** Copy an int array to a float array
  @param in Input matrix to convert
  @param length Length of the array to convert
  @param out Ouput matrix of the new datatype
  @return SUCCESS or Error Code

  @ingroup radarDSP_conversions
*/
int convert_array_int_to_sp(int *in, int length, float *out);

/** Copy an int array to a double array
  @param in Input matrix to convert
  @param length Length of the array to convert
  @param out Ouput matrix of the new datatype
  @return SUCCESS or Error Code

  @ingroup radarDSP_conversions
*/
int convert_array_int_to_dp(int *in, int length, double *out);

/** Copy a float array to an int array
  @param in Input matrix to convert
  @param length Length of the array to convert
  @param out Ouput matrix of the new datatype
  @return SUCCESS or Error Code

  @ingroup radarDSP_conversions
*/
int convert_array_sp_to_int(float *in, int length, int *out);

/** Copy a float array to a double array
  @param in Input matrix to convert
  @param length Length of the array to convert
  @param out Ouput matrix of the new datatype
  @return SUCCESS or Error Code

  @ingroup radarDSP_conversions
*/
int convert_array_sp_to_dp(float *in, int length, double *out);

/** Copy a double array to an integer array
  @param in Input matrix to convert
  @param length Length of the array to convert
  @param out Ouput matrix of the new datatype
  @return SUCCESS or Error Code

  @ingroup radarDSP_conversions
*/
int convert_array_dp_to_int(double *in, int length, int *out);

/** Copy a double array to an float array
  @param in Input matrix to convert
  @param length Length of the array to convert
  @param out Ouput matrix of the new datatype
  @return SUCCESS or Error Code

  @ingroup radarDSP_conversions
*/
int convert_array_dp_to_sp(double *in, int length, float *out);


/** Solve a system of equations using gaussian elimination.

  This solves a system of equations A*C=B.  This function is equivalent to doing inv(A)*B=C, but avoids explicitly computing the inverse.

  @param A A (rows x colsA) matrix
  @param B A (rows x colsB) matrix
  @param C A (rows x colsB) matrix with the solution
  @param rows Number of rows in all the matrices
  @param colsA Number of columns in the A matrix
  @param colsB Number of columns in the B matrix

  @returns SUCCESS or error code

  @ingroup matrix
*/
int matrix_sp_ldivide(float *A, float *B, int rows, int colsA, int colsB, float *C);


/** Solve a system of equations using gaussian elimination.

  This solves a system of equations A*C=B.  This function is equivalent to doing inv(A)*B=C, but avoids explicitly computing the inverse.

  @param A A (rows x colsA) matrix
  @param B A (rows x colsB) matrix
  @param C A (rows x colsB) matrix with the solution
  @param rows Number of rows in all the matrices
  @param colsA Number of columns in the A matrix
  @param colsB Number of columns in the B matrix

  @returns SUCCESS or error code

  @ingroup matrix
*/
int matrix_dp_ldivide(double *A, double *B, int rows, int colsA, int colsB, double *C);


/** Solve a system of equations using gaussian elimination.

  This solves a system of equations.  This function is equivalent to doing A*inv(B)=C, but avoids explicitly computing the inverse.

  @param A A (rows x colsA) matrix
  @param B A (rows x colsB) matrix
  @param C A (rows x colsB) matrix with the solution
  @param rowsB Number of rows in matrix B
  @param cols Number of columns in all the matrices
  @param rowsA Number of rows in matrix A

  @returns SUCCESS or error code

  @ingroup matrix
*/
int matrix_sp_rdivide(float *B, float *A, int rowsB, int cols, int rowsA, float *C);


/** Solve a system of equations using gaussian elimination.

  This solves a system of equations.  This function is equivalent to doing A*inv(B)=C, but avoids explicitly computing the inverse.

  @param A A (rows x colsA) matrix
  @param B A (rows x colsB) matrix
  @param C A (rows x colsB) matrix with the solution
  @param rowsB Number of rows in matrix B
  @param cols Number of columns in all the matrices
  @param rowsA Number of rows in matrix A


  @returns SUCCESS or error code

  @ingroup matrix
*/
int matrix_dp_rdivide(double *B, double *A, int rowsB, int cols, int rowsA, double *C);

/** Create a linked-list based circular buffer
  @param cbuffer pointer to a circbuffer_t structure which forms the root of the buffer
  @returns SUCCESS or error code

  @note Beta functionality, API is subject to change....
*/
int circBuffer_initialize(circBuffer_t *cbuffer);

/** Add another element to the circular buffer structure
  @param cbuffer pointer of the circular buffer to add a node to
  @param datasize Number of bytes to allow to be written to the node
  @returns SUCCESS or error code

  @note Beta functionality, API is subject to change....
*/
int circBuffer_add(circBuffer_t *cbuffer, int datasize);

/** Write data to the current node and increment the write pointer
  @param cbuffer pointer of the circular buffer use
  @param data Pointer to the data to write to the buffer node
  @param length Number of bytes to allow to be write, must be less then the length of the node
  @returns SUCCESS or error code

  @note Beta functionality, API is subject to change....
*/
int circBuffer_push(circBuffer_t *cbuffer, void *data, int length);

/** Read data to the current node and increment the read pointer
  @param cbuffer pointer of the circular buffer use
  @param data Pointer to an array to put the data read from the buffer into
  @returns SUCCESS or error code

  @note Beta functionality, API is subject to change....
*/
int circBuffer_pull(circBuffer_t *cbuffer, void *data);

/** Delete a node from the cirular buffer
  @param cbuffer pointer of the circular buffer use
  @returns SUCCESS or error code

  @note Beta functionality, API is subject to change....
*/
int circBuffer_remove(circBuffer_t *cbuffer);

/** Read data to the current node and increment the read pointer
  @param cbuffer pointer of the circular buffer use
  @returns Pointer to the data in the array

  @note Beta functionality, API is subject to change....
*/
void *circBuffer_pull_dataAddress(circBuffer_t *cbuffer);


/** Initialize a simple contiguous memory circular buffer
  @param cbuffer A pointer to a circBuffer_simple_t instance to initialize
  @param bufferLength The size of each block in the buffer
  @param elements The length of the buffer before it rolls over
  @returns SUCCESS or error code

  @note Beta functionality, API is subject to change....
*/
int circBuffer_simple_init(circBuffer_simple_t *cbuffer, int elements, int bufferLength);

/** Push data into the circular buffer and increment the write pointer
  @param cbuffer A pointer to a circBuffer_simple_t instance to use
  @param data Pointer to an array of length bufferLength to write to the circular buffer
  @returns SUCCESS or error code

  @note Beta functionality, API is subject to change....
*/
int circBuffer_simple_push(circBuffer_simple_t *cbuffer, void *data);

/** Pull data into the circular buffer without modifying pointers
  @param cbuffer A pointer to a circBuffer_simple_t instance to use
  @param data Pointer to put the read data into
  @returns SUCCESS or error code

  @note Beta functionality, API is subject to change....
*/
int circBuffer_simple_pop(circBuffer_simple_t *cbuffer, void *data);

/** Pull data into the circular buffer and increment the read pointer
  @param cbuffer A pointer to a circBuffer_simple_t instance to use
  @param data Pointer to put the read data into
  @returns SUCCESS or error code

  @note Beta functionality, API is subject to change....
*/
int circBuffer_simple_pull(circBuffer_simple_t *cbuffer, void *data);

/** Get the data in an element in the array (0 indexed from the last element written)

  For example, this would allow for the data from the last written array to be read, and then the one prior....
  This creates a copy of the data that can be used without affecting the buffer contents

  @param cbuffer A pointer to a circBuffer_simple_t instance to use
  @param data Pointer to put the read data into
  @param elementNumber Element to read (generally negative number to get to the time index n=-1, etc)
  @returns SUCCESS or error code

  @note Beta functionality, API is subject to change....
  @note Similar function: circBuffer_simple_elementDataAddress()
*/
int circBuffer_simple_AccessElementN(circBuffer_simple_t *cbuffer, void *data, int elementNumber);

/** Get the circBuffer in an array (0 indexed from the last element written)

  @param cbuffer  A pointer to a circBuffer_simple_t instance to use
  @param *data    Pointer to put the array into
  @param elementStart Element to start outputting from
  @param elementEnd  Element to stop outputting at
*/
int circBuffer_simple_toArray(circBuffer_simple_t *cbuffer, void *data, int elementStart, int elementEnd);

/** Release the memory allocated for the circular buffer
  @param cbuffer Buffer to release the memory from
  @returns SUCCESS or error code
*/
int circBuffer_simple_free(circBuffer_simple_t *cbuffer);

/** Get the memory address to a node in the circular buffer
  @param cbuffer A pointer to a circBuffer_simple_t instance to use
  @param elementNumber Element to read (generally negative number to get to the time index n=-1, etc)
  @return Address of the first element of the node and the data in the buffer will be affected by write to this pointer

  @note Similar function: circBuffer_simple_AccessElementN()
*/
void *circBuffer_simple_elementDataAddress(circBuffer_simple_t *cbuffer, int elementNumber);

/** Block matrix matrix insert

  @param A Pointer to the first element of the full matrix A of size (RA x CA)
  @param RA Number of rows in the matrix A
  @param CA Number of columns in the matrix A
  @param C Pointer to the first element of the full matrix of size (RC x CC)
  @param RC Number of rows in the matrix RC
  @param CC Number of columns in the matrix CC
  @param sRC Starting row to place the resulting matrix in C
  @param sCC Starting column to place the resulting matrix in C

  @returns SUCCESS or error code
*/
int matrixAdv_block_insert(int *A, int RA, int CA, int *C, int RC, int CC, int sRC, int sCC);

/** Block matrix matrix insert

  @param A Pointer to the first element of the full matrix A of size (RA x CA)
  @param RA Number of rows in the matrix A
  @param CA Number of columns in the matrix A
  @param C Pointer to the first element of the full matrix of size (RC x CC)
  @param RC Number of rows in the matrix RC
  @param CC Number of columns in the matrix CC
  @param sRC Starting row to place the resulting matrix in C
  @param sCC Starting column to place the resulting matrix in C

  @returns SUCCESS or error code
*/
int matrixAdv_sp_block_insert(float *A, int RA, int CA, float *C, int RC, int CC, int sRC, int sCC);

/** Block matrix matrix insert

  @param A Pointer to the first element of the full matrix A of size (RA x CA)
  @param RA Number of rows in the matrix A
  @param CA Number of columns in the matrix A
  @param C Pointer to the first element of the full matrix of size (RC x CC)
  @param RC Number of rows in the matrix RC
  @param CC Number of columns in the matrix CC
  @param sRC Starting row to place the resulting matrix in C
  @param sCC Starting column to place the resulting matrix in C

  @returns SUCCESS or error code
*/
int matrixAdv_dp_block_insert(double *A, int RA, int CA, double *C, int RC, int CC, int sRC, int sCC);


//These will probably be private functions
int matrix_dp_ltri_augmented_gaussian_elimination_private(double *A, int rows, int cols);
int matrix_dp_ltri_augmented_gaussian_elimination_pp_private(double *A, int rows, int cols);
int matrixAdv_dp_inv_QR(double *A, const int rows, const int cols, double *invA);
int dp_inv_utri(double *A, int rows);
int sp_inv_diag(float *A, int rows);
int matrix_dp_utri_augmented_backsub_private(double *A, int rows, int cols);
int matrix_dp_chol_pp_private(double *A, int cols, double *R);
int matrix_dp_submatrix(double *A, int rowA, int colA, int startRow, int endRow, int startCol, int endCol, double *B);
int matrix_sp_chol_pp_private(float *A, int cols, float *R);
int matrix_sp_submatrix(float *A, int rowA, int colA, int startRow, int endRow, int startCol, int endCol, float *B);

/** Multiply a matrix (A) by the transpose of matrix (B)
  @param A Pointer to the first matrix (rowsA x colsA)
  @param B Pointer to the second matrix, which will be transposed (rowsB x colsA)
  @param rowsA Number of rows in matrix A
  @param colsA Number of columns in both matricies
  @param rowsB Number of rows in matrix B
  @param C Resultant matrix of size (rowsA x rowsB)
  @return SUCCESS or error code
  @ingroup matrix_mult
*/
int matrix_mult_xxt(int *A, int *B, int rowsA, int colsA, int rowsB, int *C);

/** Multiply a matrix (A) by the transpose of matrix (B)
  @param A Pointer to the first matrix (rowsA x colsA)
  @param B Pointer to the second matrix, which will be transposed (rowsB x colsA)
  @param rowsA Number of rows in matrix A
  @param colsA Number of columns in both matricies
  @param rowsB Number of rows in matrix B
  @param C Resultant matrix of size (rowsA x rowsB)
  @return SUCCESS or error code
  @ingroup matrix_mult
*/
int matrix_sp_mult_xxt(float *A, float *B, int rowsA, int colsA, int rowsB, float *C);

/** Multiply a matrix (A) by the transpose of matrix (B)
  @param A Pointer to the first matrix (rowsA x colsA)
  @param B Pointer to the second matrix, which will be transposed (rowsB x colsA)
  @param rowsA Number of rows in matrix A
  @param colsA Number of columns in both matricies
  @param rowsB Number of rows in matrix B
  @param C Resultant matrix of size (rowsA x rowsB)
  @return SUCCESS or error code
  @ingroup matrix_mult
*/
int matrix_dp_mult_xxt(double *A, double *B, int rowsA, int colsA, int rowsB, double *C);

/** Multiply the transpose of matrix (A) by the matrix (B)
  @param A Pointer to the first matrix (rowsA x colsA)
  @param B Pointer to the second matrix, which will be transposed (rowsA x colsB)
  @param rowsA Number of rows in both matricies
  @param colsA Number of columns in matrix A
  @param colsB Number of columns in matrix B
  @param C Resultant matrix of size (colsA x colsB)
  @return SUCCESS or error code
  @ingroup matrix_mult
*/
int matrix_mult_xtx(int *A, int *B, int rowsA, int colsA, int colsB, int *C);

/** Multiply the transpose of matrix (A) by the matrix (B)
  @param A Pointer to the first matrix (rowsA x colsA)
  @param B Pointer to the second matrix, which will be transposed (rowsA x colsB)
  @param rowsA Number of rows in both matricies
  @param colsA Number of columns in matrix A
  @param colsB Number of columns in matrix B
  @param C Resultant matrix of size (colsA x colsB)
  @return SUCCESS or error code
  @ingroup matrix_mult
*/
int matrix_sp_mult_xtx(float *A, float *B, int rowsA, int colsA, int colsB, float *C);

/** Multiply the transpose of matrix (A) by the matrix (B)
  @param A Pointer to the first matrix (rowsA x colsA)
  @param B Pointer to the second matrix, which will be transposed (rowsA x colsB)
  @param rowsA Number of rows in both matricies
  @param colsA Number of columns in matrix A
  @param colsB Number of columns in matrix B
  @param C Resultant matrix of size (colsA x colsB)
  @return SUCCESS or error code
  @ingroup matrix_mult
*/
int matrix_dp_mult_xtx(double *A, double *B, int rowsA, int colsA, int colsB, double *C);

/** Compute a single precision QR decomposition
  @param A Input matrix
  @param rows Number of rows in matrix A
  @param cols Number of columns in matrix A
  @param Q Resulting quadrature matrix (rows x rows)
  @param R Resulting right triangular matrix

*/
int matrixAdv_sp_qr(float *A, int rows, int cols, float *Q, float *R);

/** Compute a matrix inverse using the QR Decomposition
  @param A A square input matrix
  @param rows Number of rows in A
  @param cols Number of columns in A
  @param invA Memory region to put the resulting inverse matrix
*/
int matrixAdv_sp_inv_QR(float *A, int rows, int cols, float *invA);
int sp_inv_utri(float *A, int rows);

/** Compute a double precision QR decomposition
  @param A Input matrix
  @param rows Number of rows in matrix A
  @param cols Number of columns in matrix A
  @param Q Resulting quadrature matrix (rows x rows)
  @param R Resulting right triangular matrix
*/
int matrixAdv_dp_qr(double *A, const int rows, const int cols, double *Q, double *R);

/** Return the sign of a value (1 if >0, -1 if <0, 0 if =0)
  @param x The parameter to get the sign of
  @return The sign of x
*/
int int_sign(int x);

/** Return the sign of a value (1 if >0, -1 if <0, 0 if =0)
  @param x The parameter to get the sign of
  @return The sign of x
*/
float sp_sign(float x);

/** Return the sign of a value (1 if >0, -1 if <0, 0 if =0)
  @param x The parameter to get the sign of
  @return The sign of x
*/
double dp_sign(double x);

/** Compute a single precision Moore-Penrose Pseudo-Inverse
  This is the equivalent to B = A'*inv(A*A')
  @param A Input matrix of size (rows x cols)
  @param rows Number of rows in A
  @param cols Number of columns in A
  @param B Output pseudo-inverse result of size (rows x cols)
*/
int sp_pseudoInverse(float *A, int rows, int cols, float *B);

/** Compute a double precision Moore-Penrose Pseudo-Inverse
  This is the equivalent to B = A'*inv(A*A')
  @param A Input matrix of size (rows x cols)
  @param rows Number of rows in A
  @param cols Number of columns in A
  @param B Output pseudo-inverse result of size (rows x cols)
*/
int dp_pseudoInverse(double *A, int rows, int cols, double *B);
int matrixAdv_dp_eig_qr(double *A, int rows, double *V, double *D, double eps);
int matrixAdv_sp_make_augFull_utri_gaussian(float *A, int rows, int cols);
int matrixAdv_sp_make_augFull_utri_gaussian_pp(float *A, int rows, int cols);
int matrixAdv_sp_make_augUtri_diag_jordan(float *A, int rows, int cols);
int matrixAdv_sp_normalize_augDiag(float *A, int rows, int cols);
int matrixAdv_dp_make_augFull_utri_gaussian(double *A, int rows, int cols);
int matrixAdv_dp_make_augFull_utri_gaussian_pp(double *A, int rows, int cols);
int matrixAdv_dp_make_augUtri_diag_jordan(double *A, int rows, int cols);
int matrixAdv_dp_normalize_augDiag(double *A, int rows, int cols);

int matrixAdv_sp_make_full_utri_gaussian(float *A, float *B, int rows, int colsA, int colsB);
int matrixAdv_sp_make_full_utri_gaussian_pp(float *A, float *B, int rows, int colsA, int colsB);
int matrixAdv_sp_make_utri_diag_jordan(float *A, float *B, int rows, int colsA, int colsB);
int matrixAdv_sp_normalize_diag(float *A, float *B, int rows, int colsA, int colsB);

int matrixAdv_block_add(int *A, int *B, int RA, int CA, int RB, int CB, int sRA, int sCA);
int matrixAdv_sp_block_add(float *A, float *B, int RA, int CA, int RB, int CB, int sRA, int sCA);
int matrixAdv_dp_block_add(double *A, double *B, int RA, int CA, int RB, int CB, int sRA, int sCA);

int matrixAdv_block_subtract(int *A, int *B, int RA, int CA, int RB, int CB, int sRA, int sCA);
int matrixAdv_sp_block_subtract(float *A, float *B, int RA, int CA, int RB, int CB, int sRA, int sCA);
int matrixAdv_dp_block_subtract(double *A, double *B, int RA, int CA, int RB, int CB, int sRA, int sCA);


/** Compute a single precision matrix inverse of a full square matrix
  @param A Input square matrix of size (rows x rows)
  @param rows Number of rows in the input matrix
  @param invA the resulting inverse matrix

  @note Passthrough is supported in the function
*/
int matrix_sp_inv(float *A, int rows, float *invA);

/** Compute a double precision matrix inverse of a full square matrix
  @param A Input square matrix of size (rows x rows)
  @param rows Number of rows in the input matrix
  @param invA the resulting inverse matrix

  @note Passthrough is supported in the function
*/
int matrix_dp_inv(double *A, int rows, double *invA);

/** Compute a matrix inverse of a diagonal matrix
  @param A Input square matrix of size (rows x rows)
  @param rows Number of rows in the input matrix

  @note Passthrough is supported in the function

  @warning This function does not verify the input matrix is diagonal

  @todo Change the function prototype to match inv
*/
int sp_inv_diag(float *A, int rows);

/** Compute a matrix inverse of a diagonal matrix
  @param A Input square matrix of size (rows x rows)
  @param rows Number of rows in the input matrix

  @note Passthrough is supported in the function

  @warning This function does not verify the input matrix is diagonal

  @todo Change the function prototype to match inv
*/
int dp_inv_diag(double *A, int rows);


/** Print an integer matrix in the terminal for debugging
  @param A Flattened matrix to print
  @param rows Number of rows in the matrix
  @param cols Number of columns in the matrix
  @return SUCCESS
*/
int print_matrix(int *A, int rows, int cols);

/** Print a float matrix in the terminal for debugging
  @param A Flattened matrix to print
  @param rows Number of rows in the matrix
  @param cols Number of columns in the matrix
  @return SUCCESS
*/
int print_sp_matrix(float *A, int rows, int cols);

/** Print a double matrix in the terminal for debugging
  @param A Flattened matrix to print
  @param rows Number of rows in the matrix
  @param cols Number of columns in the matrix
  @return SUCCESS
*/
int print_dp_matrix(double *A, int rows, int cols);

int matrixAdv_block_mult(int *A, int *B, int RA, int CA, int RB, int CB, int sRA, int sCA, int sRB, int sCB, int bRA, int bCA, int bCB, int *C, int RC, int CC, int sRC, int sCC);
int matrixAdv_sp_block_mult(float *A, float *B, int RA, int CA, int RB, int CB, int sRA, int sCA, int sRB, int sCB, int bRA, int bCA, int bCB, float *C, int RC, int CC, int sRC, int sCC);
int matrixAdv_dp_block_mult(double *A, double *B, int RA, int CA, int RB, int CB, int sRA, int sCA, int sRB, int sCB, int bRA, int bCA, int bCB, double *C, int RC, int CC, int sRC, int sCC);

/** Compute the matrix inverse using gauss-jordan elimination
  This function will compute: invA=inv(A)

  @param A Input matrix to compute the inverse of
  @param rows Number of rows or columns in the matrix, matrix must be square
  @param invA Output containing the inverse matrix

  @return SUCCESS or error code

  @note This function supports passthrough operation

  @ingroup matrix
*/
int matrixAdv_sp_inv_guass(float *A, int rows, float *invA);


/** Compute the matrix inverse using gauss-jordan elimination
  This function will compute: invA=inv(A)

  @param A Input matrix to compute the inverse of
  @param rows Number of rows or columns in the matrix, matrix must be square
  @param invA Output containing the inverse matrix

  @return SUCCESS or error code

  @note This function supports passthrough operation

  @ingroup matrix
*/
int matrixAdv_dp_inv_guass(double *A, int rows, double *invA);

// -------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------
// END Alpha/Beta functions
// -------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------

// -------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------
// BEGIN Release-candidate level functions.  I reserve the right to change these, but its more unlikely....
// -------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------

/** Perform an autocorrallation using FFT's (integer version)
  @param a Input vector to do the autocorr on
  @param lenA Length of vector a
  @param c Resulting autocorr of length (2*len-1)
  @return SUCCESS or error code
  @note This function can be used in-place
*/
int radarDSP_autocorr_fft(int *a, int lenA, int *c);

/** Perform an autocorrallation using FFT's  (float version)
  @param a Input vector to do the autocorr on
  @param lenA Length of vector a
  @param c Resulting autocorr of length (2*len-1)
  @return SUCCESS or error code
  @note This function can be used in-place
*/
int radarDSP_sp_autocorr_fft(float *a, int lenA, float *c);

/** Perform an autocorrallation using FFT's  (double version)
  @param a Input vector to do the autocorr on
  @param lenA Length of vector a
  @param c Resulting autocorr of length (2*len-1)
  @return SUCCESS or error code
  @note This function can be used in-place
*/
int radarDSP_dp_autocorr_fft(double *a, int lenA, double *c);

/** Do a fftshift to center the spectra in the vector (Integer version)
  @param Ain Input vector to shift
  @param len Length of the vector
  @param Aout Output vector which has been shifted
  @return SUCCESS or error code
  @note This function can be used in-place

  @ingroup radarDSP_fft
*/
int radarDSP_fftshift(int *Ain, int len, int *Aout);

/** Do a fftshift to center the spectra in the vector (Float version)
  @param Ain Input vector to shift
  @param len Length of the vector
  @param Aout Output vector which has been shifted
  @return SUCCESS or error code
  @note This function can be used in-place

  @ingroup radarDSP_fft
*/
int radarDSP_sp_fftshift(float *Ain, int len, float *Aout);

/** Do a fftshift to center the spectra in the vector (Double version)
  @param Ain Input vector to shift
  @param len Length of the vector
  @param Aout Output vector which has been shifted
  @return SUCCESS or error code
  @note This function can be used in-place

  @ingroup radarDSP_fft
*/
int radarDSP_dp_fftshift(double *Ain, int len, double *Aout);

/** Compute the Moore-Penrose Pseudo-inverse for a flattened integer array
  @param A Input and output matrix, this should probably be split into separate input and output arrays (rows x cols)
  @param rows Number of rows in the input matrix
  @param cols Number of columns in the input matrix
  @param B Output matrix to hold the resulting pseudoinverse (rows x rows).

  @returns SUCCESS or error code

  @ingroup matrix_pseudoinverse
*/

int matrix_sp_pseudoInverse(float *A, int rows, int cols, float *B);

/** Compute the Moore-Penrose Pseudo-inverse for a flattened double array
  @param A Input and output matrix, this should probably be split into separate input and output arrays (rows x cols)
  @param rows Number of rows in the input matrix
  @param cols Number of columns in the input matrix
  @param B Output matrix to hold the resulting pseudoinverse (rows x rows)

  @returns SUCCESS or error code

  @note This function can be used in-place, but it is the users responsibility to ensure that B is large enough to hold the
  resulting matrix.

  @ingroup matrix_pseudoinverse
*/
int matrix_dp_pseudoInverse(double *A, int rows, int cols, double *B);

/** Compute the dot product of integer vectors (aka the "scaler product")  This had the form (A.B=C)
  @param A First integer operand vector
  @param B Second integer operand vector
  @param length Number of elements in the vector
  @param C Integer pointer to place the result scaler value
  @returns SUCCESS or error code

  @ingroup matrix_dot
*/
int vector_dot(int *A, int *B, int length, int *C);

/** Compute the dot product of float vectors (aka the "scaler product")  This had the form (A.B=C)
  @param A First float operand vector
  @param B Second float operand vector
  @param length Number of elements in the vector
  @param C Float pointer to place the result scaler value
  @returns SUCCESS or error code

  @ingroup matrix_dot
*/
int vector_sp_dot(float *A, float *B, int length, float *C);

/** Compute the dot product of double vectors (aka the "scaler product")  This had the form (A.B=C)
  @param A First double operand vector
  @param B Second double operand vector
  @param length Number of elements in the vector
  @param C Double pointer to place the result scaler value
  @returns SUCCESS or error code

  @ingroup matrix_dot
*/
int vector_dp_dot(double *A, double *B, int length, double *C);

/** Compute the cross product (vector product) of integer vectors.  Geometrically this returns a vector perpendicular to A and B.
  @param A First integer operand vector (int[3] array)
  @param B Second integer operand vector (int[3] array)
  @param C Integer pointer to place the resulting int[3] vector
  @returns SUCCESS or error code

  @ingroup matrix_cross
*/
int vector_cross(int *A, int *B, int *C);

/** Compute the cross product (vector product) of integer vectors.  Geometrically this returns a vector perpendicular to A and B.
  @param A First integer operand vector (int[3] array)
  @param B Second integer operand vector (int[3] array)
  @param C Integer pointer to place the resulting int[3] vector
  @returns SUCCESS or error code

  @ingroup matrix_cross
*/
int vector_sp_cross(float *A, float *B, float *C);

/** Compute the cross product (vector product) of integer vectors.  Geometrically this returns a vector perpendicular to A and B.
  @param A First integer operand vector (int[3] array)
  @param B Second integer operand vector (int[3] array)
  @param C Integer pointer to place the resulting int[3] vector
  @returns SUCCESS or error code

  @ingroup matrix_cross
*/
int vector_dp_cross(double *A, double *B, double *C);

/** Create an identity matrix (flattened integer version)
  @param matrix Memory region to place the identity matrix
  @param rows Number of rows in the matrix
  @param cols Number of columns in the matrix
  @returns SUCCESS

  @ingroup matrix_init
*/
int matrix_eye(int *matrix, int rows, int cols);

/** Create an identity matrix (flattened float version)
  @param matrix Memory region to place the identity matrix
  @param rows Number of rows in the matrix
  @param cols Number of columns in the matrix
  @returns SUCCESS

  @ingroup matrix_init
*/
int matrix_sp_eye(float *matrix, int rows, int cols);

/** Create an identity matrix (flattened double version)
  @param matrix Memory region to place the identity matrix
  @param rows Number of rows in the matrix
  @param cols Number of columns in the matrix
  @returns SUCCESS

  @ingroup matrix_init
*/
int matrix_dp_eye(double *matrix, int rows, int cols);


/** Create an identity matrix (complex integer version)
  @param matrix Memory region to place the identity matrix
  @param rows Number of rows in the matrix
  @param cols Number of columns in the matrix
  @returns SUCCESS

  @ingroup matrix_init
*/
int matrix_z_eye(int *matrix, int rows, int cols);

/** Create an identity matrix (complex float version)
  @param matrix Memory region to place the identity matrix
  @param rows Number of rows in the matrix
  @param cols Number of columns in the matrix
  @returns SUCCESS

  @ingroup matrix_init
*/
int matrix_spz_eye(float *matrix, int rows, int cols);

/** Create an identity matrix (complex double version)
  @param matrix Memory region to place the identity matrix
  @param rows Number of rows in the matrix
  @param cols Number of columns in the matrix
  @returns SUCCESS

  @ingroup matrix_init
*/
int matrix_dpz_eye(double *matrix, int rows, int cols);

/** Create a zero matrix (flattened integer version)
  @param matrix Memory region to place the identity matrix
  @param rows Number of rows in the matrix
  @param cols Number of columns in the matrix
  @returns SUCCESS

  @ingroup matrix_init
*/
int matrix_zeros(int *matrix, int rows, int cols);

/** Create a zero matrix (flattened float version)
  @param matrix Memory region to place the identity matrix
  @param rows Number of rows in the matrix
  @param cols Number of columns in the matrix
  @returns SUCCESS

  @ingroup matrix_init
*/
int matrix_sp_zeros(float *matrix, int rows, int cols);

/** Create a zero matrix (flattened double version)
  @param matrix Memory region to place the identity matrix
  @param rows Number of rows in the matrix
  @param cols Number of columns in the matrix
  @returns SUCCESS

  @ingroup matrix_init
*/
int matrix_dp_zeros(double *matrix, int rows, int cols);

/** Create a ones matrix (flattened integer version)
  @param matrix Memory region to place the identity matrix
  @param rows Number of rows in the matrix
  @param cols Number of columns in the matrix
  @returns SUCCESS

  @ingroup matrix_init
*/


/** Create a zero matrix (flattened complex integer version)
  @param matrix Memory region to place the identity matrix
  @param rows Number of rows in the matrix
  @param cols Number of columns in the matrix
  @returns SUCCESS

  @ingroup matrix_init
*/
int matrix_z_zeros(int *matrix, int rows, int cols);

/** Create a zero matrix (flattened complex float version)
  @param matrix Memory region to place the identity matrix
  @param rows Number of rows in the matrix
  @param cols Number of columns in the matrix
  @returns SUCCESS

  @ingroup matrix_init
*/
int matrix_spz_zeros(float *matrix, int rows, int cols);

/** Create a zero matrix (flattened complex double version)
  @param matrix Memory region to place the identity matrix
  @param rows Number of rows in the matrix
  @param cols Number of columns in the matrix
  @returns SUCCESS

  @ingroup matrix_init
*/
int matrix_dpz_zeros(double *matrix, int rows, int cols);

/** Create a ones matrix (flattened integer version)
  @param matrix Memory region to place the identity matrix
  @param rows Number of rows in the matrix
  @param cols Number of columns in the matrix
  @returns SUCCESS

  @ingroup matrix_init
*/



int matrix_ones(int *matrix, int rows, int cols);

/** Create a ones matrix (flattened float version)
  @param matrix Memory region to place the identity matrix
  @param rows Number of rows in the matrix
  @param cols Number of columns in the matrix
  @returns SUCCESS

  @ingroup matrix_init
*/
int matrix_sp_ones(float *matrix, int rows, int cols);

/** Create a ones matrix (flattened double version)
  @param matrix Memory region to place the identity matrix
  @param rows Number of rows in the matrix
  @param cols Number of columns in the matrix
  @returns SUCCESS

  @ingroup matrix_init
*/
int matrix_dp_ones(double *matrix, int rows, int cols);

/** Create a ones matrix (complex int version)
  @param matrix Memory region to place the identity matrix
  @param rows Number of rows in the matrix
  @param cols Number of columns in the matrix
  @returns SUCCESS

  @ingroup matrix_init
*/
int matrix_z_ones(int *matrix, int rows, int cols);

/** Create a ones matrix (complex float version)
  @param matrix Memory region to place the identity matrix
  @param rows Number of rows in the matrix
  @param cols Number of columns in the matrix
  @returns SUCCESS

  @ingroup matrix_init
*/
int matrix_spz_ones(float *matrix, int rows, int cols);

/** Create a ones matrix (complex double version)
  @param matrix Memory region to place the identity matrix
  @param rows Number of rows in the matrix
  @param cols Number of columns in the matrix
  @returns SUCCESS

  @ingroup matrix_init
*/
int matrix_dpz_ones(double *matrix, int rows, int cols);

/** Copy a matrix from one memory location to a different location (integer matrices)
  @param dest Memory location to copy the matrix to
  @param src Memory location to copy the matrix from
  @param rows Number of rows in the matrix
  @param cols Number of columns in the matrix
  @returns SUCCESS

  @note This is just a memcpy, but included to make code look more logical

  @ingroup matrix_copy
*/
int matrix_copy(int *dest, int *src, int rows, int cols);

/** Copy a matrix from one memory location to a different location (float matrices)
  @param dest Memory location to copy the matrix to
  @param src Memory location to copy the matrix from
  @param rows Number of rows in the matrix
  @param cols Number of columns in the matrix
  @returns SUCCESS

  @note This is just a memcpy, but included to make code look more logical

  @ingroup matrix_copy
*/
int matrix_sp_copy(float *dest, float *src, int rows, int cols);

/** Copy a matrix from one memory location to a different location (double matrices)
  @param dest Memory location to copy the matrix to
  @param src Memory location to copy the matrix from
  @param rows Number of rows in the matrix
  @param cols Number of columns in the matrix
  @returns SUCCESS

  @note This is just a memcpy, but included to make code look more logical

  @ingroup matrix_copy
*/
int matrix_dp_copy(double *dest, double *src, int rows, int cols);

/** Copy a matrix from one memory location to a different location (complex integer matrices)
  @param dest Memory location to copy the matrix to
  @param src Memory location to copy the matrix from
  @param rows Number of rows in the matrix
  @param cols Number of columns in the matrix
  @returns SUCCESS

  @note This is just a memcpy, but included to make code look more logical

  @ingroup matrix_copy
*/
int matrix_z_copy(int *dest, int *src, int rows, int cols);

/** Copy a matrix from one memory location to a different location (complex float matrices)
  @param dest Memory location to copy the matrix to
  @param src Memory location to copy the matrix from
  @param rows Number of rows in the matrix
  @param cols Number of columns in the matrix
  @returns SUCCESS

  @note This is just a memcpy, but included to make code look more logical

  @ingroup matrix_copy
*/
int matrix_spz_copy(float *dest, float *src, int rows, int cols);

/** Copy a matrix from one memory location to a different location (complex double matrices)
  @param dest Memory location to copy the matrix to
  @param src Memory location to copy the matrix from
  @param rows Number of rows in the matrix
  @param cols Number of columns in the matrix
  @returns SUCCESS

  @note This is just a memcpy, but included to make code look more logical

  @ingroup matrix_copy
*/
int matrix_dpz_copy(double *dest, double *src, int rows, int cols);






/** Multiply two flattened integer matrices together with the form C = A*B
  @param A The first multiplicand matrix (rowsA x colsA)
  @param B The second multiplicand matrix (colsA x colsB)
  @param C The output matrix (rowsA x colsB)
  @param rowsA Number of rows in the A matrix
  @param colsA The number of columns in A with must equal the rows in B
  @param colsB The number columns in B

  @returns SUCCESS or error code

  @note The function supports in-place matrix operations

  @ingroup matrix_mult
*/
int matrix_mult(int *A, int *B, int rowsA, int colsA, int colsB, int *C);


/** Multiply two flattened single matrices together with the form C = A*B
  @param A The first multiplicand matrix (rowsA x colsA)
  @param B The second multiplicand matrix (colsA x colsB)
  @param C The output matrix (rowsA x colsB)
  @param rowsA Number of rows in the A matrix
  @param colsA The number of columns in A with must equal the rows in B
  @param colsB The number columns in B

  @returns SUCCESS or error code

  @note The function supports in-place matrix operations

  @ingroup matrix_mult
*/
int matrix_sp_mult(float *A, float *B, int rowsA, int colsA, int colsB, float *C);


/** Multiply two flattened double matrices together with the form C = A*B
  @param A The first multiplicand matrix (rowsA x colsA)
  @param B The second multiplicand matrix (colsA x colsB)
  @param C The output matrix (rowsA x colsB)
  @param rowsA Number of rows in the A matrix
  @param colsA The number of columns in A with must equal the rows in B
  @param colsB The number columns in B

  @returns SUCCESS or error code

  @note The function supports in-place matrix operations

  @ingroup matrix_mult
*/
int matrix_dp_mult(double *A, double *B, int rowsA, int colsA, int colsB, double *C);


/** Transpose an integer flattened matrix
  @param A Input matrix
  @param B Transposed output matrix
  @param rowsA The number of rows in the input matrix
  @param colsA The number of columns in the output matrix

  @returns SUCCESS or error code

  @note - This function supports in-place matrix operations

  @ingroup matrix
*/
int matrix_transpose(int *A, int *B, int rowsA, int colsA);


/** Transpose a single precision flattened matrix
  @param A Input matrix
  @param B Transposed output matrix
  @param rowsA The number of rows in the input matrix
  @param colsA The number of columns in the output matrix

  @returns SUCCESS or error code

  @note - This function supports in-place matrix operations

  @ingroup matrix
*/
int matrix_sp_transpose(float *A, float *B, int rowsA, int colsA);


/** Transpose a double precision flattened matrix
  @param A Input matrix
  @param B Transposed output matrix
  @param rowsA The number of rows in the input matrix
  @param colsA The number of columns in the output matrix

  @returns SUCCESS or error code

  @note - This function supports in-place matrix operations

  @ingroup matrix
*/
int matrix_dp_transpose(double *A, double *B, int rowsA, int colsA);


/** Find the inverse of a square double precision matrix
  @param A Flattened double precision matrix of size rows x rows, inputs A, returns inv(A)
  @param rows Number of rows in the input/output matrices
  @param invA Pointer to a matrix to place the inverse result

  @returns SUCCESS or error code

  @note I may change this function definition to have explicit input and output matrices

  @ingroup matrix
*/
int matrix_dp_inv(double *A, int rows, double *invA);


/** Single precision QR decomposition
  Compute the single precision QR Decomposition (Q*R=A) where Q is a diagonal matrix and R is a
  right triangular matrix.

  When A is a mxn matrix, Q will be mxm and R will be mxn.

  @param A Flatted float matrix to take the decomposition of
  @param rows Number of rows in A
  @param cols Number of columns in A
  @param Q Matrix pointer to hold the quadrature output matrix (rows x rows)
  @param R Matrix pointer to hold the right triangular output matrix (rows x cols)
  @returns SUCCESS or error code

  @ingroup matrix
*/
int matrix_sp_qr(float *A, int rows, int cols, float *Q, float *R);

/** Double precision QR decomposition
  Compute the single precision QR Decomposition (Q*R=A) where Q is a diagonal matrix and R is a
  right triangular matrix.

  When A is a mxn matrix, Q will be mxm and R will be mxn.

  @param A Flatted float matrix to take the decomposition of
  @param rows Number of rows in A
  @param cols Number of columns in A
  @param Q Matrix pointer to hold the quadrature output matrix (rows x rows)
  @param R Matrix pointer to hold the right triangular output matrix (rows x cols)
  @returns SUCCESS or error code

  @ingroup matrix
*/
int matrix_dp_qr(double *A, int rows, int cols, double *Q, double *R);


/** Compute the rref (row reduced echelon form) for an augmented matrix using Gauss-Jordan with partial pivots
  @param A Float input matrix of size (row x cols)
  @param rows Number of rows in the input and output matrices
  @param cols Number of columns in the input and output matrices
  @param B Float matrix pointer to hold the outputs

  @return SUCCESS or failure code

  @note This function can be used in place on the matrices

  @ingroup matrix
*/
int matrix_sp_rref(float *A, int rows, int cols, float *B);

/** Compute the rref (row reduced echelon form) for an augmented matrix using Gauss-Jordan with partial pivots
  @param A Double input matrix of size (row x cols)
  @param rows Number of rows in the input and output matrices
  @param cols Number of columns in the input and output matrices
  @param B Double matrix pointer to hold the outputs

  @return SUCCESS or failure code

  @note This function can be used in place on the matrices

  @ingroup matrix
*/
int matrix_dp_rref(double *A, int rows, int cols, double *B);


/** Integer elementwise matrix addition C = A+B
  @param A Pointer to the first operand matrix
  @param B Pointer to the second operand matrix
  @param rows Number of rows in the matrices
  @param cols Number of columns in the matrices
  @param C Output matrix pointer to get the result of A+B
  @note The function can operate in-place

  @ingroup matrix_add
*/
int matrix_add(int *A, int *B, int rows, int cols, int *C);

/** Float elementwise matrix addition C = A+B
  @param A Pointer to the first operand matrix
  @param B Pointer to the second operand matrix
  @param rows Number of rows in the matrices
  @param cols Number of columns in the matrices
  @param C Output matrix pointer to get the result of A+B
  @note The function can operate in-place

  @ingroup matrix_add
*/
int matrix_sp_add(float *A, float *B, int rows, int cols, float *C);

/** Double elementwise matrix addition C = A+B
  @param A Pointer to the first operand matrix
  @param B Pointer to the second operand matrix
  @param rows Number of rows in the matrices
  @param cols Number of columns in the matrices
  @param C Output matrix pointer to get the result of A+B
  @note The function can operate in-place

  @ingroup matrix_add
*/
int matrix_dp_add(double *A, double *B, int rows, int cols, double *C);

/** Integer elementwise matrix addition C = A+B
  @param A Pointer to the first operand matrix
  @param B Pointer to the second operand matrix
  @param rows Number of rows in the matrices
  @param cols Number of columns in the matrices
  @param C Output matrix pointer to get the result of A+B
  @note The function can operate in-place

  @ingroup matrix_add
*/
int matrix_z_add(int *A, int *B, int rows, int cols, int *C);

/** Float elementwise matrix addition C = A+B
  @param A Pointer to the first operand matrix
  @param B Pointer to the second operand matrix
  @param rows Number of rows in the matrices
  @param cols Number of columns in the matrices
  @param C Output matrix pointer to get the result of A+B
  @note The function can operate in-place

  @ingroup matrix_add
*/
int matrix_spz_add(float *A, float *B, int rows, int cols, float *C);

/** Double elementwise matrix addition C = A+B
  @param A Pointer to the first operand matrix
  @param B Pointer to the second operand matrix
  @param rows Number of rows in the matrices
  @param cols Number of columns in the matrices
  @param C Output matrix pointer to get the result of A+B
  @note The function can operate in-place

  @ingroup matrix_add
*/
int matrix_dpz_add(double *A, double *B, int rows, int cols, double *C);

/** Integer elementwise matrix subtraction C = A-B
  @param A Pointer to the first operand matrix
  @param B Pointer to the second operand matrix
  @param rows Number of rows in the matrices
  @param cols Number of columns in the matrices
  @param C Output matrix pointer to get the result of A-B
  @note The function can operate in-place

  @ingroup matrix_subtract
*/
int matrix_subtract(int *A, int *B, int rows, int cols, int *C);

/** Float elementwise matrix subtraction C = A-B
  @param A Pointer to the first operand matrix
  @param B Pointer to the second operand matrix
  @param rows Number of rows in the matrices
  @param cols Number of columns in the matrices
  @param C Output matrix pointer to get the result of A-B
  @note The function can operate in-place

  @ingroup matrix_subtract
*/
int matrix_sp_subtract(float *A, float *B, int rows, int cols, float *C);

/** Double elementwise matrix subtraction C = A-B
  @param A Pointer to the first operand matrix
  @param B Pointer to the second operand matrix
  @param rows Number of rows in the matrices
  @param cols Number of columns in the matrices
  @param C Output matrix pointer to get the result of A-B
  @note The function can operate in-place

  @ingroup matrix_subtract
*/
int matrix_dp_subtract(double *A, double *B, int rows, int cols, double *C);

/** Integer Hadamard Product (Elementwise Multiplication)
  @param A Pointer to the first operand matrix
  @param B Pointer to the second operand matrix
  @param rows Number of rows in the matrices
  @param cols Number of columns in the matrices
  @param C Output matrix pointer to get the result of A.*B
  @note The function can operate in-place

  @ingroup matrix_hproduct
*/
int matrix_hProduct(int *A, int *B, int rows, int cols, int *C);

/** Float Hadamard Product (Elementwise Multiplication)
  @param A Pointer to the first operand matrix
  @param B Pointer to the second operand matrix
  @param rows Number of rows in the matrices
  @param cols Number of columns in the matrices
  @param C Output matrix pointer to get the result of A.*B
  @note The function can operate in-place

  @ingroup matrix_hproduct
*/
int matrix_sp_hProduct(float *A, float *B, int rows, int cols, float *C);

/** Double Hadamard Product (Elementwise Multiplication)
  @param A Pointer to the first operand matrix
  @param B Pointer to the second operand matrix
  @param rows Number of rows in the matrices
  @param cols Number of columns in the matrices
  @param C Output matrix pointer to get the result of A.*B
  @note The function can operate in-place

  @ingroup matrix_hproduct
*/
int matrix_dp_hProduct(double *A, double *B, int rows, int cols, double *C);

/** Integer Hadamard Division (Elementwise division)
  @param A Pointer to the first operand matrix
  @param B Pointer to the second operand matrix
  @param rows Number of rows in the matrices
  @param cols Number of columns in the matrices
  @param C Output matrix pointer to get the result of A./B
  @note The function can operate in-place

  @ingroup matrix_hdivision
*/
int matrix_hDivision(int *A, int *B, int rows, int cols, int *C);

/** Float Hadamard Division (Elementwise division)
  @param A Pointer to the first operand matrix
  @param B Pointer to the second operand matrix
  @param rows Number of rows in the matrices
  @param cols Number of columns in the matrices
  @param C Output matrix pointer to get the result of A./B
  @note The function can operate in-place

  @ingroup matrix_hdivision
*/
int matrix_sp_hDivision(float *A, float *B, int rows, int cols, float *C);

/** Double Hadamard Division (Elementwise division)
  @param A Pointer to the first operand matrix
  @param B Pointer to the second operand matrix
  @param rows Number of rows in the matrices
  @param cols Number of columns in the matrices
  @param C Output matrix pointer to get the result of A./B
  @note The function can operate in-place

  @ingroup matrix_hdivision
*/
int matrix_dp_hDivision(double *A, double *B, int rows, int cols, double *C);

/** Float Hadamard Inverse (Elementwise inverse)
  @param A Pointer to the first operand matrix
  @param rows Number of rows in the matrices
  @param cols Number of columns in the matrices
  @param C Output matrix pointer to get the result of A.^-1
  @note The function can operate in-place

  @ingroup matrix_hinverse
*/
int matrix_sp_hInverse(float *A, int rows, int cols, float *C);

/** Double Hadamard Inverse (Elementwise inverse)
  @param A Pointer to the matrix to invert
  @param rows Number of rows in the matrices
  @param cols Number of columns in the matrices
  @param C Output matrix pointer to get the result of A.^-1
  @note The function can operate in-place

  @ingroup matrix_hinverse
*/
int matrix_dp_hInverse(double *A, int rows, int cols, double *C);

/** Float Hadamard Square Root (Elementwise sqrt)
  @param A Pointer to the matrix to sqrt
  @param rows Number of rows in the matrices
  @param cols Number of columns in the matrices
  @param C Output matrix pointer to get the result of A.^0.5
  @note The function can operate in-place

  @ingroup matrix_hroot
*/
int matrix_sp_hRoot(float *A, int rows, int cols, float *C);

/** Double Hadamard Square Root (Elementwise sqrt)
  @param A Pointer to the matrix to sqrt
  @param rows Number of rows in the matrices
  @param cols Number of columns in the matrices
  @param C Output matrix pointer to get the result of A.^0.5
  @note The function can operate in-place

  @ingroup matrix_hroot
*/
int matrix_dp_hRoot(double *A, int rows, int cols, double *C);


/** Perform a convolution with complex values
  @param aR Real portion of the first vector
  @param aI Imaginary portion of the first vector
  @param bR Real portion of the second vector
  @param bI Imaginary portion of the second vector
  @param lenA Length of the first vector
  @param lenB Length of the second vector
  @param cR Real portion of the resulting vector
  @param cI Imaginary portion of the resulting vector

  @ingroup matrix_convolution
*/
int radarDSP_convz(int *aR, int *aI, int *bR, int *bI, int lenA, int lenB, int *cR, int *cI);

/** Perform a convolution with complex values
  @param aR Real portion of the first vector
  @param aI Imaginary portion of the first vector
  @param bR Real portion of the second vector
  @param bI Imaginary portion of the second vector
  @param lenA Length of the first vector
  @param lenB Length of the second vector
  @param cR Real portion of the resulting vector
  @param cI Imaginary portion of the resulting vector

  @ingroup matrix_convolution
*/
int radarDSP_sp_convz(float *aR, float *aI, float *bR, float *bI, int lenA, int lenB, float *cR, float *cI);

/** Perform a convolution with complex values
  @param aR Real portion of the first vector
  @param aI Imaginary portion of the first vector
  @param bR Real portion of the second vector
  @param bI Imaginary portion of the second vector
  @param lenA Length of the first vector
  @param lenB Length of the second vector
  @param cR Real portion of the resulting vector
  @param cI Imaginary portion of the resulting vector

  @ingroup matrix_convolution
*/
int radarDSP_dp_convz(double *aR, double *aI, double *bR, double *bI, int lenA, int lenB, double *cR, double *cI);

/** Perform a convolution with complex values with the output is the same length as the first input vector
  @param aR Real portion of the first vector
  @param aI Imaginary portion of the first vector
  @param bR Real portion of the second vector
  @param bI Imaginary portion of the second vector
  @param lenA Length of the first vector
  @param lenB Length of the second vector
  @param cR Real portion of the resulting vector
  @param cI Imaginary portion of the resulting vector

  @ingroup matrix_convolution
*/
int radarDSP_convz_same(int *aR, int *aI, int *bR, int *bI, int lenA, int lenB, int *cR, int *cI);

/** Perform a convolution with complex values with the output is the same length as the first input vector
  @param aR Real portion of the first vector
  @param aI Imaginary portion of the first vector
  @param bR Real portion of the second vector
  @param bI Imaginary portion of the second vector
  @param lenA Length of the first vector
  @param lenB Length of the second vector
  @param cR Real portion of the resulting vector
  @param cI Imaginary portion of the resulting vector

  @ingroup matrix_convolution
*/
int radarDSP_sp_convz_same(float *aR, float *aI, float *bR, float *bI, int lenA, int lenB, float *cR, float *cI);

/** Perform a convolution with complex values with the output is the same length as the first input vector
  @param aR Real portion of the first vector
  @param aI Imaginary portion of the first vector
  @param bR Real portion of the second vector
  @param bI Imaginary portion of the second vector
  @param lenA Length of the first vector
  @param lenB Length of the second vector
  @param cR Real portion of the resulting vector
  @param cI Imaginary portion of the resulting vector

  @ingroup matrix_convolution
*/
int radarDSP_dp_convz_same(double *aR, double *aI, double *bR, double *bI, int lenA, int lenB, double *cR, double *cI);

/** Perform a convolution with complex values with the output is the same length as the second input vector
  @param aR Real portion of the first vector
  @param aI Imaginary portion of the first vector
  @param bR Real portion of the second vector
  @param bI Imaginary portion of the second vector
  @param lenA Length of the first vector
  @param lenB Length of the second vector
  @param cR Real portion of the resulting vector
  @param cI Imaginary portion of the resulting vector

  @ingroup matrix_convolution
*/
int radarDSP_convz_valid(int *aR, int *aI, int *bR, int *bI, int lenA, int lenB, int *cR, int *cI);

/** Perform a convolution with complex values with the output is the same length as the second input vector
  @param aR Real portion of the first vector
  @param aI Imaginary portion of the first vector
  @param bR Real portion of the second vector
  @param bI Imaginary portion of the second vector
  @param lenA Length of the first vector
  @param lenB Length of the second vector
  @param cR Real portion of the resulting vector
  @param cI Imaginary portion of the resulting vector

  @ingroup matrix_convolution
*/
int radarDSP_sp_convz_valid(float *aR, float *aI, float *bR, float *bI, int lenA, int lenB, float *cR, float *cI);

/** Perform a convolution with complex values with the output is the same length as the second input vector
  @param aR Real portion of the first vector
  @param aI Imaginary portion of the first vector
  @param bR Real portion of the second vector
  @param bI Imaginary portion of the second vector
  @param lenA Length of the first vector
  @param lenB Length of the second vector
  @param cR Real portion of the resulting vector
  @param cI Imaginary portion of the resulting vector

  @ingroup matrix_convolution
*/
int radarDSP_dp_convz_valid(double *aR, double *aI, double *bR, double *bI, int lenA, int lenB, double *cR, double *cI);

/** Perform a convolution with complex values
  @param a The first input vector
  @param b The second input vector
  @param lenA The length of the first input vector
  @param lenB The length of the second input vector
  @param c The resulting vector

  @ingroup matrix_convolution
*/
int radarDSP_z_convz(int *a, int *b, int lenA, int lenB, int *c);

/** Perform a convolution with complex values
  @param a The first input vector
  @param b The second input vector
  @param lenA The length of the first input vector
  @param lenB The length of the second input vector
  @param c The resulting vector

  @ingroup matrix_convolution
*/
int radarDSP_spz_convz(float *a, float *b, int lenA, int lenB, float *c);

/** Perform a convolution with complex values
  @param a The first input vector
  @param b The second input vector
  @param lenA The length of the first input vector
  @param lenB The length of the second input vector
  @param c The resulting vector

  @ingroup matrix_convolution
*/
int radarDSP_dpz_convz(double *a, double *b, int lenA, int lenB, double *c);

/** Perform a convolution with complex values where the output is the same length as the first input vector
  @param a The first input vector
  @param b The second input vector
  @param lenA The length of the first input vector
  @param lenB The length of the second input vector
  @param c The resulting vector

  @ingroup matrix_convolution
*/
int radarDSP_z_convz_same(int *a, int *b, int lenA, int lenB, int *c);

/** Perform a convolution with complex values where the output is the same length as the first input vector
  @param a The first input vector
  @param b The second input vector
  @param lenA The length of the first input vector
  @param lenB The length of the second input vector
  @param c The resulting vector

  @ingroup matrix_convolution
*/
int radarDSP_spz_convz_same(float *a, float *b, int lenA, int lenB, float *c);

/** Perform a convolution with complex values where the output is the same length as the first input vector
  @param a The first input vector
  @param b The second input vector
  @param lenA The length of the first input vector
  @param lenB The length of the second input vector
  @param c The resulting vector

  @ingroup matrix_convolution
*/
int radarDSP_dpz_convz_same(double *a, double *b, int lenA, int lenB, double *c);

/** Perform a convolution with complex values where the output is the same length as the second input vector
  @param a The first input vector
  @param b The second input vector
  @param lenA The length of the first input vector
  @param lenB The length of the second input vector
  @param c The resulting vector

  @ingroup matrix_convolution
*/
int radarDSP_z_convz_valid(int *a, int *b, int lenA, int lenB, int *c);

/** Perform a convolution with complex values where the output is the same length as the second input vector
  @param a The first input vector
  @param b The second input vector
  @param lenA The length of the first input vector
  @param lenB The length of the second input vector
  @param c The resulting vector

  @ingroup matrix_convolution
*/
int radarDSP_spz_convz_valid(float *a, float *b, int lenA, int lenB, float *c);

/** Perform a convolution with complex values where the output is the same length as the second input vector
  @param a The first input vector
  @param b The second input vector
  @param lenA The length of the first input vector
  @param lenB The length of the second input vector
  @param c The resulting vector

  @ingroup matrix_convolution
*/
int radarDSP_dpz_convz_valid(double *a, double *b, int lenA, int lenB, double *c);


/* START Alpha Functions, subject to change.... */
/** Simple IIR Filter (int)
  @param Bin Numerator coefficents for the filter
  @param Bn Number of numerator coefficients
  @param Ain Denominator coefficients for the filter
  @param An Number of Denomerator coefficients
  @param Xin Data vector to filter
  @param Xn Length of vector X
  @param Xout Vector to hold the resulting filtered data

  @note This will be slower then the lower functions as it creates and distroys the filter structure on each call
  @note equivalent to matlabs x = filter(b,a,x) for 1d vectors
*/
int radarDSP_iir(int *Bin, int Bn, int *Ain, int An, int *Xin, int Xn, int *Xout);

/** Simple IIR Filter (complex int)
  @param Bin Numerator coefficents for the filter
  @param Bn Number of numerator coefficients
  @param Ain Denominator coefficients for the filter
  @param An Number of Denomerator coefficients
  @param Xin Data vector to filter
  @param Xn Length of vector X
  @param Xout Vector to hold the resulting filtered data

  @note This will be slower then the lower functions as it creates and distroys the filter structure on each call
  @note equivalent to matlabs x = filter(b,a,x) for 1d vectors
*/
int radarDSP_z_iirz(int *Bin, int Bn, int *Ain, int An, int *Xin, int Xn, int *Xout);

/** Simple IIR Filter (complex float)
  @param Bin Numerator coefficents for the filter
  @param Bn Number of numerator coefficients
  @param Ain Denominator coefficients for the filter
  @param An Number of Denomerator coefficients
  @param Xin Data vector to filter
  @param Xn Length of vector X
  @param Xout Vector to hold the resulting filtered data

  @note This will be slower then the lower functions as it creates and distroys the filter structure on each call
  @note equivalent to matlabs x = filter(b,a,x) for 1d vectors
*/
int radarDSP_spz_iirz(float *Bin, int Bn, float *Ain, int An, float *Xin, int Xn, float *Xout);

/** Simple IIR Filter (complex double)
  @param Bin Numerator coefficents for the filter
  @param Bn Number of numerator coefficients
  @param Ain Denominator coefficients for the filter
  @param An Number of Denomerator coefficients
  @param Xin Data vector to filter
  @param Xn Length of vector X
  @param Xout Vector to hold the resulting filtered data

  @note This will be slower then the lower functions as it creates and distroys the filter structure on each call
  @note equivalent to matlabs x = filter(b,a,x) for 1d vectors
*/
int radarDSP_dpz_iirz(double *Bin, int Bn, double *Ain, int An, double *Xin, int Xn, double *Xout);

/** Create an IIR Filter (complex int)
  @param config instance of an int_filter_conf to initialize
  @param a Denomerator coefficients to initialize
  @param an Number of denomerator coefficients
  @param b Numerator coefficients to initialize
  @param bn Number of numerator coefficients
*/
int radarDSP_z_iirz_init(int_filter_conf *config, int *a, int an, int *b, int bn);

/** Process a vector using an IIR filter
  @param config An initialized instance of an int_filter_conf to use
  @param x Data vector to process
  @param xn Length of the vector x
  @param xp Output filtered data vector
*/
int radarDSP_z_iirz_filter(int_filter_conf *config, int *x, int xn, int *xp);

/** Free the memory associated with a complex IIR filter
  @param config An instance of type int_filter_conf to free the memory of
*/
int radarDSP_z_iirz_free(int_filter_conf *config);

/** Create an IIR Filter (complex float)
  @param config instance of an sp_filter_conf to initialize
  @param a Denomerator coefficients to initialize
  @param b Numerator coefficients to initialize
  @param an Number of denomerator coefficients
*/
int radarDSP_spz_iirz_init(sp_filter_conf *config, float *a, int an, float *b);

/** Process a vector using an IIR filter
  @param config An initialized instance of an sp_filter_conf to use
  @param x Data vector to process
  @param xn Length of the vector x
  @param xp Output filtered data vector
*/
int radarDSP_spz_iirz_filter(sp_filter_conf *config, float *x, int xn, float *xp);

/** Free the memory associated with a complex IIR filter
  @param config An instance of type sp_filter_conf to free the memory of
*/
int radarDSP_spz_iirz_free(sp_filter_conf *config);

/** Create an IIR Filter (complex double)
  @param config instance of an dp_filter_conf to initialize
  @param a Denomerator coefficients to initialize
  @param b Numerator coefficients to initialize
  @param an Number of denomerator coefficients
*/
int radarDSP_dpz_iirz_init(dp_filter_conf *config, double *a, int an, double *b);

/** Process a vector using an IIR filter
  @param config An initialized instance of an dp_filter_conf to use
  @param x Data vector to process
  @param xn Length of the vector x
  @param xp Output filtered data vector
*/
int radarDSP_dpz_iirz_filter(dp_filter_conf *config, double *x, int xn, double *xp);

/** Free the memory associated with a complex IIR filter
  @param config An instance of type dp_filter_conf to free the memory of
*/
int radarDSP_dpz_iirz_free(dp_filter_conf *config);

/** Create an IIR Filter (integer)
  @param config instance of an int_filter_conf to initialize
  @param a Denomerator coefficients to initialize
  @param b Numerator coefficients to initialize
  @param an Number of denomerator coefficients
  @param bn Number of numerator coefficients
*/
int radarDSP_iir_init(int_filter_conf *config, int *a, int an, int *b, int bn);

/** Process a vector using an IIR filter
  @param config An initialized instance of an int_filter_conf to use
  @param x Data vector to process
  @param xn Length of the vector x
  @param xp Output filtered data vector
*/
int radarDSP_iir_filter(int_filter_conf *config, int *x, int xn, int *xp);

/** Free the memory associated with a complex IIR filter
  @param config An instance of type int_filter_conf to free the memory of
*/
int radarDSP_iir_free(int_filter_conf *config);

/** Create an IIR Filter (float)
  @param config instance of an sp_filter_conf to initialize
  @param a Denomerator coefficients to initialize
  @param b Numerator coefficients to initialize
  @param an Number of denomerator coefficients
*/
int radarDSP_sp_iir_init(sp_filter_conf *config, float *a, int an, float *b);

/** Process a vector using an IIR filter
  @param config An initialized instance of an sp_filter_conf to use
  @param x Data vector to process
  @param xn Length of the vector x
  @param xp Output filtered data vector
*/
int radarDSP_sp_iir_filter(sp_filter_conf *config, float *x, int xn, float *xp);

/** Free the memory associated with a complex IIR filter
  @param config An instance of type sp_filter_conf to free the memory of
*/
int radarDSP_sp_iir_free(sp_filter_conf *config);

/** Create an IIR Filter (double)
  @param config instance of an dp_filter_conf to initialize
  @param a Denomerator coefficients to initialize
  @param b Numerator coefficients to initialize
  @param an Number of denomerator coefficients
*/
int radarDSP_dp_iir_init(dp_filter_conf *config, double *a, int an, double *b);

/** Process a vector using an IIR filter
  @param config An initialized instance of an dp_filter_conf to use
  @param x Data vector to process
  @param xn Length of the vector x
  @param xp Output filtered data vector
*/
int radarDSP_dp_iir_filter(dp_filter_conf *config, double *x, int xn, double *xp);

/** Free the memory associated with a complex IIR filter
  @param config An instance of type dp_filter_conf to free the memory of
*/
int radarDSP_dp_iir_free(dp_filter_conf *config);





/** Create a vector of equally spaced points
  @param start Value for the first elements
  @param stop Value of the last element
  @param length Length of the sequence to create
  @param sequence Vector to hold the resulting list
  @return SUCCESS
*/
int radarDSP_linspace(int start, int stop, int length, int *sequence);

/** Create a vector of equally spaced points
  @param start Value for the first elements
  @param stop Value of the last element
  @param length Length of the sequence to create
  @param sequence Vector to hold the resulting list
  @return SUCCESS
*/
int radarDSP_sp_linspace(float start, float stop, int length, float *sequence);

/** Create a vector of equally spaced points
  @param start Value for the first elements
  @param stop Value of the last element
  @param length Length of the sequence to create
  @param sequence Vector to hold the resulting list
  @return SUCCESS
*/
int radarDSP_dp_linspace(double start, double stop, int length, double *sequence);




/* END Alpha Functions, subject to change.... */

// -------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------
// END Release-candidate level functions.
// -------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------



#ifdef __cplusplus
}
#endif

#endif
