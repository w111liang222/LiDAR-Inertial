//============================================================================================================
//!
//! \file NComRxC.c
//!
//! \brief NCom C decoder.
//!
//============================================================================================================




//############################################################################################################
//##                                                                                                        ##
//##  Includes and Definitions                                                                              ##
//##                                                                                                        ##
//############################################################################################################


//============================================================================================================
// Includes.
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include <decoder/NComRxC.h>


void report(const NComRxC *nrx);
void print(FILE *fp, const NComRxC *nrx);
//============================================================================================================
// Definitions.

// General constants.
#define NOUTPUT_PACKET_LENGTH  (72)       //!< NCom packet length.
#define NCOM_SYNC           (0xE7)        //!< NCom sync byte.
#define PKT_PERIOD          (0.01)        //!< 10ms updates.
#define TIME2SEC            (1e-3)        //!< Units of 1 ms.
#define FINETIME2SEC        (4e-6)        //!< Units of 4 us.
#define TIMECYCLE           (60000)       //!< Units of TIME2SEC (i.e. 60 seconds).
#define WEEK2CYCLES         (10080)       //!< Time cycles in a week.
#define ACC2MPS2            (1e-4)        //!< Units of 0.1 mm/s^2.
#define RATE2RPS            (1e-5)        //!< Units of 0.01 mrad/s.
#define VEL2MPS             (1e-4)        //!< Units of 0.1 mm/s.
#define ANG2RAD             (1e-6)        //!< Units of 0.001 mrad.
#define INNFACTOR           (0.1)         //!< Resolution of 0.1.
#define POSA2M              (1e-3)        //!< Units of 1 mm.
#define VELA2MPS            (1e-3)        //!< Units of 1 mm/s.
#define ANGA2RAD            (1e-5)        //!< Units of 0.01 mrad.
#define GB2RPS              (5e-6)        //!< Units of 0.005 mrad/s.
#define AB2MPS2             (1e-4)        //!< Units of 0.1 mm/s^2.
#define GSFACTOR            (1e-6)        //!< Units of 1 ppm.
#define ASFACTOR            (1e-6)        //!< Units of 1 ppm.
#define GBA2RPS             (1e-6)        //!< Units of 0.001 mrad/s.
#define ABA2MPS2            (1e-5)        //!< Units of 0.01 mm/s^2.
#define GSAFACTOR           (1e-6)        //!< Units of 1 ppm.
#define ASAFACTOR           (1e-6)        //!< Units of 1 ppm.
#define GPSPOS2M            (1e-3)        //!< Units of 1 mm.
#define GPSATT2RAD          (1e-4)        //!< Units of 0.1 mrad.
#define GPSPOSA2M           (1e-4)        //!< Units of 0.1 mm.
#define GPSATTA2RAD         (1e-5)        //!< Units of 0.01 mrad.
#define INNFACTOR           (0.1)         //!< Resolution of 0.1.
#define DIFFAGE2SEC         (1e-2)        //!< Units of 0.01 s.
#define REFPOS2M            (0.0012)      //!< Units of 1.2 mm.
#define REFANG2RAD          (1e-4)        //!< Units of 0.1 mrad.
#define OUTPOS2M            (1e-3)        //!< Units of 1 mm.
#define ZVPOS2M             (1e-3)        //!< Units of 1 mm.
#define ZVPOSA2M            (1e-4)        //!< Units of 0.1 mm.
#define NSPOS2M             (1e-3)        //!< Units of 1 mm.
#define NSPOSA2M            (1e-4)        //!< Units of 0.1 mm.
#define ALIGN2RAD           (1e-4)        //!< Units of 0.1 mrad.
#define ALIGNA2RAD          (1e-5)        //!< Units of 0.01 mrad.
#define SZVDELAY2S          (1.0)         //!< Units of 1.0 s.
#define SZVPERIOD2S         (0.1)         //!< Units of 0.1 s.
#define TOPSPEED2MPS        (0.5)         //!< Units of 0.5 m/s.
#define NSDELAY2S           (0.1)         //!< Units of 0.1 s.
#define NSPERIOD2S          (0.02)        //!< Units of 0.02 s.
#define NSACCEL2MPS2        (0.04)        //!< Units of 0.04 m/s^2.
#define NSSPEED2MPS         (0.1)         //!< Units of 0.1 m/s.
#define NSRADIUS2M          (0.5)         //!< Units of 0.5 m.
#define INITSPEED2MPS       (0.1)         //!< Units of 0.1 m/s.
#define HLDELAY2S           (1.0)         //!< Units of 1.0 s.
#define HLPERIOD2S          (0.1)         //!< Units of 0.1 s.
#define STATDELAY2S         (1.0)         //!< Units of 1.0 s.
#define STATSPEED2MPS       (0.01)        //!< Units of 1.0 cm/s.
#define WSPOS2M             (1e-3)        //!< Units of 1 mm.
#define WSPOSA2M            (1e-4)        //!< Units of 0.1 mm.
#define WSSF2PPM            (0.1)         //!< Units of 0.1 pulse per metre (ppm).
#define WSSFA2PC            (0.002)       //!< Units of 0.002% of scale factor.
#define WSDELAY2S           (0.1)         //!< Units of 0.1 s.
#define WSNOISE2CNT         (0.1)         //!< Units of 0.1 count for wheel speed noise.
#define UNDUL2M             (0.005)       //!< Units of 5 mm.
#define DOPFACTOR           (0.1)         //!< Resolution of 0.1.
#define OMNISTAR_MIN_FREQ   (1.52e9)      //!< (Hz) i.e. 1520.0 MHz.
#define OMNIFREQ2HZ         (1000.0)      //!< Resolution of 1 kHz.
#define SNR2DB              (0.2)         //!< Resolution of 0.2 dB.
#define LTIME2SEC           (1.0)         //!< Resolution of 1.0 s.
#define TEMPK_OFFSET        (203.15)      //!< Temperature offset in degrees K.
#define ABSZERO_TEMPC       (-273.15)     //!< Absolute zero (i.e. 0 deg K) in deg C.

// For more accurate and complete local coordinates
#define FINEANG2RAD (1.74532925199433e-9) //!< Units of 0.1 udeg.
#define ALT2M               (1e-3)        //!< Units of 1 mm.

// For GPS supply voltage
#define SUPPLYV2V           (0.1)         //!< Units of 0.1 V.

// Mathematical constant definitions
#ifndef M_PI
#define M_PI (3.1415926535897932384626433832795)  //!< Pi.
#endif
#define DEG2RAD             (M_PI/180.0)  //!< Convert degrees to radians.
#define RAD2DEG             (180.0/M_PI)  //!< Convert radians to degrees.
#define POS_INT_24          (8388607)     //!< Maximum value of a two's complement 24 bit integer.
#define NEG_INT_24          (-8388607)    //!< Minimum value of a two's complement 24 bit integer.
#define INV_INT_24          (-8388608)    //!< Represents an invalid two's complement 24 bit integer.

#define NCOM_COUNT_TOO_OLD  (150)         //!< Cycle counter for data too old.
#define NCOM_STDCNT_MAX     (0xFF)        //!< Definition for the RTBNS accuracy counter.
#define MIN_HORZ_SPEED      (0.07)        //!< 0.07 m/s hold distance.
#define MIN_VERT_SPEED      (0.07)        //!< 0.07 m/s hold distance.
#define SPEED_HOLD_FACTOR   (2.0)         //!< Hold distance when speed within 2 sigma of 0.
#define MINUTES_IN_WEEK     (10080)       //!< Number of minutes in a week.

// OmniStar status definitions
#define NCOM_OMNI_STATUS_UNKNOWN      (0xFF)
#define NCOM_OMNI_STATUS_VBSEXPIRED   (0x01)
#define NCOM_OMNI_STATUS_VBSREGION    (0x02)
#define NCOM_OMNI_STATUS_VBSNOBASE    (0x04)
#define NCOM_OMNI_STATUS_HPEXPIRED    (0x08)
#define NCOM_OMNI_STATUS_HPREGION     (0x10)
#define NCOM_OMNI_STATUS_HPNOBASE     (0x20)
#define NCOM_OMNI_STATUS_HPNOCONVERGE (0x40)
#define NCOM_OMNI_STATUS_HPKEYINVALID (0x80)

// GPS hardware status definitions
#define NCOM_GPS_ANT_STATUS_BITMASK   (0x03)
#define NCOM_GPS_ANT_STATUS_DONTKNOW  (0x03)
#define NCOM_GPS_ANT_STATUS_BITSHIFT  (0)
#define NCOM_GPS_ANT_POWER_BITMASK    (0x0C)
#define NCOM_GPS_ANT_POWER_DONTKNOW   (0x0C)
#define NCOM_GPS_ANT_POWER_BITSHIFT   (2)

// GPS feature set 1 definitions
#define NCOM_GPS_FEATURE_PSRDIFF      (0x01)
#define NCOM_GPS_FEATURE_SBAS         (0x02)
#define NCOM_GPS_FEATURE_OMNIVBS      (0x08)
#define NCOM_GPS_FEATURE_OMNIHP       (0x10)
#define NCOM_GPS_FEATURE_L1DIFF       (0x20)
#define NCOM_GPS_FEATURE_L1L2DIFF     (0x40)

// GPS feature set 2 definitions
#define NCOM_GPS_FEATURE_GLONASS      (0x01)
#define NCOM_GPS_FEATURE_GALILEO      (0x02)
#define NCOM_GPS_FEATURE_RAWRNG       (0x04)
#define NCOM_GPS_FEATURE_RAWDOP       (0x08)
#define NCOM_GPS_FEATURE_RAWL1        (0x10)
#define NCOM_GPS_FEATURE_RAWL2        (0x20)
#define NCOM_GPS_FEATURE_RAWL5        (0x40)

// GPS feature valid definition
#define NCOM_GPS_FEATURE_VALID        (0x80)

// The start of GPS time in a time_t style. In this version it is a constant, but this constant assumes that
// the local machine uses 00:00:00 01/01/1970 as its Epoch time. If your machine is different then you need to
// convert 00:00:00 06/01/1980 in to the local machine's time_t time.
#define GPS_TIME_START_TIME_T         (315964800)

// Second order filter class
#define INPUT_JITTER_TOLERANCE     (0.01)  // i.e. 1%

// Some print format strings
static const char PNV_S_[] = "%s%s\t%d\t%s\n";           //!< Format string: prefix, name, valid, string
static const char PNV_I_[] = "%s%s\t%d\t%d\n";           //!< Format string: prefix, name, valid, int
static const char PNV_IE[] = "%s%s\t%d\t%d (%s)\n";      //!< Format string: prefix, name, valid, int, enumerated
static const char PNV_D_[] = "%s%s\t%d\t%1.16e\n";       //!< Format string: prefix, name, valid, double
static const char PNA_D_[] = "%s%s\t%d\t%1.16e\n";       //!< Format string: prefix, name, age, double
static const char PNVXD_[] = "%s%s\t%d\t%d\t%1.16e\n";    //!< Format string: prefix, name, valid, approx, double
static const char PNVCD_[] = "%s%s\t%d\t%d\t%1.16e\n";    //!< Format string: prefix, name, valid, config, double
static const char PNVND_[] = "%s%s\t%d\t%d\t%1.16e\n";    //!< Format string: prefix, name, valid, new, double
static const char PNV_B_[] = "%s%s\t%d\t%" PRId16 "\n";  //!< Format string: prefix, name, valid, int16_t
static const char PNV_Q_[] = "%s%s\t%d\t%" PRId32 "\n";  //!< Format string: prefix, name, valid, int32_t
static const char PNV_O_[] = "%s%s\t%d\t%" PRId64 "\n";  //!< Format string: prefix, name, valid, int64_t
static const char PNV_b_[] = "%s%s\t%d\t%" PRIu16 "\n";  //!< Format string: prefix, name, valid, uint16_t
static const char PNV_q_[] = "%s%s\t%d\t%" PRIu32 "\n";  //!< Format string: prefix, name, valid, uint32_t
static const char PNV_o_[] = "%s%s\t%d\t%" PRIu64 "\n";  //!< Format string: prefix, name, valid, uint64_t

// Array range checks
#define COM_INVALID(x) ( ((int)(sizeof(x)/sizeof(x[0]))) - 1 )
#define COM_CK_VALIDITY(x,y) (x>COM_INVALID(y) ? COM_INVALID(y) : x )




//############################################################################################################
//##                                                                                                        ##
//##  Static declarations                                                                                   ##
//##                                                                                                        ##
//############################################################################################################


//============================================================================================================
// Types.

//! Various packet parsing packet states.
typedef enum
{
	PARSED_PACKET_VALID,      //!< Parsed packet in good shape.
	PARSED_PACKET_INCOMPLETE, //!< Parsed packet is incomplete.
	PARSED_PACKET_CORRUPT     //!< Parsed packet is corrupted.
} ParsedPacketType;


//============================================================================================================
// Functions.

static ParsedPacketType PktStatus(size_t Len, const unsigned char *p);
static void RemoveFromBuffer(NComRxCInternal *Com, int n);
static void UpdateNavInvalidate(NComRxC *Com);
void UpdateNav(NComRxC *Com);
static void FilteredOutputsInvalidate(NComRxC *Com);
static void FilteredOutputsCompute(NComRxC *Com);
static void RotateOutputsInvalidate(NComRxC *Com);
static void RotateOutputsCompute(NComRxC *Com);
static void SpeedSlipInvalidate(NComRxC *Com);
static void SpeedSlipCompute(NComRxC *Com);
static void DistanceInvalidate(NComRxC *Com);
static void DistanceCompute(NComRxC *Com, char trig);
static void NComSetLinAccFiltFreq(NComRxC *Com, double freq);
static void NComSetLinAccFiltZeta(NComRxC *Com, double zeta);
static void NComFixLinAccFilt(NComRxC *Com, double freq, double zeta);
static void NComClearLinAccFilt(NComRxC *Com);
static void NComSetAngAccFiltFreq(NComRxC *Com, double freq);
static void NComSetAngAccFiltZeta(NComRxC *Com, double zeta);
static void NComFixAngAccFilt(NComRxC *Com, double freq, double zeta);
static void NComClearAngAccFilt(NComRxC *Com);
static void SetRefFrame(NComRxC *Com, double lat, double lon, double alt, double heading);
static char IsDevIDAfter(NComRxC *Com, char* datestr );

// Decode functions
static void DecodeStatusMsg(NComRxC *Com);
static void DecodeExtra0(NComRxC *Com);
static void DecodeExtra1(NComRxC *Com);
static void DecodeExtra2(NComRxC *Com);
static void DecodeExtra3(NComRxC *Com);
static void DecodeExtra4(NComRxC *Com);
static void DecodeExtra5(NComRxC *Com);
static void DecodeExtra6(NComRxC *Com);
static void DecodeExtra7(NComRxC *Com);
static void DecodeExtra8(NComRxC *Com);
static void DecodeExtra9(NComRxC *Com);
static void DecodeExtra10(NComRxC *Com);
static void DecodeExtra11(NComRxC *Com);
static void DecodeExtra12(NComRxC *Com);
static void DecodeExtra13(NComRxC *Com);
static void DecodeExtra14(NComRxC *Com);
static void DecodeExtra15(NComRxC *Com);
static void DecodeExtra16(NComRxC *Com);
static void DecodeExtra17(NComRxC *Com);
static void DecodeExtra18(NComRxC *Com);
static void DecodeExtra19(NComRxC *Com);
static void DecodeExtra20(NComRxC *Com);
static void DecodeExtra21(NComRxC *Com);
static void DecodeExtra22(NComRxC *Com);
static void DecodeExtra23(NComRxC *Com);
static void DecodeExtra24(NComRxC *Com);
static void DecodeExtra25(NComRxC *Com);
static void DecodeExtra26(NComRxC *Com);
static void DecodeExtra27(NComRxC *Com);
static void DecodeExtra28(NComRxC *Com);
static void DecodeExtra29(NComRxC *Com);
static void DecodeExtra30(NComRxC *Com);
static void DecodeExtra31(NComRxC *Com);
static void DecodeExtra32(NComRxC *Com);
static void DecodeExtra33(NComRxC *Com);
static void DecodeExtra34(NComRxC *Com);
static void DecodeExtra35(NComRxC *Com);
static void DecodeExtra36(NComRxC *Com);
static void DecodeExtra37(NComRxC *Com);
static void DecodeExtra38(NComRxC *Com);
static void DecodeExtra39(NComRxC *Com);
static void DecodeExtra41(NComRxC *Com);
static void DecodeExtra42(NComRxC *Com);
static void DecodeExtra43(NComRxC *Com);
static void DecodeExtra44(NComRxC *Com);
static void DecodeExtra45(NComRxC *Com);
static void DecodeExtra46(NComRxC *Com);
static void DecodeExtra47(NComRxC *Com);
static void DecodeExtra48(NComRxC *Com);
static void DecodeExtra49(NComRxC *Com);
static void DecodeExtra50(NComRxC *Com);
static void DecodeExtra55(NComRxC *Com);
static void DecodeExtra56(NComRxC *Com);
static void DecodeExtra57(NComRxC *Com);
static void DecodeExtra59(NComRxC *Com);
static void DecodeExtra61(NComRxC *Com);
static void DecodeExtra62(NComRxC *Com);
static void DecodeExtra63(NComRxC *Com);
static void DecodeExtra64(NComRxC *Com);
static void DecodeExtra65(NComRxC *Com);
static void DecodeExtra66(NComRxC *Com);
static void DecodeExtra67(NComRxC *Com);
static void DecodeExtra72(NComRxC *Com);
static void DecodeExtra73(NComRxC *Com);
static void DecodeExtra74(NComRxC *Com);
static void DecodeExtraGpsReceived(const unsigned char *mCurStatus, NComRxCGps *Com);
static void DecodeExtraGpsStatus  (const unsigned char *mCurStatus, NComRxCGps *Com);

// Utilities
static double   cast_8_byte_to_double(const uint8_t *b);
static float    cast_4_byte_to_float (const uint8_t *b);
static  int32_t cast_4_byte_to_int32 (const uint8_t *b);
static uint32_t cast_4_byte_to_uint32(const uint8_t *b);
static  int32_t cast_3_byte_to_int32 (const uint8_t *b);
static uint32_t cast_3_byte_to_uint32(const uint8_t *b);
static  int16_t cast_2_byte_to_int16 (const uint8_t *b);
static uint16_t cast_2_byte_to_uint16(const uint8_t *b);
static uint32_t incr_2_byte_to_uint32(const uint8_t *b, uint32_t z);
static uint32_t incr_1_byte_to_uint32(const uint8_t *b, uint32_t z);
static void     strgrab(char *destination, int destination_length, const char *source, int source_length);

// Filter functions
static Filt2ndOrder *Filt2ndOrderCreate();
static void Filt2ndOrderDestroy(Filt2ndOrder *Filt);
static void Filt2ndOrderReset(Filt2ndOrder *Filt);
static void Filt2ndOrderSetCharacteristics(Filt2ndOrder *Filt, double freq, double zeta);
static void Filt2ndOrderNewInput(Filt2ndOrder *Filt, double t, double x);
static void Filt2ndOrderInitialise(Filt2ndOrder *Filt);

// Matrix library
static int MatAllocR(Mat *R, long r, long c);
static int MatFillR(Mat *R, long r, long c, ...);
static int MatFree(Mat *A);
static int MatMultRAB(Mat *R, Mat *A, Mat *B);
static int MatMultRAtB(Mat *r, Mat *a, Mat *b);
static int Euler2DirCos2(Mat *C, Mat *E);
static int Euler2DirCos2_1(Mat *C, Mat *E);
static int Euler2DirCosH(Mat *C, Mat *E);
static int compute_earth_curvature(double *rho_e, double *rho_n, double lat);


//============================================================================================================
// Enumerated types.

static const char *OutputPacketName[8] =
{
	"Invalid",
	"Empty",
	"Regular",
	"Status",
	"In 1 Down",
	"In 1 Up",
	"Out 1",
	"Interpolated"
};

static const char *NavigationStatusName[24] =
{
   "Invalid",
   "Raw Inertial Data",
   "Ready to Initialise",
   "Locking On",
   "Real Time",
   "Unlocked",
   "Firmware Expired",
   "Reserved",
   "Reserved",
   "Reserved",
   "Status Only",
   "Reserved",
   "Reserved",
   "Reserved",
   "Reserved",
   "Reserved",
   "Reserved",
   "Reserved",
   "Reserved",
   "Reserved",
   "Trigger (Initialise)",
   "Trigger (Locking On)",
   "Trigger (Real Time)",
   "Unknown"
};

static const char *ComUmacStatus[11] =
{
   "Error",
   "Time Valid",
   "Speed Threshold",
   "Output Lag",
   "Aligning Axis",
   "Bad Position",
   "Poor Position",
   "SPS Position",
   "Differential Position",
   "RTK Position",
   "Unknown"
};

static const char *ComOptionVehicle[3] =
{
   "Initially not level",
   "Initially level",
   "Invalid"
};

static const char *ComOptionVibration[4] =
{
   "Normal",
   "High",
   "Very High",
   "Invalid"
};

static const char *ComOptionGpsAccuracy[4] =
{
   "Some Obstructions",
   "Open Sky",
   "Frequent Obstructions",
   "Invalid"
};

static const char *ComOptionOutput[16] =
{
   "NCOM",
   "TCOM",
   "ABD",
   "TSS1",
   "THALES",
   "NMEA",
   "ACOM",
   "MCOM",
   "EM3000",
   "EM1000",
   "TSS HHRP",
   "PASHR",
   "PRDID",
   "MGCOM1",
   "Javad I+RTK",
   "Invalid"
};

static const char *ComOptionHeading[5] =
{
   "Never",
   "No Search",
   "After Initialisation",
   "Always",
   "Invalid"
};

static const char *ComHeadQuality[5] =
{
   "None",
   "Poor",
   "RTK Float",
   "RTK Integer",
   "Invalid"
};

static const char *ComHeadSearchType[5] =
{
   "Idle",
   "L1",
   "L2",
   "L1/L2",
   "Invalid"
};

static const char *ComHeadSearchStatus[20] =
{
   "OK",
   "No Spare CPU",
   "No Seed",
   "No Master",
   "No Slave1",
   "No Slave2",
   "No Slave3",
   "Bad Length",
   "No matching ambiguities",
   "Too many ambiguities",
   "Lost Master",
   "Lost Slave1",
   "Lost Slave2",
   "Lost Slave3",
   "Sat Constellation Too Poor",
   "Covariance Error",
   "Ambiguous Ambiguities",
   "Lost Lock",
   "Disabled",
   "Invalid"
};

static const char *ComHeadSearchReady[3] =
{
   "Waiting",
   "Processing",
   "Invalid"
};

static const char *ImuTypeName[8]=
{
   "SiIMU-A",
   "R&D IMU",
   "IMU2",
   "IMU2X",
   "IMU3",
   "IMU3X",
   "IMU5",
   "Unknown"
};

static const char *InterPcbTypeName[5]=
{
   "14P0008A",
   "14P0008B",
   "14P0008C",
   "14P0008D",
   "Unknown"
};

static const char *FrontPcbTypeName[8]=
{
   "14P0007A",
   "14P0009A",
   "14P0009B",
   "14P0009C",
   "14P0019C",
   "14P0019D",
   "14P0034C",
   "Unknown"
};

static const char *InterSwIdName[13]=
{
   "None",
   "030528.14an",
   "030724.14an",
   "030731.14an",
   "031023.14an",
   "031107.14an",
   "040131.14an",
   "050110.14an",
   "060105.14an",
   "080102.14an",
   "080204.14an",
   "081215.14bj",
   "Unknown"
};

static const char *DeployCfgName[6]=
{
   "Integral Small Box",
   "Integral Standard Box",
   "Pod and Rack",
   "Extruded Standard Box",
   "Pod and Extrusion",
   "Unknown"
};

static const char *CpuPcbTypeName[3]=
{
   "TP400B",
   "TP500",
   "Unknown"
};

static const char *DualPortRamStatusName[12] =
{
   "Not Fitted",
   "Failed To Initialise",
   "Dead",
   "Down",
   "Overloaded",
   "Sporadic",
   "Slow",
   "Acceptable",
   "OK",
   "Good",
   "Excellent",
   "Unknown"
};

static const char *ComOptionSerBaud[16] =
{
   "Disabled",
   "300",
   "600",
   "1200",
   "2400",
   "4800",
   "9600",
   "19200",
   "38400",
   "57600",
   "76800",
   "115200",
   "230400",
   "460800",
   "921600",
   "Invalid"
};

static const char *ComOptionCanBaud[8] =
{
   "Disabled",
   "100000",
   "125000",
   "200000",
   "250000",
   "500000",
   "1000000",
   "Invalid"
};

static const char *ComGpsTypeName[11]=
{
   "BeeLine",
   "OEM4",
   "None",
   "OEMV",
   "LEA4",
   "Generic",
   "Trimble 5700/5800",
   "Trimble AgGPS 132",
   "Topcon GB-500",
   "NavCom Sapphire",
   "Unknown"
};

static const char *ComGpsFormatName[10]=
{
   "OEM3 Binary",
   "OEM4 Binary",
   "UBX",
   "NMEA",
   "GSOF",
   "TSIP",
   "GRIL",
   "Debug",
   "NCT Binary",
   "Unknown"
};

static const char *ComGpsXRateName[8] =
{
	"Disabled",
	"1",
	"2",
	"4",
	"5",
	"10",
	"20",
	"Invalid"
};

static const char *ComGpsAntStatusName[4] =
{
   "OK",
   "Open",
   "Short",
   "Unknown"
};

static const char *ComGpsAntPowerName[3] =
{
   "On",
   "Off",
   "Unknown"
};

static const char *ComGpsXModeName[31] =
{
   "None",
   "Search",
   "Doppler",
   "SPS",
   "Differential",
   "RTK Float",
   "RTK Integer",
   "WAAS",
   "Omnistar",
   "Omnistar HP",
   "No Data",
   "Blanked",
   "Doppler (PP)",
   "SPS (PP)",
   "Differential (PP)",
   "RTK Float (PP)",
   "RTK Integer (PP)",
   "Omnistar XP",
   "CDGPS",
   "Not Recognised",
   "gxDoppler",
   "gxSPS",
   "gxDifferential",
   "gxFloat",
   "gxInteger",
   "ixDoppler",
   "ixSPS",
   "ixDifferential",
   "ixFloat",
   "ixInteger",
   "Unknown"
};




//############################################################################################################
//##                                                                                                        ##
//##  NComRxCInteral                                                                                        ##
//##                                                                                                        ##
//############################################################################################################


//============================================================================================================
//! \brief Invalidation of internally used data space of the decoder.

static void NComInternalInvalidate(NComRxCInternal *Com)
{
	Com->mCurLen          =  0;
	Com->mPktProcessed    =  0;

	//Timing
	Com->mMilliSecs       = -1;
	Com->mMinutes         = -1;

	// Triggers
	Com->mTrigCount       =  0;
	Com->mTrig2Count      =  0;
	Com->mDigitalOutCount =  0;

	// Clear the parameters for calculating distance travelled
	Com->mPrevDist2dValid   = 0;
	Com->mPrevDist2dTime    = 0.0;
	Com->mPrevDist2dSpeed   = 0.0;
	Com->mPrevDist2d        = 0.0;
	Com->mPrevDist3dValid   = 0;
	Com->mPrevDist3dTime    = 0.0;
	Com->mPrevDist3dSpeed   = 0.0;
	Com->mPrevDist3d        = 0.0;

	// Clear the parameters for calculating wheel-speed tacho frequency
	Com->mIsOldWSpeedTimeValid      = 0; Com->mOldWSpeedTime      = 0.0;
	Com->mIsOldWSpeedCountValid     = 0; Com->mOldWSpeedCount     = 0.0;

	// Reference frame parameters
	Com->mIsAccurateRefLatValid     = 0; Com->mAccurateRefLat     = 0.0;
	Com->mIsAccurateRefLonValid     = 0; Com->mAccurateRefLon     = 0.0;
	Com->mIsAccurateRefAltValid     = 0; Com->mAccurateRefAlt     = 0.0;
	Com->mIsAccurateRefHeadingValid = 0; Com->mAccurateRefHeading = 0.0;

	// Linear acceleration filter
	Filt2ndOrderReset(&Com->FiltForAx);
	Filt2ndOrderReset(&Com->FiltForAy);
	Filt2ndOrderReset(&Com->FiltForAz);

	// Reset the angular rate differentiation variables
	Com->mPrevWx     = 0.0;
	Com->mPrevWy     = 0.0;
	Com->mPrevWz     = 0.0;
	Com->mPrevWbTime = 0.0;

	// Angular acceleration filter
	Filt2ndOrderReset(&Com->FiltForYx);
	Filt2ndOrderReset(&Com->FiltForYy);
	Filt2ndOrderReset(&Com->FiltForYz);
}


//============================================================================================================
//! \brief Basic report to a file pointer.

static void NComInternalReportFP(const NComRxCInternal *Com, FILE *fp, const char *pre)
{
	fprintf(fp, "%s%s %" PRIu64 "\n", pre, "NumChars",     Com->mNumChars);
	fprintf(fp, "%s%s %" PRIu64 "\n", pre, "SkippedChars", Com->mSkippedChars);
	fprintf(fp, "%s%s %" PRIu64 "\n", pre, "NumPackets",   Com->mNumPackets);
}


//============================================================================================================
//! \brief Constructor for internally used data space of the decoder.
//!
//! If any of the work space matrices did not allocate, they are all freed and a flag instructs the decoder
//! that they are not available. This mean any quantity requiring these matrices will not be decoded.

static NComRxCInternal *NComInternalCreate()
{
	NComRxCInternal *Com = (NComRxCInternal *)calloc(1, sizeof(NComRxCInternal));

	if (Com)
	{
		Com->mCurStatus   = Com->mCurPkt + PI_CHANNEL_STATUS;

		Com->mMatrixHold  = MatAllocR(&Com->E,  3, 1);
		Com->mMatrixHold |= MatAllocR(&Com->Cb, 3, 3);
		Com->mMatrixHold |= MatAllocR(&Com->Cb_1, 3, 3);      //new
		Com->mMatrixHold |= MatAllocR(&Com->Ch, 3, 3);
		Com->mMatrixHold |= MatAllocR(&Com->Ab, 3, 1);
		Com->mMatrixHold |= MatAllocR(&Com->Al, 3, 1);
		Com->mMatrixHold |= MatAllocR(&Com->Wb, 3, 1);
		Com->mMatrixHold |= MatAllocR(&Com->Wl, 3, 1);
		Com->mMatrixHold |= MatAllocR(&Com->Vn, 3, 1);
		Com->mMatrixHold |= MatAllocR(&Com->Vl, 3, 1);
		Com->mMatrixHold |= MatAllocR(&Com->Vb, 3, 1);        //new
		Com->mMatrixHold |= MatAllocR(&Com->Yb, 3, 1);
		Com->mMatrixHold |= MatAllocR(&Com->Yl, 3, 1);

		if (Com->mMatrixHold)
		{
			MatFree(&Com->E);
			MatFree(&Com->Cb);
			MatFree(&Com->Cb_1);          //new
			MatFree(&Com->Ch);
			MatFree(&Com->Ab);
			MatFree(&Com->Al);
			MatFree(&Com->Wb);
			MatFree(&Com->Wl);
			MatFree(&Com->Vn);
			MatFree(&Com->Vl);
			MatFree(&Com->Vb);            //new
			MatFree(&Com->Yb);
			MatFree(&Com->Yl);
		}

		Com->mNumChars         = 0;
		Com->mSkippedChars     = 0;
		Com->mNumPackets       = 0;
		Com->mHoldDistWhenSlow = 0;
	}

	// Reset the linear acceleration filters
	Filt2ndOrderReset(&Com->FiltForAx);
	Filt2ndOrderReset(&Com->FiltForAy);
	Filt2ndOrderReset(&Com->FiltForAz);

	// Reset the angular acceleration filters
	Filt2ndOrderReset(&Com->FiltForYx);
	Filt2ndOrderReset(&Com->FiltForYy);
	Filt2ndOrderReset(&Com->FiltForYz);

	return Com;
}


//============================================================================================================
//! \brief Destructor for internally used data space of the decoder.

static void NComInternalDestroy(NComRxCInternal *Com)
{
	if (Com != NULL)
	{
		if (!Com->mMatrixHold)
		{
			MatFree(&Com->E );
			MatFree(&Com->Cb);
			MatFree(&Com->Cb_1);          //new
			MatFree(&Com->Ch);
			MatFree(&Com->Ab);
			MatFree(&Com->Al);
			MatFree(&Com->Wb);
			MatFree(&Com->Wl);
			MatFree(&Com->Vn);
			MatFree(&Com->Vl);
			MatFree(&Com->Vb);            //new
			MatFree(&Com->Yb);
			MatFree(&Com->Yl);
		}

		free(Com);
	}
}


//============================================================================================================
//! \brief Copy for internally used data space of the decoder.
//!
//! \warning mMatrixHold property: Destination->mMatrixHold = Destination->mMatrixHold or Source->mMatrixHold.
//! That is, if either destination and/or source have no matrices before copy, then the destination will have
//! no matrices after the copy.

void NComInternalCopy(NComRxCInternal *ComDestination, const NComRxCInternal *ComSource)
{
	if (ComSource->mMatrixHold)
	{
		if (ComDestination->mMatrixHold)
		{
			// No dynamic memory anywhere so do nothing here.
		}
		else
		{
			// Clear out the destination matrix memory as ComDestination->mMatrixHold will become true.

			MatFree(&ComDestination->E );
			MatFree(&ComDestination->Cb);
			MatFree(&ComDestination->Cb_1);      //new
			MatFree(&ComDestination->Ch);
			MatFree(&ComDestination->Ab);
			MatFree(&ComDestination->Al);
			MatFree(&ComDestination->Wb);
			MatFree(&ComDestination->Wl);
			MatFree(&ComDestination->Vn);
			MatFree(&ComDestination->Vl);
            MatFree(&ComDestination->Vb);        //new
			MatFree(&ComDestination->Yb);
			MatFree(&ComDestination->Yl);
		}

		// So now should have no dynamic memory and any pointers set null so low level shallow copy ok.

		memcpy(ComDestination, ComSource, sizeof(NComRxCInternal));
	}
	else
	{
		// Do a low level shallow copy and revert any clobbered pointers.

		MatElement *E  = ComDestination->E .m;
		MatElement *Cb = ComDestination->Cb.m;
		MatElement *Cb_1 = ComDestination->Cb_1.m;      //new
		MatElement *Ch = ComDestination->Ch.m;
		MatElement *Ab = ComDestination->Ab.m;
		MatElement *Al = ComDestination->Al.m;
		MatElement *Wb = ComDestination->Wb.m;
		MatElement *Wl = ComDestination->Wl.m;
		MatElement *Vn = ComDestination->Vn.m;
		MatElement *Vl = ComDestination->Vl.m;
		MatElement *Vb = ComDestination->Vb.m;         //new
		MatElement *Yb = ComDestination->Yb.m;
		MatElement *Yl = ComDestination->Yl.m;

		memcpy(ComDestination, ComSource, sizeof(NComRxCInternal));

		ComDestination->E .m = E;
		ComDestination->Cb.m = Cb;
		ComDestination->Cb_1.m = Cb_1;      //new
		ComDestination->Ch.m = Ch;
		ComDestination->Ab.m = Ab;
		ComDestination->Al.m = Al;
		ComDestination->Wb.m = Wb;
		ComDestination->Wl.m = Wl;
		ComDestination->Vn.m = Vn;
		ComDestination->Vl.m = Vl;
		ComDestination->Vb.m = Vb;         //new
		ComDestination->Yb.m = Yb;
		ComDestination->Yl.m = Yl;

		if (ComDestination->mMatrixHold)
		{
			// Even though the source used matrices, the destination does not so nothing to do.
		}
		else
		{
			// Now copy over the matrix data. ** Assumption: matrices have not been resized. **

			memcpy(ComDestination->E .m, ComSource->E .m, sizeof(ComSource->E .m));
			memcpy(ComDestination->Cb.m, ComSource->Cb.m, sizeof(ComSource->Cb.m));
			memcpy(ComDestination->Cb_1.m, ComSource->Cb_1.m, sizeof(ComSource->Cb_1.m));        //new
			memcpy(ComDestination->Ch.m, ComSource->Ch.m, sizeof(ComSource->Ch.m));
			memcpy(ComDestination->Ab.m, ComSource->Ab.m, sizeof(ComSource->Ab.m));
			memcpy(ComDestination->Al.m, ComSource->Al.m, sizeof(ComSource->Al.m));
			memcpy(ComDestination->Wb.m, ComSource->Wb.m, sizeof(ComSource->Wb.m));
			memcpy(ComDestination->Wl.m, ComSource->Wl.m, sizeof(ComSource->Wl.m));
			memcpy(ComDestination->Vn.m, ComSource->Vn.m, sizeof(ComSource->Vn.m));
			memcpy(ComDestination->Vl.m, ComSource->Vl.m, sizeof(ComSource->Vl.m));
			memcpy(ComDestination->Vb.m, ComSource->Vb.m, sizeof(ComSource->Vb.m));              //new
			memcpy(ComDestination->Yb.m, ComSource->Yb.m, sizeof(ComSource->Yb.m));
			memcpy(ComDestination->Yl.m, ComSource->Yl.m, sizeof(ComSource->Yl.m));
		}
	}

	// Ensure we do not clobber current status pointer.

	ComDestination->mCurStatus = ComDestination->mCurPkt + PI_CHANNEL_STATUS;
}



//############################################################################################################
//##                                                                                                        ##
//##  NComRxCGps                                                                                            ##
//##                                                                                                        ##
//############################################################################################################


//============================================================================================================
// Access functions.

//------------------------------------------------------------------------------------------------------------
// GPS Information

// System information

const char *NComGpsGetTypeString                     (const NComRxCGps *Com) { return ComGpsTypeName[Com->mType]; }
static void NComGpsSetTypeEnum                       (NComRxCGps *Com, uint8_t     v) { Com->mType = COM_CK_VALIDITY(v, ComGpsTypeName); Com->mIsTypeValid = 1; }
const char *NComGpsGetFormatString                   (const NComRxCGps *Com) { return ComGpsFormatName[Com->mFormat]; }
static void NComGpsSetFormatEnum                     (NComRxCGps *Com, uint8_t     v) { Com->mFormat = COM_CK_VALIDITY(v, ComGpsFormatName); Com->mIsFormatValid = 1; }

const char *NComGpsGetRawRateString                  (const NComRxCGps *Com) { return ComGpsXRateName[Com->mRawRate]; }
static void NComGpsSetRawRateEnum                    (NComRxCGps *Com, uint8_t     v) { Com->mRawRate = COM_CK_VALIDITY(v, ComGpsXRateName); Com->mIsRawRateValid = 1; }
const char *NComGpsGetPosRateString                  (const NComRxCGps *Com) { return ComGpsXRateName[Com->mPosRate]; }
static void NComGpsSetPosRateEnum                    (NComRxCGps *Com, uint8_t     v) { Com->mPosRate = COM_CK_VALIDITY(v, ComGpsXRateName); Com->mIsPosRateValid = 1; }
const char *NComGpsGetVelRateString                  (const NComRxCGps *Com) { return ComGpsXRateName[Com->mVelRate]; }
static void NComGpsSetVelRateEnum                    (NComRxCGps *Com, uint8_t     v) { Com->mVelRate = COM_CK_VALIDITY(v, ComGpsXRateName); Com->mIsVelRateValid = 1; }

const char *NComGpsGetAntStatusString                (const NComRxCGps *Com) { return ComGpsAntStatusName[Com->mAntStatus]; }
static void NComGpsSetAntStatusEnum                  (NComRxCGps *Com, uint8_t     v) { Com->mAntStatus = COM_CK_VALIDITY(v, ComGpsAntStatusName); Com->mIsAntStatusValid = 1; }
const char *NComGpsGetAntPowerString                 (const NComRxCGps *Com) { return ComGpsAntPowerName[Com->mAntPower]; }
static void NComGpsSetAntPowerEnum                   (NComRxCGps *Com, uint8_t     v) { Com->mAntPower = COM_CK_VALIDITY(v, ComGpsAntPowerName); Com->mIsAntPowerValid = 1; }
const char *NComGpsGetPosModeString                  (const NComRxCGps *Com) { return ComGpsXModeName[Com->mPosMode]; }
static void NComGpsSetPosModeEnum                    (NComRxCGps *Com, uint8_t     v) { Com->mPosMode = COM_CK_VALIDITY(v, ComGpsXModeName); Com->mIsPosModeValid = 1; }

const char *NComGpsGetSerBaudString                  (const NComRxCGps *Com) { return ComOptionSerBaud[Com->mSerBaud]; }
static void NComGpsSetSerBaudEnum                    (NComRxCGps *Com, uint8_t     v) { Com->mSerBaud = COM_CK_VALIDITY(v, ComOptionSerBaud); Com->mIsSerBaudValid = 1; }

// Status

static void NComGpsSetNumSats                        (NComRxCGps *Com, int         v) { Com->mNumSats = v; Com->mIsNumSatsValid = 1; }

static void NComGpsSetCpuUsed                        (NComRxCGps *Com, double      v) { Com->mCpuUsed = v; Com->mIsCpuUsedValid = 1; }
static void NComGpsSetCoreNoise                      (NComRxCGps *Com, double      v) { Com->mCoreNoise = v; Com->mIsCoreNoiseValid = 1; }
static void NComGpsSetCoreTemp                       (NComRxCGps *Com, double      v) { Com->mCoreTemp = v; Com->mIsCoreTempValid = 1; }
static void NComGpsSetSupplyVolt                     (NComRxCGps *Com, double      v) { Com->mSupplyVolt = v; Com->mIsSupplyVoltValid = 1; }

// Received data statistics

static void NComGpsSetChars                          (NComRxCGps *Com, uint32_t    v) { Com->mChars = v; Com->mIsCharsValid = 1; }
static void NComGpsSetCharsSkipped                   (NComRxCGps *Com, uint32_t    v) { Com->mCharsSkipped = v; Com->mIsCharsSkippedValid = 1; }
static void NComGpsSetPkts                           (NComRxCGps *Com, uint32_t    v) { Com->mPkts = v; Com->mIsPktsValid = 1; }
static void NComGpsSetOldPkts                        (NComRxCGps *Com, uint32_t    v) { Com->mOldPkts = v; Com->mIsOldPktsValid = 1; }


//============================================================================================================
//! \brief Invalidation.

void NComGpsInvalidate(NComRxCGps *Com)
{
	//--------------------------------------------------------------------------------------------------------
	// GPS Information

	// System information

	Com->mIsTypeValid = 0;                           Com->mType = 0;
	Com->mIsFormatValid = 0;                         Com->mFormat = 0;

	Com->mIsRawRateValid = 0;                        Com->mRawRate = 0;
	Com->mIsPosRateValid = 0;                        Com->mPosRate = 0;
	Com->mIsVelRateValid = 0;                        Com->mVelRate = 0;

	Com->mIsAntStatusValid = 0;                      Com->mAntStatus = 0;
	Com->mIsAntPowerValid = 0;                       Com->mAntPower = 0;
	Com->mIsPosModeValid = 0;                        Com->mPosMode = 0;

	Com->mIsSerBaudValid = 0;                        Com->mSerBaud = 0;

	// Status

	Com->mIsNumSatsValid = 0;                        Com->mNumSats = 0;

	Com->mIsCpuUsedValid = 0;                        Com->mCpuUsed = 0.0;
	Com->mIsCoreNoiseValid = 0;                      Com->mCoreNoise = 0.0;
	Com->mIsCoreTempValid = 0;                       Com->mCoreTemp = 0.0;
	Com->mIsSupplyVoltValid = 0;                     Com->mSupplyVolt = 0.0;

	// Received data statistics

	Com->mIsCharsValid = 0;                          Com->mChars = 0;
	Com->mIsCharsSkippedValid = 0;                   Com->mCharsSkipped = 0;
	Com->mIsPktsValid = 0;                           Com->mPkts = 0;
	Com->mIsOldPktsValid = 0;                        Com->mOldPkts = 0;

	// *** XML End - NComRxCGps Invalidate ***
}


//============================================================================================================
//! \brief Basic report to a file pointer.
//!
//! The user may wish to export this function. By not exporting we avoid exposing the standard library in
//! which FILE is defined to the user.

static void NComGpsReportFP(const NComRxCGps *Com, FILE *fp, const char *pre)
{
	//--------------------------------------------------------------------------------------------------------
	// GPS Information

	// System information

	fprintf(fp, PNV_IE, pre, "Type",                      Com->mIsTypeValid,                      Com->mType,                     NComGpsGetTypeString(Com));
	fprintf(fp, PNV_IE, pre, "Format",                    Com->mIsFormatValid,                    Com->mFormat,                   NComGpsGetFormatString(Com));

	fprintf(fp, PNV_IE, pre, "RawRate",                   Com->mIsRawRateValid,                   Com->mRawRate,                  NComGpsGetRawRateString(Com));
	fprintf(fp, PNV_IE, pre, "PosRate",                   Com->mIsPosRateValid,                   Com->mPosRate,                  NComGpsGetPosRateString(Com));
	fprintf(fp, PNV_IE, pre, "VelRate",                   Com->mIsVelRateValid,                   Com->mVelRate,                  NComGpsGetVelRateString(Com));

	fprintf(fp, PNV_IE, pre, "AntStatus",                 Com->mIsAntStatusValid,                 Com->mAntStatus,                NComGpsGetAntStatusString(Com));
	fprintf(fp, PNV_IE, pre, "AntPower",                  Com->mIsAntPowerValid,                  Com->mAntPower,                 NComGpsGetAntPowerString(Com));
	fprintf(fp, PNV_IE, pre, "PosMode",                   Com->mIsPosModeValid,                   Com->mPosMode,                  NComGpsGetPosModeString(Com));

	fprintf(fp, PNV_IE, pre, "SerBaud",                   Com->mIsSerBaudValid,                   Com->mSerBaud,                  NComGpsGetSerBaudString(Com));

	// Status

	fprintf(fp, PNV_I_, pre, "NumSats",                   Com->mIsNumSatsValid,                   Com->mNumSats);

	fprintf(fp, PNV_D_, pre, "CpuUsed",                   Com->mIsCpuUsedValid,                   Com->mCpuUsed);
	fprintf(fp, PNV_D_, pre, "CoreNoise",                 Com->mIsCoreNoiseValid,                 Com->mCoreNoise);
	fprintf(fp, PNV_D_, pre, "CoreTemp",                  Com->mIsCoreTempValid,                  Com->mCoreTemp);
	fprintf(fp, PNV_D_, pre, "SupplyVolt",                Com->mIsSupplyVoltValid,                Com->mSupplyVolt);

	// Received data statistics

	fprintf(fp, PNV_q_, pre, "Chars",                     Com->mIsCharsValid,                     Com->mChars);
	fprintf(fp, PNV_q_, pre, "CharsSkipped",              Com->mIsCharsSkippedValid,              Com->mCharsSkipped);
	fprintf(fp, PNV_q_, pre, "Pkts",                      Com->mIsPktsValid,                      Com->mPkts);
	fprintf(fp, PNV_q_, pre, "OldPkts",                   Com->mIsOldPktsValid,                   Com->mOldPkts);
}


//============================================================================================================
//! \brief Basic report to file.

void NComGpsReport(const NComRxCGps *Com, const char *file_name, int append)
{
	FILE *fp = fopen(file_name, append ? "a" : "w");

	if (fp == NULL) return;

	NComGpsReportFP(Com, fp, "");

	fclose(fp);
}


//============================================================================================================
//! \brief Constructor for GPS information structure.

NComRxCGps *NComGpsCreate()
{
	return (NComRxCGps *)calloc(1, sizeof(NComRxCGps));
}


//============================================================================================================
//! \brief Destructor for GPS information structure.

void NComGpsDestroy(NComRxCGps *Com)
{
	if (Com != NULL)
	{
		free(Com);
	}
}


//============================================================================================================
//! \brief Copy for GPS information structure.

void NComGpsCopy(NComRxCGps *ComDestination, const NComRxCGps *ComSource)
{
	memcpy(ComDestination, ComSource, sizeof(NComRxCGps));
}




//############################################################################################################
//##                                                                                                        ##
//##  NComRxC                                                                                               ##
//##                                                                                                        ##
//############################################################################################################


//============================================================================================================
// Access functions.

//------------------------------------------------------------------------------------------------------------
// General information

// Status

const char *NComGetOutputPacketTypeString         (const NComRxC *Com) { return OutputPacketName[Com->mOutputPacketType]; }
static void NComSetOutputPacketTypeEnum           (NComRxC *Com, uint8_t     v) { Com->mOutputPacketType = COM_CK_VALIDITY(v, OutputPacketName); Com->mIsOutputPacketTypeValid = 1; }
const char *NComGetInsNavModeString               (const NComRxC *Com) { return NavigationStatusName[Com->mInsNavMode]; }
static void NComSetInsNavModeEnum                 (NComRxC *Com, uint8_t     v) { Com->mInsNavMode = COM_CK_VALIDITY(v, NavigationStatusName); Com->mIsInsNavModeValid = 1; }

// System information

static void NComSetSerialNumber                   (NComRxC *Com, int         v) { Com->mSerialNumber = v; Com->mIsSerialNumberValid = 1; }
static void NComSetDevId                          (NComRxC *Com, const char *v, int n) { strgrab(Com->mDevId, DEV_ID_STRLEN, v, n); Com->mIsDevIdValid = 1; }

static void NComSetOsVersion1                     (NComRxC *Com, int         v) { Com->mOsVersion1 = v; Com->mIsOsVersion1Valid = 1; }
static void NComSetOsVersion2                     (NComRxC *Com, int         v) { Com->mOsVersion2 = v; Com->mIsOsVersion2Valid = 1; }
static void NComSetOsVersion3                     (NComRxC *Com, int         v) { Com->mOsVersion3 = v; Com->mIsOsVersion3Valid = 1; }
static void NComSetOsScriptId                     (NComRxC *Com, const char *v, int n) { strgrab(Com->mOsScriptId, OS_SCRIPT_ID_STRLEN, v, n); Com->mIsOsScriptIdValid = 1; }

const char *NComGetImuTypeString                  (const NComRxC *Com) { return ImuTypeName[Com->mImuType]; }
static void NComSetImuTypeEnum                    (NComRxC *Com, uint8_t     v) { Com->mImuType = COM_CK_VALIDITY(v, ImuTypeName); Com->mIsImuTypeValid = 1; }
const char *NComGetCpuPcbTypeString               (const NComRxC *Com) { return CpuPcbTypeName[Com->mCpuPcbType]; }
static void NComSetCpuPcbTypeEnum                 (NComRxC *Com, uint8_t     v) { Com->mCpuPcbType = COM_CK_VALIDITY(v, CpuPcbTypeName); Com->mIsCpuPcbTypeValid = 1; }
const char *NComGetInterPcbTypeString             (const NComRxC *Com) { return InterPcbTypeName[Com->mInterPcbType]; }
static void NComSetInterPcbTypeEnum               (NComRxC *Com, uint8_t     v) { Com->mInterPcbType = COM_CK_VALIDITY(v, InterPcbTypeName); Com->mIsInterPcbTypeValid = 1; }
const char *NComGetFrontPcbTypeString             (const NComRxC *Com) { return FrontPcbTypeName[Com->mFrontPcbType]; }
static void NComSetFrontPcbTypeEnum               (NComRxC *Com, uint8_t     v) { Com->mFrontPcbType = COM_CK_VALIDITY(v, FrontPcbTypeName); Com->mIsFrontPcbTypeValid = 1; }
const char *NComGetInterSwIdString                (const NComRxC *Com) { return InterSwIdName[Com->mInterSwId]; }
static void NComSetInterSwIdEnum                  (NComRxC *Com, uint8_t     v) { Com->mInterSwId = COM_CK_VALIDITY(v, InterSwIdName); Com->mIsInterSwIdValid = 1; }
const char *NComGetHwConfigString                 (const NComRxC *Com) { return DeployCfgName[Com->mHwConfig]; }
static void NComSetHwConfigEnum                   (NComRxC *Com, uint8_t     v) { Com->mHwConfig = COM_CK_VALIDITY(v, DeployCfgName); Com->mIsHwConfigValid = 1; }

static void NComSetDiskSpace                      (NComRxC *Com, uint64_t    v) { Com->mDiskSpace = v; Com->mIsDiskSpaceValid = 1; }
static void NComSetFileSize                       (NComRxC *Com, uint64_t    v) { Com->mFileSize = v; Com->mIsFileSizeValid = 1; }
static void NComSetUpTime                         (NComRxC *Com, uint32_t    v) { Com->mUpTime = v; Com->mIsUpTimeValid = 1; }
const char *NComGetDualPortRamStatusString        (const NComRxC *Com) { return DualPortRamStatusName[Com->mDualPortRamStatus]; }
static void NComSetDualPortRamStatusEnum          (NComRxC *Com, uint8_t     v) { Com->mDualPortRamStatus = COM_CK_VALIDITY(v, DualPortRamStatusName); Com->mIsDualPortRamStatusValid = 1; }

// IMU information

const char *NComGetUmacStatusString               (const NComRxC *Com) { return ComUmacStatus[Com->mUmacStatus]; }
static void NComSetUmacStatusEnum                 (NComRxC *Com, uint8_t     v) { Com->mUmacStatus = COM_CK_VALIDITY(v, ComUmacStatus); Com->mIsUmacStatusValid = 1; }

// Global Navigation Satellite System (GNSS) information

static void NComSetGnssGpsEnabled                 (NComRxC *Com, int        v) { Com->mGnssGpsEnabled = v; Com->mIsGnssGpsEnabledValid = 1; }
static void NComSetGnssGlonassEnabled             (NComRxC *Com, int        v) { Com->mGnssGlonassEnabled = v; Com->mIsGnssGlonassEnabledValid = 1; }
static void NComSetGnssGalileoEnabled             (NComRxC *Com, int        v) { Com->mGnssGalileoEnabled = v; Com->mIsGnssGalileoEnabledValid = 1; }

static void NComSetPsrDiffEnabled                 (NComRxC *Com, int        v) { Com->mPsrDiffEnabled = v; Com->mIsPsrDiffEnabledValid = 1; }
static void NComSetSBASEnabled                    (NComRxC *Com, int        v) { Com->mSBASEnabled = v; Com->mIsSBASEnabledValid = 1; }
static void NComSetOmniVBSEnabled                 (NComRxC *Com, int        v) { Com->mOmniVBSEnabled = v; Com->mIsOmniVBSEnabledValid = 1; }
static void NComSetOmniHPEnabled                  (NComRxC *Com, int        v) { Com->mOmniHPEnabled = v; Com->mIsOmniHPEnabledValid = 1; }
static void NComSetL1DiffEnabled                  (NComRxC *Com, int        v) { Com->mL1DiffEnabled = v; Com->mIsL1DiffEnabledValid = 1; }
static void NComSetL1L2DiffEnabled                (NComRxC *Com, int        v) { Com->mL1L2DiffEnabled = v; Com->mIsL1L2DiffEnabledValid = 1; }

static void NComSetRawRngEnabled                  (NComRxC *Com, int        v) { Com->mRawRngEnabled = v; Com->mIsRawRngEnabledValid = 1; }
static void NComSetRawDopEnabled                  (NComRxC *Com, int        v) { Com->mRawDopEnabled = v; Com->mIsRawDopEnabledValid = 1; }
static void NComSetRawL1Enabled                   (NComRxC *Com, int        v) { Com->mRawL1Enabled = v; Com->mIsRawL1EnabledValid = 1; }
static void NComSetRawL2Enabled                   (NComRxC *Com, int        v) { Com->mRawL2Enabled = v; Com->mIsRawL2EnabledValid = 1; }
static void NComSetRawL5Enabled                   (NComRxC *Com, int        v) { Com->mRawL5Enabled = v; Com->mIsRawL5EnabledValid = 1; }

const char *NComGetGpsPosModeString               (const NComRxC *Com) { return ComGpsXModeName[Com->mGpsPosMode]; }
static void NComSetGpsPosModeEnum                 (NComRxC *Com, uint8_t     v) { Com->mGpsPosMode = COM_CK_VALIDITY(v, ComGpsXModeName); Com->mIsGpsPosModeValid = 1; }
const char *NComGetGpsVelModeString               (const NComRxC *Com) { return ComGpsXModeName[Com->mGpsVelMode]; }
static void NComSetGpsVelModeEnum                 (NComRxC *Com, uint8_t     v) { Com->mGpsVelMode = COM_CK_VALIDITY(v, ComGpsXModeName); Com->mIsGpsVelModeValid = 1; }
const char *NComGetGpsAttModeString               (const NComRxC *Com) { return ComGpsXModeName[Com->mGpsAttMode]; }
static void NComSetGpsAttModeEnum                 (NComRxC *Com, uint8_t     v) { Com->mGpsAttMode = COM_CK_VALIDITY(v, ComGpsXModeName); Com->mIsGpsAttModeValid = 1; }

static void NComSetPDOP                           (NComRxC *Com, double      v) { Com->mPDOP = v; Com->mIsPDOPValid = 1; }
static void NComSetHDOP                           (NComRxC *Com, double      v) { Com->mHDOP = v; Com->mIsHDOPValid = 1; }
static void NComSetVDOP                           (NComRxC *Com, double      v) { Com->mVDOP = v; Com->mIsVDOPValid = 1; }

static void NComSetGpsNumObs                      (NComRxC *Com, int         v) { Com->mGpsNumObs = v; Com->mIsGpsNumObsValid = 1; }
static void NComSetUndulation                     (NComRxC *Com, double      v) { Com->mUndulation = v; Com->mIsUndulationValid = 1; }
static void NComSetGpsDiffAge                     (NComRxC *Com, double      v) { Com->mGpsDiffAge = v; Com->mIsGpsDiffAgeValid = 1; }
static void NComSetBaseStationId                  (NComRxC *Com, const char *v, int n) { strgrab(Com->mBaseStationId, BASE_STATION_ID_STRLEN, v, n); Com->mIsBaseStationIdValid = 1; }

// Dual antenna computation status

const char *NComGetHeadQualityString              (const NComRxC *Com) { return ComHeadQuality[Com->mHeadQuality]; }
static void NComSetHeadQualityEnum                (NComRxC *Com, uint8_t     v) { Com->mHeadQuality = COM_CK_VALIDITY(v, ComHeadQuality); Com->mIsHeadQualityValid = 1; }
const char *NComGetHeadSearchTypeString           (const NComRxC *Com) { return ComHeadSearchType[Com->mHeadSearchType]; }
static void NComSetHeadSearchTypeEnum             (NComRxC *Com, uint8_t     v) { Com->mHeadSearchType = COM_CK_VALIDITY(v, ComHeadSearchType); Com->mIsHeadSearchTypeValid = 1; }
const char *NComGetHeadSearchStatusString         (const NComRxC *Com) { return ComHeadSearchStatus[Com->mHeadSearchStatus]; }
static void NComSetHeadSearchStatusEnum           (NComRxC *Com, uint8_t     v) { Com->mHeadSearchStatus = COM_CK_VALIDITY(v, ComHeadSearchStatus); Com->mIsHeadSearchStatusValid = 1; }
const char *NComGetHeadSearchReadyString          (const NComRxC *Com) { return ComHeadSearchReady[Com->mHeadSearchReady]; }
static void NComSetHeadSearchReadyEnum            (NComRxC *Com, uint8_t     v) { Com->mHeadSearchReady = COM_CK_VALIDITY(v, ComHeadSearchReady); Com->mIsHeadSearchReadyValid = 1; }

static void NComSetHeadSearchInit                 (NComRxC *Com, int         v) { Com->mHeadSearchInit = v; Com->mIsHeadSearchInitValid = 1; }
static void NComSetHeadSearchNum                  (NComRxC *Com, int         v) { Com->mHeadSearchNum = v; Com->mIsHeadSearchNumValid = 1; }
static void NComSetHeadSearchTime                 (NComRxC *Com, int         v) { Com->mHeadSearchTime = v; Com->mIsHeadSearchTimeValid = 1; }
static void NComSetHeadSearchConstr               (NComRxC *Com, int         v) { Com->mHeadSearchConstr = v; Com->mIsHeadSearchConstrValid = 1; }

static void NComSetHeadSearchMaster               (NComRxC *Com, int         v) { Com->mHeadSearchMaster = v; Com->mIsHeadSearchMasterValid = 1; }
static void NComSetHeadSearchSlave1               (NComRxC *Com, int         v) { Com->mHeadSearchSlave1 = v; Com->mIsHeadSearchSlave1Valid = 1; }
static void NComSetHeadSearchSlave2               (NComRxC *Com, int         v) { Com->mHeadSearchSlave2 = v; Com->mIsHeadSearchSlave2Valid = 1; }
static void NComSetHeadSearchSlave3               (NComRxC *Com, int         v) { Com->mHeadSearchSlave3 = v; Com->mIsHeadSearchSlave3Valid = 1; }

// OmniSTAR information

static void NComSetOmniStarSerial                 (NComRxC *Com, const char *v, int n) { strgrab(Com->mOmniStarSerial, OMNISTAR_SERIAL_STRLEN, v, n); Com->mIsOmniStarSerialValid = 1; }
static void NComSetOmniStarFreq                   (NComRxC *Com, double      v) { Com->mOmniStarFreq = v; Com->mIsOmniStarFreqValid = 1; }
static void NComSetOmniStarSNR                    (NComRxC *Com, double      v) { Com->mOmniStarSNR = v; Com->mIsOmniStarSNRValid = 1; }
static void NComSetOmniStarLockTime               (NComRxC *Com, double      v) { Com->mOmniStarLockTime = v; Com->mIsOmniStarLockTimeValid = 1; }

static void NComSetOmniStatusVbsExpired           (NComRxC *Com, int        v) { Com->mOmniStatusVbsExpired = v; Com->mIsOmniStatusVbsExpiredValid = 1; }
static void NComSetOmniStatusVbsOutOfRegion       (NComRxC *Com, int        v) { Com->mOmniStatusVbsOutOfRegion = v; Com->mIsOmniStatusVbsOutOfRegionValid = 1; }
static void NComSetOmniStatusVbsNoRemoteSites     (NComRxC *Com, int        v) { Com->mOmniStatusVbsNoRemoteSites = v; Com->mIsOmniStatusVbsNoRemoteSitesValid = 1; }

static void NComSetOmniStatusHpExpired            (NComRxC *Com, int        v) { Com->mOmniStatusHpExpired = v; Com->mIsOmniStatusHpExpiredValid = 1; }
static void NComSetOmniStatusHpOutOfRegion        (NComRxC *Com, int        v) { Com->mOmniStatusHpOutOfRegion = v; Com->mIsOmniStatusHpOutOfRegionValid = 1; }
static void NComSetOmniStatusHpNoRemoteSites      (NComRxC *Com, int        v) { Com->mOmniStatusHpNoRemoteSites = v; Com->mIsOmniStatusHpNoRemoteSitesValid = 1; }
static void NComSetOmniStatusHpNotConverged       (NComRxC *Com, int        v) { Com->mOmniStatusHpNotConverged = v; Com->mIsOmniStatusHpNotConvergedValid = 1; }
static void NComSetOmniStatusHpKeyInvalid         (NComRxC *Com, int        v) { Com->mOmniStatusHpKeyInvalid = v; Com->mIsOmniStatusHpKeyInvalidValid = 1; }

//------------------------------------------------------------------------------------------------------------
// General user options

// General options

const char *NComGetOptionLevelString              (const NComRxC *Com) { return ComOptionVehicle[Com->mOptionLevel]; }
static void NComSetOptionLevelEnum                (NComRxC *Com, uint8_t     v) { Com->mOptionLevel = COM_CK_VALIDITY(v, ComOptionVehicle); Com->mIsOptionLevelValid = 1; }
const char *NComGetOptionVibrationString          (const NComRxC *Com) { return ComOptionVibration[Com->mOptionVibration]; }
static void NComSetOptionVibrationEnum            (NComRxC *Com, uint8_t     v) { Com->mOptionVibration = COM_CK_VALIDITY(v, ComOptionVibration); Com->mIsOptionVibrationValid = 1; }
const char *NComGetOptionGpsAccString             (const NComRxC *Com) { return ComOptionGpsAccuracy[Com->mOptionGpsAcc]; }
static void NComSetOptionGpsAccEnum               (NComRxC *Com, uint8_t     v) { Com->mOptionGpsAcc = COM_CK_VALIDITY(v, ComOptionGpsAccuracy); Com->mIsOptionGpsAccValid = 1; }
const char *NComGetOptionUdpString                (const NComRxC *Com) { return ComOptionOutput[Com->mOptionUdp]; }
static void NComSetOptionUdpEnum                  (NComRxC *Com, uint8_t     v) { Com->mOptionUdp = COM_CK_VALIDITY(v, ComOptionOutput); Com->mIsOptionUdpValid = 1; }
const char *NComGetOptionSer1String               (const NComRxC *Com) { return ComOptionOutput[Com->mOptionSer1]; }
static void NComSetOptionSer1Enum                 (NComRxC *Com, uint8_t     v) { Com->mOptionSer1 = COM_CK_VALIDITY(v, ComOptionOutput); Com->mIsOptionSer1Valid = 1; }
const char *NComGetOptionSer2String               (const NComRxC *Com) { return ComOptionOutput[Com->mOptionSer2]; }
static void NComSetOptionSer2Enum                 (NComRxC *Com, uint8_t     v) { Com->mOptionSer2 = COM_CK_VALIDITY(v, ComOptionOutput); Com->mIsOptionSer2Valid = 1; }
const char *NComGetOptionSer3String               (const NComRxC *Com) { return ComOptionOutput[Com->mOptionSer3]; }
static void NComSetOptionSer3Enum                 (NComRxC *Com, uint8_t     v) { Com->mOptionSer3 = COM_CK_VALIDITY(v, ComOptionOutput); Com->mIsOptionSer3Valid = 1; }
const char *NComGetOptionHeadingString            (const NComRxC *Com) { return ComOptionHeading[Com->mOptionHeading]; }
static void NComSetOptionHeadingEnum              (NComRxC *Com, uint8_t     v) { Com->mOptionHeading = COM_CK_VALIDITY(v, ComOptionHeading); Com->mIsOptionHeadingValid = 1; }

static void NComSetOptionInitSpeed                (NComRxC *Com, double      v) { Com->mOptionInitSpeed = v; Com->mIsOptionInitSpeedValid = 1; Com->mIsOptionInitSpeedConfig = 1; }
static void NComSetOptionTopSpeed                 (NComRxC *Com, double      v) { Com->mOptionTopSpeed = v; Com->mIsOptionTopSpeedValid = 1; Com->mIsOptionTopSpeedConfig = 1; }

// Output baud rate settings

const char *NComGetOptionSer1BaudString           (const NComRxC *Com) { return ComOptionSerBaud[Com->mOptionSer1Baud]; }
static void NComSetOptionSer1BaudEnum             (NComRxC *Com, uint8_t     v) { Com->mOptionSer1Baud = COM_CK_VALIDITY(v, ComOptionSerBaud); Com->mIsOptionSer1BaudValid = 1; }
const char *NComGetOptionSer2BaudString           (const NComRxC *Com) { return ComOptionSerBaud[Com->mOptionSer2Baud]; }
static void NComSetOptionSer2BaudEnum             (NComRxC *Com, uint8_t     v) { Com->mOptionSer2Baud = COM_CK_VALIDITY(v, ComOptionSerBaud); Com->mIsOptionSer2BaudValid = 1; }
const char *NComGetOptionSer3BaudString           (const NComRxC *Com) { return ComOptionSerBaud[Com->mOptionSer3Baud]; }
static void NComSetOptionSer3BaudEnum             (NComRxC *Com, uint8_t     v) { Com->mOptionSer3Baud = COM_CK_VALIDITY(v, ComOptionSerBaud); Com->mIsOptionSer3BaudValid = 1; }

const char *NComGetOptionCanBaudString            (const NComRxC *Com) { return ComOptionCanBaud[Com->mOptionCanBaud]; }
static void NComSetOptionCanBaudEnum              (NComRxC *Com, uint8_t     v) { Com->mOptionCanBaud = COM_CK_VALIDITY(v, ComOptionCanBaud); Com->mIsOptionCanBaudValid = 1; }

//------------------------------------------------------------------------------------------------------------
// General measurements

// Timing

static void NComSetTime                           (NComRxC *Com, double      v) { Com->mTime = v; Com->mIsTimeValid = 1; }

static void NComSetTimeWeekCount                  (NComRxC *Com, uint32_t    v) { Com->mTimeWeekCount = v; Com->mIsTimeWeekCountValid = 1; }
static void NComSetTimeWeekSecond                 (NComRxC *Com, double      v) { Com->mTimeWeekSecond = v; Com->mIsTimeWeekSecondValid = 1; }
static void NComSetTimeUtcOffset                  (NComRxC *Com, int         v) { Com->mTimeUtcOffset = v; Com->mIsTimeUtcOffsetValid = 1; }

// Position

static void NComSetLat                            (NComRxC *Com, double      v) { Com->mLat = v; Com->mIsLatValid = 1; Com->mIsLatApprox = 1; }
static void NComSetLon                            (NComRxC *Com, double      v) { Com->mLon = v; Com->mIsLonValid = 1; Com->mIsLonApprox = 1; }
static void NComSetAlt                            (NComRxC *Com, double      v) { Com->mAlt = v; Com->mIsAltValid = 1; Com->mIsAltApprox = 1; }

static void NComSetNorthAcc                       (NComRxC *Com, double      v) { Com->mNorthAcc = v; Com->mIsNorthAccValid = 1; }
static void NComSetEastAcc                        (NComRxC *Com, double      v) { Com->mEastAcc = v; Com->mIsEastAccValid = 1; }
static void NComSetAltAcc                         (NComRxC *Com, double      v) { Com->mAltAcc = v; Com->mIsAltAccValid = 1; }

// Distance

static void NComSetDist2d                         (NComRxC *Com, double      v) { Com->mDist2d = v; Com->mIsDist2dValid = 1; }
static void NComSetDist3d                         (NComRxC *Com, double      v) { Com->mDist3d = v; Com->mIsDist3dValid = 1; }

// Velocity

static void NComSetVn                             (NComRxC *Com, double      v) { Com->mVn = v; Com->mIsVnValid = 1; Com->mIsVnApprox = 1; }
static void NComSetVe                             (NComRxC *Com, double      v) { Com->mVe = v; Com->mIsVeValid = 1; Com->mIsVeApprox = 1; }
static void NComSetVd                             (NComRxC *Com, double      v) { Com->mVd = v; Com->mIsVdValid = 1; Com->mIsVdApprox = 1; }

static void NComSetVf                             (NComRxC *Com, double      v) { Com->mVf = v; Com->mIsVfValid = 1; }
static void NComSetVl                             (NComRxC *Com, double      v) { Com->mVl = v; Com->mIsVlValid = 1; }

static void NComSetVx							  (NComRxC *Com, double      v) { Com->mVx = v; Com->mIsVxValid = 1; }
static void NComSetVy						      (NComRxC *Com, double      v) { Com->mVy = v; Com->mIsVyValid = 1; }
static void NComSetVz							  (NComRxC *Com, double      v) { Com->mVz = v; Com->mIsVzValid = 1; }               //new

static void NComSetVnAcc                          (NComRxC *Com, double      v) { Com->mVnAcc = v; Com->mIsVnAccValid = 1; }
static void NComSetVeAcc                          (NComRxC *Com, double      v) { Com->mVeAcc = v; Com->mIsVeAccValid = 1; }
static void NComSetVdAcc                          (NComRxC *Com, double      v) { Com->mVdAcc = v; Com->mIsVdAccValid = 1; }

// Speed

static void NComSetSpeed2d                        (NComRxC *Com, double      v) { Com->mSpeed2d = v; Com->mIsSpeed2dValid = 1; }
static void NComSetSpeed3d                        (NComRxC *Com, double      v) { Com->mSpeed3d = v; Com->mIsSpeed3dValid = 1; }

// Acceleration

static void NComSetAx                             (NComRxC *Com, double      v) { Com->mAx = v; Com->mIsAxValid = 1; }
static void NComSetAy                             (NComRxC *Com, double      v) { Com->mAy = v; Com->mIsAyValid = 1; }
static void NComSetAz                             (NComRxC *Com, double      v) { Com->mAz = v; Com->mIsAzValid = 1; }

static void NComSetAf                             (NComRxC *Com, double      v) { Com->mAf = v; Com->mIsAfValid = 1; }
static void NComSetAl                             (NComRxC *Com, double      v) { Com->mAl = v; Com->mIsAlValid = 1; }
static void NComSetAd                             (NComRxC *Com, double      v) { Com->mAd = v; Com->mIsAdValid = 1; }

// Filtered acceleration

static void NComSetFiltAx                         (NComRxC *Com, double      v) { Com->mFiltAx = v; Com->mIsFiltAxValid = 1; }
static void NComSetFiltAy                         (NComRxC *Com, double      v) { Com->mFiltAy = v; Com->mIsFiltAyValid = 1; }
static void NComSetFiltAz                         (NComRxC *Com, double      v) { Com->mFiltAz = v; Com->mIsFiltAzValid = 1; }

static void NComSetFiltAf                         (NComRxC *Com, double      v) { Com->mFiltAf = v; Com->mIsFiltAfValid = 1; }
static void NComSetFiltAl                         (NComRxC *Com, double      v) { Com->mFiltAl = v; Com->mIsFiltAlValid = 1; }
static void NComSetFiltAd                         (NComRxC *Com, double      v) { Com->mFiltAd = v; Com->mIsFiltAdValid = 1; }

// Orientation

static void NComSetHeading                        (NComRxC *Com, double      v) { Com->mHeading = v; Com->mIsHeadingValid = 1; Com->mIsHeadingApprox = 1; }
static void NComSetPitch                          (NComRxC *Com, double      v) { Com->mPitch = v; Com->mIsPitchValid = 1; Com->mIsPitchApprox = 1; }
static void NComSetRoll                           (NComRxC *Com, double      v) { Com->mRoll = v; Com->mIsRollValid = 1; Com->mIsRollApprox = 1; }

static void NComSetHeadingAcc                     (NComRxC *Com, double      v) { Com->mHeadingAcc = v; Com->mIsHeadingAccValid = 1; }
static void NComSetPitchAcc                       (NComRxC *Com, double      v) { Com->mPitchAcc = v; Com->mIsPitchAccValid = 1; }
static void NComSetRollAcc                        (NComRxC *Com, double      v) { Com->mRollAcc = v; Com->mIsRollAccValid = 1; }

// Special

static void NComSetTrack                          (NComRxC *Com, double      v) { Com->mTrack = v; Com->mIsTrackValid = 1; }
static void NComSetSlip                           (NComRxC *Com, double      v) { Com->mSlip = v; Com->mIsSlipValid = 1; }
static void NComSetCurvature                      (NComRxC *Com, double      v) { Com->mCurvature = v; Com->mIsCurvatureValid = 1; }

// Angular rate

static void NComSetWx                             (NComRxC *Com, double      v) { Com->mWx = v; Com->mIsWxValid = 1; }
static void NComSetWy                             (NComRxC *Com, double      v) { Com->mWy = v; Com->mIsWyValid = 1; }
static void NComSetWz                             (NComRxC *Com, double      v) { Com->mWz = v; Com->mIsWzValid = 1; }

static void NComSetWf                             (NComRxC *Com, double      v) { Com->mWf = v; Com->mIsWfValid = 1; }
static void NComSetWl                             (NComRxC *Com, double      v) { Com->mWl = v; Com->mIsWlValid = 1; }
static void NComSetWd                             (NComRxC *Com, double      v) { Com->mWd = v; Com->mIsWdValid = 1; }

// Angular acceleration

static void NComSetYx                             (NComRxC *Com, double      v) { Com->mYx = v; Com->mIsYxValid = 1; }
static void NComSetYy                             (NComRxC *Com, double      v) { Com->mYy = v; Com->mIsYyValid = 1; }
static void NComSetYz                             (NComRxC *Com, double      v) { Com->mYz = v; Com->mIsYzValid = 1; }

static void NComSetYf                             (NComRxC *Com, double      v) { Com->mYf = v; Com->mIsYfValid = 1; }
static void NComSetYl                             (NComRxC *Com, double      v) { Com->mYl = v; Com->mIsYlValid = 1; }
static void NComSetYd                             (NComRxC *Com, double      v) { Com->mYd = v; Com->mIsYdValid = 1; }

// Filtered angular acceleration

static void NComSetFiltYx                         (NComRxC *Com, double      v) { Com->mFiltYx = v; Com->mIsFiltYxValid = 1; }
static void NComSetFiltYy                         (NComRxC *Com, double      v) { Com->mFiltYy = v; Com->mIsFiltYyValid = 1; }
static void NComSetFiltYz                         (NComRxC *Com, double      v) { Com->mFiltYz = v; Com->mIsFiltYzValid = 1; }

static void NComSetFiltYf                         (NComRxC *Com, double      v) { Com->mFiltYf = v; Com->mIsFiltYfValid = 1; }
static void NComSetFiltYl                         (NComRxC *Com, double      v) { Com->mFiltYl = v; Com->mIsFiltYlValid = 1; }
static void NComSetFiltYd                         (NComRxC *Com, double      v) { Com->mFiltYd = v; Com->mIsFiltYdValid = 1; }

//------------------------------------------------------------------------------------------------------------
// Model particulars

// Innovations (discrepancy between GPS and IMU)

static void NComSetInnPosX                        (NComRxC *Com, double      v) { Com->mInnPosX = v; Com->mInnPosXAge = 0; }
static void NComSetInnPosY                        (NComRxC *Com, double      v) { Com->mInnPosY = v; Com->mInnPosYAge = 0; }
static void NComSetInnPosZ                        (NComRxC *Com, double      v) { Com->mInnPosZ = v; Com->mInnPosZAge = 0; }

static void NComSetInnVelX                        (NComRxC *Com, double      v) { Com->mInnVelX = v; Com->mInnVelXAge = 0; }
static void NComSetInnVelY                        (NComRxC *Com, double      v) { Com->mInnVelY = v; Com->mInnVelYAge = 0; }
static void NComSetInnVelZ                        (NComRxC *Com, double      v) { Com->mInnVelZ = v; Com->mInnVelZAge = 0; }

static void NComSetInnHeading                     (NComRxC *Com, double      v) { Com->mInnHeading = v; Com->mInnHeadingAge = 0; }
static void NComSetInnPitch                       (NComRxC *Com, double      v) { Com->mInnPitch = v; Com->mInnPitchAge = 0; }

// Gyroscope bias and scale factor

static void NComSetWxBias                         (NComRxC *Com, double      v) { Com->mWxBias = v; Com->mIsWxBiasValid = 1; }
static void NComSetWyBias                         (NComRxC *Com, double      v) { Com->mWyBias = v; Com->mIsWyBiasValid = 1; }
static void NComSetWzBias                         (NComRxC *Com, double      v) { Com->mWzBias = v; Com->mIsWzBiasValid = 1; }

static void NComSetWxBiasAcc                      (NComRxC *Com, double      v) { Com->mWxBiasAcc = v; Com->mIsWxBiasAccValid = 1; }
static void NComSetWyBiasAcc                      (NComRxC *Com, double      v) { Com->mWyBiasAcc = v; Com->mIsWyBiasAccValid = 1; }
static void NComSetWzBiasAcc                      (NComRxC *Com, double      v) { Com->mWzBiasAcc = v; Com->mIsWzBiasAccValid = 1; }

static void NComSetWxSf                           (NComRxC *Com, double      v) { Com->mWxSf = v; Com->mIsWxSfValid = 1; }
static void NComSetWySf                           (NComRxC *Com, double      v) { Com->mWySf = v; Com->mIsWySfValid = 1; }
static void NComSetWzSf                           (NComRxC *Com, double      v) { Com->mWzSf = v; Com->mIsWzSfValid = 1; }

static void NComSetWxSfAcc                        (NComRxC *Com, double      v) { Com->mWxSfAcc = v; Com->mIsWxSfAccValid = 1; }
static void NComSetWySfAcc                        (NComRxC *Com, double      v) { Com->mWySfAcc = v; Com->mIsWySfAccValid = 1; }
static void NComSetWzSfAcc                        (NComRxC *Com, double      v) { Com->mWzSfAcc = v; Com->mIsWzSfAccValid = 1; }

// Accelerometer bias and scale factor

static void NComSetAxBias                         (NComRxC *Com, double      v) { Com->mAxBias = v; Com->mIsAxBiasValid = 1; }
static void NComSetAyBias                         (NComRxC *Com, double      v) { Com->mAyBias = v; Com->mIsAyBiasValid = 1; }
static void NComSetAzBias                         (NComRxC *Com, double      v) { Com->mAzBias = v; Com->mIsAzBiasValid = 1; }

static void NComSetAxBiasAcc                      (NComRxC *Com, double      v) { Com->mAxBiasAcc = v; Com->mIsAxBiasAccValid = 1; }
static void NComSetAyBiasAcc                      (NComRxC *Com, double      v) { Com->mAyBiasAcc = v; Com->mIsAyBiasAccValid = 1; }
static void NComSetAzBiasAcc                      (NComRxC *Com, double      v) { Com->mAzBiasAcc = v; Com->mIsAzBiasAccValid = 1; }

static void NComSetAxSf                           (NComRxC *Com, double      v) { Com->mAxSf = v; Com->mIsAxSfValid = 1; }
static void NComSetAySf                           (NComRxC *Com, double      v) { Com->mAySf = v; Com->mIsAySfValid = 1; }
static void NComSetAzSf                           (NComRxC *Com, double      v) { Com->mAzSf = v; Com->mIsAzSfValid = 1; }

static void NComSetAxSfAcc                        (NComRxC *Com, double      v) { Com->mAxSfAcc = v; Com->mIsAxSfAccValid = 1; }
static void NComSetAySfAcc                        (NComRxC *Com, double      v) { Com->mAySfAcc = v; Com->mIsAySfAccValid = 1; }
static void NComSetAzSfAcc                        (NComRxC *Com, double      v) { Com->mAzSfAcc = v; Com->mIsAzSfAccValid = 1; }

// GPS antenna position

static void NComSetGAPx                           (NComRxC *Com, double      v) { Com->mGAPx = v; Com->mIsGAPxValid = 1; }
static void NComSetGAPy                           (NComRxC *Com, double      v) { Com->mGAPy = v; Com->mIsGAPyValid = 1; }
static void NComSetGAPz                           (NComRxC *Com, double      v) { Com->mGAPz = v; Com->mIsGAPzValid = 1; }

static void NComSetGAPxAcc                        (NComRxC *Com, double      v) { Com->mGAPxAcc = v; Com->mIsGAPxAccValid = 1; }
static void NComSetGAPyAcc                        (NComRxC *Com, double      v) { Com->mGAPyAcc = v; Com->mIsGAPyAccValid = 1; }
static void NComSetGAPzAcc                        (NComRxC *Com, double      v) { Com->mGAPzAcc = v; Com->mIsGAPzAccValid = 1; }

static void NComSetAtH                            (NComRxC *Com, double      v) { Com->mAtH = v; Com->mIsAtHValid = 1; }
static void NComSetAtP                            (NComRxC *Com, double      v) { Com->mAtP = v; Com->mIsAtPValid = 1; }

static void NComSetAtHAcc                         (NComRxC *Com, double      v) { Com->mAtHAcc = v; Com->mIsAtHAccValid = 1; }
static void NComSetAtPAcc                         (NComRxC *Com, double      v) { Com->mAtPAcc = v; Com->mIsAtPAccValid = 1; }

static void NComSetBaseLineLength                 (NComRxC *Com, double      v) { Com->mBaseLineLength = v; Com->mIsBaseLineLengthValid = 1; Com->mIsBaseLineLengthConfig = 1; }
static void NComSetBaseLineLengthAcc              (NComRxC *Com, double      v) { Com->mBaseLineLengthAcc = v; Com->mIsBaseLineLengthAccValid = 1; Com->mIsBaseLineLengthAccConfig = 1; }

//------------------------------------------------------------------------------------------------------------
// Statistics

// IMU hardware status event counters

static void NComSetImuMissedPkts                  (NComRxC *Com, uint32_t    v) { Com->mImuMissedPkts = v; Com->mIsImuMissedPktsValid = 1; }
static void NComSetImuResetCount                  (NComRxC *Com, uint32_t    v) { Com->mImuResetCount = v; Com->mIsImuResetCountValid = 1; }
static void NComSetImuErrorCount                  (NComRxC *Com, uint32_t    v) { Com->mImuErrorCount = v; Com->mIsImuErrorCountValid = 1; }

// GPS successive rejected aiding updates

static void NComSetGPSPosReject                   (NComRxC *Com, uint32_t    v) { Com->mGPSPosReject = v; Com->mIsGPSPosRejectValid = 1; }
static void NComSetGPSVelReject                   (NComRxC *Com, uint32_t    v) { Com->mGPSVelReject = v; Com->mIsGPSVelRejectValid = 1; }
static void NComSetGPSAttReject                   (NComRxC *Com, uint32_t    v) { Com->mGPSAttReject = v; Com->mIsGPSAttRejectValid = 1; }

// Received data statistics

static void NComSetImuChars                       (NComRxC *Com, uint32_t    v) { Com->mImuChars = v; Com->mIsImuCharsValid = 1; }
static void NComSetImuCharsSkipped                (NComRxC *Com, uint32_t    v) { Com->mImuCharsSkipped = v; Com->mIsImuCharsSkippedValid = 1; }
static void NComSetImuPkts                        (NComRxC *Com, uint32_t    v) { Com->mImuPkts = v; Com->mIsImuPktsValid = 1; }

static void NComSetCmdChars                       (NComRxC *Com, uint32_t    v) { Com->mCmdChars = v; Com->mIsCmdCharsValid = 1; }
static void NComSetCmdCharsSkipped                (NComRxC *Com, uint32_t    v) { Com->mCmdCharsSkipped = v; Com->mIsCmdCharsSkippedValid = 1; }
static void NComSetCmdPkts                        (NComRxC *Com, uint32_t    v) { Com->mCmdPkts = v; Com->mIsCmdPktsValid = 1; }
static void NComSetCmdErrors                      (NComRxC *Com, uint32_t    v) { Com->mCmdErrors = v; Com->mIsCmdErrorsValid = 1; }

//------------------------------------------------------------------------------------------------------------
// Transformation Euler angles

// Orientation of vehicle-frame relative to IMU-frame

static void NComSetImu2VehHeading                 (NComRxC *Com, double      v) { Com->mImu2VehHeading = v; Com->mIsImu2VehHeadingValid = 1; }
static void NComSetImu2VehPitch                   (NComRxC *Com, double      v) { Com->mImu2VehPitch = v; Com->mIsImu2VehPitchValid = 1; }
static void NComSetImu2VehRoll                    (NComRxC *Com, double      v) { Com->mImu2VehRoll = v; Com->mIsImu2VehRollValid = 1; }

//------------------------------------------------------------------------------------------------------------
// Miscellaneous items

// Triggers

static void NComSetTrigTime                       (NComRxC *Com, double      v) { Com->mTrigTime = v; Com->mIsTrigTimeValid = 1; Com->mIsTrigTimeNew = 1; }
static void NComSetTrig2Time                      (NComRxC *Com, double      v) { Com->mTrig2Time = v; Com->mIsTrig2TimeValid = 1; Com->mIsTrig2TimeNew = 1; }
static void NComSetDigitalOutTime                 (NComRxC *Com, double      v) { Com->mDigitalOutTime = v; Com->mIsDigitalOutTimeValid = 1; Com->mIsDigitalOutTimeNew = 1; }

// Remote lever arm option

static void NComSetRemoteLeverArmX                (NComRxC *Com, double      v) { Com->mRemoteLeverArmX = v; Com->mIsRemoteLeverArmXValid = 1; }
static void NComSetRemoteLeverArmY                (NComRxC *Com, double      v) { Com->mRemoteLeverArmY = v; Com->mIsRemoteLeverArmYValid = 1; }
static void NComSetRemoteLeverArmZ                (NComRxC *Com, double      v) { Com->mRemoteLeverArmZ = v; Com->mIsRemoteLeverArmZValid = 1; }

// Local reference frame (definition)

static void NComSetRefLat                         (NComRxC *Com, double      v) { Com->mRefLat = v; Com->mIsRefLatValid = 1; }
static void NComSetRefLon                         (NComRxC *Com, double      v) { Com->mRefLon = v; Com->mIsRefLonValid = 1; }
static void NComSetRefAlt                         (NComRxC *Com, double      v) { Com->mRefAlt = v; Com->mIsRefAltValid = 1; }
static void NComSetRefHeading                     (NComRxC *Com, double      v) { Com->mRefHeading = v; Com->mIsRefHeadingValid = 1; }

//------------------------------------------------------------------------------------------------------------
// Zero velocity and advanced slip

// Innovations

static void NComSetInnZeroVelX                    (NComRxC *Com, double      v) { Com->mInnZeroVelX = v; Com->mInnZeroVelXAge = 0; }
static void NComSetInnZeroVelY                    (NComRxC *Com, double      v) { Com->mInnZeroVelY = v; Com->mInnZeroVelYAge = 0; }
static void NComSetInnZeroVelZ                    (NComRxC *Com, double      v) { Com->mInnZeroVelZ = v; Com->mInnZeroVelZAge = 0; }

static void NComSetInnNoSlipH                     (NComRxC *Com, double      v) { Com->mInnNoSlipH = v; Com->mInnNoSlipHAge = 0; }

// Lever arm options

static void NComSetZeroVelLeverArmX               (NComRxC *Com, double      v) { Com->mZeroVelLeverArmX = v; Com->mIsZeroVelLeverArmXValid = 1; }
static void NComSetZeroVelLeverArmY               (NComRxC *Com, double      v) { Com->mZeroVelLeverArmY = v; Com->mIsZeroVelLeverArmYValid = 1; }
static void NComSetZeroVelLeverArmZ               (NComRxC *Com, double      v) { Com->mZeroVelLeverArmZ = v; Com->mIsZeroVelLeverArmZValid = 1; }

static void NComSetZeroVelLeverArmXAcc            (NComRxC *Com, double      v) { Com->mZeroVelLeverArmXAcc = v; Com->mIsZeroVelLeverArmXAccValid = 1; }
static void NComSetZeroVelLeverArmYAcc            (NComRxC *Com, double      v) { Com->mZeroVelLeverArmYAcc = v; Com->mIsZeroVelLeverArmYAccValid = 1; }
static void NComSetZeroVelLeverArmZAcc            (NComRxC *Com, double      v) { Com->mZeroVelLeverArmZAcc = v; Com->mIsZeroVelLeverArmZAccValid = 1; }

static void NComSetNoSlipLeverArmX                (NComRxC *Com, double      v) { Com->mNoSlipLeverArmX = v; Com->mIsNoSlipLeverArmXValid = 1; }
static void NComSetNoSlipLeverArmY                (NComRxC *Com, double      v) { Com->mNoSlipLeverArmY = v; Com->mIsNoSlipLeverArmYValid = 1; }
static void NComSetNoSlipLeverArmZ                (NComRxC *Com, double      v) { Com->mNoSlipLeverArmZ = v; Com->mIsNoSlipLeverArmZValid = 1; }

static void NComSetNoSlipLeverArmXAcc             (NComRxC *Com, double      v) { Com->mNoSlipLeverArmXAcc = v; Com->mIsNoSlipLeverArmXAccValid = 1; }
static void NComSetNoSlipLeverArmYAcc             (NComRxC *Com, double      v) { Com->mNoSlipLeverArmYAcc = v; Com->mIsNoSlipLeverArmYAccValid = 1; }
static void NComSetNoSlipLeverArmZAcc             (NComRxC *Com, double      v) { Com->mNoSlipLeverArmZAcc = v; Com->mIsNoSlipLeverArmZAccValid = 1; }

// Heading alignment

static void NComSetHeadingMisAlign                (NComRxC *Com, double      v) { Com->mHeadingMisAlign = v; Com->mIsHeadingMisAlignValid = 1; }
static void NComSetHeadingMisAlignAcc             (NComRxC *Com, double      v) { Com->mHeadingMisAlignAcc = v; Com->mIsHeadingMisAlignAccValid = 1; }

// User Options

static void NComSetOptionSZVDelay                 (NComRxC *Com, double      v) { Com->mOptionSZVDelay = v; Com->mIsOptionSZVDelayValid = 1; Com->mIsOptionSZVDelayConfig = 1; }
static void NComSetOptionSZVPeriod                (NComRxC *Com, double      v) { Com->mOptionSZVPeriod = v; Com->mIsOptionSZVPeriodValid = 1; Com->mIsOptionSZVPeriodConfig = 1; }

static void NComSetOptionNSDelay                  (NComRxC *Com, double      v) { Com->mOptionNSDelay = v; Com->mIsOptionNSDelayValid = 1; Com->mIsOptionNSDelayConfig = 1; }
static void NComSetOptionNSPeriod                 (NComRxC *Com, double      v) { Com->mOptionNSPeriod = v; Com->mIsOptionNSPeriodValid = 1; Com->mIsOptionNSPeriodConfig = 1; }
static void NComSetOptionNSAngleStd               (NComRxC *Com, double      v) { Com->mOptionNSAngleStd = v; Com->mIsOptionNSAngleStdValid = 1; Com->mIsOptionNSAngleStdConfig = 1; }
static void NComSetOptionNSHAccel                 (NComRxC *Com, double      v) { Com->mOptionNSHAccel = v; Com->mIsOptionNSHAccelValid = 1; Com->mIsOptionNSHAccelConfig = 1; }
static void NComSetOptionNSVAccel                 (NComRxC *Com, double      v) { Com->mOptionNSVAccel = v; Com->mIsOptionNSVAccelValid = 1; Com->mIsOptionNSVAccelConfig = 1; }
static void NComSetOptionNSSpeed                  (NComRxC *Com, double      v) { Com->mOptionNSSpeed = v; Com->mIsOptionNSSpeedValid = 1; Com->mIsOptionNSSpeedConfig = 1; }
static void NComSetOptionNSRadius                 (NComRxC *Com, double      v) { Com->mOptionNSRadius = v; Com->mIsOptionNSRadiusValid = 1; Com->mIsOptionNSRadiusConfig = 1; }

//------------------------------------------------------------------------------------------------------------
// Wheel speed input

// Innovations

static void NComSetInnWSpeed                      (NComRxC *Com, double      v) { Com->mInnWSpeed = v; Com->mInnWSpeedAge = 0; }

// Wheel speed lever arm option

static void NComSetWSpeedLeverArmX                (NComRxC *Com, double      v) { Com->mWSpeedLeverArmX = v; Com->mIsWSpeedLeverArmXValid = 1; }
static void NComSetWSpeedLeverArmY                (NComRxC *Com, double      v) { Com->mWSpeedLeverArmY = v; Com->mIsWSpeedLeverArmYValid = 1; }
static void NComSetWSpeedLeverArmZ                (NComRxC *Com, double      v) { Com->mWSpeedLeverArmZ = v; Com->mIsWSpeedLeverArmZValid = 1; }

static void NComSetWSpeedLeverArmXAcc             (NComRxC *Com, double      v) { Com->mWSpeedLeverArmXAcc = v; Com->mIsWSpeedLeverArmXAccValid = 1; }
static void NComSetWSpeedLeverArmYAcc             (NComRxC *Com, double      v) { Com->mWSpeedLeverArmYAcc = v; Com->mIsWSpeedLeverArmYAccValid = 1; }
static void NComSetWSpeedLeverArmZAcc             (NComRxC *Com, double      v) { Com->mWSpeedLeverArmZAcc = v; Com->mIsWSpeedLeverArmZAccValid = 1; }

// Wheel speed input model

static void NComSetWSpeedScale                    (NComRxC *Com, double      v) { Com->mWSpeedScale = v; Com->mIsWSpeedScaleValid = 1; Com->mIsWSpeedScaleConfig = 1; }
static void NComSetWSpeedScaleStd                 (NComRxC *Com, double      v) { Com->mWSpeedScaleStd = v; Com->mIsWSpeedScaleStdValid = 1; Com->mIsWSpeedScaleStdConfig = 1; }

static void NComSetOptionWSpeedDelay              (NComRxC *Com, double      v) { Com->mOptionWSpeedDelay = v; Com->mIsOptionWSpeedDelayValid = 1; Com->mIsOptionWSpeedDelayConfig = 1; }
static void NComSetOptionWSpeedZVDelay            (NComRxC *Com, double      v) { Com->mOptionWSpeedZVDelay = v; Com->mIsOptionWSpeedZVDelayValid = 1; Com->mIsOptionWSpeedZVDelayConfig = 1; }
static void NComSetOptionWSpeedNoiseStd           (NComRxC *Com, double      v) { Com->mOptionWSpeedNoiseStd = v; Com->mIsOptionWSpeedNoiseStdValid = 1; Com->mIsOptionWSpeedNoiseStdConfig = 1; }

// Wheel speed tacho measurements

static void NComSetWSpeedTime                     (NComRxC *Com, double      v) { Com->mWSpeedTime = v; Com->mIsWSpeedTimeValid = 1; }
static void NComSetWSpeedCount                    (NComRxC *Com, double      v) { Com->mWSpeedCount = v; Com->mIsWSpeedCountValid = 1; }
static void NComSetWSpeedTimeUnchanged            (NComRxC *Com, double      v) { Com->mWSpeedTimeUnchanged = v; Com->mIsWSpeedTimeUnchangedValid = 1; }
static void NComSetWSpeedFreq                     (NComRxC *Com, double      v) { Com->mWSpeedFreq = v; Com->mIsWSpeedFreqValid = 1; }

//------------------------------------------------------------------------------------------------------------
// Heading lock

// Innovations

static void NComSetInnHeadingH                    (NComRxC *Com, double      v) { Com->mInnHeadingH = v; Com->mInnHeadingHAge = 0; }

// User options

static void NComSetOptionHLDelay                  (NComRxC *Com, double      v) { Com->mOptionHLDelay = v; Com->mIsOptionHLDelayValid = 1; Com->mIsOptionHLDelayConfig = 1; }
static void NComSetOptionHLPeriod                 (NComRxC *Com, double      v) { Com->mOptionHLPeriod = v; Com->mIsOptionHLPeriodValid = 1; Com->mIsOptionHLPeriodConfig = 1; }
static void NComSetOptionHLAngleStd               (NComRxC *Com, double      v) { Com->mOptionHLAngleStd = v; Com->mIsOptionHLAngleStdValid = 1; Com->mIsOptionHLAngleStdConfig = 1; }

static void NComSetOptionStatDelay                (NComRxC *Com, double      v) { Com->mOptionStatDelay = v; Com->mIsOptionStatDelayValid = 1; Com->mIsOptionStatDelayConfig = 1; }
static void NComSetOptionStatSpeed                (NComRxC *Com, double      v) { Com->mOptionStatSpeed = v; Com->mIsOptionStatSpeedValid = 1; Com->mIsOptionStatSpeedConfig = 1; }

//------------------------------------------------------------------------------------------------------------
// For use in testing

// Reserved for testing

static void NComSetTimeMismatch                   (NComRxC *Com, int         v) { Com->mTimeMismatch = v; Com->mIsTimeMismatchValid = 1; }
static void NComSetImuTimeDiff                    (NComRxC *Com, int         v) { Com->mImuTimeDiff = v; Com->mIsImuTimeDiffValid = 1; }
static void NComSetImuTimeMargin                  (NComRxC *Com, int         v) { Com->mImuTimeMargin = v; Com->mIsImuTimeMarginValid = 1; }
static void NComSetImuLoopTime                    (NComRxC *Com, int         v) { Com->mImuLoopTime = v; Com->mIsImuLoopTimeValid = 1; }
static void NComSetOpLoopTime                     (NComRxC *Com, int         v) { Com->mOpLoopTime = v; Com->mIsOpLoopTimeValid = 1; }

static void NComSetBnsLag                         (NComRxC *Com, int         v) { Com->mBnsLag = v; Com->mIsBnsLagValid = 1; }
static void NComSetBnsLagFilt                     (NComRxC *Com, double      v) { Com->mBnsLagFilt = v; Com->mIsBnsLagFiltValid = 1; }


//============================================================================================================
//! \brief Invalidation.

void NComInvalidate(NComRxC *Com)
{
	NComInternalInvalidate(Com->mInternal);

	//--------------------------------------------------------------------------------------------------------
	// Other structures.

	NComGpsInvalidate(Com->mGpsPrimary);
	NComGpsInvalidate(Com->mGpsSecondary);
	NComGpsInvalidate(Com->mGpsExternal);

	//--------------------------------------------------------------------------------------------------------
	// General information

	// Status

	Com->mIsOutputPacketTypeValid = 0;               Com->mOutputPacketType = 0;
	Com->mIsInsNavModeValid = 0;                     Com->mInsNavMode = 0;

	// System information

	Com->mIsSerialNumberValid = 0;                   Com->mSerialNumber = 0;
	Com->mIsDevIdValid = 0;                          Com->mDevId[0] = '\0';

	Com->mIsOsVersion1Valid = 0;                     Com->mOsVersion1 = 0;
	Com->mIsOsVersion2Valid = 0;                     Com->mOsVersion2 = 0;
	Com->mIsOsVersion3Valid = 0;                     Com->mOsVersion3 = 0;
	Com->mIsOsScriptIdValid = 0;                     Com->mOsScriptId[0] = '\0';

	Com->mIsImuTypeValid = 0;                        Com->mImuType = 0;
	Com->mIsCpuPcbTypeValid = 0;                     Com->mCpuPcbType = 0;
	Com->mIsInterPcbTypeValid = 0;                   Com->mInterPcbType = 0;
	Com->mIsFrontPcbTypeValid = 0;                   Com->mFrontPcbType = 0;
	Com->mIsInterSwIdValid = 0;                      Com->mInterSwId = 0;
	Com->mIsHwConfigValid = 0;                       Com->mHwConfig = 0;

	Com->mIsDiskSpaceValid = 0;                      Com->mDiskSpace = 0;
	Com->mIsFileSizeValid = 0;                       Com->mFileSize = 0;
	Com->mIsUpTimeValid = 0;                         Com->mUpTime = 0;
	Com->mIsDualPortRamStatusValid = 0;              Com->mDualPortRamStatus = 0;

	// IMU information

	Com->mIsUmacStatusValid = 0;                     Com->mUmacStatus = 0;

	// Global Navigation Satellite System (GNSS) information

	Com->mIsGnssGpsEnabledValid = 0;                 Com->mGnssGpsEnabled = 0;
	Com->mIsGnssGlonassEnabledValid = 0;             Com->mGnssGlonassEnabled = 0;
	Com->mIsGnssGalileoEnabledValid = 0;             Com->mGnssGalileoEnabled = 0;

	Com->mIsPsrDiffEnabledValid = 0;                 Com->mPsrDiffEnabled = 0;
	Com->mIsSBASEnabledValid = 0;                    Com->mSBASEnabled = 0;
	Com->mIsOmniVBSEnabledValid = 0;                 Com->mOmniVBSEnabled = 0;
	Com->mIsOmniHPEnabledValid = 0;                  Com->mOmniHPEnabled = 0;
	Com->mIsL1DiffEnabledValid = 0;                  Com->mL1DiffEnabled = 0;
	Com->mIsL1L2DiffEnabledValid = 0;                Com->mL1L2DiffEnabled = 0;

	Com->mIsRawRngEnabledValid = 0;                  Com->mRawRngEnabled = 0;
	Com->mIsRawDopEnabledValid = 0;                  Com->mRawDopEnabled = 0;
	Com->mIsRawL1EnabledValid = 0;                   Com->mRawL1Enabled = 0;
	Com->mIsRawL2EnabledValid = 0;                   Com->mRawL2Enabled = 0;
	Com->mIsRawL5EnabledValid = 0;                   Com->mRawL5Enabled = 0;

	Com->mIsGpsPosModeValid = 0;                     Com->mGpsPosMode = 0;
	Com->mIsGpsVelModeValid = 0;                     Com->mGpsVelMode = 0;
	Com->mIsGpsAttModeValid = 0;                     Com->mGpsAttMode = 0;

	Com->mIsPDOPValid = 0;                           Com->mPDOP = 0.0;
	Com->mIsHDOPValid = 0;                           Com->mHDOP = 0.0;
	Com->mIsVDOPValid = 0;                           Com->mVDOP = 0.0;

	Com->mIsGpsNumObsValid = 0;                      Com->mGpsNumObs = 0;
	Com->mIsUndulationValid = 0;                     Com->mUndulation = 0.0;
	Com->mIsGpsDiffAgeValid = 0;                     Com->mGpsDiffAge = 0.0;
	Com->mIsBaseStationIdValid = 0;                  Com->mBaseStationId[0] = '\0';

	// Dual antenna computation status

	Com->mIsHeadQualityValid = 0;                    Com->mHeadQuality = 0;
	Com->mIsHeadSearchTypeValid = 0;                 Com->mHeadSearchType = 0;
	Com->mIsHeadSearchStatusValid = 0;               Com->mHeadSearchStatus = 0;
	Com->mIsHeadSearchReadyValid = 0;                Com->mHeadSearchReady = 0;

	Com->mIsHeadSearchInitValid = 0;                 Com->mHeadSearchInit = 0;
	Com->mIsHeadSearchNumValid = 0;                  Com->mHeadSearchNum = 0;
	Com->mIsHeadSearchTimeValid = 0;                 Com->mHeadSearchTime = 0;
	Com->mIsHeadSearchConstrValid = 0;               Com->mHeadSearchConstr = 0;

	Com->mIsHeadSearchMasterValid = 0;               Com->mHeadSearchMaster = 0;
	Com->mIsHeadSearchSlave1Valid = 0;               Com->mHeadSearchSlave1 = 0;
	Com->mIsHeadSearchSlave2Valid = 0;               Com->mHeadSearchSlave2 = 0;
	Com->mIsHeadSearchSlave3Valid = 0;               Com->mHeadSearchSlave3 = 0;

	// OmniSTAR information

	Com->mIsOmniStarSerialValid = 0;                 Com->mOmniStarSerial[0] = '\0';
	Com->mIsOmniStarFreqValid = 0;                   Com->mOmniStarFreq = 0.0;
	Com->mIsOmniStarSNRValid = 0;                    Com->mOmniStarSNR = 0.0;
	Com->mIsOmniStarLockTimeValid = 0;               Com->mOmniStarLockTime = 0.0;

	Com->mIsOmniStatusVbsExpiredValid = 0;           Com->mOmniStatusVbsExpired = 0;
	Com->mIsOmniStatusVbsOutOfRegionValid = 0;       Com->mOmniStatusVbsOutOfRegion = 0;
	Com->mIsOmniStatusVbsNoRemoteSitesValid = 0;     Com->mOmniStatusVbsNoRemoteSites = 0;

	Com->mIsOmniStatusHpExpiredValid = 0;            Com->mOmniStatusHpExpired = 0;
	Com->mIsOmniStatusHpOutOfRegionValid = 0;        Com->mOmniStatusHpOutOfRegion = 0;
	Com->mIsOmniStatusHpNoRemoteSitesValid = 0;      Com->mOmniStatusHpNoRemoteSites = 0;
	Com->mIsOmniStatusHpNotConvergedValid = 0;       Com->mOmniStatusHpNotConverged = 0;
	Com->mIsOmniStatusHpKeyInvalidValid = 0;         Com->mOmniStatusHpKeyInvalid = 0;

	//--------------------------------------------------------------------------------------------------------
	// General user options

	// General options

	Com->mIsOptionLevelValid = 0;                    Com->mOptionLevel = 0;
	Com->mIsOptionVibrationValid = 0;                Com->mOptionVibration = 0;
	Com->mIsOptionGpsAccValid = 0;                   Com->mOptionGpsAcc = 0;
	Com->mIsOptionUdpValid = 0;                      Com->mOptionUdp = 0;
	Com->mIsOptionSer1Valid = 0;                     Com->mOptionSer1 = 0;
	Com->mIsOptionSer2Valid = 0;                     Com->mOptionSer2 = 0;
	Com->mIsOptionSer3Valid = 0;                     Com->mOptionSer3 = 0;
	Com->mIsOptionHeadingValid = 0;                  Com->mOptionHeading = 0;

	Com->mIsOptionInitSpeedValid = 0;                Com->mIsOptionInitSpeedConfig = 0;          Com->mOptionInitSpeed = 0.0;
	Com->mIsOptionTopSpeedValid = 0;                 Com->mIsOptionTopSpeedConfig = 0;           Com->mOptionTopSpeed = 0.0;

	// Output baud rate settings

	Com->mIsOptionSer1BaudValid = 0;                 Com->mOptionSer1Baud = 0;
	Com->mIsOptionSer2BaudValid = 0;                 Com->mOptionSer2Baud = 0;
	Com->mIsOptionSer3BaudValid = 0;                 Com->mOptionSer3Baud = 0;

	Com->mIsOptionCanBaudValid = 0;                  Com->mOptionCanBaud = 0;

	//--------------------------------------------------------------------------------------------------------
	// General measurements

	// Timing

	Com->mIsTimeValid = 0;                           Com->mTime = 0.0;

	Com->mIsTimeWeekCountValid = 0;                  Com->mTimeWeekCount = 0;
	Com->mIsTimeWeekSecondValid = 0;                 Com->mTimeWeekSecond = 0.0;
	Com->mIsTimeUtcOffsetValid = 0;                  Com->mTimeUtcOffset = 0;

	// Position

	Com->mIsLatValid = 0;                            Com->mIsLatApprox = 0;                      Com->mLat = 0.0;
	Com->mIsLonValid = 0;                            Com->mIsLonApprox = 0;                      Com->mLon = 0.0;
	Com->mIsAltValid = 0;                            Com->mIsAltApprox = 0;                      Com->mAlt = 0.0;

	Com->mIsNorthAccValid = 0;                       Com->mNorthAcc = 0.0;
	Com->mIsEastAccValid = 0;                        Com->mEastAcc = 0.0;
	Com->mIsAltAccValid = 0;                         Com->mAltAcc = 0.0;

	// Distance

	Com->mIsDist2dValid = 0;                         Com->mDist2d = 0.0;
	Com->mIsDist3dValid = 0;                         Com->mDist3d = 0.0;

	// Velocity

	Com->mIsVnValid = 0;                             Com->mIsVnApprox = 0;                       Com->mVn = 0.0;
	Com->mIsVeValid = 0;                             Com->mIsVeApprox = 0;                       Com->mVe = 0.0;
	Com->mIsVdValid = 0;                             Com->mIsVdApprox = 0;                       Com->mVd = 0.0;

	Com->mIsVfValid = 0;                             Com->mVf = 0.0;
	Com->mIsVlValid = 0;                             Com->mVl = 0.0;

	Com->mIsVnAccValid = 0;                          Com->mVnAcc = 0.0;
	Com->mIsVeAccValid = 0;                          Com->mVeAcc = 0.0;
	Com->mIsVdAccValid = 0;                          Com->mVdAcc = 0.0;

	// Speed

	Com->mIsSpeed2dValid = 0;                        Com->mSpeed2d = 0.0;
	Com->mIsSpeed3dValid = 0;                        Com->mSpeed3d = 0.0;

	// Acceleration

	Com->mIsAxValid = 0;                             Com->mAx = 0.0;
	Com->mIsAyValid = 0;                             Com->mAy = 0.0;
	Com->mIsAzValid = 0;                             Com->mAz = 0.0;

	Com->mIsAfValid = 0;                             Com->mAf = 0.0;
	Com->mIsAlValid = 0;                             Com->mAl = 0.0;
	Com->mIsAdValid = 0;                             Com->mAd = 0.0;

	// Filtered acceleration

	Com->mIsFiltAxValid = 0;                         Com->mFiltAx = 0.0;
	Com->mIsFiltAyValid = 0;                         Com->mFiltAy = 0.0;
	Com->mIsFiltAzValid = 0;                         Com->mFiltAz = 0.0;

	Com->mIsFiltAfValid = 0;                         Com->mFiltAf = 0.0;
	Com->mIsFiltAlValid = 0;                         Com->mFiltAl = 0.0;
	Com->mIsFiltAdValid = 0;                         Com->mFiltAd = 0.0;

	// Orientation

	Com->mIsHeadingValid = 0;                        Com->mIsHeadingApprox = 0;                  Com->mHeading = 0.0;
	Com->mIsPitchValid = 0;                          Com->mIsPitchApprox = 0;                    Com->mPitch = 0.0;
	Com->mIsRollValid = 0;                           Com->mIsRollApprox = 0;                     Com->mRoll = 0.0;

	Com->mIsHeadingAccValid = 0;                     Com->mHeadingAcc = 0.0;
	Com->mIsPitchAccValid = 0;                       Com->mPitchAcc = 0.0;
	Com->mIsRollAccValid = 0;                        Com->mRollAcc = 0.0;

	// Special

	Com->mIsTrackValid = 0;                          Com->mTrack = 0.0;
	Com->mIsSlipValid = 0;                           Com->mSlip = 0.0;
	Com->mIsCurvatureValid = 0;                      Com->mCurvature = 0.0;

	// Angular rate

	Com->mIsWxValid = 0;                             Com->mWx = 0.0;
	Com->mIsWyValid = 0;                             Com->mWy = 0.0;
	Com->mIsWzValid = 0;                             Com->mWz = 0.0;

	Com->mIsWfValid = 0;                             Com->mWf = 0.0;
	Com->mIsWlValid = 0;                             Com->mWl = 0.0;
	Com->mIsWdValid = 0;                             Com->mWd = 0.0;

	// Angular acceleration

	Com->mIsYxValid = 0;                             Com->mYx = 0.0;
	Com->mIsYyValid = 0;                             Com->mYy = 0.0;
	Com->mIsYzValid = 0;                             Com->mYz = 0.0;

	Com->mIsYfValid = 0;                             Com->mYf = 0.0;
	Com->mIsYlValid = 0;                             Com->mYl = 0.0;
	Com->mIsYdValid = 0;                             Com->mYd = 0.0;

	// Filtered angular acceleration

	Com->mIsFiltYxValid = 0;                         Com->mFiltYx = 0.0;
	Com->mIsFiltYyValid = 0;                         Com->mFiltYy = 0.0;
	Com->mIsFiltYzValid = 0;                         Com->mFiltYz = 0.0;

	Com->mIsFiltYfValid = 0;                         Com->mFiltYf = 0.0;
	Com->mIsFiltYlValid = 0;                         Com->mFiltYl = 0.0;
	Com->mIsFiltYdValid = 0;                         Com->mFiltYd = 0.0;

	// Filter characteristics

	Com->mIsLinAccFiltFreqValid = 0;                 Com->mLinAccFiltFreq = -1.0;
	Com->mIsLinAccFiltZetaValid = 0;                 Com->mLinAccFiltZeta = -1.0;
	Com->mIsLinAccFiltFixed = 0;
	Com->mHasLinAccFiltChanged = 0;
	Com->mIsLinAccFiltOff = 0;

	Com->mIsAngAccFiltFreqValid = 0;                 Com->mAngAccFiltFreq = -1.0;
	Com->mIsAngAccFiltZetaValid = 0;                 Com->mAngAccFiltZeta = -1.0;
	Com->mIsAngAccFiltFixed = 0;
	Com->mHasAngAccFiltChanged = 0;
	Com->mIsAngAccFiltOff = 0;

	//--------------------------------------------------------------------------------------------------------
	// Model particulars

	// Innovations (discrepancy between GPS and IMU)

	Com->mInnPosXAge = MAX_INN_AGE;                  Com->mInnPosX = 0.0;
	Com->mInnPosYAge = MAX_INN_AGE;                  Com->mInnPosY = 0.0;
	Com->mInnPosZAge = MAX_INN_AGE;                  Com->mInnPosZ = 0.0;

	Com->mInnVelXAge = MAX_INN_AGE;                  Com->mInnVelX = 0.0;
	Com->mInnVelYAge = MAX_INN_AGE;                  Com->mInnVelY = 0.0;
	Com->mInnVelZAge = MAX_INN_AGE;                  Com->mInnVelZ = 0.0;

	Com->mInnHeadingAge = MAX_INN_AGE;               Com->mInnHeading = 0.0;
	Com->mInnPitchAge = MAX_INN_AGE;                 Com->mInnPitch = 0.0;

	// Gyroscope bias and scale factor

	Com->mIsWxBiasValid = 0;                         Com->mWxBias = 0.0;
	Com->mIsWyBiasValid = 0;                         Com->mWyBias = 0.0;
	Com->mIsWzBiasValid = 0;                         Com->mWzBias = 0.0;

	Com->mIsWxBiasAccValid = 0;                      Com->mWxBiasAcc = 0.0;
	Com->mIsWyBiasAccValid = 0;                      Com->mWyBiasAcc = 0.0;
	Com->mIsWzBiasAccValid = 0;                      Com->mWzBiasAcc = 0.0;

	Com->mIsWxSfValid = 0;                           Com->mWxSf = 0.0;
	Com->mIsWySfValid = 0;                           Com->mWySf = 0.0;
	Com->mIsWzSfValid = 0;                           Com->mWzSf = 0.0;

	Com->mIsWxSfAccValid = 0;                        Com->mWxSfAcc = 0.0;
	Com->mIsWySfAccValid = 0;                        Com->mWySfAcc = 0.0;
	Com->mIsWzSfAccValid = 0;                        Com->mWzSfAcc = 0.0;

	// Accelerometer bias and scale factor

	Com->mIsAxBiasValid = 0;                         Com->mAxBias = 0.0;
	Com->mIsAyBiasValid = 0;                         Com->mAyBias = 0.0;
	Com->mIsAzBiasValid = 0;                         Com->mAzBias = 0.0;

	Com->mIsAxBiasAccValid = 0;                      Com->mAxBiasAcc = 0.0;
	Com->mIsAyBiasAccValid = 0;                      Com->mAyBiasAcc = 0.0;
	Com->mIsAzBiasAccValid = 0;                      Com->mAzBiasAcc = 0.0;

	Com->mIsAxSfValid = 0;                           Com->mAxSf = 0.0;
	Com->mIsAySfValid = 0;                           Com->mAySf = 0.0;
	Com->mIsAzSfValid = 0;                           Com->mAzSf = 0.0;

	Com->mIsAxSfAccValid = 0;                        Com->mAxSfAcc = 0.0;
	Com->mIsAySfAccValid = 0;                        Com->mAySfAcc = 0.0;
	Com->mIsAzSfAccValid = 0;                        Com->mAzSfAcc = 0.0;

	// GPS antenna position

	Com->mIsGAPxValid = 0;                           Com->mGAPx = 0.0;
	Com->mIsGAPyValid = 0;                           Com->mGAPy = 0.0;
	Com->mIsGAPzValid = 0;                           Com->mGAPz = 0.0;

	Com->mIsGAPxAccValid = 0;                        Com->mGAPxAcc = 0.0;
	Com->mIsGAPyAccValid = 0;                        Com->mGAPyAcc = 0.0;
	Com->mIsGAPzAccValid = 0;                        Com->mGAPzAcc = 0.0;

	Com->mIsAtHValid = 0;                            Com->mAtH = 0.0;
	Com->mIsAtPValid = 0;                            Com->mAtP = 0.0;

	Com->mIsAtHAccValid = 0;                         Com->mAtHAcc = 0.0;
	Com->mIsAtPAccValid = 0;                         Com->mAtPAcc = 0.0;

	Com->mIsBaseLineLengthValid = 0;                 Com->mIsBaseLineLengthConfig = 0;           Com->mBaseLineLength = 0.0;
	Com->mIsBaseLineLengthAccValid = 0;              Com->mIsBaseLineLengthAccConfig = 0;        Com->mBaseLineLengthAcc = 0.0;

	//--------------------------------------------------------------------------------------------------------
	// Statistics

	// IMU hardware status event counters

	Com->mIsImuMissedPktsValid = 0;                  Com->mImuMissedPkts = 0;
	Com->mIsImuResetCountValid = 0;                  Com->mImuResetCount = 0;
	Com->mIsImuErrorCountValid = 0;                  Com->mImuErrorCount = 0;

	// GPS successive rejected aiding updates

	Com->mIsGPSPosRejectValid = 0;                   Com->mGPSPosReject = 0;
	Com->mIsGPSVelRejectValid = 0;                   Com->mGPSVelReject = 0;
	Com->mIsGPSAttRejectValid = 0;                   Com->mGPSAttReject = 0;

	// Received data statistics

	Com->mIsImuCharsValid = 0;                       Com->mImuChars = 0;
	Com->mIsImuCharsSkippedValid = 0;                Com->mImuCharsSkipped = 0;
	Com->mIsImuPktsValid = 0;                        Com->mImuPkts = 0;

	Com->mIsCmdCharsValid = 0;                       Com->mCmdChars = 0;
	Com->mIsCmdCharsSkippedValid = 0;                Com->mCmdCharsSkipped = 0;
	Com->mIsCmdPktsValid = 0;                        Com->mCmdPkts = 0;
	Com->mIsCmdErrorsValid = 0;                      Com->mCmdErrors = 0;

	//--------------------------------------------------------------------------------------------------------
	// Transformation Euler angles

	// Orientation of vehicle-frame relative to IMU-frame

	Com->mIsImu2VehHeadingValid = 0;                 Com->mImu2VehHeading = 0.0;
	Com->mIsImu2VehPitchValid = 0;                   Com->mImu2VehPitch = 0.0;
	Com->mIsImu2VehRollValid = 0;                    Com->mImu2VehRoll = 0.0;

	//--------------------------------------------------------------------------------------------------------
	// Miscellaneous items

	// Triggers

	Com->mIsTrigTimeValid = 0;                       Com->mIsTrigTimeNew = 0;                 Com->mTrigTime = 0.0;
	Com->mIsTrig2TimeValid = 0;                      Com->mIsTrig2TimeNew = 0;                Com->mTrig2Time = 0.0;
	Com->mIsDigitalOutTimeValid = 0;                 Com->mIsDigitalOutTimeNew = 0;           Com->mDigitalOutTime = 0.0;

	// Remote lever arm option

	Com->mIsRemoteLeverArmXValid = 0;                Com->mRemoteLeverArmX = 0.0;
	Com->mIsRemoteLeverArmYValid = 0;                Com->mRemoteLeverArmY = 0.0;
	Com->mIsRemoteLeverArmZValid = 0;                Com->mRemoteLeverArmZ = 0.0;

	// Local reference frame (definition)

	Com->mIsRefLatValid = 0;                         Com->mRefLat = 0.0;
	Com->mIsRefLonValid = 0;                         Com->mRefLon = 0.0;
	Com->mIsRefAltValid = 0;                         Com->mRefAlt = 0.0;
	Com->mIsRefHeadingValid = 0;                     Com->mRefHeading = 0.0;

	//--------------------------------------------------------------------------------------------------------
	// Zero velocity and advanced slip

	// Innovations

	Com->mInnZeroVelXAge = MAX_INN_AGE;              Com->mInnZeroVelX = 0.0;
	Com->mInnZeroVelYAge = MAX_INN_AGE;              Com->mInnZeroVelY = 0.0;
	Com->mInnZeroVelZAge = MAX_INN_AGE;              Com->mInnZeroVelZ = 0.0;

	Com->mInnNoSlipHAge = MAX_INN_AGE;               Com->mInnNoSlipH = 0.0;

	// Lever arm options

	Com->mIsZeroVelLeverArmXValid = 0;               Com->mZeroVelLeverArmX = 0.0;
	Com->mIsZeroVelLeverArmYValid = 0;               Com->mZeroVelLeverArmY = 0.0;
	Com->mIsZeroVelLeverArmZValid = 0;               Com->mZeroVelLeverArmZ = 0.0;

	Com->mIsZeroVelLeverArmXAccValid = 0;            Com->mZeroVelLeverArmXAcc = 0.0;
	Com->mIsZeroVelLeverArmYAccValid = 0;            Com->mZeroVelLeverArmYAcc = 0.0;
	Com->mIsZeroVelLeverArmZAccValid = 0;            Com->mZeroVelLeverArmZAcc = 0.0;

	Com->mIsNoSlipLeverArmXValid = 0;                Com->mNoSlipLeverArmX = 0.0;
	Com->mIsNoSlipLeverArmYValid = 0;                Com->mNoSlipLeverArmY = 0.0;
	Com->mIsNoSlipLeverArmZValid = 0;                Com->mNoSlipLeverArmZ = 0.0;

	Com->mIsNoSlipLeverArmXAccValid = 0;             Com->mNoSlipLeverArmXAcc = 0.0;
	Com->mIsNoSlipLeverArmYAccValid = 0;             Com->mNoSlipLeverArmYAcc = 0.0;
	Com->mIsNoSlipLeverArmZAccValid = 0;             Com->mNoSlipLeverArmZAcc = 0.0;

	// Heading alignment

	Com->mIsHeadingMisAlignValid = 0;                Com->mHeadingMisAlign = 0.0;
	Com->mIsHeadingMisAlignAccValid = 0;             Com->mHeadingMisAlignAcc = 0.0;

	// User Options

	Com->mIsOptionSZVDelayValid = 0;                 Com->mIsOptionSZVDelayConfig = 0;           Com->mOptionSZVDelay = 0.0;
	Com->mIsOptionSZVPeriodValid = 0;                Com->mIsOptionSZVPeriodConfig = 0;          Com->mOptionSZVPeriod = 0.0;

	Com->mIsOptionNSDelayValid = 0;                  Com->mIsOptionNSDelayConfig = 0;            Com->mOptionNSDelay = 0.0;
	Com->mIsOptionNSPeriodValid = 0;                 Com->mIsOptionNSPeriodConfig = 0;           Com->mOptionNSPeriod = 0.0;
	Com->mIsOptionNSAngleStdValid = 0;               Com->mIsOptionNSAngleStdConfig = 0;         Com->mOptionNSAngleStd = 0.0;
	Com->mIsOptionNSHAccelValid = 0;                 Com->mIsOptionNSHAccelConfig = 0;           Com->mOptionNSHAccel = 0.0;
	Com->mIsOptionNSVAccelValid = 0;                 Com->mIsOptionNSVAccelConfig = 0;           Com->mOptionNSVAccel = 0.0;
	Com->mIsOptionNSSpeedValid = 0;                  Com->mIsOptionNSSpeedConfig = 0;            Com->mOptionNSSpeed = 0.0;
	Com->mIsOptionNSRadiusValid = 0;                 Com->mIsOptionNSRadiusConfig = 0;           Com->mOptionNSRadius = 0.0;

	//--------------------------------------------------------------------------------------------------------
	// Wheel speed input

	// Innovations

	Com->mInnWSpeedAge = MAX_INN_AGE;                Com->mInnWSpeed = 0.0;

	// Wheel speed lever arm option

	Com->mIsWSpeedLeverArmXValid = 0;                Com->mWSpeedLeverArmX = 0.0;
	Com->mIsWSpeedLeverArmYValid = 0;                Com->mWSpeedLeverArmY = 0.0;
	Com->mIsWSpeedLeverArmZValid = 0;                Com->mWSpeedLeverArmZ = 0.0;

	Com->mIsWSpeedLeverArmXAccValid = 0;             Com->mWSpeedLeverArmXAcc = 0.0;
	Com->mIsWSpeedLeverArmYAccValid = 0;             Com->mWSpeedLeverArmYAcc = 0.0;
	Com->mIsWSpeedLeverArmZAccValid = 0;             Com->mWSpeedLeverArmZAcc = 0.0;

	// Wheel speed input model

	Com->mIsWSpeedScaleValid = 0;                    Com->mIsWSpeedScaleConfig = 0;              Com->mWSpeedScale = 0.0;
	Com->mIsWSpeedScaleStdValid = 0;                 Com->mIsWSpeedScaleStdConfig = 0;           Com->mWSpeedScaleStd = 0.0;

	Com->mIsOptionWSpeedDelayValid = 0;              Com->mIsOptionWSpeedDelayConfig = 0;        Com->mOptionWSpeedDelay = 0.0;
	Com->mIsOptionWSpeedZVDelayValid = 0;            Com->mIsOptionWSpeedZVDelayConfig = 0;      Com->mOptionWSpeedZVDelay = 0.0;
	Com->mIsOptionWSpeedNoiseStdValid = 0;           Com->mIsOptionWSpeedNoiseStdConfig = 0;     Com->mOptionWSpeedNoiseStd = 0.0;

	// Wheel speed tacho measurements

	Com->mIsWSpeedTimeValid = 0;                     Com->mWSpeedTime = 0.0;
	Com->mIsWSpeedCountValid = 0;                    Com->mWSpeedCount = 0.0;
	Com->mIsWSpeedTimeUnchangedValid = 0;            Com->mWSpeedTimeUnchanged = 0.0;
	Com->mIsWSpeedFreqValid = 0;                     Com->mWSpeedFreq = 0.0;

	//--------------------------------------------------------------------------------------------------------
	// Heading lock

	// Innovations

	Com->mInnHeadingHAge = MAX_INN_AGE;              Com->mInnHeadingH = 0.0;

	// User options

	Com->mIsOptionHLDelayValid = 0;                  Com->mIsOptionHLDelayConfig = 0;            Com->mOptionHLDelay = 0.0;
	Com->mIsOptionHLPeriodValid = 0;                 Com->mIsOptionHLPeriodConfig = 0;           Com->mOptionHLPeriod = 0.0;
	Com->mIsOptionHLAngleStdValid = 0;               Com->mIsOptionHLAngleStdConfig = 0;         Com->mOptionHLAngleStd = 0.0;

	Com->mIsOptionStatDelayValid = 0;                Com->mIsOptionStatDelayConfig = 0;          Com->mOptionStatDelay = 0.0;
	Com->mIsOptionStatSpeedValid = 0;                Com->mIsOptionStatSpeedConfig = 0;          Com->mOptionStatSpeed = 0.0;

	//--------------------------------------------------------------------------------------------------------
	// For use in testing

	// Reserved for testing

	Com->mIsTimeMismatchValid = 0;                   Com->mTimeMismatch = 0;
	Com->mIsImuTimeDiffValid = 0;                    Com->mImuTimeDiff = 0;
	Com->mIsImuTimeMarginValid = 0;                  Com->mImuTimeMargin = 0;
	Com->mIsImuLoopTimeValid = 0;                    Com->mImuLoopTime = 0;
	Com->mIsOpLoopTimeValid = 0;                     Com->mOpLoopTime = 0;

	Com->mIsBnsLagValid = 0;                         Com->mBnsLag = 0;
	Com->mIsBnsLagFiltValid = 0;                     Com->mBnsLagFilt = 0.0;
}


//============================================================================================================
//! \brief Basic report to a file pointer.
//!
//! The user may wish to export this function. By not exporting we avoid exposing the standard library in
//! which FILE is defined to the user.

void NComReportFP(const NComRxC *Com, FILE *fp, const char *pre)
{
	fprintf(fp, "----------------------------------------------------------------------\n");

	NComInternalReportFP(Com->mInternal, fp, "Internal.");

	//--------------------------------------------------------------------------------------------------------
	// Other structures.

	NComGpsReportFP(Com->mGpsPrimary,    fp, "GpsPrimary.");
	NComGpsReportFP(Com->mGpsSecondary,  fp, "GpsSecondary.");
	NComGpsReportFP(Com->mGpsExternal,   fp, "GpsExternal.");

	//--------------------------------------------------------------------------------------------------------
	// General information

	// Status

	fprintf(fp, PNV_IE, pre, "OutputPacketType",          Com->mIsOutputPacketTypeValid,          Com->mOutputPacketType,         NComGetOutputPacketTypeString(Com));
	fprintf(fp, PNV_IE, pre, "InsNavMode",                Com->mIsInsNavModeValid,                Com->mInsNavMode,               NComGetInsNavModeString(Com));

	// System information

	fprintf(fp, PNV_I_, pre, "SerialNumber",              Com->mIsSerialNumberValid,              Com->mSerialNumber);
	fprintf(fp, PNV_S_, pre, "DevId",                     Com->mIsDevIdValid,                     Com->mDevId);

	fprintf(fp, PNV_I_, pre, "OsVersion1",                Com->mIsOsVersion1Valid,                Com->mOsVersion1);
	fprintf(fp, PNV_I_, pre, "OsVersion2",                Com->mIsOsVersion2Valid,                Com->mOsVersion2);
	fprintf(fp, PNV_I_, pre, "OsVersion3",                Com->mIsOsVersion3Valid,                Com->mOsVersion3);
	fprintf(fp, PNV_S_, pre, "OsScriptId",                Com->mIsOsScriptIdValid,                Com->mOsScriptId);

	fprintf(fp, PNV_IE, pre, "ImuType",                   Com->mIsImuTypeValid,                   Com->mImuType,                  NComGetImuTypeString(Com));
	fprintf(fp, PNV_IE, pre, "CpuPcbType",                Com->mIsCpuPcbTypeValid,                Com->mCpuPcbType,               NComGetCpuPcbTypeString(Com));
	fprintf(fp, PNV_IE, pre, "InterPcbType",              Com->mIsInterPcbTypeValid,              Com->mInterPcbType,             NComGetInterPcbTypeString(Com));
	fprintf(fp, PNV_IE, pre, "FrontPcbType",              Com->mIsFrontPcbTypeValid,              Com->mFrontPcbType,             NComGetFrontPcbTypeString(Com));
	fprintf(fp, PNV_IE, pre, "InterSwId",                 Com->mIsInterSwIdValid,                 Com->mInterSwId,                NComGetInterSwIdString(Com));
	fprintf(fp, PNV_IE, pre, "HwConfig",                  Com->mIsHwConfigValid,                  Com->mHwConfig,                 NComGetHwConfigString(Com));

	fprintf(fp, PNV_o_, pre, "DiskSpace",                 Com->mIsDiskSpaceValid,                 Com->mDiskSpace);
	fprintf(fp, PNV_o_, pre, "FileSize",                  Com->mIsFileSizeValid,                  Com->mFileSize);
	fprintf(fp, PNV_q_, pre, "UpTime",                    Com->mIsUpTimeValid,                    Com->mUpTime);
	fprintf(fp, PNV_IE, pre, "DualPortRamStatus",         Com->mIsDualPortRamStatusValid,         Com->mDualPortRamStatus,        NComGetDualPortRamStatusString(Com));

	// IMU information

	fprintf(fp, PNV_IE, pre, "UmacStatus",                Com->mIsUmacStatusValid,                Com->mUmacStatus,               NComGetUmacStatusString(Com));

	// Global Navigation Satellite System (GNSS) information

	fprintf(fp, PNV_I_, pre, "GnssGpsEnabled",            Com->mIsGnssGpsEnabledValid,            Com->mGnssGpsEnabled);
	fprintf(fp, PNV_I_, pre, "GnssGlonassEnabled",        Com->mIsGnssGlonassEnabledValid,        Com->mGnssGlonassEnabled);
	fprintf(fp, PNV_I_, pre, "GnssGalileoEnabled",        Com->mIsGnssGalileoEnabledValid,        Com->mGnssGalileoEnabled);

	fprintf(fp, PNV_I_, pre, "PsrDiffEnabled",            Com->mIsPsrDiffEnabledValid,            Com->mPsrDiffEnabled);
	fprintf(fp, PNV_I_, pre, "SBASEnabled",               Com->mIsSBASEnabledValid,               Com->mSBASEnabled);
	fprintf(fp, PNV_I_, pre, "OmniVBSEnabled",            Com->mIsOmniVBSEnabledValid,            Com->mOmniVBSEnabled);
	fprintf(fp, PNV_I_, pre, "OmniHPEnabled",             Com->mIsOmniHPEnabledValid,             Com->mOmniHPEnabled);
	fprintf(fp, PNV_I_, pre, "L1DiffEnabled",             Com->mIsL1DiffEnabledValid,             Com->mL1DiffEnabled);
	fprintf(fp, PNV_I_, pre, "L1L2DiffEnabled",           Com->mIsL1L2DiffEnabledValid,           Com->mL1L2DiffEnabled);

	fprintf(fp, PNV_I_, pre, "RawRngEnabled",             Com->mIsRawRngEnabledValid,             Com->mRawRngEnabled);
	fprintf(fp, PNV_I_, pre, "RawDopEnabled",             Com->mIsRawDopEnabledValid,             Com->mRawDopEnabled);
	fprintf(fp, PNV_I_, pre, "RawL1Enabled",              Com->mIsRawL1EnabledValid,              Com->mRawL1Enabled);
	fprintf(fp, PNV_I_, pre, "RawL2Enabled",              Com->mIsRawL2EnabledValid,              Com->mRawL2Enabled);
	fprintf(fp, PNV_I_, pre, "RawL5Enabled",              Com->mIsRawL5EnabledValid,              Com->mRawL5Enabled);

	fprintf(fp, PNV_IE, pre, "GpsPosMode",                Com->mIsGpsPosModeValid,                Com->mGpsPosMode,               NComGetGpsPosModeString(Com));
	fprintf(fp, PNV_IE, pre, "GpsVelMode",                Com->mIsGpsVelModeValid,                Com->mGpsVelMode,               NComGetGpsVelModeString(Com));
	fprintf(fp, PNV_IE, pre, "GpsAttMode",                Com->mIsGpsAttModeValid,                Com->mGpsAttMode,               NComGetGpsAttModeString(Com));

	fprintf(fp, PNV_D_, pre, "PDOP",                      Com->mIsPDOPValid,                      Com->mPDOP);
	fprintf(fp, PNV_D_, pre, "HDOP",                      Com->mIsHDOPValid,                      Com->mHDOP);
	fprintf(fp, PNV_D_, pre, "VDOP",                      Com->mIsVDOPValid,                      Com->mVDOP);

	fprintf(fp, PNV_I_, pre, "GpsNumObs",                 Com->mIsGpsNumObsValid,                 Com->mGpsNumObs);
	fprintf(fp, PNV_D_, pre, "Undulation",                Com->mIsUndulationValid,                Com->mUndulation);
	fprintf(fp, PNV_D_, pre, "GpsDiffAge",                Com->mIsGpsDiffAgeValid,                Com->mGpsDiffAge);
	fprintf(fp, PNV_S_, pre, "BaseStationId",             Com->mIsBaseStationIdValid,             Com->mBaseStationId);

	// Dual antenna computation status

	fprintf(fp, PNV_IE, pre, "HeadQuality",               Com->mIsHeadQualityValid,               Com->mHeadQuality,              NComGetHeadQualityString(Com));
	fprintf(fp, PNV_IE, pre, "HeadSearchType",            Com->mIsHeadSearchTypeValid,            Com->mHeadSearchType,           NComGetHeadSearchTypeString(Com));
	fprintf(fp, PNV_IE, pre, "HeadSearchStatus",          Com->mIsHeadSearchStatusValid,          Com->mHeadSearchStatus,         NComGetHeadSearchStatusString(Com));
	fprintf(fp, PNV_IE, pre, "HeadSearchReady",           Com->mIsHeadSearchReadyValid,           Com->mHeadSearchReady,          NComGetHeadSearchReadyString(Com));

	fprintf(fp, PNV_I_, pre, "HeadSearchInit",            Com->mIsHeadSearchInitValid,            Com->mHeadSearchInit);
	fprintf(fp, PNV_I_, pre, "HeadSearchNum",             Com->mIsHeadSearchNumValid,             Com->mHeadSearchNum);
	fprintf(fp, PNV_I_, pre, "HeadSearchTime",            Com->mIsHeadSearchTimeValid,            Com->mHeadSearchTime);
	fprintf(fp, PNV_I_, pre, "HeadSearchConstr",          Com->mIsHeadSearchConstrValid,          Com->mHeadSearchConstr);

	fprintf(fp, PNV_I_, pre, "HeadSearchMaster",          Com->mIsHeadSearchMasterValid,          Com->mHeadSearchMaster);
	fprintf(fp, PNV_I_, pre, "HeadSearchSlave1",          Com->mIsHeadSearchSlave1Valid,          Com->mHeadSearchSlave1);
	fprintf(fp, PNV_I_, pre, "HeadSearchSlave2",          Com->mIsHeadSearchSlave2Valid,          Com->mHeadSearchSlave2);
	fprintf(fp, PNV_I_, pre, "HeadSearchSlave3",          Com->mIsHeadSearchSlave3Valid,          Com->mHeadSearchSlave3);

	// OmniSTAR information

	fprintf(fp, PNV_S_, pre, "OmniStarSerial",            Com->mIsOmniStarSerialValid,            Com->mOmniStarSerial);
	fprintf(fp, PNV_D_, pre, "OmniStarFreq",              Com->mIsOmniStarFreqValid,              Com->mOmniStarFreq);
	fprintf(fp, PNV_D_, pre, "OmniStarSNR",               Com->mIsOmniStarSNRValid,               Com->mOmniStarSNR);
	fprintf(fp, PNV_D_, pre, "OmniStarLockTime",          Com->mIsOmniStarLockTimeValid,          Com->mOmniStarLockTime);

	fprintf(fp, PNV_I_, pre, "OmniStatusVbsExpired",      Com->mIsOmniStatusVbsExpiredValid,      Com->mOmniStatusVbsExpired);
	fprintf(fp, PNV_I_, pre, "OmniStatusVbsOutOfRegion",  Com->mIsOmniStatusVbsOutOfRegionValid,  Com->mOmniStatusVbsOutOfRegion);
	fprintf(fp, PNV_I_, pre, "OmniStatusVbsNoRemoteSites",Com->mIsOmniStatusVbsNoRemoteSitesValid, Com->mOmniStatusVbsNoRemoteSites);

	fprintf(fp, PNV_I_, pre, "OmniStatusHpExpired",       Com->mIsOmniStatusHpExpiredValid,       Com->mOmniStatusHpExpired);
	fprintf(fp, PNV_I_, pre, "OmniStatusHpOutOfRegion",   Com->mIsOmniStatusHpOutOfRegionValid,   Com->mOmniStatusHpOutOfRegion);
	fprintf(fp, PNV_I_, pre, "OmniStatusHpNoRemoteSites", Com->mIsOmniStatusHpNoRemoteSitesValid, Com->mOmniStatusHpNoRemoteSites);
	fprintf(fp, PNV_I_, pre, "OmniStatusHpNotConverged",  Com->mIsOmniStatusHpNotConvergedValid,  Com->mOmniStatusHpNotConverged);
	fprintf(fp, PNV_I_, pre, "OmniStatusHpKeyInvalid",    Com->mIsOmniStatusHpKeyInvalidValid,    Com->mOmniStatusHpKeyInvalid);

	//--------------------------------------------------------------------------------------------------------
	// General user options

	// General options

	fprintf(fp, PNV_IE, pre, "OptionLevel",               Com->mIsOptionLevelValid,               Com->mOptionLevel,              NComGetOptionLevelString(Com));
	fprintf(fp, PNV_IE, pre, "OptionVibration",           Com->mIsOptionVibrationValid,           Com->mOptionVibration,          NComGetOptionVibrationString(Com));
	fprintf(fp, PNV_IE, pre, "OptionGpsAcc",              Com->mIsOptionGpsAccValid,              Com->mOptionGpsAcc,             NComGetOptionGpsAccString(Com));
	fprintf(fp, PNV_IE, pre, "OptionUdp",                 Com->mIsOptionUdpValid,                 Com->mOptionUdp,                NComGetOptionUdpString(Com));
	fprintf(fp, PNV_IE, pre, "OptionSer1",                Com->mIsOptionSer1Valid,                Com->mOptionSer1,               NComGetOptionSer1String(Com));
	fprintf(fp, PNV_IE, pre, "OptionSer2",                Com->mIsOptionSer2Valid,                Com->mOptionSer2,               NComGetOptionSer2String(Com));
	fprintf(fp, PNV_IE, pre, "OptionSer3",                Com->mIsOptionSer3Valid,                Com->mOptionSer3,               NComGetOptionSer3String(Com));
	fprintf(fp, PNV_IE, pre, "OptionHeading",             Com->mIsOptionHeadingValid,             Com->mOptionHeading,            NComGetOptionHeadingString(Com));

	fprintf(fp, PNVCD_, pre, "OptionInitSpeed",           Com->mIsOptionInitSpeedValid,           Com->mIsOptionInitSpeedConfig,          Com->mOptionInitSpeed);
	fprintf(fp, PNVCD_, pre, "OptionTopSpeed",            Com->mIsOptionTopSpeedValid,            Com->mIsOptionTopSpeedConfig,           Com->mOptionTopSpeed);

	// Output baud rate settings

	fprintf(fp, PNV_IE, pre, "OptionSer1Baud",            Com->mIsOptionSer1BaudValid,            Com->mOptionSer1Baud,           NComGetOptionSer1BaudString(Com));
	fprintf(fp, PNV_IE, pre, "OptionSer2Baud",            Com->mIsOptionSer2BaudValid,            Com->mOptionSer2Baud,           NComGetOptionSer2BaudString(Com));
	fprintf(fp, PNV_IE, pre, "OptionSer3Baud",            Com->mIsOptionSer3BaudValid,            Com->mOptionSer3Baud,           NComGetOptionSer3BaudString(Com));

	fprintf(fp, PNV_IE, pre, "OptionCanBaud",             Com->mIsOptionCanBaudValid,             Com->mOptionCanBaud,            NComGetOptionCanBaudString(Com));

	//--------------------------------------------------------------------------------------------------------
	// General measurements

	// Timing

	fprintf(fp, PNV_D_, pre, "Time",                      Com->mIsTimeValid,                      Com->mTime);

	fprintf(fp, PNV_q_, pre, "TimeWeekCount",             Com->mIsTimeWeekCountValid,             Com->mTimeWeekCount);
	fprintf(fp, PNV_D_, pre, "TimeWeekSecond",            Com->mIsTimeWeekSecondValid,            Com->mTimeWeekSecond);
	fprintf(fp, PNV_I_, pre, "TimeUtcOffset",             Com->mIsTimeUtcOffsetValid,             Com->mTimeUtcOffset);

	// Position

	fprintf(fp, PNVXD_, pre, "Lat",                       Com->mIsLatValid,                       Com->mIsLatApprox,                      Com->mLat);
	fprintf(fp, PNVXD_, pre, "Lon",                       Com->mIsLonValid,                       Com->mIsLonApprox,                      Com->mLon);
	fprintf(fp, PNVXD_, pre, "Alt",                       Com->mIsAltValid,                       Com->mIsAltApprox,                      Com->mAlt);

	fprintf(fp, PNV_D_, pre, "NorthAcc",                  Com->mIsNorthAccValid,                  Com->mNorthAcc);
	fprintf(fp, PNV_D_, pre, "EastAcc",                   Com->mIsEastAccValid,                   Com->mEastAcc);
	fprintf(fp, PNV_D_, pre, "AltAcc",                    Com->mIsAltAccValid,                    Com->mAltAcc);

	// Distance

	fprintf(fp, PNV_D_, pre, "Dist2d",                    Com->mIsDist2dValid,                    Com->mDist2d);
	fprintf(fp, PNV_D_, pre, "Dist3d",                    Com->mIsDist3dValid,                    Com->mDist3d);

	// Velocity

	fprintf(fp, PNVXD_, pre, "Vn",                        Com->mIsVnValid,                        Com->mIsVnApprox,                       Com->mVn);
	fprintf(fp, PNVXD_, pre, "Ve",                        Com->mIsVeValid,                        Com->mIsVeApprox,                       Com->mVe);
	fprintf(fp, PNVXD_, pre, "Vd",                        Com->mIsVdValid,                        Com->mIsVdApprox,                       Com->mVd);

	fprintf(fp, PNV_D_, pre, "Vf",                        Com->mIsVfValid,                        Com->mVf);
	fprintf(fp, PNV_D_, pre, "Vl",                        Com->mIsVlValid,                        Com->mVl);

	fprintf(fp, "%s\t%1.16e\n", "Vx",                        Com->mVx);
	fprintf(fp, "%s\t%1.16e\n", "Vy",                        Com->mVy);
	fprintf(fp, "%s\t%1.16e\n", "Vz",                        Com->mVz);            //new


	fprintf(fp, PNV_D_, pre, "VnAcc",                     Com->mIsVnAccValid,                     Com->mVnAcc);
	fprintf(fp, PNV_D_, pre, "VeAcc",                     Com->mIsVeAccValid,                     Com->mVeAcc);
	fprintf(fp, PNV_D_, pre, "VdAcc",                     Com->mIsVdAccValid,                     Com->mVdAcc);

	// Speed

	fprintf(fp, PNV_D_, pre, "Speed2d",                   Com->mIsSpeed2dValid,                   Com->mSpeed2d);
	fprintf(fp, PNV_D_, pre, "Speed3d",                   Com->mIsSpeed3dValid,                   Com->mSpeed3d);

	// Acceleration

	fprintf(fp, PNV_D_, pre, "Ax",                        Com->mIsAxValid,                        Com->mAx);
	fprintf(fp, PNV_D_, pre, "Ay",                        Com->mIsAyValid,                        Com->mAy);
	fprintf(fp, PNV_D_, pre, "Az",                        Com->mIsAzValid,                        Com->mAz);

	fprintf(fp, PNV_D_, pre, "Af",                        Com->mIsAfValid,                        Com->mAf);
	fprintf(fp, PNV_D_, pre, "Al",                        Com->mIsAlValid,                        Com->mAl);
	fprintf(fp, PNV_D_, pre, "Ad",                        Com->mIsAdValid,                        Com->mAd);
	// Filtered acceleration

	fprintf(fp, PNV_D_, pre, "FiltAx",                    Com->mIsFiltAxValid,                    Com->mFiltAx);
	fprintf(fp, PNV_D_, pre, "FiltAy",                    Com->mIsFiltAyValid,                    Com->mFiltAy);
	fprintf(fp, PNV_D_, pre, "FiltAz",                    Com->mIsFiltAzValid,                    Com->mFiltAz);

	fprintf(fp, PNV_D_, pre, "FiltAf",                    Com->mIsFiltAfValid,                    Com->mFiltAf);
	fprintf(fp, PNV_D_, pre, "FiltAl",                    Com->mIsFiltAlValid,                    Com->mFiltAl);
	fprintf(fp, PNV_D_, pre, "FiltAd",                    Com->mIsFiltAdValid,                    Com->mFiltAd);

	// Orientation

	fprintf(fp, PNVXD_, pre, "Heading",                   Com->mIsHeadingValid,                   Com->mIsHeadingApprox,                  Com->mHeading);
	fprintf(fp, PNVXD_, pre, "Pitch",                     Com->mIsPitchValid,                     Com->mIsPitchApprox,                    Com->mPitch);
	fprintf(fp, PNVXD_, pre, "Roll",                      Com->mIsRollValid,                      Com->mIsRollApprox,                     Com->mRoll);

	fprintf(fp, PNV_D_, pre, "HeadingAcc",                Com->mIsHeadingAccValid,                Com->mHeadingAcc);
	fprintf(fp, PNV_D_, pre, "PitchAcc",                  Com->mIsPitchAccValid,                  Com->mPitchAcc);
	fprintf(fp, PNV_D_, pre, "RollAcc",                   Com->mIsRollAccValid,                   Com->mRollAcc);

	// Special

	fprintf(fp, PNV_D_, pre, "Track",                     Com->mIsTrackValid,                     Com->mTrack);
	fprintf(fp, PNV_D_, pre, "Slip",                      Com->mIsSlipValid,                      Com->mSlip);
	fprintf(fp, PNV_D_, pre, "Curvature",                 Com->mIsCurvatureValid,                 Com->mCurvature);

	// Angular rate

	fprintf(fp, PNV_D_, pre, "Wx",                        Com->mIsWxValid,                        Com->mWx);
	fprintf(fp, PNV_D_, pre, "Wy",                        Com->mIsWyValid,                        Com->mWy);
	fprintf(fp, PNV_D_, pre, "Wz",                        Com->mIsWzValid,                        Com->mWz);

	fprintf(fp, PNV_D_, pre, "Wf",                        Com->mIsWfValid,                        Com->mWf);
	fprintf(fp, PNV_D_, pre, "Wl",                        Com->mIsWlValid,                        Com->mWl);
	fprintf(fp, PNV_D_, pre, "Wd",                        Com->mIsWdValid,                        Com->mWd);

	// Angular acceleration

	fprintf(fp, PNV_D_, pre, "Yx",                        Com->mIsYxValid,                        Com->mYx);
	fprintf(fp, PNV_D_, pre, "Yy",                        Com->mIsYyValid,                        Com->mYy);
	fprintf(fp, PNV_D_, pre, "Yz",                        Com->mIsYzValid,                        Com->mYz);

	fprintf(fp, PNV_D_, pre, "Yf",                        Com->mIsYfValid,                        Com->mYf);
	fprintf(fp, PNV_D_, pre, "Yl",                        Com->mIsYlValid,                        Com->mYl);
	fprintf(fp, PNV_D_, pre, "Yd",                        Com->mIsYdValid,                        Com->mYd);

	// Filtered angular acceleration

	fprintf(fp, PNV_D_, pre, "FiltYx",                    Com->mIsFiltYxValid,                    Com->mFiltYx);
	fprintf(fp, PNV_D_, pre, "FiltYy",                    Com->mIsFiltYyValid,                    Com->mFiltYy);
	fprintf(fp, PNV_D_, pre, "FiltYz",                    Com->mIsFiltYzValid,                    Com->mFiltYz);

	fprintf(fp, PNV_D_, pre, "FiltYf",                    Com->mIsFiltYfValid,                    Com->mFiltYf);
	fprintf(fp, PNV_D_, pre, "FiltYl",                    Com->mIsFiltYlValid,                    Com->mFiltYl);
	fprintf(fp, PNV_D_, pre, "FiltYd",                    Com->mIsFiltYdValid,                    Com->mFiltYd);

	// Filter characteristics

	fprintf(fp, PNV_D_, pre, "LinAccFiltFreq",            Com->mIsLinAccFiltFreqValid,            Com->mLinAccFiltFreq);
	fprintf(fp, PNV_D_, pre, "LinAccFiltZeta",            Com->mIsLinAccFiltZetaValid,            Com->mLinAccFiltZeta);

	fprintf(fp, PNV_D_, pre, "AngAccFiltFreq",            Com->mIsAngAccFiltFreqValid,            Com->mAngAccFiltFreq);
	fprintf(fp, PNV_D_, pre, "AngAccFiltZeta",            Com->mIsAngAccFiltZetaValid,            Com->mAngAccFiltZeta);

	//--------------------------------------------------------------------------------------------------------
	// Model particulars

	// Innovations (discrepancy between GPS and IMU)

	fprintf(fp, PNA_D_, pre, "InnPosX",                   Com->mInnPosXAge,                       Com->mInnPosX);
	fprintf(fp, PNA_D_, pre, "InnPosY",                   Com->mInnPosYAge,                       Com->mInnPosY);
	fprintf(fp, PNA_D_, pre, "InnPosZ",                   Com->mInnPosZAge,                       Com->mInnPosZ);

	fprintf(fp, PNA_D_, pre, "InnVelX",                   Com->mInnVelXAge,                       Com->mInnVelX);
	fprintf(fp, PNA_D_, pre, "InnVelY",                   Com->mInnVelYAge,                       Com->mInnVelY);
	fprintf(fp, PNA_D_, pre, "InnVelZ",                   Com->mInnVelZAge,                       Com->mInnVelZ);

	fprintf(fp, PNA_D_, pre, "InnHeading",                Com->mInnHeadingAge,                    Com->mInnHeading);
	fprintf(fp, PNA_D_, pre, "InnPitch",                  Com->mInnPitchAge,                      Com->mInnPitch);

	// Gyroscope bias and scale factor

	fprintf(fp, PNV_D_, pre, "WxBias",                    Com->mIsWxBiasValid,                    Com->mWxBias);
	fprintf(fp, PNV_D_, pre, "WyBias",                    Com->mIsWyBiasValid,                    Com->mWyBias);
	fprintf(fp, PNV_D_, pre, "WzBias",                    Com->mIsWzBiasValid,                    Com->mWzBias);

	fprintf(fp, PNV_D_, pre, "WxBiasAcc",                 Com->mIsWxBiasAccValid,                 Com->mWxBiasAcc);
	fprintf(fp, PNV_D_, pre, "WyBiasAcc",                 Com->mIsWyBiasAccValid,                 Com->mWyBiasAcc);
	fprintf(fp, PNV_D_, pre, "WzBiasAcc",                 Com->mIsWzBiasAccValid,                 Com->mWzBiasAcc);

	fprintf(fp, PNV_D_, pre, "WxSf",                      Com->mIsWxSfValid,                      Com->mWxSf);
	fprintf(fp, PNV_D_, pre, "WySf",                      Com->mIsWySfValid,                      Com->mWySf);
	fprintf(fp, PNV_D_, pre, "WzSf",                      Com->mIsWzSfValid,                      Com->mWzSf);

	fprintf(fp, PNV_D_, pre, "WxSfAcc",                   Com->mIsWxSfAccValid,                   Com->mWxSfAcc);
	fprintf(fp, PNV_D_, pre, "WySfAcc",                   Com->mIsWySfAccValid,                   Com->mWySfAcc);
	fprintf(fp, PNV_D_, pre, "WzSfAcc",                   Com->mIsWzSfAccValid,                   Com->mWzSfAcc);

	// Accelerometer bias and scale factor

	fprintf(fp, PNV_D_, pre, "AxBias",                    Com->mIsAxBiasValid,                    Com->mAxBias);
	fprintf(fp, PNV_D_, pre, "AyBias",                    Com->mIsAyBiasValid,                    Com->mAyBias);
	fprintf(fp, PNV_D_, pre, "AzBias",                    Com->mIsAzBiasValid,                    Com->mAzBias);

	fprintf(fp, PNV_D_, pre, "AxBiasAcc",                 Com->mIsAxBiasAccValid,                 Com->mAxBiasAcc);
	fprintf(fp, PNV_D_, pre, "AyBiasAcc",                 Com->mIsAyBiasAccValid,                 Com->mAyBiasAcc);
	fprintf(fp, PNV_D_, pre, "AzBiasAcc",                 Com->mIsAzBiasAccValid,                 Com->mAzBiasAcc);

	fprintf(fp, PNV_D_, pre, "AxSf",                      Com->mIsAxSfValid,                      Com->mAxSf);
	fprintf(fp, PNV_D_, pre, "AySf",                      Com->mIsAySfValid,                      Com->mAySf);
	fprintf(fp, PNV_D_, pre, "AzSf",                      Com->mIsAzSfValid,                      Com->mAzSf);

	fprintf(fp, PNV_D_, pre, "AxSfAcc",                   Com->mIsAxSfAccValid,                   Com->mAxSfAcc);
	fprintf(fp, PNV_D_, pre, "AySfAcc",                   Com->mIsAySfAccValid,                   Com->mAySfAcc);
	fprintf(fp, PNV_D_, pre, "AzSfAcc",                   Com->mIsAzSfAccValid,                   Com->mAzSfAcc);

	// GPS antenna position

	fprintf(fp, PNV_D_, pre, "GAPx",                      Com->mIsGAPxValid,                      Com->mGAPx);
	fprintf(fp, PNV_D_, pre, "GAPy",                      Com->mIsGAPyValid,                      Com->mGAPy);
	fprintf(fp, PNV_D_, pre, "GAPz",                      Com->mIsGAPzValid,                      Com->mGAPz);

	fprintf(fp, PNV_D_, pre, "GAPxAcc",                   Com->mIsGAPxAccValid,                   Com->mGAPxAcc);
	fprintf(fp, PNV_D_, pre, "GAPyAcc",                   Com->mIsGAPyAccValid,                   Com->mGAPyAcc);
	fprintf(fp, PNV_D_, pre, "GAPzAcc",                   Com->mIsGAPzAccValid,                   Com->mGAPzAcc);

	fprintf(fp, PNV_D_, pre, "AtH",                       Com->mIsAtHValid,                       Com->mAtH);
	fprintf(fp, PNV_D_, pre, "AtP",                       Com->mIsAtPValid,                       Com->mAtP);

	fprintf(fp, PNV_D_, pre, "AtHAcc",                    Com->mIsAtHAccValid,                    Com->mAtHAcc);
	fprintf(fp, PNV_D_, pre, "AtPAcc",                    Com->mIsAtPAccValid,                    Com->mAtPAcc);

	fprintf(fp, PNVCD_, pre, "BaseLineLength",            Com->mIsBaseLineLengthValid,            Com->mIsBaseLineLengthConfig,           Com->mBaseLineLength);
	fprintf(fp, PNVCD_, pre, "BaseLineLengthAcc",         Com->mIsBaseLineLengthAccValid,         Com->mIsBaseLineLengthAccConfig,        Com->mBaseLineLengthAcc);

	//--------------------------------------------------------------------------------------------------------
	// Statistics

	// IMU hardware status event counters

	fprintf(fp, PNV_q_, pre, "ImuMissedPkts",             Com->mIsImuMissedPktsValid,             Com->mImuMissedPkts);
	fprintf(fp, PNV_q_, pre, "ImuResetCount",             Com->mIsImuResetCountValid,             Com->mImuResetCount);
	fprintf(fp, PNV_q_, pre, "ImuErrorCount",             Com->mIsImuErrorCountValid,             Com->mImuErrorCount);

	// GPS successive rejected aiding updates

	fprintf(fp, PNV_q_, pre, "GPSPosReject",              Com->mIsGPSPosRejectValid,              Com->mGPSPosReject);
	fprintf(fp, PNV_q_, pre, "GPSVelReject",              Com->mIsGPSVelRejectValid,              Com->mGPSVelReject);
	fprintf(fp, PNV_q_, pre, "GPSAttReject",              Com->mIsGPSAttRejectValid,              Com->mGPSAttReject);

	// Received data statistics

	fprintf(fp, PNV_q_, pre, "ImuChars",                  Com->mIsImuCharsValid,                  Com->mImuChars);
	fprintf(fp, PNV_q_, pre, "ImuCharsSkipped",           Com->mIsImuCharsSkippedValid,           Com->mImuCharsSkipped);
	fprintf(fp, PNV_q_, pre, "ImuPkts",                   Com->mIsImuPktsValid,                   Com->mImuPkts);

	fprintf(fp, PNV_q_, pre, "CmdChars",                  Com->mIsCmdCharsValid,                  Com->mCmdChars);
	fprintf(fp, PNV_q_, pre, "CmdCharsSkipped",           Com->mIsCmdCharsSkippedValid,           Com->mCmdCharsSkipped);
	fprintf(fp, PNV_q_, pre, "CmdPkts",                   Com->mIsCmdPktsValid,                   Com->mCmdPkts);
	fprintf(fp, PNV_q_, pre, "CmdErrors",                 Com->mIsCmdErrorsValid,                 Com->mCmdErrors);

	//--------------------------------------------------------------------------------------------------------
	// Transformation Euler angles

	// Orientation of vehicle-frame relative to IMU-frame

	fprintf(fp, PNV_D_, pre, "Imu2VehHeading",            Com->mIsImu2VehHeadingValid,            Com->mImu2VehHeading);
	fprintf(fp, PNV_D_, pre, "Imu2VehPitch",              Com->mIsImu2VehPitchValid,              Com->mImu2VehPitch);
	fprintf(fp, PNV_D_, pre, "Imu2VehRoll",               Com->mIsImu2VehRollValid,               Com->mImu2VehRoll);

	//--------------------------------------------------------------------------------------------------------
	// Miscellaneous items

	// Triggers

	fprintf(fp, PNVND_, pre, "TrigTime",                  Com->mIsTrigTimeValid,                  Com->mIsTrigTimeNew,                 Com->mTrigTime);
	fprintf(fp, PNVND_, pre, "Trig2Time",                 Com->mIsTrig2TimeValid,                 Com->mIsTrig2TimeNew,                Com->mTrig2Time);
	fprintf(fp, PNVND_, pre, "DigitalOutTime",            Com->mIsDigitalOutTimeValid,            Com->mIsDigitalOutTimeNew,           Com->mDigitalOutTime);

	// Remote lever arm

	fprintf(fp, PNV_D_, pre, "RemoteLeverArmX",           Com->mIsRemoteLeverArmXValid,           Com->mRemoteLeverArmX);
	fprintf(fp, PNV_D_, pre, "RemoteLeverArmY",           Com->mIsRemoteLeverArmYValid,           Com->mRemoteLeverArmY);
	fprintf(fp, PNV_D_, pre, "RemoteLeverArmZ",           Com->mIsRemoteLeverArmZValid,           Com->mRemoteLeverArmZ);

	// Local reference frame (definition)

	fprintf(fp, PNV_D_, pre, "RefLat",                    Com->mIsRefLatValid,                    Com->mRefLat);
	fprintf(fp, PNV_D_, pre, "RefLon",                    Com->mIsRefLonValid,                    Com->mRefLon);
	fprintf(fp, PNV_D_, pre, "RefAlt",                    Com->mIsRefAltValid,                    Com->mRefAlt);

	fprintf(fp, PNV_D_, pre, "RefHeading",                Com->mIsRefHeadingValid,                Com->mRefHeading);

	//--------------------------------------------------------------------------------------------------------
	// Zero velocity and advanced slip

	// Innovations

	fprintf(fp, PNA_D_, pre, "InnZeroVelX",               Com->mInnZeroVelXAge,                   Com->mInnZeroVelX);
	fprintf(fp, PNA_D_, pre, "InnZeroVelY",               Com->mInnZeroVelYAge,                   Com->mInnZeroVelY);
	fprintf(fp, PNA_D_, pre, "InnZeroVelZ",               Com->mInnZeroVelZAge,                   Com->mInnZeroVelZ);

	fprintf(fp, PNA_D_, pre, "InnNoSlipH",                Com->mInnNoSlipHAge,                    Com->mInnNoSlipH);

	// Lever arm options

	fprintf(fp, PNV_D_, pre, "ZeroVelLeverArmX",          Com->mIsZeroVelLeverArmXValid,          Com->mZeroVelLeverArmX);
	fprintf(fp, PNV_D_, pre, "ZeroVelLeverArmY",          Com->mIsZeroVelLeverArmYValid,          Com->mZeroVelLeverArmY);
	fprintf(fp, PNV_D_, pre, "ZeroVelLeverArmZ",          Com->mIsZeroVelLeverArmZValid,          Com->mZeroVelLeverArmZ);

	fprintf(fp, PNV_D_, pre, "ZeroVelLeverArmXAcc",       Com->mIsZeroVelLeverArmXAccValid,       Com->mZeroVelLeverArmXAcc);
	fprintf(fp, PNV_D_, pre, "ZeroVelLeverArmYAcc",       Com->mIsZeroVelLeverArmYAccValid,       Com->mZeroVelLeverArmYAcc);
	fprintf(fp, PNV_D_, pre, "ZeroVelLeverArmZAcc",       Com->mIsZeroVelLeverArmZAccValid,       Com->mZeroVelLeverArmZAcc);

	fprintf(fp, PNV_D_, pre, "NoSlipLeverArmX",           Com->mIsNoSlipLeverArmXValid,           Com->mNoSlipLeverArmX);
	fprintf(fp, PNV_D_, pre, "NoSlipLeverArmY",           Com->mIsNoSlipLeverArmYValid,           Com->mNoSlipLeverArmY);
	fprintf(fp, PNV_D_, pre, "NoSlipLeverArmZ",           Com->mIsNoSlipLeverArmZValid,           Com->mNoSlipLeverArmZ);

	fprintf(fp, PNV_D_, pre, "NoSlipLeverArmXAcc",        Com->mIsNoSlipLeverArmXAccValid,        Com->mNoSlipLeverArmXAcc);
	fprintf(fp, PNV_D_, pre, "NoSlipLeverArmYAcc",        Com->mIsNoSlipLeverArmYAccValid,        Com->mNoSlipLeverArmYAcc);
	fprintf(fp, PNV_D_, pre, "NoSlipLeverArmZAcc",        Com->mIsNoSlipLeverArmZAccValid,        Com->mNoSlipLeverArmZAcc);

	// Heading alignment

	fprintf(fp, PNV_D_, pre, "HeadingMisAlign",           Com->mIsHeadingMisAlignValid,           Com->mHeadingMisAlign);
	fprintf(fp, PNV_D_, pre, "HeadingMisAlignAcc",        Com->mIsHeadingMisAlignAccValid,        Com->mHeadingMisAlignAcc);

	// User Options

	fprintf(fp, PNVCD_, pre, "OptionSZVDelay",            Com->mIsOptionSZVDelayValid,            Com->mIsOptionSZVDelayConfig,           Com->mOptionSZVDelay);
	fprintf(fp, PNVCD_, pre, "OptionSZVPeriod",           Com->mIsOptionSZVPeriodValid,           Com->mIsOptionSZVPeriodConfig,          Com->mOptionSZVPeriod);

	fprintf(fp, PNVCD_, pre, "OptionNSDelay",             Com->mIsOptionNSDelayValid,             Com->mIsOptionNSDelayConfig,            Com->mOptionNSDelay);
	fprintf(fp, PNVCD_, pre, "OptionNSPeriod",            Com->mIsOptionNSPeriodValid,            Com->mIsOptionNSPeriodConfig,           Com->mOptionNSPeriod);
	fprintf(fp, PNVCD_, pre, "OptionNSAngleStd",          Com->mIsOptionNSAngleStdValid,          Com->mIsOptionNSAngleStdConfig,         Com->mOptionNSAngleStd);
	fprintf(fp, PNVCD_, pre, "OptionNSHAccel",            Com->mIsOptionNSHAccelValid,            Com->mIsOptionNSHAccelConfig,           Com->mOptionNSHAccel);
	fprintf(fp, PNVCD_, pre, "OptionNSVAccel",            Com->mIsOptionNSVAccelValid,            Com->mIsOptionNSVAccelConfig,           Com->mOptionNSVAccel);
	fprintf(fp, PNVCD_, pre, "OptionNSSpeed",             Com->mIsOptionNSSpeedValid,             Com->mIsOptionNSSpeedConfig,            Com->mOptionNSSpeed);
	fprintf(fp, PNVCD_, pre, "OptionNSRadius",            Com->mIsOptionNSRadiusValid,            Com->mIsOptionNSRadiusConfig,           Com->mOptionNSRadius);

	//--------------------------------------------------------------------------------------------------------
	// Wheel speed input

	// Innovations

	fprintf(fp, PNA_D_, pre, "InnWSpeed",                 Com->mInnWSpeedAge,                     Com->mInnWSpeed);

	// Wheel speed lever arm option

	fprintf(fp, PNV_D_, pre, "WSpeedLeverArmX",           Com->mIsWSpeedLeverArmXValid,           Com->mWSpeedLeverArmX);
	fprintf(fp, PNV_D_, pre, "WSpeedLeverArmY",           Com->mIsWSpeedLeverArmYValid,           Com->mWSpeedLeverArmY);
	fprintf(fp, PNV_D_, pre, "WSpeedLeverArmZ",           Com->mIsWSpeedLeverArmZValid,           Com->mWSpeedLeverArmZ);

	fprintf(fp, PNV_D_, pre, "WSpeedLeverArmXAcc",        Com->mIsWSpeedLeverArmXAccValid,        Com->mWSpeedLeverArmXAcc);
	fprintf(fp, PNV_D_, pre, "WSpeedLeverArmYAcc",        Com->mIsWSpeedLeverArmYAccValid,        Com->mWSpeedLeverArmYAcc);
	fprintf(fp, PNV_D_, pre, "WSpeedLeverArmZAcc",        Com->mIsWSpeedLeverArmZAccValid,        Com->mWSpeedLeverArmZAcc);

	// Wheel speed input model

	fprintf(fp, PNVCD_, pre, "WSpeedScale",               Com->mIsWSpeedScaleValid,               Com->mIsWSpeedScaleConfig,              Com->mWSpeedScale);
	fprintf(fp, PNVCD_, pre, "WSpeedScaleStd",            Com->mIsWSpeedScaleStdValid,            Com->mIsWSpeedScaleStdConfig,           Com->mWSpeedScaleStd);

	fprintf(fp, PNVCD_, pre, "OptionWSpeedDelay",         Com->mIsOptionWSpeedDelayValid,         Com->mIsOptionWSpeedDelayConfig,        Com->mOptionWSpeedDelay);
	fprintf(fp, PNVCD_, pre, "OptionWSpeedZVDelay",       Com->mIsOptionWSpeedZVDelayValid,       Com->mIsOptionWSpeedZVDelayConfig,      Com->mOptionWSpeedZVDelay);
	fprintf(fp, PNVCD_, pre, "OptionWSpeedNoiseStd",      Com->mIsOptionWSpeedNoiseStdValid,      Com->mIsOptionWSpeedNoiseStdConfig,     Com->mOptionWSpeedNoiseStd);

	// Wheel speed tacho measurements

	fprintf(fp, PNV_D_, pre, "WSpeedTime",                Com->mIsWSpeedTimeValid,                Com->mWSpeedTime);
	fprintf(fp, PNV_D_, pre, "WSpeedCount",               Com->mIsWSpeedCountValid,               Com->mWSpeedCount);
	fprintf(fp, PNV_D_, pre, "WSpeedTimeUnchanged",       Com->mIsWSpeedTimeUnchangedValid,       Com->mWSpeedTimeUnchanged);
	fprintf(fp, PNV_D_, pre, "WSpeedFreq",                Com->mIsWSpeedFreqValid,                Com->mWSpeedFreq);

	//--------------------------------------------------------------------------------------------------------
	// Heading lock

	// Innovations

	fprintf(fp, PNA_D_, pre, "InnHeadingH",               Com->mInnHeadingHAge,                   Com->mInnHeadingH);

	// User options

	fprintf(fp, PNVCD_, pre, "OptionHLDelay",             Com->mIsOptionHLDelayValid,             Com->mIsOptionHLDelayConfig,            Com->mOptionHLDelay);
	fprintf(fp, PNVCD_, pre, "OptionHLPeriod",            Com->mIsOptionHLPeriodValid,            Com->mIsOptionHLPeriodConfig,           Com->mOptionHLPeriod);
	fprintf(fp, PNVCD_, pre, "OptionHLAngleStd",          Com->mIsOptionHLAngleStdValid,          Com->mIsOptionHLAngleStdConfig,         Com->mOptionHLAngleStd);

	fprintf(fp, PNVCD_, pre, "OptionStatDelay",           Com->mIsOptionStatDelayValid,           Com->mIsOptionStatDelayConfig,          Com->mOptionStatDelay);
	fprintf(fp, PNVCD_, pre, "OptionStatSpeed",           Com->mIsOptionStatSpeedValid,           Com->mIsOptionStatSpeedConfig,          Com->mOptionStatSpeed);

	//--------------------------------------------------------------------------------------------------------
	// For use in testing

	// Reserved for testing

	fprintf(fp, PNV_I_, pre, "TimeMismatch",              Com->mIsTimeMismatchValid,              Com->mTimeMismatch);
	fprintf(fp, PNV_I_, pre, "ImuTimeDiff",               Com->mIsImuTimeDiffValid,               Com->mImuTimeDiff);
	fprintf(fp, PNV_I_, pre, "ImuTimeMargin",             Com->mIsImuTimeMarginValid,             Com->mImuTimeMargin);
	fprintf(fp, PNV_I_, pre, "ImuLoopTime",               Com->mIsImuLoopTimeValid,               Com->mImuLoopTime);
	fprintf(fp, PNV_I_, pre, "OpLoopTime",                Com->mIsOpLoopTimeValid,                Com->mOpLoopTime);

	fprintf(fp, PNV_I_, pre, "BnsLag",                    Com->mIsBnsLagValid,                    Com->mBnsLag);
	fprintf(fp, PNV_D_, pre, "BnsLagFilt",                Com->mIsBnsLagFiltValid,                Com->mBnsLagFilt);
}


//============================================================================================================
//! \brief Basic report to file.

void NComReport(const NComRxC *Com, const char *file_name,int append)
{
	FILE *fp = fopen(file_name, append ? "a" : "w");

	if (fp == NULL) return;

	NComReportFP(Com, fp, "");

	fclose(fp);
}


//============================================================================================================
//! \brief Constructor.

NComRxC *NComCreateNComRxC()
{
	NComRxC *Com = (NComRxC *)calloc(1, sizeof(NComRxC));

	if (Com == NULL) return NULL;

	Com->mInternal     = NComInternalCreate();
	Com->mGpsPrimary   = NComGpsCreate();
	Com->mGpsSecondary = NComGpsCreate();
	Com->mGpsExternal  = NComGpsCreate();

	if (Com->mInternal == NULL || Com->mGpsPrimary == NULL || Com->mGpsSecondary == NULL || Com->mGpsExternal == NULL)
	{
		NComDestroyNComRxC(Com);
		return NULL;
	};

	NComInvalidate(Com);
	return Com;
}


//============================================================================================================
//! \brief Destructor.

void NComDestroyNComRxC(NComRxC *Com)
{
	if (Com != NULL)
	{
		// Free the the internal space
		NComInternalDestroy(Com->mInternal);

		// Free the the GPS information
		NComGpsDestroy(Com->mGpsPrimary);
		NComGpsDestroy(Com->mGpsSecondary);
		NComGpsDestroy(Com->mGpsExternal);

		// Free the Com
		free(Com);
	}
}


//============================================================================================================
//! \brief Copy for NCom structure.

void NComCopy(NComRxC *ComDestination, const NComRxC *ComSource)
{
	// Keep track of pointers before mem copy.

	NComRxCGps *xGpsPrimary   = ComDestination->mGpsPrimary;
	NComRxCGps *xGpsSecondary = ComDestination->mGpsSecondary;
	NComRxCGps *xGpsExternal  = ComDestination->mGpsExternal;

	NComRxCInternal *xInternal = ComDestination->mInternal;

	// Copy this structure

	memcpy(ComDestination, ComSource, sizeof(NComRxC));

	// Recover pointers

	ComDestination->mGpsPrimary   = xGpsPrimary;
	ComDestination->mGpsSecondary = xGpsSecondary;
	ComDestination->mGpsExternal  = xGpsExternal;

	ComDestination->mInternal = xInternal;

	// Copy pointed to structures

	NComGpsCopy(ComDestination->mGpsPrimary,   ComSource->mGpsPrimary);
	NComGpsCopy(ComDestination->mGpsSecondary, ComSource->mGpsSecondary);
	NComGpsCopy(ComDestination->mGpsExternal,  ComSource->mGpsExternal);

	NComInternalCopy(ComDestination->mInternal, ComSource->mInternal);
}


//============================================================================================================
//! \brief Returns the version of NCOM CODE.

const char *NComDecoderVersionString()
{
	return NCOMRXC_DEV_ID ".NCOM";
}


//============================================================================================================
//! \brief Default file extension.

const char *NComFileDefaultExt(const NComRxC *Com)
{
	return "ncom";
}


//============================================================================================================
//! \brief File filter.

const char *NComFileFilter(const NComRxC *Com)
{
	return "NCOM Files (*.ncom)|*.ncom|All Files (*.*)|*.*";
}


//============================================================================================================
//! \brief Baud rate.

const int32_t NComBaudRate(const NComRxC *Com)
{
	return 115200;
}


//============================================================================================================
//! \brief Parse the incoming character for the packet.

ComResponse NComNewChar(NComRxC *Com, unsigned char c)
{
	return NComNewChars(Com, &c, 1);
}


//============================================================================================================
//! \brief Parse the incoming characters for the packet.

ComResponse NComNewChars(NComRxC *Com, const unsigned char *data, int num)
{
	int extra;
	ParsedPacketType pktStatus = PARSED_PACKET_INCOMPLETE;
	int offset = 0;

	NComRxCInternal *ComI = Com->mInternal;

	// If a processed packet is still in the buffer, remove it
	if (ComI->mPktProcessed)
	{
		RemoveFromBuffer(ComI, NOUTPUT_PACKET_LENGTH);
	}

	// If we have new data to add, and the buffer is full, get rid of
	// enough characters to accommodate the new characters
	// This is expensive, but should not happen often!
	if ((extra = ComI->mCurLen + num - NCOMRX_BUFFER_SIZE) > 0)
	{
		ComI->mCurLen = NCOMRX_BUFFER_SIZE - num;

		// Check that the amount of data passed is not beyond acceptable limits
		if (ComI->mCurLen > 0)
		{
			memmove(ComI->mCurPkt, ComI->mCurPkt + extra, ComI->mCurLen);
		}
		else if (ComI->mCurLen < 0) // (num > NCOMRX_BUFFER_SIZE) - i.e. too much data for the buffer
		{
			num += ComI->mCurLen; // Only accept as much data as the buffer size
			ComI->mCurLen = 0; // All previous data is lost
		}

		ComI->mSkippedChars += extra;
	}

	// Add the new characters (if available)
	if (num > 0)
	{
		memcpy(ComI->mCurPkt + ComI->mCurLen, data, num);
		ComI->mCurLen   += num;
		ComI->mNumChars += num;
	}

	// Check to see if we have a complete packet
	while (offset < ComI->mCurLen)
	{
		// Is there a valid packet at current offset?
		pktStatus = PktStatus(ComI->mCurLen - offset, ComI->mCurPkt + offset);

		// If we have found valid data
		if (pktStatus != PARSED_PACKET_CORRUPT)
			break; // Stop searching
		else
			offset++;
	}

	// If some corrupt data was found, realign the data
	if (offset > 0)
	{
		RemoveFromBuffer(ComI, offset);
	}

	// If a complete packet has been detected
	if (pktStatus == PARSED_PACKET_VALID)
	{
		// Extract data from the packet and update the navigation quantities
		UpdateNav(Com);

		// No packets so far then ignore the errors
		if (ComI->mNumPackets == 0)
			ComI->mSkippedChars = 0;

		// We've done a packet
		ComI->mNumPackets++;
		ComI->mPktProcessed = 1;

		// Indicate that a complete packet has been received
		return COM_NEW_UPDATE;
	}
	else
	{
		// Indicate that no complete packet has been received
		return COM_NO_UPDATE;
	}
}


//============================================================================================================
//! \brief Number of characters.

uint64_t NComNumChars(const NComRxC *Com)
{
	return (Com != NULL && Com->mInternal != NULL) ? Com->mInternal->mNumChars : 0;
}


//============================================================================================================
//! \brief Skipped characters.

uint64_t NComSkippedChars(const NComRxC *Com)
{
	return (Com != NULL && Com->mInternal != NULL) ? Com->mInternal->mSkippedChars : 0;
}


//============================================================================================================
//! \brief Number of packets.

uint64_t NComNumPackets(const NComRxC *Com)
{
	return (Com != NULL && Com->mInternal != NULL) ? Com->mInternal->mNumPackets : 0;
}


//============================================================================================================
//! \brief Update innovation age.

void NComUpdateInnAge(NComRxC *Com)
{
	if (Com->mInnPosXAge     < MAX_INN_AGE) Com->mInnPosXAge++;
	if (Com->mInnPosYAge     < MAX_INN_AGE) Com->mInnPosYAge++;
	if (Com->mInnPosZAge     < MAX_INN_AGE) Com->mInnPosZAge++;

	if (Com->mInnVelXAge     < MAX_INN_AGE) Com->mInnVelXAge++;
	if (Com->mInnVelYAge     < MAX_INN_AGE) Com->mInnVelYAge++;
	if (Com->mInnVelZAge     < MAX_INN_AGE) Com->mInnVelZAge++;

	if (Com->mInnHeadingAge  < MAX_INN_AGE) Com->mInnHeadingAge++;
	if (Com->mInnPitchAge    < MAX_INN_AGE) Com->mInnPitchAge++;

	if (Com->mInnZeroVelXAge < MAX_INN_AGE) Com->mInnZeroVelXAge++;
	if (Com->mInnZeroVelYAge < MAX_INN_AGE) Com->mInnZeroVelYAge++;
	if (Com->mInnZeroVelZAge < MAX_INN_AGE) Com->mInnZeroVelZAge++;

	if (Com->mInnNoSlipHAge  < MAX_INN_AGE) Com->mInnNoSlipHAge++;
	if (Com->mInnHeadingHAge < MAX_INN_AGE) Com->mInnHeadingHAge++;
	if (Com->mInnWSpeedAge   < MAX_INN_AGE) Com->mInnWSpeedAge++;
}


//============================================================================================================
//! \brief Interpolate between packets.
//!
//! \todo This needs to be reviewed in light of all available data members.

void NComInterpolate(NComRxC *Com, double a, const NComRxC *A, double b, const NComRxC *B)
{
	NComInvalidate(Com);

	if (A->mIsTimeValid && B->mIsTimeValid) NComSetTime(Com, a * A->mTime + b * B->mTime);
	if (A->mIsLatValid  && B->mIsLatValid ) NComSetLat (Com, a * A->mLat  + b * B->mLat );
	if (A->mIsLonValid  && B->mIsLonValid ) NComSetLon (Com, a * A->mLon  + b * B->mLon );
	if (A->mIsAltValid  && B->mIsAltValid ) NComSetAlt (Com, a * A->mAlt  + b * B->mAlt );
	if (A->mIsVnValid   && B->mIsVnValid  ) NComSetVn  (Com, a * A->mVn   + b * B->mVn  );
	if (A->mIsVeValid   && B->mIsVeValid  ) NComSetVe  (Com, a * A->mVe   + b * B->mVe  );
	if (A->mIsVdValid   && B->mIsVdValid  ) NComSetVd  (Com, a * A->mVd   + b * B->mVd  );

	if (A->mIsHeadingValid && B->mIsHeadingValid)
	{
		double d = A->mHeading - B->mHeading;
		if (d > 180.0)
			d -= 360.0;
		else if (d < -180.0)
			d += 360.0;
		d = A->mHeading + b * d;
		if (d < 0.0)
			d += 360.0;
		else if (d > 360.0)
			d -= 360.0;
		NComSetHeading(Com, d);
	}

	if (A->mIsPitchValid   && B->mIsPitchValid  ) NComSetPitch  (Com, a * A->mPitch   + b * B->mPitch  );
	if (A->mIsRollValid    && B->mIsRollValid   ) NComSetRoll   (Com, a * A->mRoll    + b * B->mRoll   );
	if (A->mIsDist2dValid  && B->mIsDist2dValid ) NComSetDist2d (Com, a * A->mDist2d  + b * B->mDist2d );
	if (A->mIsDist3dValid  && B->mIsDist3dValid ) NComSetDist3d (Com, a * A->mDist3d  + b * B->mDist3d );
	if (A->mIsVfValid      && B->mIsVfValid     ) NComSetVf     (Com, a * A->mVf      + b * B->mVf     );
	if (A->mIsVlValid      && B->mIsVlValid     ) NComSetVl     (Com, a * A->mVl      + b * B->mVl     );
	if (A->mIsSpeed2dValid && B->mIsSpeed2dValid) NComSetSpeed2d(Com, a * A->mSpeed2d + b * B->mSpeed2d);
	if (A->mIsSpeed3dValid && B->mIsSpeed3dValid) NComSetSpeed3d(Com, a * A->mSpeed3d + b * B->mSpeed3d);
	if (A->mIsAxValid      && B->mIsAxValid     ) NComSetAx     (Com, a * A->mAx      + b * B->mAx     );
	if (A->mIsAyValid      && B->mIsAyValid     ) NComSetAy     (Com, a * A->mAy      + b * B->mAy     );
	if (A->mIsAzValid      && B->mIsAzValid     ) NComSetAz     (Com, a * A->mAz      + b * B->mAz     );
	if (A->mIsAfValid      && B->mIsAfValid     ) NComSetAf     (Com, a * A->mAf      + b * B->mAf     );
	if (A->mIsAlValid      && B->mIsAlValid     ) NComSetAl     (Com, a * A->mAl      + b * B->mAl     );
	if (A->mIsAdValid      && B->mIsAdValid     ) NComSetAd     (Com, a * A->mAd      + b * B->mAd     );
	if (A->mIsWxValid      && B->mIsWxValid     ) NComSetWx     (Com, a * A->mWx      + b * B->mWx     );
	if (A->mIsWyValid      && B->mIsWyValid     ) NComSetWy     (Com, a * A->mWy      + b * B->mWy     );
	if (A->mIsWzValid      && B->mIsWzValid     ) NComSetWz     (Com, a * A->mWz      + b * B->mWz     );
	if (A->mIsWfValid      && B->mIsWfValid     ) NComSetWf     (Com, a * A->mWf      + b * B->mWf     );
	if (A->mIsWlValid      && B->mIsWlValid     ) NComSetWl     (Com, a * A->mWl      + b * B->mWl     );
	if (A->mIsWdValid      && B->mIsWdValid     ) NComSetWd     (Com, a * A->mWd      + b * B->mWd     );
	if (A->mIsYxValid      && B->mIsYxValid     ) NComSetYx     (Com, a * A->mYx      + b * B->mYx     );
	if (A->mIsYyValid      && B->mIsYyValid     ) NComSetYy     (Com, a * A->mYy      + b * B->mYy     );
	if (A->mIsYzValid      && B->mIsYzValid     ) NComSetYz     (Com, a * A->mYz      + b * B->mYz     );
	if (A->mIsYfValid      && B->mIsYfValid     ) NComSetYf     (Com, a * A->mYf      + b * B->mYf     );
	if (A->mIsYlValid      && B->mIsYlValid     ) NComSetYl     (Com, a * A->mYl      + b * B->mYl     );
	if (A->mIsYdValid      && B->mIsYdValid     ) NComSetYd     (Com, a * A->mYd      + b * B->mYd     );

	if (A->mIsSlipValid && B->mIsSlipValid)
	{
		double d = A->mSlip - B->mSlip;
		if (d > 180.0)
			d -= 360.0;
		else if (d < -180.0)
			d += 360.0;
		d = A->mSlip + b * d;
		if (d < 0.0)
			d += 360.0;
		else if (d > 360.0)
			d -= 360.0;
		NComSetSlip(Com, d);
	}

	if (A->mIsFiltAxValid  && B->mIsFiltAxValid ) NComSetFiltAx (Com, a * A->mFiltAx  + b * B->mFiltAx );
	if (A->mIsFiltAyValid  && B->mIsFiltAyValid ) NComSetFiltAy (Com, a * A->mFiltAy  + b * B->mFiltAy );
	if (A->mIsFiltAzValid  && B->mIsFiltAzValid ) NComSetFiltAz (Com, a * A->mFiltAz  + b * B->mFiltAz );
	if (A->mIsFiltAfValid  && B->mIsFiltAfValid ) NComSetFiltAf (Com, a * A->mFiltAf  + b * B->mFiltAf );
	if (A->mIsFiltAlValid  && B->mIsFiltAlValid ) NComSetFiltAl (Com, a * A->mFiltAl  + b * B->mFiltAl );
	if (A->mIsFiltAdValid  && B->mIsFiltAdValid ) NComSetFiltAd (Com, a * A->mFiltAd  + b * B->mFiltAd );
	if (A->mIsFiltYxValid  && B->mIsFiltYxValid ) NComSetFiltYx (Com, a * A->mFiltYx  + b * B->mFiltYx );
	if (A->mIsFiltYyValid  && B->mIsFiltYyValid ) NComSetFiltYy (Com, a * A->mFiltYy  + b * B->mFiltYy );
	if (A->mIsFiltYzValid  && B->mIsFiltYzValid ) NComSetFiltYz (Com, a * A->mFiltYz  + b * B->mFiltYz );
	if (A->mIsFiltYfValid  && B->mIsFiltYfValid ) NComSetFiltYf (Com, a * A->mFiltYf  + b * B->mFiltYf );
	if (A->mIsFiltYlValid  && B->mIsFiltYlValid ) NComSetFiltYl (Com, a * A->mFiltYl  + b * B->mFiltYl );
	if (A->mIsFiltYdValid  && B->mIsFiltYdValid ) NComSetFiltYd (Com, a * A->mFiltYd  + b * B->mFiltYd );

	if (A->mIsVnAccValid && B->mIsVnAccValid) NComSetVnAcc(Com, a * A->mVnAcc + b * B->mVnAcc);
	if (A->mIsVeAccValid && B->mIsVeAccValid) NComSetVeAcc(Com, a * A->mVeAcc + b * B->mVeAcc);
	if (A->mIsVdAccValid && B->mIsVdAccValid) NComSetVdAcc(Com, a * A->mVdAcc + b * B->mVdAcc);
}




//############################################################################################################
//##                                                                                                        ##
//##  NComRxC - Private Functions                                                                           ##
//##                                                                                                        ##
//############################################################################################################


//============================================================================================================
//! \brief Check packet validity.

static ParsedPacketType PktStatus(size_t Len, const unsigned char *p)
{
	int i;
	unsigned char csum3;

	// Has a complete header arrived?
	if (Len < 1)
		return PARSED_PACKET_INCOMPLETE;

	// Next check the SYNC character
	if (p[0] != NCOM_SYNC)
		return PARSED_PACKET_CORRUPT;

	// Find out if all of the packet has arrived
	if (Len < NOUTPUT_PACKET_LENGTH)
		return PARSED_PACKET_INCOMPLETE;

	// Evaluate the checksum
	for (i = 1, csum3 = 0; i < (NOUTPUT_PACKET_LENGTH - 1); i++)
		csum3 += p[i];

	// Verify the checksum
	if (csum3 != p[NOUTPUT_PACKET_LENGTH - 1])
		return PARSED_PACKET_CORRUPT;

	// If we have got here then there is a valid packet!
	return PARSED_PACKET_VALID;
}


//============================================================================================================
//! \brief Removes n characters from the input buffer.

static void RemoveFromBuffer(NComRxCInternal *Com, int n)
{
	// Discard the first n characters from the buffer
	Com->mCurLen -= n;

	// Realign the data in the buffer
	if (Com->mCurLen > 0)
	{
		memmove(Com->mCurPkt, Com->mCurPkt + n, Com->mCurLen);
	}
	else if (Com->mCurLen < 0)
	{
		// Adjust the number of characters to be rejected
		n += Com->mCurLen;
		Com->mCurLen = 0;
	}

	// Update the housekeeping parameters
	if (Com->mPktProcessed)
		Com->mPktProcessed = 0;
	else
		Com->mSkippedChars += n;
}


//============================================================================================================
//! \brief Test how recent the development identification is.

static char IsDevIDAfter(NComRxC *Com, char *datestr)
{
	if (Com->mIsDevIdValid)
	{
		return (strcmp(Com->mDevId, datestr) >= 0);
	}
	else
	{
		return 0;
	}
}


//============================================================================================================
//! \brief Update navigation helper function.
//!
//! Clear some of the validity flags. The intention of this is to invalidate only the data that gets
//! updated rapidly. This data will be "raw" data from the NCom packet, and any data derived from it (such
//! as rotated outputs). It is not intended that any slowly changing data (such as approximate values,
//! accuracies, innovations) or data that persists across packets (such as configuration information) be
//! invalidated. Note: depending on triggers being used, we still need to call this even though we may not
//! recompute the data.

static void UpdateNavInvalidate(NComRxC *Com)
{
	// Invalidate fundamental data.
	Com->mIsTimeValid                                                = 0;  // Time
	Com->mIsAxValid       = Com->mIsAyValid     = Com->mIsAzValid    = 0;  // Acceleration.
	Com->mIsWxValid       = Com->mIsWyValid     = Com->mIsWzValid    = 0;  // Angular rate.
	Com->mIsLatValid      = Com->mIsLonValid    = Com->mIsAltValid   = 0;  // Position.
	Com->mIsVnValid       = Com->mIsVeValid     = Com->mIsVdValid    = 0;  // Velocity.
	Com->mIsHeadingValid  = Com->mIsPitchValid  = Com->mIsRollValid  = 0;  // Orientation.

#if 0
	// Invalidate approximate fundamental data.
	Com->mIsLatApprox     = Com->mIsLonApprox   = Com->mIsAltApprox  = 0;  // Position.
	Com->mIsVnApprox      = Com->mIsVeApprox    = Com->mIsVdApprox   = 0;  // Velocity.
	Com->mIsHeadingApprox = Com->mIsPitchApprox = Com->mIsRollApprox = 0;  // Orientation.
#endif

	// Invalidate high rate derived data only.
	Com->mIsTimeWeekCountValid  = 0;
	Com->mIsTimeWeekSecondValid = 0;
	Com->mIsYxValid       = Com->mIsYyValid     = Com->mIsYzValid    = 0;  // Angular acceleration.
	FilteredOutputsInvalidate(Com);
	RotateOutputsInvalidate(Com);
	SpeedSlipInvalidate(Com);
	DistanceInvalidate(Com);
}


//============================================================================================================
//! \brief Update navigation with a legal packet.

 void UpdateNav(NComRxC *Com)
{
	int32_t x, y, z;

	// Easy access to internal values.
	NComRxCInternal *ComI = Com->mInternal;

	// Easy access to current packet buffer.
	const unsigned char *mCurPkt = ComI->mCurPkt;

	// Flag used to distinguish between regular and trigger processing
	char trig = 0;

	// Should not be necessary, here for safety.
	NComSetOutputPacketTypeEnum(Com, OUTPUT_PACKET_INVALID);

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// Navigation Mode

	NComSetInsNavModeEnum(Com, mCurPkt[PI_INS_NAV_MODE]);

	switch (Com->mInsNavMode)
	{
		// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
		// This first block tend to be trivial decode and return out of this function

		case NAVIGATION_STATUS_NOTHING     : // Supported, but no useful data so everything invalid.
		{
			UpdateNavInvalidate(Com);
			NComSetOutputPacketTypeEnum(Com, OUTPUT_PACKET_EMPTY);
			return;
		}
		break;

		case NAVIGATION_STATUS_EXPIRED     : // If firmware has expired then no useful data to extract.
		{
			UpdateNavInvalidate(Com);
			NComSetOutputPacketTypeEnum(Com, OUTPUT_PACKET_EMPTY);
			return;
		}
		break;

		case NAVIGATION_STATUS_STATUSONLY  : // Update only status info.
		{
			UpdateNavInvalidate(Com);
			DecodeStatusMsg(Com);
			NComSetOutputPacketTypeEnum(Com, OUTPUT_PACKET_STATUS);
			return;
		}
		break;

		// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
		// This second block tend to be non-trivial decode and continue through this function

		case NAVIGATION_STATUS_RAWIMU      : // Regular style packets
		case NAVIGATION_STATUS_INIT        :
		case NAVIGATION_STATUS_LOCKING     :
		case NAVIGATION_STATUS_LOCKED      :
		{
			UpdateNavInvalidate(Com);
			NComSetOutputPacketTypeEnum(Com, OUTPUT_PACKET_REGULAR);
		}
		break;

		case NAVIGATION_STATUS_TRIGINIT    : // Trigger before full init.
		case NAVIGATION_STATUS_TRIGLOCKING : // Trigger whilst locking.
		case NAVIGATION_STATUS_TRIGLOCKED  : // Trigger in real-time mode.
		{
			UpdateNavInvalidate(Com);
			// Indicate that we are decoding a trigger
			trig = 1;
			// Decode the status message now, as it contains trigger time
			DecodeStatusMsg(Com);
			// Record exact time (from trigger time) and trigger type
			switch (mCurPkt[PI_CHANNEL_INDEX])
			{
				case  24:
					NComSetTime(Com, Com->mTrigTime);
					NComSetOutputPacketTypeEnum(Com, OUTPUT_PACKET_IN1DOWN);
					break;
				case  43:
					NComSetTime(Com, Com->mTrig2Time);
					NComSetOutputPacketTypeEnum(Com, OUTPUT_PACKET_IN1UP);
					break;
				case  65:
					NComSetTime(Com, Com->mDigitalOutTime);
					NComSetOutputPacketTypeEnum(Com, OUTPUT_PACKET_OUT1);
					break;
				default:
					NComSetOutputPacketTypeEnum(Com, OUTPUT_PACKET_EMPTY);
					return;
			}
			// Reset the navigation mode appropriately
			switch (Com->mInsNavMode)
			{
				case NAVIGATION_STATUS_TRIGINIT    : NComSetInsNavModeEnum(Com, NAVIGATION_STATUS_INIT   ); break;
				case NAVIGATION_STATUS_TRIGLOCKING : NComSetInsNavModeEnum(Com, NAVIGATION_STATUS_LOCKING); break;
				case NAVIGATION_STATUS_TRIGLOCKED  : NComSetInsNavModeEnum(Com, NAVIGATION_STATUS_LOCKED ); break;
				default : break;
			}
      }
		break;

		// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
		// If we get here we pretend the packet did not happen (other than nav mode being recorded)

		default :
		{
			NComSetOutputPacketTypeEnum(Com, OUTPUT_PACKET_EMPTY);
			return;
		}
		break;
	}

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// Can decode acceleration and angular rate.

	x = cast_3_byte_to_int32(mCurPkt+PI_ACCEL_X);
	y = cast_3_byte_to_int32(mCurPkt+PI_ACCEL_Y);
	z = cast_3_byte_to_int32(mCurPkt+PI_ACCEL_Z);

	if (x != INV_INT_24) NComSetAx(Com, x * ACC2MPS2);
	if (y != INV_INT_24) NComSetAy(Com, y * ACC2MPS2);
	if (z != INV_INT_24) NComSetAz(Com, z * ACC2MPS2);

	x = cast_3_byte_to_int32(mCurPkt+PI_ANG_RATE_X);
	y = cast_3_byte_to_int32(mCurPkt+PI_ANG_RATE_Y);
	z = cast_3_byte_to_int32(mCurPkt+PI_ANG_RATE_Z);

	if (x != INV_INT_24) NComSetWx(Com, x * (RATE2RPS * RAD2DEG));
	if (y != INV_INT_24) NComSetWy(Com, y * (RATE2RPS * RAD2DEG));
	if (z != INV_INT_24) NComSetWz(Com, z * (RATE2RPS * RAD2DEG));

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// Timing.

	if ((trig == 0) && (Com->mInsNavMode > NAVIGATION_STATUS_RAWIMU))
	{
		int32_t ms = cast_2_byte_to_uint16(mCurPkt+PI_TIME);

		// Check if seconds have wrapped.
		if ((ms < INT32_C(2000)) && (ComI->mMilliSecs > INT32_C(2000)) && (ComI->mMinutes >= INT32_C(0)))
		{
			ComI->mMinutes++;
		}

		ComI->mMilliSecs = ms;

		if (ComI->mMinutes >= INT32_C(0))
		{
			// Update time stamp.
			NComSetTime(Com, ((double) ComI->mMinutes) * 60.0 + ((double) ComI->mMilliSecs) * 0.001);

			// Also store the GPS time for good measure.
			NComSetTimeWeekCount (Com,           ComI->mMinutes / MINUTES_IN_WEEK);
			NComSetTimeWeekSecond(Com, ((double)(ComI->mMinutes % MINUTES_IN_WEEK)) * 60.0 + ((double) ComI->mMilliSecs) * 0.001);
		}
	}

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// Compute some navigation measurements.

	if (Com->mInsNavMode >= NAVIGATION_STATUS_INIT)
	{
		// Decode basic navigation measurements

		NComSetLat(Com, cast_8_byte_to_double(mCurPkt+PI_POS_LAT) * RAD2DEG);
		NComSetLon(Com, cast_8_byte_to_double(mCurPkt+PI_POS_LON) * RAD2DEG);
		NComSetAlt(Com, cast_4_byte_to_float (mCurPkt+PI_POS_ALT)          );

		x = cast_3_byte_to_int32(mCurPkt+PI_VEL_N);
		y = cast_3_byte_to_int32(mCurPkt+PI_VEL_E);
		z = cast_3_byte_to_int32(mCurPkt+PI_VEL_D);

		if (x != INV_INT_24) NComSetVn(Com, x * VEL2MPS);
		if (y != INV_INT_24) NComSetVe(Com, y * VEL2MPS);
		if (z != INV_INT_24) NComSetVd(Com, z * VEL2MPS);

		x = cast_3_byte_to_int32(mCurPkt+PI_ORIEN_H);
		y = cast_3_byte_to_int32(mCurPkt+PI_ORIEN_P);
		z = cast_3_byte_to_int32(mCurPkt+PI_ORIEN_R);

		if (x != INV_INT_24) { NComSetHeading(Com, x * (ANG2RAD * RAD2DEG)); if (Com->mHeading < 0.0) NComSetHeading(Com, Com->mHeading + 360.0); }
		if (y != INV_INT_24)   NComSetPitch  (Com, y * (ANG2RAD * RAD2DEG));
		if (z != INV_INT_24)   NComSetRoll   (Com, z * (ANG2RAD * RAD2DEG));

		// If we're in approximate mode then flag so.
		if (Com->mInsNavMode == NAVIGATION_STATUS_INIT)
		{
			if (Com->mIsLatValid    ) { Com->mIsLatValid     = 0; Com->mIsLatApprox     = 1; }
			if (Com->mIsLonValid    ) { Com->mIsLonValid     = 0; Com->mIsLonApprox     = 1; }
			if (Com->mIsAltValid    ) { Com->mIsAltValid     = 0; Com->mIsAltApprox     = 1; }

			if (Com->mIsVnValid     ) { Com->mIsVnValid      = 0; Com->mIsVnApprox      = 1; }
			if (Com->mIsVeValid     ) { Com->mIsVeValid      = 0; Com->mIsVeApprox      = 1; }
			if (Com->mIsVdValid     ) { Com->mIsVdValid      = 0; Com->mIsVdApprox      = 1; }

			if (Com->mIsHeadingValid) { Com->mIsHeadingValid = 0; Com->mIsHeadingApprox = 1; }
			if (Com->mIsPitchValid  ) { Com->mIsPitchValid   = 0; Com->mIsPitchApprox   = 1; }
			if (Com->mIsRollValid   ) { Com->mIsRollValid    = 0; Com->mIsRollApprox    = 1; }
		}
	}

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// Compute derived angular accelerations.

	if ((trig == 0) && Com->mIsWxValid && Com->mIsWyValid && Com->mIsWzValid && Com->mIsTimeValid)
	{
		// Differentiate angular rates to get angular accelerations

		double dt = Com->mTime - ComI->mPrevWbTime;
		if (dt > 0.0)
		{
			NComSetYx(Com, (Com->mWx - ComI->mPrevWx)/dt);
			NComSetYy(Com, (Com->mWy - ComI->mPrevWy)/dt);
			NComSetYz(Com, (Com->mWz - ComI->mPrevWz)/dt);
		}

		// Record values for next time round

		ComI->mPrevWx     = Com->mWx;
		ComI->mPrevWy     = Com->mWy;
		ComI->mPrevWz     = Com->mWz;
		ComI->mPrevWbTime = Com->mTime;
	}

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// Compute derived quantities

	if (trig == 0)
	{
		FilteredOutputsCompute(Com);
	}

	if (Com->mInsNavMode > NAVIGATION_STATUS_INIT)
	{
		if (!ComI->mMatrixHold)
		{
			RotateOutputsCompute(Com);    // Rotated quantities
		}

		SpeedSlipCompute(Com);            // Compute speed and slip
		DistanceCompute(Com, trig);       // Compute distance travelled
	}

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// Triggers

	if (trig == 0)
	{
		NComUpdateInnAge(Com);
		DecodeStatusMsg(Com);
	}
}


//============================================================================================================
//! \brief Invalidate all variables that may be computed by filtered outputs.

static void FilteredOutputsInvalidate(NComRxC *Com)
{
	Com->mIsFiltAxValid = Com->mIsFiltAyValid = Com->mIsFiltAzValid = 0; // Filtered accelerations.
	Com->mIsFiltYxValid = Com->mIsFiltYyValid = Com->mIsFiltYzValid = 0; // Filtered angular accelerYtions.
}


//============================================================================================================
//! \brief Filtered outputs.

static void FilteredOutputsCompute(NComRxC *Com)
{
	if (Com->mIsAxValid && Com->mIsAyValid && Com->mIsAzValid && Com->mIsTimeValid)
	{
		NComRxCInternal *ComI = Com->mInternal;

		// Pass valid raw accelerations to the respective filter (if appropriate)

		if (Com->mIsLinAccFiltOff)
		{
			NComSetFiltAx(Com, Com->mAx);
		}
		else
		{
			Filt2ndOrderNewInput(&ComI->FiltForAx, Com->mTime, Com->mAx);
			if (ComI->FiltForAx.mOutputValid)
				NComSetFiltAx(Com, ComI->FiltForAx.mU0);
		}

		if (Com->mIsLinAccFiltOff)
		{
			NComSetFiltAy(Com, Com->mAy);
		}
		else
		{
			Filt2ndOrderNewInput(&ComI->FiltForAy, Com->mTime, Com->mAy);
			if (ComI->FiltForAy.mOutputValid)
				NComSetFiltAy(Com, ComI->FiltForAy.mU0);
		}

		if (Com->mIsLinAccFiltOff)
		{
			NComSetFiltAz(Com, Com->mAz);
		}
		else
		{
			Filt2ndOrderNewInput(&ComI->FiltForAz, Com->mTime, Com->mAz);
			if (ComI->FiltForAz.mOutputValid)
				NComSetFiltAz(Com, ComI->FiltForAz.mU0);
		}
	}

	if (Com->mIsYxValid && Com->mIsYyValid && Com->mIsYzValid && Com->mIsTimeValid)
	{
		NComRxCInternal *ComI = Com->mInternal;

		// Pass valid raw angular accelerations to the respective filter (if appropriate)

		if (Com->mIsAngAccFiltOff)
		{
			NComSetFiltYx(Com, Com->mYx);
		}
		else
		{
			Filt2ndOrderNewInput(&ComI->FiltForYx, Com->mTime, Com->mYx);
			if (ComI->FiltForYx.mOutputValid)
				NComSetFiltYx(Com, ComI->FiltForYx.mU0);
		}

		if (Com->mIsAngAccFiltOff)
		{
			NComSetFiltYy(Com, Com->mYy);
		}
		else
		{
			Filt2ndOrderNewInput(&ComI->FiltForYy, Com->mTime, Com->mYy);
			if (ComI->FiltForYy.mOutputValid)
				NComSetFiltYy(Com, ComI->FiltForYy.mU0);
		}

		if (Com->mIsAngAccFiltOff)
		{
			NComSetFiltYz(Com, Com->mYz);
		}
		else
		{
			Filt2ndOrderNewInput(&ComI->FiltForYz, Com->mTime, Com->mYz);
			if (ComI->FiltForYz.mOutputValid)
				NComSetFiltYz(Com, ComI->FiltForYz.mU0);
		}
	}
}


//============================================================================================================
//! \brief Invalidate all variables that may be computed by rotate outputs.

static void RotateOutputsInvalidate(NComRxC *Com)
{
	Com->mIsAfValid    = Com->mIsAlValid    = Com->mIsAdValid    = 0; // Acceleration.
	Com->mIsWfValid    = Com->mIsWlValid    = Com->mIsWdValid    = 0; // Angular rate.
	Com->mIsVfValid    = Com->mIsVlValid                         = 0; // Velocity.
	Com->mIsYfValid    = Com->mIsYlValid    = Com->mIsYdValid    = 0; // Angular acceleration.

	Com->mIsFiltAfValid = Com->mIsFiltAlValid = Com->mIsFiltAdValid = 0; // Filtered acceleration.
	Com->mIsFiltYfValid = Com->mIsFiltYlValid = Com->mIsFiltYdValid = 0; // Filtered angular acceleration.
}


//============================================================================================================
//! \brief Rotate outputs.

static void RotateOutputsCompute(NComRxC *Com)
{
	if (Com->mIsHeadingValid && Com->mIsPitchValid && Com->mIsRollValid)
	{
		NComRxCInternal *ComI = Com->mInternal;

		MatFillR(&ComI->E, 3, 1, Com->mHeading * DEG2RAD, Com->mPitch * DEG2RAD, Com->mRoll * DEG2RAD);

		Euler2DirCos2(&ComI->Cb, &ComI->E);  // P/R rotation from body
		Euler2DirCos2_1(&ComI->Cb_1, &ComI->E);       //new  P/R rotation from level
		Euler2DirCosH(&ComI->Ch, &ComI->E);  // Heading rotation from NED

		// Acceleration in FLD frame.
		if (Com->mIsAxValid && Com->mIsAyValid && Com->mIsAzValid)
		{
			MatFillR(&ComI->Ab, 3, 1, Com->mAx, Com->mAy, Com->mAz);
			MatMultRAB(&ComI->Al, &ComI->Cb, &ComI->Ab);
			NComSetAf(Com, e(&ComI->Al, 0, 0));
			NComSetAl(Com, e(&ComI->Al, 1, 0));
			NComSetAd(Com, e(&ComI->Al, 2, 0));
		}

		// Filtered acceleration in FLD frame.
		if (Com->mIsFiltAxValid && Com->mIsFiltAyValid && Com->mIsFiltAzValid)
		{
			MatFillR(&ComI->Ab, 3, 1, Com->mFiltAx, Com->mFiltAy, Com->mFiltAz);
			MatMultRAB(&ComI->Al, &ComI->Cb, &ComI->Ab);
			NComSetFiltAf(Com, e(&ComI->Al, 0, 0));
			NComSetFiltAl(Com, e(&ComI->Al, 1, 0));
			NComSetFiltAd(Com, e(&ComI->Al, 2, 0));
		}

		// Angular rate in FLD frame.
		if (Com->mIsWxValid && Com->mIsWyValid && Com->mIsWzValid)
		{
			MatFillR(&ComI->Wb, 3, 1, Com->mWx, Com->mWy, Com->mWz);
			MatMultRAB(&ComI->Wl, &ComI->Cb, &ComI->Wb);
			NComSetWf(Com, e(&ComI->Wl, 0, 0));
			NComSetWl(Com, e(&ComI->Wl, 1, 0));
			NComSetWd(Com, e(&ComI->Wl, 2, 0));
		}

		// Velocity in FLD frame.
		if (Com->mIsVnValid && Com->mIsVeValid && Com->mIsVdValid)
		{
			MatFillR(&ComI->Vn, 3, 1, Com->mVn, Com->mVe, Com->mVd);
			MatMultRAtB(&ComI->Vl, &ComI->Ch, &ComI->Vn);
			NComSetVf(Com, e(&ComI->Vl, 0, 0));
			NComSetVl(Com, e(&ComI->Vl, 1, 0));
			// Vd already known!
			MatMultRAtB(&ComI->Vb, &ComI->Cb_1, &ComI->Vl);       //new
			NComSetVx(Com, e(&ComI->Vb, 0, 0));
			NComSetVy(Com, e(&ComI->Vb, 1, 0));
			NComSetVz(Com, e(&ComI->Vb, 2, 0));                   //new
		}

		// Angular acceleration in FLD frame.
		if (Com->mIsYxValid && Com->mIsYyValid && Com->mIsYzValid)
		{
			MatFillR(&ComI->Yb, 3, 1, Com->mYx, Com->mYy, Com->mYz);
			MatMultRAB(&ComI->Yl, &ComI->Cb, &ComI->Yb);
			NComSetYf(Com, e(&ComI->Yl, 0, 0));
			NComSetYl(Com, e(&ComI->Yl, 1, 0));
			NComSetYd(Com, e(&ComI->Yl, 2, 0));
		}

		// Filtered angular acceleration in FLD frame.
		if (Com->mIsFiltYxValid && Com->mIsFiltYyValid && Com->mIsFiltYzValid)
		{
			MatFillR(&ComI->Yb, 3, 1, Com->mFiltYx, Com->mFiltYy, Com->mFiltYz);
			MatMultRAB(&ComI->Yl, &ComI->Cb, &ComI->Yb);
			NComSetFiltYf(Com, e(&ComI->Yl, 0, 0));
			NComSetFiltYl(Com, e(&ComI->Yl, 1, 0));
			NComSetFiltYd(Com, e(&ComI->Yl, 2, 0));
		}
	}
}


//============================================================================================================
//! \brief Invalidate all variables that my be computed by speed and slip.

static void SpeedSlipInvalidate(NComRxC *Com)
{
	Com->mIsSpeed2dValid   = 0;
	Com->mIsSpeed3dValid   = 0;

	Com->mIsTrackValid     = 0;
	Com->mIsSlipValid      = 0;
	Com->mIsCurvatureValid = 0;
}


//============================================================================================================
//! \brief Speed and slip computation.

static void SpeedSlipCompute(NComRxC *Com)
{
	NComRxCInternal *ComI = Com->mInternal;

	double accuracy2d = 1e308;

	// Compute speeds and speed accuracies (if accuracy not available then assume a value)

	if (Com->mIsVeValid && Com->mIsVnValid)
	{
		double s, a;

		// 2d speed (optionally clamping to zero if small) and accuracy

		s = Com->mVe * Com->mVe + Com->mVn * Com->mVn;

		if (Com->mIsVnAccValid && Com->mIsVeAccValid)
			a = SPEED_HOLD_FACTOR * SPEED_HOLD_FACTOR * (Com->mVnAcc * Com->mVnAcc + Com->mVeAcc * Com->mVeAcc);
		else
			a = SPEED_HOLD_FACTOR * SPEED_HOLD_FACTOR * (MIN_HORZ_SPEED * MIN_HORZ_SPEED);

		NComSetSpeed2d(Com, (ComI->mHoldDistWhenSlow && s <= a) ? 0.0 : sqrt(s) );

		accuracy2d = sqrt(a);

		// 3d speed (optionally clamping to zero if small) and accuracy

		if (Com->mIsVdValid)
		{
			s += Com->mVd * Com->mVd;

			if (Com->mIsVdAccValid)
				a += SPEED_HOLD_FACTOR * SPEED_HOLD_FACTOR * (Com->mVdAcc * Com->mVdAcc);
			else
				a += SPEED_HOLD_FACTOR * SPEED_HOLD_FACTOR * (MIN_VERT_SPEED * MIN_VERT_SPEED);

			NComSetSpeed3d(Com, (ComI->mHoldDistWhenSlow && s <= a) ? 0.0 : sqrt(s) );
		}
	}

	// Compute slip angle, track angle and curvature (when 2D speed is larger than 2D speed accuracy)

	if (Com->mIsSpeed2dValid && Com->mSpeed2d > accuracy2d)
	{
		if (Com->mIsVeValid && Com->mIsVnValid)
		{
			NComSetTrack(Com, atan2(Com->mVe, Com->mVn) * RAD2DEG);
		}

		if (Com->mIsTrackValid && Com->mIsHeadingValid)
		{
			double d = Com->mTrack - Com->mHeading;

			if (d > 180.0)
				d -= 360.0;
			else if (d <= -180.0)
				d += 360.0;

			NComSetSlip(Com, d);
		}

		if (Com->mIsWdValid)
		{
			NComSetCurvature(Com, (Com->mWd * DEG2RAD) / Com->mSpeed2d);
		}
	}
}


//============================================================================================================
//! \brief Invalidate all variables that my be computed by distance.

static void DistanceInvalidate(NComRxC *Com)
{
	Com->mIsDist2dValid = 0;
	Com->mIsDist3dValid = 0;
}


//============================================================================================================
//! \brief Distance computation.

static void DistanceCompute(NComRxC *Com, char trig)
{
	NComRxCInternal *ComI = Com->mInternal;

	if (Com->mIsSpeed2dValid && Com->mIsTimeValid)
	{
		if (trig)
		{
			if (ComI->mPrevDist2dValid)
			{
				// Use trapezoidal integration
				NComSetDist2d(Com, ComI->mPrevDist2d + 0.5 * (Com->mSpeed2d + ComI->mPrevDist2dSpeed) * (Com->mTime - ComI->mPrevDist2dTime));
			}
		}
		else
		{
			if (ComI->mPrevDist2dValid)
			{
				// Use trapezoidal integration
				ComI->mPrevDist2d += 0.5 * (Com->mSpeed2d + ComI->mPrevDist2dSpeed) * (Com->mTime - ComI->mPrevDist2dTime);
			}
			else
			{
				ComI->mPrevDist2d      = 0.0;
				ComI->mPrevDist2dValid = 1;
			}

			ComI->mPrevDist2dTime  = Com->mTime;
			ComI->mPrevDist2dSpeed = Com->mSpeed2d;

			NComSetDist2d(Com, ComI->mPrevDist2d);
		}
	}

	if (Com->mIsSpeed3dValid && Com->mIsTimeValid)
	{
		if (trig)
		{
			if (ComI->mPrevDist3dValid)
			{
				// Use trapezoidal integration
				NComSetDist3d(Com, ComI->mPrevDist3d + 0.5 * (Com->mSpeed3d + ComI->mPrevDist3dSpeed) * (Com->mTime - ComI->mPrevDist3dTime));
			}
		}
		else
		{
			if (ComI->mPrevDist3dValid)
			{
				// Use trapezoidal integration
				ComI->mPrevDist3d += 0.5 * (Com->mSpeed3d + ComI->mPrevDist3dSpeed) * (Com->mTime - ComI->mPrevDist3dTime);
			}
			else
			{
				ComI->mPrevDist3d      = 0.0;
				ComI->mPrevDist3dValid = 1;
			}

			ComI->mPrevDist3dTime  = Com->mTime;
			ComI->mPrevDist3dSpeed = Com->mSpeed3d;

			NComSetDist3d(Com, ComI->mPrevDist3d);
		}
	}
}


//============================================================================================================
//! \brief Set linear acceleration low-pass filter cut-off frequency.

static void NComSetLinAccFiltFreq(NComRxC *Com, double freq)
{
	// Check if the filter frequency is currently invalid
	if (!Com->mIsLinAccFiltFreqValid)
	{
		Com->mLinAccFiltFreq       = freq;
		Com->mHasLinAccFiltChanged = 1;
	}
	// See if the filter characteristics are not fixed
	else if (!Com->mIsLinAccFiltFixed)
	{
		// Monitor changes to the filter frequency
		if (Com->mLinAccFiltFreq != freq)
			Com->mHasLinAccFiltChanged = 1;
		// Update the filter frequency
		Com->mLinAccFiltFreq = freq;
	}
	// Check whether the filter has been switched off
	if ((Com->mLinAccFiltFreq == 0.0) && Com->mIsLinAccFiltZetaValid && (Com->mLinAccFiltZeta == 0.0))
		Com->mIsLinAccFiltOff = 1;
	// Indicate that the filter frequency is now valid
	Com->mIsLinAccFiltFreqValid = 1;
}


//============================================================================================================
//! \brief Set linear acceleration low-pass filter damping ratio.

static void NComSetLinAccFiltZeta(NComRxC *Com, double zeta)
{
	// Check if the filter damping ratio is currently invalid
	if (!Com->mIsLinAccFiltZetaValid)
	{
		Com->mLinAccFiltZeta       = zeta;
		Com->mHasLinAccFiltChanged = 1;
	}
	// See if the filter characteristics are not fixed
	else if (!Com->mIsLinAccFiltFixed)
	{
		// Monitor changes to the filter damping ratio
		if (Com->mLinAccFiltZeta != zeta)
			Com->mHasLinAccFiltChanged = 1;
		// Update the filter damping ratio
		Com->mLinAccFiltZeta = zeta;
	}
	// Check whether the filter has been switched off
	if (Com->mIsLinAccFiltFreqValid && (Com->mLinAccFiltFreq == 0.0) && (Com->mLinAccFiltZeta == 0.0))
		Com->mIsLinAccFiltOff = 1;
	// Indicate that the filter damping ratio is now valid
	Com->mIsLinAccFiltZetaValid = 1;
}


//============================================================================================================
//! \brief Fix parameters of linear acceleration low-pass filter.

static void NComFixLinAccFilt(NComRxC *Com, double freq, double zeta)
{
	// Clear the linear acceleration filter fixed flag
	Com->mIsLinAccFiltFixed = 0;
	// Set the filter frequency
	NComSetLinAccFiltFreq(Com, freq);
	// Set the filter damping ratio
	NComSetLinAccFiltZeta(Com, zeta);
	// Set the linear acceleration filter fixed flag
	Com->mIsLinAccFiltFixed = 1;
}


//============================================================================================================
//! \brief Clear all parameters of linear acceleration low-pass filter.

static void NComClearLinAccFilt(NComRxC *Com)
{
	Com->mIsLinAccFiltFixed = 0;
	Com->mIsLinAccFiltFreqValid = 0;
	Com->mIsLinAccFiltZetaValid = 0;
	Com->mHasLinAccFiltChanged = 0;
	Com->mIsLinAccFiltOff = 0;
}


//============================================================================================================
//! \brief Set angular acceleration low-pass filter cut-off frequency.

static void NComSetAngAccFiltFreq(NComRxC *Com, double freq)
{
	// Check if the filter frequency is currently invalid
	if (!Com->mIsAngAccFiltFreqValid)
	{
		Com->mAngAccFiltFreq       = freq;
		Com->mHasAngAccFiltChanged = 1;
	}
	// See if the filter characteristics are not fixed
	else if (!Com->mIsAngAccFiltFixed)
	{
		// Monitor changes to the filter frequency
		if (Com->mAngAccFiltFreq != freq)
			Com->mHasAngAccFiltChanged = 1;
		// Update the filter frequency
		Com->mAngAccFiltFreq = freq;
	}
	// Check whether the filter has been switched off
	if ((Com->mAngAccFiltFreq == 0.0) && Com->mIsAngAccFiltZetaValid && (Com->mAngAccFiltZeta == 0.0))
		Com->mIsAngAccFiltOff = 1;
	// Indicate that the filter frequency is now valid
	Com->mIsAngAccFiltFreqValid = 1;
}


//============================================================================================================
//! \brief Set angular acceleration low-pass filter damping ratio.

static void NComSetAngAccFiltZeta(NComRxC *Com, double zeta)
{
	// Check if the filter damping ratio is currently invalid
	if (!Com->mIsAngAccFiltZetaValid)
	{
		Com->mAngAccFiltZeta       = zeta;
		Com->mHasAngAccFiltChanged = 1;
	}
	// See if the filter characteristics are not fixed
	else if (!Com->mIsAngAccFiltFixed)
	{
		// Monitor changes to the filter damping ratio
		if (Com->mAngAccFiltZeta != zeta)
			Com->mHasAngAccFiltChanged = 1;
		// Update the filter damping ratio
		Com->mAngAccFiltZeta = zeta;
	}
	// Check whether the filter has been switched off
	if (Com->mIsAngAccFiltFreqValid && (Com->mAngAccFiltFreq == 0.0) && (Com->mAngAccFiltZeta == 0.0))
		Com->mIsAngAccFiltOff = 1;
	// Indicate that the filter damping ratio is now valid
	Com->mIsAngAccFiltZetaValid = 1;
}


//============================================================================================================
//! \brief Fix parameters of angular acceleration low-pass filter.

static void NComFixAngAccFilt(NComRxC *Com, double freq, double zeta)
{
	// Clear the angular acceleration filter fixed flag
	Com->mIsAngAccFiltFixed = 0;
	// Set the filter frequency
	NComSetAngAccFiltFreq(Com, freq);
	// Set the filter damping ratio
	NComSetAngAccFiltZeta(Com, zeta);
	// Set the angular acceleration filter fixed flag
	Com->mIsAngAccFiltFixed = 1;
}


//============================================================================================================
//! \brief Clear all parameters of angular acceleration low-pass filter.

static void NComClearAngAccFilt(NComRxC *Com)
{
	Com->mIsAngAccFiltFixed = 0;
	Com->mIsAngAccFiltFreqValid = 0;
	Com->mIsAngAccFiltZetaValid = 0;
	Com->mHasAngAccFiltChanged = 0;
	Com->mIsAngAccFiltOff = 0;
}


//============================================================================================================
//! \brief Set reference frame.

static void SetRefFrame(NComRxC *Com, double lat, double lon, double alt, double heading)
{
	if (heading > 180.0)
		heading -= 360.0;
	else if (heading <= -180.0)
		heading += 360.0;

	NComSetRefLat(Com, lat);
	NComSetRefLon(Com, lon);
	NComSetRefAlt(Com, alt);

	NComSetRefHeading(Com, heading);
}


//============================================================================================================
//! \brief Decode status message.

static void DecodeStatusMsg(NComRxC *Com)
{
	Com->mIsTrigTimeNew       = 0;
	Com->mIsTrig2TimeNew      = 0;
	Com->mIsDigitalOutTimeNew = 0;

	switch (Com->mInternal->mCurPkt[PI_CHANNEL_INDEX])
	{
		case  0: DecodeExtra0(Com);  break;
		case  1: DecodeExtra1(Com);  break;
		case  2: DecodeExtra2(Com);  break;
		case  3: DecodeExtra3(Com);  break;
		case  4: DecodeExtra4(Com);  break;
		case  5: DecodeExtra5(Com);  break;
		case  6: DecodeExtra6(Com);  break;
		case  7: DecodeExtra7(Com);  break;
		case  8: DecodeExtra8(Com);  break;
		case  9: DecodeExtra9(Com);  break;
		case 10: DecodeExtra10(Com); break;
		case 11: DecodeExtra11(Com); break;
		case 12: DecodeExtra12(Com); break;
		case 13: DecodeExtra13(Com); break;
		case 14: DecodeExtra14(Com); break;
		case 15: DecodeExtra15(Com); break;
		case 16: DecodeExtra16(Com); break;
		case 17: DecodeExtra17(Com); break;
		case 18: DecodeExtra18(Com); break;
		case 19: DecodeExtra19(Com); break;
		case 20: DecodeExtra20(Com); break;
		case 21: DecodeExtra21(Com); break;
		case 22: DecodeExtra22(Com); break;
		case 23: DecodeExtra23(Com); break;
		case 24: DecodeExtra24(Com); break;
		case 25: DecodeExtra25(Com); break;
		case 26: DecodeExtra26(Com); break;
		case 27: DecodeExtra27(Com); break;
		case 28: DecodeExtra28(Com); break;
		case 29: DecodeExtra29(Com); break;
		case 30: DecodeExtra30(Com); break;
		case 31: DecodeExtra31(Com); break;
		case 32: DecodeExtra32(Com); break;
		case 33: DecodeExtra33(Com); break;
		case 34: DecodeExtra34(Com); break;
		case 35: DecodeExtra35(Com); break;
		case 36: DecodeExtra36(Com); break;
		case 37: DecodeExtra37(Com); break;
		case 38: DecodeExtra38(Com); break;
		case 39: DecodeExtra39(Com); break;
		case 41: DecodeExtra41(Com); break;
		case 42: DecodeExtra42(Com); break;
		case 43: DecodeExtra43(Com); break;
		case 44: DecodeExtra44(Com); break;
		case 45: DecodeExtra45(Com); break;
		case 46: DecodeExtra46(Com); break;
		case 47: DecodeExtra47(Com); break;
		case 48: DecodeExtra48(Com); break;
		case 49: DecodeExtra49(Com); break;
		case 50: DecodeExtra50(Com); break;
		case 55: DecodeExtra55(Com); break;
		case 56: DecodeExtra56(Com); break;
		case 57: DecodeExtra57(Com); break;
		case 59: DecodeExtra59(Com); break;
		case 61: DecodeExtra61(Com); break;
		case 62: DecodeExtra62(Com); break;
		case 63: DecodeExtra63(Com); break;
		case 64: DecodeExtra64(Com); break;
		case 65: DecodeExtra65(Com); break;
		case 66: DecodeExtra66(Com); break;
		case 67: DecodeExtra67(Com); break;
		case 72: DecodeExtra72(Com); break;
		case 73: DecodeExtra73(Com); break;
		case 74: DecodeExtra74(Com); break;
	}
}


//============================================================================================================
//! \brief 0. Full time, number of satellites, position, velocity and dual antenna modes.

static void DecodeExtra0(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	if (Com->mInsNavMode >= NAVIGATION_STATUS_INIT)
	{
		Com->mInternal->mMinutes = cast_4_byte_to_int32(mCurStatus+0);
	}

	if (mCurStatus[4] & 0x80) Com->mIsGpsNumObsValid  = 0; else NComSetGpsNumObs     (Com, mCurStatus[4]);
	if (mCurStatus[5] & 0x80) Com->mIsGpsPosModeValid = 0; else NComSetGpsPosModeEnum(Com, mCurStatus[5]);
	if (mCurStatus[6] & 0x80) Com->mIsGpsVelModeValid = 0; else NComSetGpsVelModeEnum(Com, mCurStatus[6]);
	if (mCurStatus[7] & 0x80) Com->mIsGpsAttModeValid = 0; else NComSetGpsAttModeEnum(Com, mCurStatus[7]);
}


//============================================================================================================
//! \brief 1. Kalman filter innovations for position, velocity and attitude.

static void DecodeExtra1(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	// Valid flag packed in lsb.

	if (mCurStatus[0] & 0x1) NComSetInnPosX   (Com, (((int8_t) mCurStatus[0]) >> 1) * INNFACTOR);
	if (mCurStatus[1] & 0x1) NComSetInnPosY   (Com, (((int8_t) mCurStatus[1]) >> 1) * INNFACTOR);
	if (mCurStatus[2] & 0x1) NComSetInnPosZ   (Com, (((int8_t) mCurStatus[2]) >> 1) * INNFACTOR);
	if (mCurStatus[3] & 0x1) NComSetInnVelX   (Com, (((int8_t) mCurStatus[3]) >> 1) * INNFACTOR);
	if (mCurStatus[4] & 0x1) NComSetInnVelY   (Com, (((int8_t) mCurStatus[4]) >> 1) * INNFACTOR);
	if (mCurStatus[5] & 0x1) NComSetInnVelZ   (Com, (((int8_t) mCurStatus[5]) >> 1) * INNFACTOR);
	if (mCurStatus[6] & 0x1) NComSetInnPitch  (Com, (((int8_t) mCurStatus[6]) >> 1) * INNFACTOR);
	if (mCurStatus[7] & 0x1) NComSetInnHeading(Com, (((int8_t) mCurStatus[7]) >> 1) * INNFACTOR);
}


//============================================================================================================
//! \brief 2. Received information about the primary GPS receiver.

static void DecodeExtra2(NComRxC *Com)
{
	DecodeExtraGpsReceived(Com->mInternal->mCurStatus, Com->mGpsPrimary);
}


//============================================================================================================
//! \brief 3. Position accuracy and UMAC status.

static void DecodeExtra3(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	if (mCurStatus[6] < NCOM_COUNT_TOO_OLD)
	{
		NComSetNorthAcc(Com, cast_2_byte_to_uint16(mCurStatus+0) * POSA2M);
		NComSetEastAcc (Com, cast_2_byte_to_uint16(mCurStatus+2) * POSA2M);
		NComSetAltAcc  (Com, cast_2_byte_to_uint16(mCurStatus+4) * POSA2M);
	}
	else
	{
		Com->mIsNorthAccValid = 0;
		Com->mIsEastAccValid  = 0;
		Com->mIsAltAccValid   = 0;
	}

	if (mCurStatus[7] != 0xFF)
	{
		NComSetUmacStatusEnum(Com, mCurStatus[7]);
	}
	else
	{
		Com->mIsUmacStatusValid = 0;
	}
}


//============================================================================================================
//! \brief 4. Velocity accuracy.

static void DecodeExtra4(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	if (mCurStatus[6] < NCOM_COUNT_TOO_OLD)
	{
		NComSetVnAcc(Com, cast_2_byte_to_uint16(mCurStatus+0) * VELA2MPS);
		NComSetVeAcc(Com, cast_2_byte_to_uint16(mCurStatus+2) * VELA2MPS);
		NComSetVdAcc(Com, cast_2_byte_to_uint16(mCurStatus+4) * VELA2MPS);
	}
	else
	{
		Com->mIsVnAccValid = 0;
		Com->mIsVeAccValid = 0;
		Com->mIsVdAccValid = 0;
	}
}


//============================================================================================================
//! \brief 5. Orientation accuracy.

static void DecodeExtra5(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	if (mCurStatus[6] < NCOM_COUNT_TOO_OLD)
	{
		NComSetHeadingAcc(Com, cast_2_byte_to_uint16(mCurStatus+0) * ANGA2RAD * RAD2DEG);
		NComSetPitchAcc  (Com, cast_2_byte_to_uint16(mCurStatus+2) * ANGA2RAD * RAD2DEG);
		NComSetRollAcc   (Com, cast_2_byte_to_uint16(mCurStatus+4) * ANGA2RAD * RAD2DEG);
	}
	else
	{
		Com->mIsHeadingAccValid = 0;
		Com->mIsPitchAccValid   = 0;
		Com->mIsRollAccValid    = 0;
	}
}


//============================================================================================================
//! \brief 6. Gyroscope bias.

static void DecodeExtra6(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	if (mCurStatus[6] < NCOM_COUNT_TOO_OLD)
	{
		NComSetWxBias(Com, cast_2_byte_to_int16(mCurStatus+0) * GB2RPS * RAD2DEG);
		NComSetWyBias(Com, cast_2_byte_to_int16(mCurStatus+2) * GB2RPS * RAD2DEG);
		NComSetWzBias(Com, cast_2_byte_to_int16(mCurStatus+4) * GB2RPS * RAD2DEG);
	}
	else
	{
		Com->mIsWxBiasValid = 0;
		Com->mIsWyBiasValid = 0;
		Com->mIsWzBiasValid = 0;
	}
}


//============================================================================================================
//! \brief 7. Accelerometer bias.

static void DecodeExtra7(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	if (mCurStatus[6] < NCOM_COUNT_TOO_OLD)
	{
		NComSetAxBias(Com, cast_2_byte_to_int16(mCurStatus+0) * AB2MPS2);
		NComSetAyBias(Com, cast_2_byte_to_int16(mCurStatus+2) * AB2MPS2);
		NComSetAzBias(Com, cast_2_byte_to_int16(mCurStatus+4) * AB2MPS2);
	}
	else
	{
		Com->mIsAxBiasValid = 0;
		Com->mIsAyBiasValid = 0;
		Com->mIsAzBiasValid = 0;
	}
}


//============================================================================================================
//! \brief 8. Gyroscope scale factor.

static void DecodeExtra8(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	if (mCurStatus[6] < NCOM_COUNT_TOO_OLD)
	{
		NComSetWxSf(Com, cast_2_byte_to_int16(mCurStatus+0) * GSFACTOR);
		NComSetWySf(Com, cast_2_byte_to_int16(mCurStatus+2) * GSFACTOR);
		NComSetWzSf(Com, cast_2_byte_to_int16(mCurStatus+4) * GSFACTOR);
	}
	else
	{
		Com->mIsWxSfValid = 0;
		Com->mIsWySfValid = 0;
		Com->mIsWzSfValid = 0;
	}
}


//============================================================================================================
//! \brief 9. Gyroscope bias accuracy.

static void DecodeExtra9(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	if (mCurStatus[6] < NCOM_COUNT_TOO_OLD)
	{
		NComSetWxBiasAcc(Com, cast_2_byte_to_uint16(mCurStatus+0) * GBA2RPS * RAD2DEG);
		NComSetWyBiasAcc(Com, cast_2_byte_to_uint16(mCurStatus+2) * GBA2RPS * RAD2DEG);
		NComSetWzBiasAcc(Com, cast_2_byte_to_uint16(mCurStatus+4) * GBA2RPS * RAD2DEG);
	}
	else
	{
		Com->mIsWxBiasAccValid = 0;
		Com->mIsWyBiasAccValid = 0;
		Com->mIsWzBiasAccValid = 0;
	}
}


//============================================================================================================
//! \brief 10. Accelerometer bias accuracy.

static void DecodeExtra10(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	if (mCurStatus[6] < NCOM_COUNT_TOO_OLD)
	{
		NComSetAxBiasAcc(Com, cast_2_byte_to_uint16(mCurStatus+0) * ABA2MPS2);
		NComSetAyBiasAcc(Com, cast_2_byte_to_uint16(mCurStatus+2) * ABA2MPS2);
		NComSetAzBiasAcc(Com, cast_2_byte_to_uint16(mCurStatus+4) * ABA2MPS2);
	}
	else
	{
		Com->mIsAxBiasAccValid = 0;
		Com->mIsAyBiasAccValid = 0;
		Com->mIsAzBiasAccValid = 0;
	}
}


//============================================================================================================
//! \brief 11. Gyroscope scale factor accuracy.

static void DecodeExtra11(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	if (mCurStatus[6] < NCOM_COUNT_TOO_OLD)
	{
		NComSetWxSfAcc(Com, cast_2_byte_to_uint16(mCurStatus+0) * GSAFACTOR);
		NComSetWySfAcc(Com, cast_2_byte_to_uint16(mCurStatus+2) * GSAFACTOR);
		NComSetWzSfAcc(Com, cast_2_byte_to_uint16(mCurStatus+4) * GSAFACTOR);
	}
	else
	{
		Com->mIsWxSfAccValid = 0;
		Com->mIsWySfAccValid = 0;
		Com->mIsWzSfAccValid = 0;
	}
}


//============================================================================================================
//! \brief 12. Position estimate of the primary GPS antenna.

static void DecodeExtra12(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	if (mCurStatus[6] < NCOM_COUNT_TOO_OLD)
	{
		NComSetGAPx(Com, cast_2_byte_to_int16(mCurStatus+0) * GPSPOS2M);
		NComSetGAPy(Com, cast_2_byte_to_int16(mCurStatus+2) * GPSPOS2M);
		NComSetGAPz(Com, cast_2_byte_to_int16(mCurStatus+4) * GPSPOS2M);
	}
	else
	{
		Com->mIsGAPxValid = 0;
		Com->mIsGAPyValid = 0;
		Com->mIsGAPzValid = 0;
	}
}


//============================================================================================================
//! \brief 13. Orientation estimate of dual antenna systems.

static void DecodeExtra13(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	if (mCurStatus[6] < NCOM_COUNT_TOO_OLD)
	{
		NComSetAtH(Com, ((double) cast_2_byte_to_int16(mCurStatus+0)) * GPSATT2RAD * RAD2DEG);
		NComSetAtP(Com, ((double) cast_2_byte_to_int16(mCurStatus+2)) * GPSATT2RAD * RAD2DEG);
	}
	else
	{
		Com->mIsAtHValid = 0;
		Com->mIsAtPValid = 0;
	}

	if ((mCurStatus[6] < NCOM_COUNT_TOO_OLD) && IsDevIDAfter(Com, "030321"))
	{
		uint16_t us = cast_2_byte_to_uint16(mCurStatus+4);

		if ((us == UINT16_C(0)) || (us == UINT16_C(0xFFFF)))
		{
			NComSetBaseLineLength(Com, -1.0); Com->mIsBaseLineLengthValid = 0; Com->mIsBaseLineLengthConfig = 1;
		}
		else
		{
			NComSetBaseLineLength(Com, ((double) us) * GPSPOS2M);
		}
	}
	else
	{
		Com->mIsBaseLineLengthValid = 0;
	}
}


//============================================================================================================
//! \brief 14. Accuracy of position of the primary GPS antenna.

static void DecodeExtra14(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	if (mCurStatus[6] < NCOM_COUNT_TOO_OLD)
	{
		NComSetGAPxAcc(Com, cast_2_byte_to_uint16(mCurStatus+0) * GPSPOSA2M);
		NComSetGAPyAcc(Com, cast_2_byte_to_uint16(mCurStatus+2) * GPSPOSA2M);
		NComSetGAPzAcc(Com, cast_2_byte_to_uint16(mCurStatus+4) * GPSPOSA2M);
	}
	else
	{
		Com->mIsGAPxAccValid = 0;
		Com->mIsGAPyAccValid = 0;
		Com->mIsGAPzAccValid = 0;
	}
}


//============================================================================================================
//! \brief 15. Accuracy of the orientation of dual antenna systems.

static void DecodeExtra15(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	if (mCurStatus[6] < NCOM_COUNT_TOO_OLD)
	{
		NComSetAtHAcc(Com, ((double) cast_2_byte_to_uint16(mCurStatus+0)) * GPSATTA2RAD * RAD2DEG);
		NComSetAtPAcc(Com, ((double) cast_2_byte_to_uint16(mCurStatus+2)) * GPSATTA2RAD * RAD2DEG);
	}
	else
	{
		Com->mIsAtHAccValid = 0;
		Com->mIsAtPAccValid = 0;
	}

	if ((mCurStatus[6] < NCOM_COUNT_TOO_OLD) && IsDevIDAfter(Com, "030321"))
	{
		uint16_t us = cast_2_byte_to_uint16(mCurStatus+4);

		if (us == UINT16_C(0xFFFF))
		{
			NComSetBaseLineLengthAcc(Com, -1.0); Com->mIsBaseLineLengthAccValid = 0; Com->mIsBaseLineLengthAccConfig = 1;
		}
		else
		{
			NComSetBaseLineLengthAcc(Com, ((double) us) * GPSPOSA2M);
		}
	}
	else
	{
		Com->mIsBaseLineLengthAccValid = 0;
	}
}


//============================================================================================================
//! \brief 16. RT to vehicle rotation.

static void DecodeExtra16(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	if (mCurStatus[6] == 0x00)
	{
		NComSetImu2VehHeading(Com, cast_2_byte_to_int16(mCurStatus+0) * GPSATT2RAD * RAD2DEG);
		NComSetImu2VehPitch  (Com, cast_2_byte_to_int16(mCurStatus+2) * GPSATT2RAD * RAD2DEG);
		NComSetImu2VehRoll   (Com, cast_2_byte_to_int16(mCurStatus+4) * GPSATT2RAD * RAD2DEG);
	}
	else
	{
		Com->mIsImu2VehHeadingValid = 0;
		Com->mIsImu2VehPitchValid   = 0;
		Com->mIsImu2VehRollValid    = 0;
	}

	if (IsDevIDAfter(Com, "030128") && (mCurStatus[7] & 0x1)) // means valid
	{
		NComSetTimeUtcOffset(Com, ((int8_t) mCurStatus[7]) >> 1);
	}
	else
	{
		Com->mIsTimeUtcOffsetValid = 0;
	}
}


//============================================================================================================
//! \brief 17. Received information about the secondary GPS receiver.

static void DecodeExtra17(NComRxC *Com)
{
	DecodeExtraGpsReceived(Com->mInternal->mCurStatus, Com->mGpsSecondary);
}


//============================================================================================================
//! \brief 18. Internal information about inertial measurement unit.

static void DecodeExtra18(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	NComSetImuChars       (Com, cast_4_byte_to_uint32(mCurStatus+0                       ));
	NComSetImuPkts        (Com, incr_2_byte_to_uint32(mCurStatus+4, Com->mImuPkts        ));
	NComSetImuCharsSkipped(Com, incr_2_byte_to_uint32(mCurStatus+6, Com->mImuCharsSkipped));
}


//============================================================================================================
//! \brief 19. Software version running on RT.

static void DecodeExtra19(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	NComSetDevId(Com, (char *) (mCurStatus+0), 8);
}


//============================================================================================================
//! \brief 20. Differential corrections configuration.

static void DecodeExtra20(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	NComSetGpsDiffAge(Com, cast_2_byte_to_int16(mCurStatus+0) * DIFFAGE2SEC);

	if (mCurStatus[2] != '\0')
	{
		NComSetBaseStationId(Com, (char *) (mCurStatus+2), 4);
	}
	else
	{
		Com->mIsBaseStationIdValid = 0;
	}
}


//============================================================================================================
//! \brief 21. Disk space and size of current internal log file.

static void DecodeExtra21(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	int32_t ds = cast_4_byte_to_int32(mCurStatus+0);  // These are
	int32_t fs = cast_4_byte_to_int32(mCurStatus+4);  // in KiBs.

	if (ds >= 0) NComSetDiskSpace(Com, ((uint64_t)ds) << 10); else Com->mIsDiskSpaceValid = 0;
	if (fs >= 0) NComSetFileSize (Com, ((uint64_t)fs) << 10); else Com->mIsFileSizeValid  = 0;
}


//============================================================================================================
//! \brief 22. Internal information on timing of real-time processing.

static void DecodeExtra22(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	NComSetTimeMismatch (Com, cast_2_byte_to_uint16(mCurStatus+0)); if (Com->mTimeMismatch  == 65535) Com->mIsTimeMismatchValid  = 0;
	NComSetImuTimeDiff  (Com,                       mCurStatus[2]); if (Com->mImuTimeDiff   ==   255) Com->mIsImuTimeDiffValid   = 0;
	NComSetImuTimeMargin(Com,                       mCurStatus[3]); if (Com->mImuTimeMargin ==   255) Com->mIsImuTimeMarginValid = 0;
	NComSetImuLoopTime  (Com, cast_2_byte_to_uint16(mCurStatus+4)); if (Com->mImuLoopTime   == 65535) Com->mIsImuLoopTimeValid   = 0;
	NComSetOpLoopTime   (Com, cast_2_byte_to_uint16(mCurStatus+6)); if (Com->mOpLoopTime    == 65535) Com->mIsOpLoopTimeValid    = 0;
}


//============================================================================================================
//! \brief 23. System up time and number of consecutive GPS rejections.

static void DecodeExtra23(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	uint16_t us;

	NComSetBnsLag(Com, cast_2_byte_to_uint16(mCurStatus+0));
	if (Com->mBnsLag == 65535)
	{
		Com->mIsBnsLagValid = 0;
	}
	else if (Com->mIsBnsLagFiltValid)
	{
		NComSetBnsLagFilt(Com, Com->mBnsLagFilt*0.99 + (double)Com->mBnsLag*0.01);
	}
	else
	{
		NComSetBnsLagFilt(Com, (double)Com->mBnsLag);
	}

	us = cast_2_byte_to_uint16(mCurStatus+2);
	if (us == UINT16_C(0xFFFF))
	{
		Com->mIsUpTimeValid = 0;
	}
	else if (IsDevIDAfter(Com, "030124"))
	{
		// From firmware version 030124 onwards, time compression is used
		if (us > UINT16_C(20700))
		{
			NComSetUpTime(Com, (((uint32_t) us) - UINT32_C(20532)) * UINT32_C(3600));  // Hours to seconds
		}
		else if (us > UINT16_C(10800))
		{
			NComSetUpTime(Com, (((uint32_t) us) - UINT32_C(10620)) * UINT32_C(60));  // Minutes to seconds
		}
		else
		{
			NComSetUpTime(Com, (uint32_t) us);  // Seconds
		}
	}
	else if (Com->mIsDevIdValid)
	{
		// Older firmware versions do not compress UpTime
		NComSetUpTime(Com, us);
	}

	NComSetGPSPosReject(Com, mCurStatus[4]); if (Com->mGPSPosReject == 255) Com->mIsGPSPosRejectValid = 0;
	NComSetGPSVelReject(Com, mCurStatus[5]); if (Com->mGPSVelReject == 255) Com->mIsGPSVelRejectValid = 0;
	NComSetGPSAttReject(Com, mCurStatus[6]); if (Com->mGPSAttReject == 255) Com->mIsGPSAttRejectValid = 0;
}


//============================================================================================================
//! \brief 24. Trigger event timing (falling edge triggers).

static void DecodeExtra24(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	if (Com->mInternal->mTrigCount != mCurStatus[7])
	{
		double min, ms, c;

		Com->mInternal->mTrigCount = mCurStatus[7];

		min = cast_4_byte_to_int32 (mCurStatus+0);
		ms  = cast_2_byte_to_uint16(mCurStatus+4);
		c   = (int8_t)              mCurStatus[6];

		NComSetTrigTime(Com, min * 60.0 + ms * 0.001 + c * FINETIME2SEC);
	}
}


//============================================================================================================
//! \brief 25. (Reserved)

static void DecodeExtra25(NComRxC *Com)
{
	NComRxCInternal *ComI = Com->mInternal;
	const unsigned char *mCurStatus = ComI->mCurStatus;

	// Find out if the local reference is invalid
	if (((mCurStatus[6] == 0x00) && (mCurStatus[7] == 0x80)) || // heading set beyond -PI
	    ((mCurStatus[0] == 0x80) && (mCurStatus[1] == 0x00)  && // This condition matches the
	     (mCurStatus[2] == 0x00) && (mCurStatus[3] == 0x00)  && // incorrect settings in the
	     (mCurStatus[4] == 0x80) && (mCurStatus[5] == 0x00)  && // transmitter (up to version 0x08)
	     (mCurStatus[6] == 0x00) && (mCurStatus[7] == 0x00)))   // for invalid local reference frame
		; // do nothing as local reference frame is not valid
	else if (ComI->mIsAccurateRefLatValid && ComI->mIsAccurateRefLonValid &&
	         ComI->mIsAccurateRefAltValid && ComI->mIsAccurateRefHeadingValid)
		; // do nothing as the "accurate" local reference frame is valid
	else if (Com->mIsLatValid && Com->mIsLonValid && Com->mIsAltValid)
	{
		// Decode the local frame coordinates
		int32_t X = cast_3_byte_to_int32(mCurStatus+0);
		int32_t Y = cast_3_byte_to_int32(mCurStatus+3);
		// Are the coordinates within the allowed range?
		if ((X < POS_INT_24) && (X > NEG_INT_24) && (Y < POS_INT_24) && (Y > NEG_INT_24))
		{
			double dX, dY, rH, cosH, sinH;
			double dLat, dLon, rLat, rLon, rAlt;
			double refLatRad, curLatRad, oldLatRad, rho_e, rho_n;
			// Calculate local coordinates in m
			dX = ((double)X) * REFPOS2M;
			dY = ((double)Y) * REFPOS2M;
			// Calculate reference heading
			rH = ((double)cast_2_byte_to_int16(mCurStatus+6)) * REFANG2RAD * RAD2DEG;
			if (rH > 180.0)
				rH -= 360.0;
			else if (rH < -180.0)
				rH += 360.0;
			// Estimate the latitude and longitude offsets
			cosH = cos(rH * DEG2RAD);
			sinH = sin(rH * DEG2RAD);
			dLat = dX * cosH + dY * sinH;
			dLon = dX * sinH - dY * cosH;
			// Iterate until the reference origin latitude does not change
			curLatRad = Com->mLat * DEG2RAD;
			refLatRad = curLatRad;
			rAlt      = Com->mAlt;
			do
			{
				compute_earth_curvature(&rho_e, &rho_n, refLatRad);
				oldLatRad = refLatRad;
				refLatRad = curLatRad - dLat / (rho_e + rAlt);
			} while (fabs(refLatRad - oldLatRad) > 1e-6);
			// Estimate the local reference frame origin
			rLat = Com->mLat - (dLat / (rho_e + rAlt)) * RAD2DEG;
			rLon = Com->mLon - (dLon / ((rho_n + rAlt) * cos(refLatRad))) * RAD2DEG;
			// Set the local reference frame
			SetRefFrame(Com, rLat, rLon, rAlt, rH);
		}
	}
}


//============================================================================================================
//! \brief 26. (Reserved)

static void DecodeExtra26(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	if (mCurStatus[6] == 0x00)
	{
		NComSetRemoteLeverArmX(Com, cast_2_byte_to_int16(mCurStatus+0) * OUTPOS2M);
		NComSetRemoteLeverArmY(Com, cast_2_byte_to_int16(mCurStatus+2) * OUTPOS2M);
		NComSetRemoteLeverArmZ(Com, cast_2_byte_to_int16(mCurStatus+4) * OUTPOS2M);
	}
	else
	{
		Com->mIsRemoteLeverArmXValid = 0;
		Com->mIsRemoteLeverArmYValid = 0;
		Com->mIsRemoteLeverArmZValid = 0;
	}
}


//============================================================================================================
//! \brief 27. Internal information about dual antenna ambiguity searches.

static void DecodeExtra27(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	uint16_t us;

	NComSetHeadQualityEnum     (Com, mCurStatus[0]);
	NComSetHeadSearchTypeEnum  (Com, mCurStatus[1]);
	NComSetHeadSearchStatusEnum(Com, mCurStatus[2]);
	NComSetHeadSearchReadyEnum (Com, mCurStatus[3]);

	us = cast_2_byte_to_uint16(mCurStatus+4);
	if (us == UINT16_C(0xFFFF))
		Com->mIsHeadSearchInitValid = 0;
	else
		NComSetHeadSearchInit(Com, us);

	us = cast_2_byte_to_uint16(mCurStatus+6);
	if (us == UINT16_C(0xFFFF))
		Com->mIsHeadSearchNumValid = 0;
	else
		NComSetHeadSearchNum(Com, us);
}


//============================================================================================================
//! \brief 28. Internal information about dual antenna ambiguity searches.

static void DecodeExtra28(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	NComSetHeadSearchMaster(Com, 1 + mCurStatus[0]);
	NComSetHeadSearchSlave1(Com, 1 + mCurStatus[1]);
	NComSetHeadSearchSlave2(Com, 1 + mCurStatus[2]);
	NComSetHeadSearchSlave3(Com, 1 + mCurStatus[3]);

	NComSetHeadSearchTime  (Com, cast_2_byte_to_uint16(mCurStatus+4));
	NComSetHeadSearchConstr(Com, cast_2_byte_to_uint16(mCurStatus+6));
}


//============================================================================================================
//! \brief 29. Details on the initial settings.

static void DecodeExtra29(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	if (mCurStatus[0] & 0x80) Com->mIsOptionLevelValid     = 0; else NComSetOptionLevelEnum    (Com, mCurStatus[0]);
	if (mCurStatus[1] & 0x80) Com->mIsOptionVibrationValid = 0; else NComSetOptionVibrationEnum(Com, mCurStatus[1]);
	if (mCurStatus[2] & 0x80) Com->mIsOptionGpsAccValid    = 0; else NComSetOptionGpsAccEnum   (Com, mCurStatus[2]);
	if (mCurStatus[3] & 0x80) Com->mIsOptionUdpValid       = 0; else NComSetOptionUdpEnum      (Com, mCurStatus[3]);
	if (mCurStatus[4] & 0x80) Com->mIsOptionSer1Valid      = 0; else NComSetOptionSer1Enum     (Com, mCurStatus[4]);
	if (mCurStatus[5] & 0x80) Com->mIsOptionSer2Valid      = 0; else NComSetOptionSer2Enum     (Com, mCurStatus[5]);
	if (mCurStatus[6] & 0x80) Com->mIsOptionHeadingValid   = 0; else NComSetOptionHeadingEnum  (Com, mCurStatus[6]);
}


//============================================================================================================
//! \brief 30. Operating system and script version information.

static void DecodeExtra30(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	int32_t i;

	// Operating System Version
	if (mCurStatus[0] != 0xFF)
	{
		NComSetOsVersion1(Com, mCurStatus[0]);
	}
	else
	{
		Com->mIsOsVersion1Valid = 0;
	}

	if (mCurStatus[1] != 0xFF)
	{
		NComSetOsVersion2(Com, mCurStatus[1]);
	}
	else
	{
		Com->mIsOsVersion2Valid = 0;
	}

	if (mCurStatus[2] != 0xFF)
	{
		NComSetOsVersion3(Com, mCurStatus[2]);
	}
	else
	{
		Com->mIsOsVersion3Valid = 0;
	}

	// Startup Script ID
	if ((i = cast_3_byte_to_int32(mCurStatus+3)) >= 0)
	{
		char buffer[12];

		sprintf(buffer, "%06" PRId32, i);

		NComSetOsScriptId(Com, buffer, 11);
	}
	else
	{
		Com->mIsOsScriptIdValid = 0;
	}

	// Serial Number
	if ((mCurStatus[6] != 0xFF) || (mCurStatus[7] != 0xFF))
	{
		NComSetSerialNumber(Com, cast_2_byte_to_uint16(mCurStatus+6));
	}
	else
	{
		Com->mIsSerialNumberValid = 0;
	}
}


//============================================================================================================
//! \brief 31. Hardware configuration information.

static void DecodeExtra31(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	unsigned char gf;

	if (mCurStatus[1] == 0xFF) Com->mGpsPrimary->mIsTypeValid = 0; else NComGpsSetTypeEnum(Com->mGpsPrimary, mCurStatus[1]);

	if (mCurStatus[2] == 0xFF) Com->mGpsSecondary->mIsTypeValid = 0; else NComGpsSetTypeEnum(Com->mGpsSecondary, mCurStatus[2]);

	if (mCurStatus[0] == 0xFF) Com->mIsImuTypeValid      = 0; else NComSetImuTypeEnum     (Com, mCurStatus[0]);
	if (mCurStatus[3] == 0xFF) Com->mIsInterPcbTypeValid = 0; else NComSetInterPcbTypeEnum(Com, mCurStatus[3]);
	if (mCurStatus[4] == 0xFF) Com->mIsFrontPcbTypeValid = 0; else NComSetFrontPcbTypeEnum(Com, mCurStatus[4]);
	if (mCurStatus[5] == 0xFF) Com->mIsInterSwIdValid    = 0; else NComSetInterSwIdEnum   (Com, mCurStatus[5]);
	if (mCurStatus[6] == 0xFF) Com->mIsHwConfigValid     = 0; else NComSetHwConfigEnum    (Com, mCurStatus[6]);

	if ((gf = ~(mCurStatus[7])) & NCOM_GPS_FEATURE_VALID)
	{
		NComSetPsrDiffEnabled (Com, (gf & NCOM_GPS_FEATURE_PSRDIFF ) ? 1 : 0);
		NComSetSBASEnabled    (Com, (gf & NCOM_GPS_FEATURE_SBAS    ) ? 1 : 0);
		NComSetOmniVBSEnabled (Com, (gf & NCOM_GPS_FEATURE_OMNIVBS ) ? 1 : 0);
		NComSetOmniHPEnabled  (Com, (gf & NCOM_GPS_FEATURE_OMNIHP  ) ? 1 : 0);
		NComSetL1DiffEnabled  (Com, (gf & NCOM_GPS_FEATURE_L1DIFF  ) ? 1 : 0);
		NComSetL1L2DiffEnabled(Com, (gf & NCOM_GPS_FEATURE_L1L2DIFF) ? 1 : 0);
	}
	else
	{
		Com->mIsPsrDiffEnabledValid  = 0;
		Com->mIsSBASEnabledValid     = 0;
		Com->mIsOmniVBSEnabledValid  = 0;
		Com->mIsOmniHPEnabledValid   = 0;
		Com->mIsL1DiffEnabledValid   = 0;
		Com->mIsL1L2DiffEnabledValid = 0;
	}
}


//============================================================================================================
//! \brief 32. Kalman filter innovations for zero velocity and advanced slip.

static void DecodeExtra32(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	// Valid flag packed in lsb.

	if (mCurStatus[0] & 0x1) NComSetInnZeroVelX(Com, (((int8_t) mCurStatus[0]) >> 1) * INNFACTOR);
	if (mCurStatus[1] & 0x1) NComSetInnZeroVelY(Com, (((int8_t) mCurStatus[1]) >> 1) * INNFACTOR);
	if (mCurStatus[2] & 0x1) NComSetInnZeroVelZ(Com, (((int8_t) mCurStatus[2]) >> 1) * INNFACTOR);
	if (mCurStatus[3] & 0x1) NComSetInnNoSlipH (Com, (((int8_t) mCurStatus[3]) >> 1) * INNFACTOR);
	if (mCurStatus[4] & 0x1) NComSetInnHeadingH(Com, (((int8_t) mCurStatus[4]) >> 1) * INNFACTOR);
	if (mCurStatus[5] & 0x1) NComSetInnWSpeed  (Com, (((int8_t) mCurStatus[5]) >> 1) * INNFACTOR);
}


//============================================================================================================
//! \brief 33. Zero velocity lever arm.

static void DecodeExtra33(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	if (mCurStatus[6] == 0x00)
	{
		NComSetZeroVelLeverArmX(Com, cast_2_byte_to_int16(mCurStatus+0) * ZVPOS2M);
		NComSetZeroVelLeverArmY(Com, cast_2_byte_to_int16(mCurStatus+2) * ZVPOS2M);
		NComSetZeroVelLeverArmZ(Com, cast_2_byte_to_int16(mCurStatus+4) * ZVPOS2M);
	}
	else
	{
		Com->mIsZeroVelLeverArmXValid = 0;
		Com->mIsZeroVelLeverArmYValid = 0;
		Com->mIsZeroVelLeverArmZValid = 0;
	}
}


//============================================================================================================
//! \brief 34. Zero velocity lever arm accuracy.

static void DecodeExtra34(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	if (mCurStatus[6] == 0x00)
	{
		NComSetZeroVelLeverArmXAcc(Com, cast_2_byte_to_uint16(mCurStatus+0) * ZVPOSA2M);
		NComSetZeroVelLeverArmYAcc(Com, cast_2_byte_to_uint16(mCurStatus+2) * ZVPOSA2M);
		NComSetZeroVelLeverArmZAcc(Com, cast_2_byte_to_uint16(mCurStatus+4) * ZVPOSA2M);
	}
	else
	{
		Com->mIsZeroVelLeverArmXAccValid = 0;
		Com->mIsZeroVelLeverArmYAccValid = 0;
		Com->mIsZeroVelLeverArmZAccValid = 0;
	}
}


//============================================================================================================
//! \brief 35. Advanced slip lever arm.

static void DecodeExtra35(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	if (mCurStatus[6] == 0x00)
	{
		NComSetNoSlipLeverArmX(Com, cast_2_byte_to_int16(mCurStatus+0) * NSPOS2M);
		NComSetNoSlipLeverArmY(Com, cast_2_byte_to_int16(mCurStatus+2) * NSPOS2M);
		NComSetNoSlipLeverArmZ(Com, cast_2_byte_to_int16(mCurStatus+4) * NSPOS2M);
	}
	else
	{
		Com->mIsNoSlipLeverArmXValid = 0;
		Com->mIsNoSlipLeverArmYValid = 0;
		Com->mIsNoSlipLeverArmZValid = 0;
	}
}


//============================================================================================================
//! \brief 36. Advanced slip lever arm accuracy.

static void DecodeExtra36(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	if (mCurStatus[6] == 0x00)
	{
		NComSetNoSlipLeverArmXAcc(Com, cast_2_byte_to_uint16(mCurStatus+0) * NSPOSA2M);
		NComSetNoSlipLeverArmYAcc(Com, cast_2_byte_to_uint16(mCurStatus+2) * NSPOSA2M);
		NComSetNoSlipLeverArmZAcc(Com, cast_2_byte_to_uint16(mCurStatus+4) * NSPOSA2M);
	}
	else
	{
		Com->mIsNoSlipLeverArmXAccValid = 0;
		Com->mIsNoSlipLeverArmYAccValid = 0;
		Com->mIsNoSlipLeverArmZAccValid = 0;
	}
}


//============================================================================================================
//! \brief 37. Heading misalignment angle and accuracy.

static void DecodeExtra37(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	if ((mCurStatus[6] < NCOM_STDCNT_MAX) && IsDevIDAfter(Com, "030710"))
	{
		NComSetHeadingMisAlign(Com, cast_2_byte_to_int16(mCurStatus+0) * ALIGN2RAD  * RAD2DEG);
	}
	else
	{
		Com->mIsHeadingMisAlignValid = 0;
	}

	if ((mCurStatus[6] < NCOM_STDCNT_MAX) && IsDevIDAfter(Com, "030710"))
	{
		NComSetHeadingMisAlignAcc(Com, cast_2_byte_to_uint16(mCurStatus+2) * ALIGNA2RAD * RAD2DEG);
	}
	else
	{
		Com->mIsHeadingMisAlignAccValid = 0;
	}
}


//============================================================================================================
//! \brief 38. Zero velocity option settings.

static void DecodeExtra38(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	// To interpret quantities encoded with d2uc():
	//   1) If value of 0xFF then minus 1 for not set
	//   2) Else Convert to double then scale by constant

	if (mCurStatus[0] != 0xFF)
	{
		NComSetOptionSZVDelay(Com, (double) (mCurStatus[0]) * SZVDELAY2S);
	}
	else
	{
		NComSetOptionSZVDelay(Com, -1.0); Com->mIsOptionSZVDelayValid = 0; Com->mIsOptionSZVDelayConfig = 1;
	}

	if (mCurStatus[1] != 0xFF)
	{
		NComSetOptionSZVPeriod(Com, (double) (mCurStatus[1]) * SZVPERIOD2S);
	}
	else
	{
		NComSetOptionSZVPeriod(Com, -1.0); Com->mIsOptionSZVPeriodValid = 0; Com->mIsOptionSZVPeriodConfig = 1;
	}

	if ((mCurStatus[2] != 0xFF) || (mCurStatus[3] != 0xFF))
	{
		NComSetOptionTopSpeed(Com, cast_2_byte_to_uint16(mCurStatus+2) * TOPSPEED2MPS);
	}
	else
	{
		NComSetOptionTopSpeed(Com, -1.0); Com->mIsOptionTopSpeedValid = 0; Com->mIsOptionTopSpeedConfig = 1;
	}

	if (mCurStatus[4] != 0xFF)
	{
		NComSetOptionInitSpeed(Com, (double) (mCurStatus[4]) * INITSPEED2MPS);
	}
	else
	{
		NComSetOptionInitSpeed(Com, -1.0); Com->mIsOptionInitSpeedValid = 0; Com->mIsOptionInitSpeedConfig = 1;
	}

	if (mCurStatus[5] & 0x80) Com->mIsOptionSer3Valid = 0; else NComSetOptionSer3Enum(Com, mCurStatus[5]);
}


//============================================================================================================
//! \brief 39. No slip option settings.

static void DecodeExtra39(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	if (mCurStatus[0] != 0xFF)
	{
		NComSetOptionNSDelay(Com, (double) (mCurStatus[0]) * NSDELAY2S);
	}
	else
	{
		NComSetOptionNSDelay(Com, -1.0); Com->mIsOptionNSDelayValid = 0; Com->mIsOptionNSDelayConfig = 1;
	}

	if (mCurStatus[1] != 0xFF)
	{
		NComSetOptionNSPeriod(Com, (double) (mCurStatus[1]) * NSPERIOD2S);
	}
	else
	{
		NComSetOptionNSPeriod(Com, -1.0); Com->mIsOptionNSPeriodValid = 0; Com->mIsOptionNSPeriodConfig = 1;
	}

	if ((mCurStatus[2] != 0xFF) || (mCurStatus[3] != 0xFF))
	{
		NComSetOptionNSAngleStd(Com, cast_2_byte_to_uint16(mCurStatus+2) * ANGA2RAD * RAD2DEG);
	}
	else
	{
		NComSetOptionNSAngleStd(Com, -1.0); Com->mIsOptionNSAngleStdValid = 0; Com->mIsOptionNSAngleStdConfig = 1;
	}

	if (mCurStatus[4] != 0xFF)
	{
		NComSetOptionNSHAccel(Com, (double) (mCurStatus[4]) * NSACCEL2MPS2);
	}
	else
	{
		NComSetOptionNSHAccel(Com, -1.0); Com->mIsOptionNSHAccelValid = 0; Com->mIsOptionNSHAccelConfig = 1;
	}

	if (mCurStatus[5] != 0xFF)
	{
		NComSetOptionNSVAccel(Com, (double) (mCurStatus[5]) * NSACCEL2MPS2);
	}
	else
	{
		NComSetOptionNSVAccel(Com, -1.0); Com->mIsOptionNSVAccelValid = 0; Com->mIsOptionNSVAccelConfig = 1;
	}

	if (mCurStatus[6] != 0xFF)
	{
		NComSetOptionNSSpeed(Com, (double) (mCurStatus[6]) * NSSPEED2MPS);
	}
	else
	{
		NComSetOptionNSSpeed(Com, -1.0); Com->mIsOptionNSSpeedValid = 0; Com->mIsOptionNSSpeedConfig = 1;
	}

	if (mCurStatus[7] != 0xFF)
	{
		NComSetOptionNSRadius(Com, (double) (mCurStatus[7]) * NSRADIUS2M);
	}
	else
	{
		NComSetOptionNSRadius(Com, -1.0); Com->mIsOptionNSRadiusValid = 0; Com->mIsOptionNSRadiusConfig = 1;
	}
}


//============================================================================================================


//============================================================================================================
//! \brief 41. Output baud rates.

static void DecodeExtra41(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	NComSetOptionSer1BaudEnum(Com, mCurStatus[0] & 0x0F);
	NComSetOptionSer2BaudEnum(Com, mCurStatus[1] & 0x0F);
	NComSetOptionSer3BaudEnum(Com, mCurStatus[2] & 0x0F);
	NComSetOptionCanBaudEnum (Com, mCurStatus[3] & 0x0F);
}


//============================================================================================================
//! \brief 42. Heading lock options.

static void DecodeExtra42(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	if (mCurStatus[0] != 0xFF)
	{
		NComSetOptionHLDelay(Com, (double) (mCurStatus[0]) * HLDELAY2S);
	}
	else
	{
		NComSetOptionHLDelay(Com, -1.0); Com->mIsOptionHLDelayValid = 0; Com->mIsOptionHLDelayConfig = 1;
	}

	if (mCurStatus[1] != 0xFF)
	{
		NComSetOptionHLPeriod(Com, (double) (mCurStatus[1]) * HLPERIOD2S);
	}
	else
	{
		NComSetOptionHLPeriod(Com, -1.0); Com->mIsOptionHLPeriodValid = 0; Com->mIsOptionHLPeriodConfig = 1;
	}

	if ((mCurStatus[2] != 0xFF) || (mCurStatus[3] != 0xFF))
	{
		NComSetOptionHLAngleStd(Com, (double) (cast_2_byte_to_uint16(mCurStatus+2)) * ANGA2RAD * RAD2DEG);
	}
	else
	{
		NComSetOptionHLAngleStd(Com, -1.0); Com->mIsOptionHLAngleStdValid = 0; Com->mIsOptionHLAngleStdConfig = 1;
	}

	if (mCurStatus[4] != 0xFF)
	{
		NComSetOptionStatDelay(Com, (double) (mCurStatus[4]) * STATDELAY2S);
	}
	else
	{
		NComSetOptionStatDelay(Com, -1.0); Com->mIsOptionStatDelayValid = 0; Com->mIsOptionStatDelayConfig = 1;
	}

	if (mCurStatus[5] != 0xFF)
	{
		NComSetOptionStatSpeed(Com, (double) (mCurStatus[5]) * STATSPEED2MPS);
	}
	else
	{
		NComSetOptionStatSpeed(Com, -1.0); Com->mIsOptionStatSpeedValid = 0; Com->mIsOptionStatSpeedConfig = 1;
	}
}


//============================================================================================================
//! \brief 43. Trigger2 event timing (rising edge triggers).

static void DecodeExtra43(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	if (Com->mInternal->mTrig2Count != mCurStatus[7])
	{
		double min, ms, c;

		Com->mInternal->mTrig2Count = mCurStatus[7];

		min = cast_4_byte_to_int32 (mCurStatus+0);
		ms  = cast_2_byte_to_uint16(mCurStatus+4);
		c   = (int8_t)              mCurStatus[6];

		NComSetTrig2Time(Com, min * 60.0 + ms * 0.001 + c * FINETIME2SEC);
	}
}


//============================================================================================================
//! \brief 44. Wheel speed configuration.

static void DecodeExtra44(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	if ((mCurStatus[0] != 0xFF) || (mCurStatus[1] != 0xFF))
	{
		NComSetWSpeedScale(Com, (double) (cast_2_byte_to_uint16(mCurStatus+0)) * WSSF2PPM);
	}
	else
	{
		NComSetWSpeedScale(Com, -1.0); Com->mIsWSpeedScaleValid = 0; Com->mIsWSpeedScaleConfig = 1;
	}

	if ((mCurStatus[2] != 0xFF) || (mCurStatus[3] != 0xFF))
	{
		NComSetWSpeedScaleStd(Com, (double) (cast_2_byte_to_uint16(mCurStatus+2)) * WSSFA2PC);
	}
	else
	{
		NComSetWSpeedScaleStd(Com, -1.0); Com->mIsWSpeedScaleStdValid = 0; Com->mIsWSpeedScaleStdConfig = 1;
	}

	if (mCurStatus[4] != 0xFF)
	{
		NComSetOptionWSpeedDelay(Com, (double) (mCurStatus[4]) * WSDELAY2S);
	}
	else
	{
		NComSetOptionWSpeedDelay(Com, -1.0); Com->mIsOptionWSpeedDelayValid = 0; Com->mIsOptionWSpeedDelayConfig = 1;
	}

	if (mCurStatus[5] != 0xFF)
	{
		NComSetOptionWSpeedZVDelay(Com, (double) (mCurStatus[5]) * WSDELAY2S);
	}
	else
	{
		NComSetOptionWSpeedZVDelay(Com, -1.0); Com->mIsOptionWSpeedZVDelayValid = 0; Com->mIsOptionWSpeedZVDelayConfig = 1;
	}

	if (mCurStatus[6] != 0xFF)
	{
		NComSetOptionWSpeedNoiseStd(Com, (double) (mCurStatus[6]) * WSNOISE2CNT);
	}
	else
	{
		NComSetOptionWSpeedNoiseStd(Com, -1.0); Com->mIsOptionWSpeedNoiseStdValid = 0; Com->mIsOptionWSpeedNoiseStdConfig = 1;
	}
}


//============================================================================================================
//! \brief 45. Wheel speed counts.

static void DecodeExtra45(NComRxC *Com)
{
	NComRxCInternal     *ComI    = Com->mInternal;
	const unsigned char *mCurStatus = ComI->mCurStatus;

	// Wheel speed tacho measurements
	NComSetWSpeedCount(Com, (double) (cast_4_byte_to_uint32(mCurStatus+0)));

	// Timestamp needs aligning to global time stamp
	if (((mCurStatus[4] != 0xFF) || (mCurStatus[5] != 0xFF)) && Com->mIsTimeValid)
	{
		double ms      = cast_2_byte_to_uint16(mCurStatus+4);
		double fix_min = floor(0.5 + ((Com->mTime - ms * 0.001) / 60.0));

		NComSetWSpeedTime(Com, 60.0 * fix_min + ms * 0.001);
	}
	else
	{
		Com->mIsWSpeedTimeValid = 0;
	}

	// Duration since last tacho input change
	if (mCurStatus[6] != 0xFF)
	{
		NComSetWSpeedTimeUnchanged(Com, (double) (mCurStatus[6]) * WSDELAY2S);
	}
	else
	{
		Com->mIsWSpeedTimeUnchangedValid = 0;
	}

	// Local computation of tacho frequency
	if (Com->mIsWSpeedTimeValid && Com->mIsWSpeedCountValid)
	{
		// Compute tacho frequency if possible
		if (ComI->mIsOldWSpeedTimeValid && ComI->mIsOldWSpeedCountValid && (Com->mWSpeedTime > ComI->mOldWSpeedTime))
		{
			NComSetWSpeedFreq(Com, (Com->mWSpeedCount - ComI->mOldWSpeedCount) / (Com->mWSpeedTime - ComI->mOldWSpeedTime));
		}
		else
		{
			Com->mIsWSpeedFreqValid = 0;
		}

		// Update housekeeping parameters
		ComI->mIsOldWSpeedTimeValid  = 1; ComI->mOldWSpeedTime  = Com->mWSpeedTime;
		ComI->mIsOldWSpeedCountValid = 1; ComI->mOldWSpeedCount = Com->mWSpeedCount;
	}
}


//============================================================================================================
//! \brief 46. Wheel speed lever arm.

static void DecodeExtra46(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	if (mCurStatus[6] == 0x00)
	{
		NComSetWSpeedLeverArmX(Com, cast_2_byte_to_int16(mCurStatus+0) * WSPOS2M);
		NComSetWSpeedLeverArmY(Com, cast_2_byte_to_int16(mCurStatus+2) * WSPOS2M);
		NComSetWSpeedLeverArmZ(Com, cast_2_byte_to_int16(mCurStatus+4) * WSPOS2M);
	}
	else
	{
		Com->mIsWSpeedLeverArmXValid = 0;
		Com->mIsWSpeedLeverArmYValid = 0;
		Com->mIsWSpeedLeverArmZValid = 0;
	}
}


//============================================================================================================
//! \brief 47. Wheel speed lever arm accuracy.

static void DecodeExtra47(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	if (mCurStatus[6] == 0x00)
	{
		NComSetWSpeedLeverArmXAcc(Com, cast_2_byte_to_uint16(mCurStatus+0) * WSPOSA2M);
		NComSetWSpeedLeverArmYAcc(Com, cast_2_byte_to_uint16(mCurStatus+2) * WSPOSA2M);
		NComSetWSpeedLeverArmZAcc(Com, cast_2_byte_to_uint16(mCurStatus+4) * WSPOSA2M);
	}
	else
	{
		Com->mIsWSpeedLeverArmXAccValid = 0;
		Com->mIsWSpeedLeverArmYAccValid = 0;
		Com->mIsWSpeedLeverArmZAccValid = 0;
	}
}


//============================================================================================================
//! \brief 48. Undulation and dilution of precision of GPS.

static void DecodeExtra48(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	double pdop, hdop;

	// Undulation
	if ((mCurStatus[0] != 0x00) || (mCurStatus[1] != 0x80))
		NComSetUndulation(Com, (double) (cast_2_byte_to_int16(mCurStatus+0)) * UNDUL2M);

	// Dilution Of Precision
	if (mCurStatus[2] != 0xFF)
		NComSetHDOP(Com, (double) (mCurStatus[2]) * DOPFACTOR);
	else
		Com->mIsHDOPValid = 0;

	if (mCurStatus[3] != 0xFF)
		NComSetPDOP(Com, (double) (mCurStatus[3]) * DOPFACTOR);
	else
		Com->mIsPDOPValid = 0;

	if (Com->mIsHDOPValid && Com->mIsPDOPValid && ((pdop = Com->mPDOP) > (hdop = Com->mHDOP)))
		NComSetVDOP(Com, sqrt(pdop * pdop - hdop * hdop));
	else
		Com->mIsVDOPValid = 0;
}


//============================================================================================================
//! \brief 49. OmniStar tracking information.

static void DecodeExtra49(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	// OmniStar frequency
	if ((mCurStatus[0] != 0xFF) || (mCurStatus[1] != 0xFF))
	{
		NComSetOmniStarFreq(Com, (double) (cast_2_byte_to_uint16(mCurStatus+0)) * OMNIFREQ2HZ + OMNISTAR_MIN_FREQ);
	}
	else
	{
		Com->mIsOmniStarFreqValid = 0;
	}

	// OmniStar SNR
	if (mCurStatus[2] != 0xFF)
	{
		NComSetOmniStarSNR(Com, (double) (mCurStatus[2]) * SNR2DB);
	}
	else
	{
		Com->mIsOmniStarSNRValid = 0;
	}

	// OmniStar lock time
	if (mCurStatus[3] != 0xFF)
	{
		NComSetOmniStarLockTime(Com, (double) (mCurStatus[3]) * LTIME2SEC);
	}
	else
	{
		Com->mIsOmniStarLockTimeValid = 0;
	}

	// OmniStar Virtual Base Station status
	if (mCurStatus[4] != NCOM_OMNI_STATUS_UNKNOWN)
	{
		NComSetOmniStatusVbsExpired      (Com, (mCurStatus[4] & NCOM_OMNI_STATUS_VBSEXPIRED) != 0);
		NComSetOmniStatusVbsOutOfRegion  (Com, (mCurStatus[4] & NCOM_OMNI_STATUS_VBSREGION ) != 0);
		NComSetOmniStatusVbsNoRemoteSites(Com, (mCurStatus[4] & NCOM_OMNI_STATUS_VBSNOBASE ) != 0);
	}
	else
	{
		Com->mIsOmniStatusVbsExpiredValid       = 0;
		Com->mIsOmniStatusVbsOutOfRegionValid   = 0;
		Com->mIsOmniStatusVbsNoRemoteSitesValid = 0;
	}

	// OmniStar High Performance status
	if (mCurStatus[4] != NCOM_OMNI_STATUS_UNKNOWN)
	{
		NComSetOmniStatusHpExpired      (Com, (mCurStatus[4] & NCOM_OMNI_STATUS_HPEXPIRED   ) != 0);
		NComSetOmniStatusHpOutOfRegion  (Com, (mCurStatus[4] & NCOM_OMNI_STATUS_HPREGION    ) != 0);
		NComSetOmniStatusHpNoRemoteSites(Com, (mCurStatus[4] & NCOM_OMNI_STATUS_HPNOBASE    ) != 0);
		NComSetOmniStatusHpNotConverged (Com, (mCurStatus[4] & NCOM_OMNI_STATUS_HPNOCONVERGE) != 0);
		NComSetOmniStatusHpKeyInvalid   (Com, (mCurStatus[4] & NCOM_OMNI_STATUS_HPKEYINVALID) != 0);
	}
	else
	{
		Com->mIsOmniStatusHpExpiredValid       = 0;
		Com->mIsOmniStatusHpOutOfRegionValid   = 0;
		Com->mIsOmniStatusHpNoRemoteSitesValid = 0;
		Com->mIsOmniStatusHpNotConvergedValid  = 0;
		Com->mIsOmniStatusHpKeyInvalidValid    = 0;
	}

	// OmniStar serial number
	if ((mCurStatus[5] != 0xFF) || (mCurStatus[6] != 0xFF) || (mCurStatus[7] != 0xFF))
	{
		//! \todo Improve serial transmission format as the current 24 bit number does not suffice.

		char buffer[16];

		uint32_t sn = cast_3_byte_to_uint32(mCurStatus+5);

		//! \todo We can end up with the wrong format if channel 49 is received before channel 31.

		if (Com->mGpsPrimary->mIsTypeValid && (Com->mGpsPrimary->mType == GPS_TYPE_OEMV) && (sn < 700000))
		{
			sprintf(buffer, "007-0%06" PRIu32, sn);
		}
		else
		{
			sprintf(buffer, "%" PRIu32, sn);
		}

		NComSetOmniStarSerial(Com, buffer, 15);
	}
	else
	{
		Com->mIsOmniStarSerialValid = 0;
	}
}


//============================================================================================================
//! \brief 50. Information sent to the command decoder.

static void DecodeExtra50(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	NComSetCmdChars       (Com, incr_2_byte_to_uint32(mCurStatus+0, Com->mCmdChars       ));
	NComSetCmdPkts        (Com, incr_2_byte_to_uint32(mCurStatus+2, Com->mCmdPkts        ));
	NComSetCmdCharsSkipped(Com, incr_2_byte_to_uint32(mCurStatus+4, Com->mCmdCharsSkipped));
	NComSetCmdErrors      (Com, incr_2_byte_to_uint32(mCurStatus+6, Com->mCmdErrors      ));
}


//============================================================================================================


//============================================================================================================


//============================================================================================================


//============================================================================================================


//============================================================================================================
//! \brief 55. Information about the primary GPS receiver.

static void DecodeExtra55(NComRxC *Com)
{
	DecodeExtraGpsStatus(Com->mInternal->mCurStatus, Com->mGpsPrimary);
}


//============================================================================================================
//! \brief 56. Information about the secondary GPS receiver.

static void DecodeExtra56(NComRxC *Com)
{
	DecodeExtraGpsStatus(Com->mInternal->mCurStatus, Com->mGpsSecondary);
}


//============================================================================================================
//! \brief 57. Position estimate of the primary GPS antenna (extended range).

static void DecodeExtra57(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	if ((mCurStatus[6] < NCOM_COUNT_TOO_OLD) && (mCurStatus[7] != 0x00))
	{
		double sf;

		if (mCurStatus[7] == 0xFF) // saturation condition
			sf = 1.0;              // to match standard saturation values
		else
			sf = mCurStatus[7];

		NComSetGAPx(Com, ((double) cast_2_byte_to_int16(mCurStatus+0)) * sf * GPSPOS2M);
		NComSetGAPy(Com, ((double) cast_2_byte_to_int16(mCurStatus+2)) * sf * GPSPOS2M);
		NComSetGAPz(Com, ((double) cast_2_byte_to_int16(mCurStatus+4)) * sf * GPSPOS2M);
	}
	else
	{
		Com->mIsGAPxValid = 0;
		Com->mIsGAPyValid = 0;
		Com->mIsGAPzValid = 0;
	}
}


//============================================================================================================


//============================================================================================================
//! \brief 59. IMU decoding status.

static void DecodeExtra59(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	NComSetImuMissedPkts(Com, incr_2_byte_to_uint32(mCurStatus+0, Com->mImuMissedPkts));
	NComSetImuResetCount(Com, incr_1_byte_to_uint32(mCurStatus+2, Com->mImuResetCount));
	NComSetImuErrorCount(Com, incr_1_byte_to_uint32(mCurStatus+3, Com->mImuErrorCount));
}


//============================================================================================================


//============================================================================================================
//! \brief 61. Received information about external GPS receiver.

static void DecodeExtra61(NComRxC *Com)
{
	DecodeExtraGpsReceived(Com->mInternal->mCurStatus, Com->mGpsExternal);
}


//============================================================================================================
//! \brief 62. Information about the external GPS receiver.

static void DecodeExtra62(NComRxC *Com)
{
	DecodeExtraGpsStatus(Com->mInternal->mCurStatus, Com->mGpsExternal);
}


//============================================================================================================


//============================================================================================================
//! \brief 63. Characteristics of the angular acceleration low-pass filter.

static void DecodeExtra63(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	float f;

	// Decode the angular acceleration filter frequency
	f = cast_4_byte_to_float(mCurStatus+0);
	if (f < 0.0)
		Com->mIsAngAccFiltFreqValid = 0;
	else
		NComSetAngAccFiltFreq(Com, (double) f);

	// Decode the angular acceleration filter damping ratio
	f = cast_4_byte_to_float(mCurStatus+4);
	if (f < 0.0)
		Com->mIsAngAccFiltZetaValid = 0;
	else
		NComSetAngAccFiltZeta(Com, (double) f);

	// Pass the parameters to the angular acceleration filters
	if (Com->mIsAngAccFiltFreqValid && Com->mIsAngAccFiltZetaValid && Com->mHasAngAccFiltChanged)
	{
		NComRxCInternal *ComI = Com->mInternal;
		Filt2ndOrderSetCharacteristics(&ComI->FiltForYx, Com->mAngAccFiltFreq, Com->mAngAccFiltZeta);
		Filt2ndOrderSetCharacteristics(&ComI->FiltForYy, Com->mAngAccFiltFreq, Com->mAngAccFiltZeta);
		Filt2ndOrderSetCharacteristics(&ComI->FiltForYz, Com->mAngAccFiltFreq, Com->mAngAccFiltZeta);

		Com->mHasAngAccFiltChanged = 0;
	}
}


//============================================================================================================


//============================================================================================================
//! \brief 64. Hardware information and GPS receiver configurations.

static void DecodeExtra64(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	if (mCurStatus[0] == 0xFF) Com->mIsCpuPcbTypeValid = 0; else NComSetCpuPcbTypeEnum(Com, mCurStatus[0]);

	if (mCurStatus[1] == 0xFF) Com->mGpsExternal->mIsTypeValid   = 0; else NComGpsSetTypeEnum  (Com->mGpsExternal, mCurStatus[1]);
	if (mCurStatus[2] == 0xFF) Com->mGpsExternal->mIsFormatValid = 0; else NComGpsSetFormatEnum(Com->mGpsExternal, mCurStatus[2]);

	if (mCurStatus[3] == 0xFF) Com->mIsDualPortRamStatusValid = 0; else NComSetDualPortRamStatusEnum(Com, mCurStatus[3]);

	{
		uint8_t g1_pos = mCurStatus[4] & UINT8_C(0x0F);
		uint8_t g1_vel = mCurStatus[4] >> 4;
		uint8_t g1_raw = mCurStatus[5] & UINT8_C(0x0F);

		if (g1_pos == UINT8_C(0x0F)) Com->mGpsPrimary->mIsPosRateValid = 0; else NComGpsSetPosRateEnum(Com->mGpsPrimary, g1_pos);
		if (g1_vel == UINT8_C(0x0F)) Com->mGpsPrimary->mIsVelRateValid = 0; else NComGpsSetVelRateEnum(Com->mGpsPrimary, g1_vel);
		if (g1_raw == UINT8_C(0x0F)) Com->mGpsPrimary->mIsRawRateValid = 0; else NComGpsSetRawRateEnum(Com->mGpsPrimary, g1_raw);
	}

	{
		uint8_t g2_raw = mCurStatus[5] >> 4;

		if (g2_raw == UINT8_C(0x0F)) Com->mGpsSecondary->mIsRawRateValid = 0; else NComGpsSetRawRateEnum(Com->mGpsSecondary, g2_raw);
	}

	{
		NComSetGnssGpsEnabled(Com, 1);
	}

	{
		unsigned char gf = ~mCurStatus[6];

		if (gf & NCOM_GPS_FEATURE_VALID)
		{
			NComSetGnssGlonassEnabled(Com, (gf & NCOM_GPS_FEATURE_GLONASS) ? 1 : 0);
			NComSetGnssGalileoEnabled(Com, (gf & NCOM_GPS_FEATURE_GALILEO) ? 1 : 0);
		}

		if (gf & NCOM_GPS_FEATURE_VALID)
		{
			NComSetRawRngEnabled(Com, (gf & NCOM_GPS_FEATURE_RAWRNG) ? 1 : 0);
			NComSetRawDopEnabled(Com, (gf & NCOM_GPS_FEATURE_RAWDOP) ? 1 : 0);
			NComSetRawL1Enabled (Com, (gf & NCOM_GPS_FEATURE_RAWL1 ) ? 1 : 0);
			NComSetRawL2Enabled (Com, (gf & NCOM_GPS_FEATURE_RAWL2 ) ? 1 : 0);
			NComSetRawL5Enabled (Com, (gf & NCOM_GPS_FEATURE_RAWL5 ) ? 1 : 0);
		}
	}
}


//============================================================================================================
//! \brief 65. Camera output event timing.

static void DecodeExtra65(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	if (Com->mInternal->mDigitalOutCount != mCurStatus[7])
	{
		double min, ms, c;

		Com->mInternal->mDigitalOutCount = mCurStatus[7];

		min = cast_4_byte_to_int32 (mCurStatus+0);
		ms  = cast_2_byte_to_uint16(mCurStatus+4);
		c   = (int8_t)              mCurStatus[6];

		NComSetDigitalOutTime(Com, min * 60.0 + ms * 0.001 + c * FINETIME2SEC);
	}
}


//============================================================================================================
//! \brief 66. Extended local co-ordinate definition for latitude and longitude.

static void DecodeExtra66(NComRxC *Com)
{
	NComRxCInternal *ComI = Com->mInternal;
	const unsigned char *mCurStatus = ComI->mCurStatus;

	int32_t l;

	// Decode the latitude information
	l = cast_4_byte_to_int32(mCurStatus+0);
	if (l != ((int32_t)(UINT32_C(0x80000000))))
	{
		// Update the accurate representation of latitude
		ComI->mAccurateRefLat = ((double) l) * FINEANG2RAD * RAD2DEG;
		ComI->mIsAccurateRefLatValid = 1;
	}
	else
	{
		ComI->mIsAccurateRefLatValid = 0;
	}

	// Decode the longitude information
	l = cast_4_byte_to_int32(mCurStatus+4);
	if (l != ((int32_t)(UINT32_C(0x80000000))))
	{
		// Update the accurate representation of longitude
		ComI->mAccurateRefLon = ((double) l) * FINEANG2RAD * RAD2DEG;
		ComI->mIsAccurateRefLonValid = 1;
	}
	else
	{
		ComI->mIsAccurateRefLonValid = 0;
	}

	// If we have all the required information set the local reference frame
	if (ComI->mIsAccurateRefLatValid && ComI->mIsAccurateRefLonValid && ComI->mIsAccurateRefAltValid && ComI->mIsAccurateRefHeadingValid)
	{
		SetRefFrame(Com, ComI->mAccurateRefLat, ComI->mAccurateRefLon, ComI->mAccurateRefAlt, ComI->mAccurateRefHeading);
	}
}


//============================================================================================================
//! \brief 67. Extended local co-ordinate definition for altitude and heading.

static void DecodeExtra67(NComRxC *Com)
{
	NComRxCInternal *ComI = Com->mInternal;
	const unsigned char *mCurStatus = ComI->mCurStatus;

	int32_t l;

	// Decode the altitude information
	l = cast_4_byte_to_int32(mCurStatus+0);
	if (l != ((int32_t)(UINT32_C(0x80000000))))
	{
		// Update the accurate representation of altitude
		ComI->mAccurateRefAlt = ((double) l) * ALT2M;
		ComI->mIsAccurateRefAltValid = 1;
	}
	else
	{
		ComI->mIsAccurateRefAltValid = 0;
	}

	// Decode the heading information
	l = cast_4_byte_to_int32(mCurStatus+4);
	if (l != ((int32_t)(UINT32_C(0x80000000))))
	{
		// Update the accurate representation of heading
		ComI->mAccurateRefHeading = ((double) l) * FINEANG2RAD * RAD2DEG;
		ComI->mIsAccurateRefHeadingValid = 1;
	}
	else
	{
		ComI->mIsAccurateRefHeadingValid = 0;
	}

	// If we have all the required information set the local reference frame
	if (ComI->mIsAccurateRefLatValid && ComI->mIsAccurateRefLonValid && ComI->mIsAccurateRefAltValid && ComI->mIsAccurateRefHeadingValid)
	{
		SetRefFrame(Com, ComI->mAccurateRefLat, ComI->mAccurateRefLon, ComI->mAccurateRefAlt, ComI->mAccurateRefHeading);
	}
}


//============================================================================================================


//============================================================================================================


//============================================================================================================


//============================================================================================================


//============================================================================================================
//! \brief 72. Accelerometer scale factor.

static void DecodeExtra72(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	if (mCurStatus[6] < NCOM_COUNT_TOO_OLD)
	{
		NComSetAxSf(Com, cast_2_byte_to_int16(mCurStatus+0) * ASFACTOR);
		NComSetAySf(Com, cast_2_byte_to_int16(mCurStatus+2) * ASFACTOR);
		NComSetAzSf(Com, cast_2_byte_to_int16(mCurStatus+4) * ASFACTOR);
	}
	else
	{
		Com->mIsAxSfValid = 0;
		Com->mIsAySfValid = 0;
		Com->mIsAzSfValid = 0;
	}
}


//============================================================================================================
//! \brief 73. Accelerometer scale factor accuracy.

static void DecodeExtra73(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	if (mCurStatus[6] < NCOM_COUNT_TOO_OLD)
	{
		NComSetAxSfAcc(Com, cast_2_byte_to_uint16(mCurStatus+0) * ASAFACTOR);
		NComSetAySfAcc(Com, cast_2_byte_to_uint16(mCurStatus+2) * ASAFACTOR);
		NComSetAzSfAcc(Com, cast_2_byte_to_uint16(mCurStatus+4) * ASAFACTOR);
	}
	else
	{
		Com->mIsAxSfAccValid = 0;
		Com->mIsAySfAccValid = 0;
		Com->mIsAzSfAccValid = 0;
	}
}


//============================================================================================================


//============================================================================================================
//! \brief 74. Characteristics of the linear acceleration low-pass filter.

static void DecodeExtra74(NComRxC *Com)
{
	const unsigned char *mCurStatus = Com->mInternal->mCurStatus;

	float f;

	// Decode the linear acceleration filter frequency
	f = cast_4_byte_to_float(mCurStatus+0);
	if (f < 0.0)
		Com->mIsLinAccFiltFreqValid = 0;
	else
		NComSetLinAccFiltFreq(Com, (double) f);

	// Decode the linear acceleration filter damping ratio
	f = cast_4_byte_to_float(mCurStatus+4);
	if (f < 0.0)
		Com->mIsLinAccFiltZetaValid = 0;
	else
		NComSetLinAccFiltZeta(Com, (double) f);

	// Pass the parameters to the linear acceleration filters
	if (Com->mIsLinAccFiltFreqValid && Com->mIsLinAccFiltZetaValid && Com->mHasLinAccFiltChanged)
	{
		NComRxCInternal *ComI = Com->mInternal;
		Filt2ndOrderSetCharacteristics(&ComI->FiltForAx, Com->mLinAccFiltFreq, Com->mLinAccFiltZeta);
		Filt2ndOrderSetCharacteristics(&ComI->FiltForAy, Com->mLinAccFiltFreq, Com->mLinAccFiltZeta);
		Filt2ndOrderSetCharacteristics(&ComI->FiltForAz, Com->mLinAccFiltFreq, Com->mLinAccFiltZeta);

		Com->mHasLinAccFiltChanged = 0;
	}
}


//============================================================================================================
//! \brief Decode received information about a GPS receiver.

static void DecodeExtraGpsReceived(const unsigned char *mCurStatus, NComRxCGps *Com)
{
	NComGpsSetChars       (Com, incr_2_byte_to_uint32(mCurStatus+0, Com->mChars       ));
	NComGpsSetPkts        (Com, incr_2_byte_to_uint32(mCurStatus+2, Com->mPkts        ));
	NComGpsSetCharsSkipped(Com, incr_2_byte_to_uint32(mCurStatus+4, Com->mCharsSkipped));
	NComGpsSetOldPkts     (Com, incr_2_byte_to_uint32(mCurStatus+6, Com->mOldPkts     ));
}


//============================================================================================================
//! \brief Decode status information about a GPS receiver.

static void DecodeExtraGpsStatus(const unsigned char *mCurStatus, NComRxCGps *Com)
{
	uint8_t x = 0;

	// Antenna status and power, position mode

	x = mCurStatus[0] & NCOM_GPS_ANT_STATUS_BITMASK;

	if (x == NCOM_GPS_ANT_STATUS_DONTKNOW)
	{
		Com->mIsAntStatusValid = 0;
	}
	else
	{
		NComGpsSetAntStatusEnum(Com, x >> NCOM_GPS_ANT_STATUS_BITSHIFT);
	}

	x = mCurStatus[0] & NCOM_GPS_ANT_POWER_BITMASK;

	if (x == NCOM_GPS_ANT_POWER_DONTKNOW)
	{
		Com->mIsAntPowerValid = 0;
	}
	else
	{
		NComGpsSetAntPowerEnum(Com, x >> NCOM_GPS_ANT_POWER_BITSHIFT);
	}

	if (mCurStatus[5] == 0xFF) Com->mIsPosModeValid = 0; else NComGpsSetPosModeEnum(Com, mCurStatus[5]);

	// Serial baud

	if (mCurStatus[3] == 0xFF) Com->mIsSerBaudValid = 0; else NComGpsSetSerBaudEnum(Com, mCurStatus[3]);

	// Number of satellites

	if (mCurStatus[4] == 0xFF) Com->mIsNumSatsValid = 0; else NComGpsSetNumSats(Com, mCurStatus[4]);

	// CPU, noise, temp, voltage

	if (mCurStatus[1] == 0xFF) Com->mIsCpuUsedValid    = 0; else NComGpsSetCpuUsed   (Com, mCurStatus[1]);
	if (mCurStatus[2] == 0xFF) Com->mIsCoreNoiseValid  = 0; else NComGpsSetCoreNoise (Com, mCurStatus[2]);
	if (mCurStatus[6] == 0xFF) Com->mIsCoreTempValid   = 0; else NComGpsSetCoreTemp  (Com, ((double) (mCurStatus[6])) + (TEMPK_OFFSET + ABSZERO_TEMPC));
	if (mCurStatus[7] == 0xFF) Com->mIsSupplyVoltValid = 0; else NComGpsSetSupplyVolt(Com, ((double) (mCurStatus[7])) * SUPPLYV2V);
}




//############################################################################################################
//##                                                                                                        ##
//##  Utilities                                                                                             ##
//##                                                                                                        ##
//############################################################################################################


//============================================================================================================
//! \brief Convert a 8 byte representation of a real to a basic 64 bit real type.

static double cast_8_byte_to_double(const uint8_t *b)
{
	union { double x; uint8_t c[8]; } u;
	u.c[0] = b[0];
	u.c[1] = b[1];
	u.c[2] = b[2];
	u.c[3] = b[3];
	u.c[4] = b[4];
	u.c[5] = b[5];
	u.c[6] = b[6];
	u.c[7] = b[7];
	return u.x;
}


//============================================================================================================
//! \brief Convert a 4 byte representation of a real to a basic 32 bit real type.

static float cast_4_byte_to_float(const uint8_t *b)
{
	union { float x; uint8_t c[4]; } u;
	u.c[0] = b[0];
	u.c[1] = b[1];
	u.c[2] = b[2];
	u.c[3] = b[3];
	return u.x;
}


//============================================================================================================
//! \brief Convert a 4 byte representation of a signed integer to a basic signed 32 bit integer type.

static int32_t cast_4_byte_to_int32(const uint8_t *b)
{
	union { int32_t x; uint8_t c[4]; } u;
	u.c[0] = b[0];
	u.c[1] = b[1];
	u.c[2] = b[2];
	u.c[3] = b[3];
	return u.x;
}


//============================================================================================================
//! \brief Convert a 4 byte representation of an unsigned integer to a basic unsigned 32 bit integer type.

static uint32_t cast_4_byte_to_uint32(const uint8_t *b)
{
	union { uint32_t x; uint8_t c[4]; } u;
	u.c[0] = b[0];
	u.c[1] = b[1];
	u.c[2] = b[2];
	u.c[3] = b[3];
	return u.x;
}


//============================================================================================================
//! \brief Convert a 3 byte representation of a signed integer to a basic signed 32 bit integer type.

static int32_t cast_3_byte_to_int32(const uint8_t *b)
{
	union { int32_t x; uint8_t c[4]; } u;
	u.c[1] = b[0];
	u.c[2] = b[1];
	u.c[3] = b[2];
	return u.x >> 8;
}


//============================================================================================================
//! \brief Convert a 3 byte representation of an unsigned integer to a basic unsigned 32 bit integer type.

static uint32_t cast_3_byte_to_uint32(const uint8_t *b)
{
	union { uint32_t x; uint8_t c[4]; } u;
	u.x    = 0;
	u.c[0] = b[0];
	u.c[1] = b[1];
	u.c[2] = b[2];
	return u.x;
}


//============================================================================================================
//! \brief Convert a 2 byte representation of a signed integer to a basic signed 16 bit integer type.

static int16_t cast_2_byte_to_int16(const uint8_t *b)
{
	union { int16_t x; uint8_t c[2]; } u;
	u.c[0] = b[0];
	u.c[1] = b[1];
	return u.x;
}


//============================================================================================================
//! \brief Convert a 2 byte representation of an unsigned integer to a basic unsigned 16 bit integer type.

static uint16_t cast_2_byte_to_uint16(const uint8_t *b)
{
	union { uint16_t x; uint8_t c[2]; } u;
	u.c[0] = b[0];
	u.c[1] = b[1];
	return u.x;
}


//============================================================================================================
//! \brief Converts.

static uint32_t incr_2_byte_to_uint32(const uint8_t *b, uint32_t z)
{
//	uint16_t lsb = cast_2_byte_to_uint16(b) - v;
//	return z - lsb;

	uint32_t x, y;

	x = cast_2_byte_to_uint16(b);
	y = z & 0x0000FFFF; // low part
	z = z & 0xFFFF0000; // high part
	if (y > x) // low part > x ?
		return (z + x + 0x10000);
	else
		return (z + x);
}


//============================================================================================================
//! \brief Converts.

static uint32_t incr_1_byte_to_uint32(const uint8_t *b, uint32_t z)
{
//	uint8_t lsb = *b - v;
//	return z - lsb;

	uint32_t x, y;

	x = *b;
	y = z & 0x000000FF; // low part
	z = z & 0xFFFFFF00; // high part
	if (y > x) // low part > x ?
		return (z + x + 0x100);
	else
		return (z + x);
}


//============================================================================================================
//! \brief Copy an unterminated string of given length into a possibly different sized target.
//!
//! The lengths of both arguments do not include the string termination null-character. The source string does
//! not need to be terminated (and if it is then the termination is not used). The destination string will
//! have a termination character written, thus the buffer length should be at least one greater than the
//! specified string length.

static void strgrab(char *destination, int destination_length, const char *source, int source_length)
{
	int n = (destination_length < source_length) ? destination_length : source_length;

	int i;

	for (i=0; i<n; i++) destination[i] = source[i];

	for (i=n; i<=destination_length; i++) destination[i] = '\0';
}




//############################################################################################################
//##                                                                                                        ##
//##  Filter functions                                                                                      ##
//##                                                                                                        ##
//############################################################################################################


//============================================================================================================
//! \brief Allocates and resets a 2nd order filter.

static Filt2ndOrder *Filt2ndOrderCreate()
{
	Filt2ndOrder *Filt = (Filt2ndOrder *)calloc(1, sizeof(Filt2ndOrder));

	if (Filt)
		Filt2ndOrderReset(Filt);

	return Filt;
}


//============================================================================================================
//! \brief Frees the 2nd order filter Filt.

static void Filt2ndOrderDestroy(Filt2ndOrder *Filt)
{
	free(Filt);
}


//============================================================================================================
//! \brief Resets 2nd order filter Filt.

static void Filt2ndOrderReset(Filt2ndOrder *Filt)
{
	// Set the design parameters as invalid

	Filt->mFreqSample = -1.0;
	Filt->mFreqCutoff = -1.0;
	Filt->mZeta       = -1.0;

	// Reset all filter coefficients

	Filt->mA0 = 0.0;
	Filt->mA1 = 0.0;
	Filt->mA2 = 0.0;
	Filt->mB0 = 0.0;
	Filt->mB1 = 0.0;
	Filt->mB2 = 0.0;

	// Set all timestamps as invalid

	Filt->mT0 = -1.0;
	Filt->mT1 = -1.0;
	Filt->mT2 = -1.0;

	// Reset all inputs and outputs

	Filt->mX0 = 0.0;
	Filt->mX1 = 0.0;
	Filt->mX2 = 0.0;
	Filt->mU0 = 0.0;
	Filt->mU1 = 0.0;
	Filt->mU2 = 0.0;

	// Indicate that the output is not valid

	Filt->mOutputValid = 0;
}


//============================================================================================================
//! \brief Sets the characteristics of 2nd order low-pass filter Filt.

static void Filt2ndOrderSetCharacteristics(Filt2ndOrder *Filt, double freq, double zeta)
{
	// Only accept strictly positive cut-off frequency and damping ratio
	// which are different from the previous design parameters

	if ((freq > 0.0) && (zeta > 0.0) && ((freq != Filt->mFreqCutoff)||(zeta != Filt->mZeta)))
	{
		Filt->mFreqCutoff = freq;
		Filt->mZeta       = zeta;

		// Attempt to initialise the filter straight away

		Filt2ndOrderInitialise(Filt);
	}
}


//============================================================================================================
//! \brief Feeds a new timed input into 2nd order filter Filt.

static void Filt2ndOrderNewInput(Filt2ndOrder *Filt, double t, double x)
{
	// Shuffle past inputs and outputs along

	Filt->mT2 = Filt->mT1;
	Filt->mX2 = Filt->mX1;
	Filt->mU2 = Filt->mU1;
	Filt->mT1 = Filt->mT0;
	Filt->mX1 = Filt->mX0;
	Filt->mU1 = Filt->mU0;

	// Read in the new input

	Filt->mT0 = t;
	Filt->mX0 = x;

	// Check if filter needs initialising

	if (!Filt->mOutputValid)             // if filter is not initialised
		Filt2ndOrderInitialise(Filt);         // Attempt to initialise the filter
	else                                 // if filter is initialised
	{
		// Could check excessive input time jitter ...
		// Compute the new output value

		Filt->mU0 = Filt->mA0 * Filt->mX0 + Filt->mA1 * Filt->mX1 + Filt->mA2 * Filt->mX2 - Filt->mB1 * Filt->mU1 - Filt->mB2 * Filt->mU2;
	}
}


//============================================================================================================
//! \brief Initialises 2nd order low-pass filter Filt if all required information is available.

static void Filt2ndOrderInitialise(Filt2ndOrder *Filt)
{
	// Check that all inputs and design parameters are valid

	if ((Filt->mT2 >= 0.0) && (Filt->mT1 >= 0.0) && (Filt->mT0 >= 0.0) && (Filt->mFreqCutoff > 0.0) && (Filt->mZeta > 0.0))
	{
		double d1, d2;

		// Try to determine the sampling frequency

		d1 = Filt->mT0 - Filt->mT1;
		d2 = Filt->mT1 - Filt->mT2;
		if ((d1 > 0.0) && (d2 > 0.0) && (fabs(d2/d1 - 1.0) < INPUT_JITTER_TOLERANCE))
		{
			double r;
			Filt->mFreqSample = 2.0 / (d1 + d2);

			// Check that the sampling frequency is greater than twice the cut-off frequency

			r = Filt->mFreqCutoff / Filt->mFreqSample;
			if (r < 0.5)
			{
				// Compute the filter coefficients

				double K = tan(M_PI * r);
				double Q = 0.5 / Filt->mZeta;
				double D = K * K + K / Q + 1.0;
				Filt->mA0 = K * K / D;
				Filt->mA1 = 2.0 * K * K / D;
				Filt->mA2 = K * K / D;
				Filt->mB0 = 1.0;
				Filt->mB1 = 2.0 * (K * K - 1.0) / D;
				Filt->mB2 = (K * K - K / Q + 1.0) / D;

				// Initialise the past outputs to suitable values

				Filt->mU2 = Filt->mX2;
				Filt->mU1 = Filt->mA0 * Filt->mX1 + Filt->mA1 * Filt->mX2 + Filt->mA2 * Filt->mX2 - Filt->mB1 * Filt->mU2 - Filt->mB2 * Filt->mU2;

				// Compute the new output value

				Filt->mU0 = Filt->mA0 * Filt->mX0 + Filt->mA1 * Filt->mX1 + Filt->mA2 * Filt->mX2 - Filt->mB1 * Filt->mU1 - Filt->mB2 * Filt->mU2;

				// Indicate that the output is valid

				Filt->mOutputValid = 1;
			}
			else
			{
				// Indicate that the output is no longer valid

				Filt->mOutputValid = 0;
			}
		}
		else
		{
			// Indicate that the output is no longer valid

			Filt->mOutputValid = 0;
		}
	}
}




//############################################################################################################
//##                                                                                                        ##
//##  Matrix Library                                                                                        ##
//##                                                                                                        ##
//############################################################################################################



//============================================================================================================
//! \brief Allocates the space for a matrix.

static int MatAllocR( Mat *R, long r, long c )
{
   R->m = (MatElement *)malloc( sizeof(MatElement) * (size_t)(r*c) );
   if (R->m == NULL)
   {
      return 1;
   }
   else
   {
      R->r = 0L;
      R->tr = r;
      R->c = 0L;
      R->tc = c;
      return 0;
   }
}


//============================================================================================================
//! \brief Fills an already allocated matrix with the elements provided by the parameter list.

static int MatFillR( Mat *R, long r, long c, ... )
{
   va_list ap;
   int i,j;

   R->r = r;
   R->c = c;

   va_start( ap, c );
   for( i = 0; i < r; i++ )
      for( j = 0; j < c; j++ )
         e(R,i,j) = (MatElement)va_arg(ap, double);
   va_end(ap);

   return 0;
}


//============================================================================================================
//! \brief Frees the matrix A.

static int MatFree( Mat *A )
{
   if (A && A->m)
   {
      free( A->m );
      A->m = NULL;
   }

   return 0;
}


//============================================================================================================
//! \brief Multiplies the matrices A and B placing the results in r.
//!
//! The operation's validity is checked only with EXTRA_CHECKS.

static int MatMultRAB( Mat *R, Mat *A, Mat *B )
{
   int i, j, k;
   MatElement sum;
   MatElement *Ap, *Aps, *Bp, *r;

   R->r = A->r;  // Set the size of the results, rows
   R->c = B->c;  // and columns

   for( i = 0; i < R->r; i++ ) // Counts the rows of r->m
   {
      r = R->m + i*(int)R->tc;  // R is the start of the row being calculated
      Aps = A->m + i*(int)A->tc;// Aps is the start of the A's row being used

      for( j = 0; j < R->c; j++ ) // Counts the columns of r->m
      {
         sum = 0.0;              // Initialise the sum
         Ap = Aps;               // Ap is the start of the row
         Bp = B->m + j;          // Bp is the start of the column
         for( k = 0; k < A->c; k++ )
         {
            sum += *(Ap++) * (*Bp);
            Bp += (int)B->tc;
         }
         *(r++) = sum;
      }
   }

   return 0;
}


//============================================================================================================
//! \brief Multiplies the matrices At and B placing the results in R.
//!
//! The operation's validity is only checked with EXTRA_CHECKS.

static int MatMultRAtB( Mat *R, Mat *A, Mat *B )
{
   int i, j, k;
   MatElement sum;
   MatElement *Ap, *Aps, *Bp, *r;

   R->r = A->c;  // Set the size of the results, rows
   R->c = B->c;  // and columns

   for( i = 0; i < R->r; i++ )     // Counts the rows of r->m
   {
      r = R->m + i*(int)R->tc;     // R is the start of the row being calculated
      Aps = A->m + i;              // Aps is the start of the A's column being used

      for( j = 0; j < R->c; j++ )  // Counts the columns of r->m
      {
         sum = 0.0;                // Initialise the sum
         Ap = Aps;                 // Ap is the start of the column
         Bp = B->m + j;            // Bp is the start of the column
         for( k = 0; k < A->r; k++ )
         {
            sum += (*Ap) * (*Bp);
            Ap += (int)A->tc;
            Bp += (int)B->tc;
         }
         *(r++) = sum;
      }
   }

   return 0;
}


//============================================================================================================
//! \brief Generates a direction cosine matrix from the pitch and roll by ignoring the heading angle
//! provided and setting the heading to 0.

static int Euler2DirCos2( Mat *C, Mat *E )
{
   double sin_p, cos_p, sin_r, cos_r;

   sin_p = sin( e(E,1,0) );
   cos_p = cos( e(E,1,0) );
   sin_r = sin( e(E,2,0) );
   cos_r = cos( e(E,2,0) );

   C->r = 3;
   C->c = 3;
   e(C,0,0) = cos_p;
   e(C,1,0) = 0.0;
   e(C,2,0) = -sin_p;
   e(C,0,1) = sin_p * sin_r;
   e(C,1,1) = cos_r;
   e(C,2,1) = cos_p * sin_r;
   e(C,0,2) = sin_p * cos_r;
   e(C,1,2) = -sin_r;
   e(C,2,2) = cos_p * cos_r;

   return 0;
}


//==================================   new

static int Euler2DirCos2_1( Mat *C, Mat *E )
{
   double sin_p, cos_p, sin_r, cos_r;

   sin_p = sin( e(E,1,0) );
   cos_p = cos( e(E,1,0) );
   sin_r = sin( e(E,2,0) );
   cos_r = cos( e(E,2,0) );

   C->r = 3;
   C->c = 3;
   e(C,0,0) = cos_p;
   e(C,1,0) = sin_p * sin_r;
   e(C,2,0) = sin_p * cos_r;
   e(C,0,1) = 0.0;
   e(C,1,1) = cos_r;
   e(C,2,1) = -sin_r;
   e(C,0,2) = -sin_p;
   e(C,1,2) = cos_p * sin_r;
   e(C,2,2) = cos_p * cos_r;

   return 0;
}



//============================================================================================================
//! \brief Generates a direction cosine matrix from the Heading.

static int Euler2DirCosH( Mat *C, Mat *E )
{
   double sin_h, cos_h;

   sin_h = sin( e(E,0,0) );
   cos_h = cos( e(E,0,0) );

   C->r = 3;
   C->c = 3;
   e(C,0,0) = cos_h;
   e(C,1,0) = sin_h;
   e(C,2,0) = 0.0;
   e(C,0,1) = -sin_h;
   e(C,1,1) = cos_h;
   e(C,2,1) = 0.0;
   e(C,0,2) = 0.0;
   e(C,1,2) = 0.0;
   e(C,2,2) = 1.0;

   return 0;
}


//============================================================================================================
// WGS-84 constants.

#define EARTH_EQUAT_RADIUS   (6378137.0)         // m
#define EARTH_ECCENTRICITY   (0.0818191908426)


//============================================================================================================
//! \brief Computes the Earth radii of curvature as a function of latitude.

static int compute_earth_curvature(double *rho_e, double *rho_n, double lat)
{
   double tmp, sqt;

   tmp = EARTH_ECCENTRICITY * sin(lat);
   tmp = 1.0 - tmp * tmp;
   sqt = sqrt(tmp);
   *rho_e = EARTH_EQUAT_RADIUS * (1.0 - EARTH_ECCENTRICITY * EARTH_ECCENTRICITY) / (sqt * tmp);
   *rho_n = EARTH_EQUAT_RADIUS / sqt;

   return 0;
}



