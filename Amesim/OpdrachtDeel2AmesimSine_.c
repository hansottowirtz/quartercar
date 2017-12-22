/************** -*-C-*- *********************************************
 * $Id: cosim.etemp 53253 2016-10-19 13:27:30Z jandre $
 *                                                                  *
 *       AMESim C code for cosimulation written by code generator.  *
 
                                 OpdrachtDeel2AmesimSine_
 *
 *******************************************************************/

/* INSERT_INTERFACE */
 
#ifdef DS_PLATFORM_SMART
#ifndef AME_SCALEXIO
#define AME_SCALEXIO
#endif
#ifndef AMERT
#define AMERT
#endif
#endif

#if defined(_DS1005) || defined(ds1005) || defined(_DS1401) || defined(ds1401)
#ifndef AME_DS1005
#define AME_DS1005
#endif
#ifndef AMERT
#define AMERT
#endif
#endif

#if defined(_DS1006) || defined(ds1006)
#ifndef AME_DS1006
#define AME_DS1006
#endif
#ifndef AMERT
#define AMERT
#endif
#endif


#include <assert.h>
#include <stdio.h>
#ifndef AME_DS1005
#ifndef AME_DS1006
#include <sys/types.h>
#include <sys/stat.h>
#include <signal.h>
#endif
#endif
#ifndef AME_DS1006
#include <fcntl.h>
#endif
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>

#ifdef _WIN32
#ifndef WIN32
#define WIN32
#endif
#endif

#include "ameutils.h"


static void ModelAmeExit(AMESIMSYSTEM *amesys, int status);
static void EndOfSimulation(AMESIMSYSTEM *amesys);
static AMESIMSYSTEM *setupAmeSystem();

#if defined (WIN32) || defined(XPCMSVISUALC)
#include <io.h>
#else
#if defined(AME_DS1005) || defined(AME_DS1006)
#else
#include <unistd.h>
#endif
#endif

#ifndef R_OK
#define R_OK 4
#endif

#if defined(_WINDOWS) || defined(_WIN32) || defined(WIN32)
#define DLLEXPORTDEF __declspec(dllexport)
#else
#define DLLEXPORTDEF
#endif

/* If we are being compiled for these platforms we are in a RT config */
#if defined (RTLAB) || defined (AME_IN_MORPHEE) || defined(AME_DS1005) || defined(AME_DS1006) || defined(LABCAR) || defined(_ADXPRTS) || defined(HWACREATE) || defined(RT) ||defined(AMEVERISTAND)
#ifndef AMERT
#define AMERT
#endif
#ifdef DLLEXPORTDEF
#undef DLLEXPORTDEF
#endif
#define DLLEXPORTDEF
#endif

/* Currently - if we are in RTLab, dSpace, Labcar, HWACREATE, ADI or if RT is set we are in a Simulink/RTW situation */
#if defined (RTLAB) || defined(AME_DS1005) || defined(AME_DS1006) || defined(LABCAR) || defined(HWACREATE) || defined(_ADXPRTS) || defined(RT)
#ifndef AMESIMULINK
#define AMESIMULINK 
#endif
#endif

/* If we are in Simulink we should set things up for a AMEMULTIDLL (only static symbols) */
#ifdef AMESIMULINK
#ifndef AMEMULTIDLL
#define AMEMULTIDLL
#endif
#endif

/* If we are in Veristand we should set things up for a AMEMULTIDLL (only static symbols) */
#ifdef AMEVERISTAND
#ifndef AMEMULTIDLL
#define AMEMULTIDLL
#endif
#endif

/* avoid globally exported functions - required for exporting several models in one executable */
#if defined AMEMULTIDLL
#define DLLEXPORTDEF_OR_STATIC static
#else
#define DLLEXPORTDEF_OR_STATIC DLLEXPORTDEF
#endif

#if defined CREATING_STATIC_LIB
#define DLLEXPORTDEF_OR_EXTERN extern 
#else
#define DLLEXPORTDEF_OR_EXTERN DLLEXPORTDEF
#endif

#define EXPLICIT_TEMPLATE

#define TIME_ROUNDOFF 1.0e-10
#define TLAST_MIN     -1E30

#if defined(AMERT) && ( (defined(WIN32) && !defined(XPCMSVISUALC) && !defined(LABVIEWCOSIM) ) || defined(LABCAR) || defined(RTLAB) || defined(linux) )
#define AME_RT_CAN_READ_FILE 1
#endif

#if defined(FMICS1) || defined(FMICS2) || defined(FMIME1)
#ifdef FMIX
#undef FMIX
#endif
#define FMI
#define MODEL_IDENTIFIER         OpdrachtDeel2AmesimSine_
#define MODEL_IDENTIFIER_STRING "OpdrachtDeel2AmesimSine_"
#endif

#include <setjmp.h>
static jmp_buf jump_env;   /* used to restore stack call if an error occurs in libAME and AMEExit() is called */


/* ============================================================== */
/* If the interface needs linearisation (cosim and Amesim) */

#ifndef AME_NO_LA
#ifndef AME_NEED_LINEAR_ANALYSIS
#define AME_NEED_LINEAR_ANALYSIS
#endif
#endif

/* for advanced solver debugging define AME_ADVANCEDDEBUG - it replaces macros by functions */
/* if AMERT is defined both LA and discontinuity handling is disabled */

#ifdef AMERT
#ifdef AME_NEED_LINEAR_ANALYSIS
#undef AME_NEED_LINEAR_ANALYSIS
#endif
#ifdef AME_ADVANCEDDEBUG
#undef AME_ADVANCEDDEBUG
#endif
#endif

#ifdef AME_ADVANCEDDEBUG
static void AME_POST_SUBMODCALL_WITH_DISCON(int *flag, int *sflag, int *oflag, int *panic, char *submodelname, int instancenum)
{
   if(*sflag < 3)*sflag = getnfg_();
#ifdef AME_NEED_LINEAR_ANALYSIS
   if(*flag == 5)
   {
      LPerturbIfNecessary(flag);
   }
   else if(*oflag != 5)
   {
      resdis(flag, sflag, oflag, submodelname, instancenum, panic);
   }
#else
   resdis(flag, sflag, oflag, submodelname, instancenum, panic);
#endif
}

static void AME_POST_SUBMODCALL_NO_DISCON(int *flag)
{
#ifdef AME_NEED_LINEAR_ANALYSIS
   if(*flag == 5)
   {
      LPerturbIfNecessary(flag);
   }
#endif
}
#endif


#ifndef AME_ADVANCEDDEBUG
#ifdef AME_NEED_LINEAR_ANALYSIS
/* Typically for normal runs and cosim */
#define AME_POST_SUBMODCALL_WITH_DISCON(flag,sflag,oflag,panic,submodelname,instancenum) if(*sflag < 3)*sflag = getnfg_(); if(*flag == 5) LPerturbIfNecessary(flag); else if(*oflag != 5) resdis(flag, sflag, oflag, submodelname, instancenum, panic)
#define AME_POST_SUBMODCALL_NO_DISCON(flag) if(*flag == 5) LPerturbIfNecessary(flag)
#else
/* Typically for code exchange (simulink for instance) */
#define AME_POST_SUBMODCALL_WITH_DISCON(flag,sflag,oflag,panic,submodelname,instancenum) if(*sflag < 3)*sflag = getnfg_(); resdis(flag, sflag, oflag,submodelname,instancenum,panic)
#define AME_POST_SUBMODCALL_NO_DISCON(flag)
#endif
#endif

#ifdef AMERT
/* We dont need LA nor resdis for real-time - so set them to (almost) empty macros. (set sflag=0) */
#undef AME_POST_SUBMODCALL_WITH_DISCON
#undef AME_POST_SUBMODCALL_NO_DISCON
#define AME_POST_SUBMODCALL_WITH_DISCON(flag,sflag,oflag,panic,submodelname,instancenum) *sflag = 0
#define AME_POST_SUBMODCALL_NO_DISCON(flag) 
#endif

/* ============================================================== */


/* ***************************************************************** 

   code inserted by AMESim
   
   *****************************************************************/

#define SUB_LENGTH        63
#define NUMSTATES         5
#define NUMINPUTS         0
#define NUMOUTPUTS        0
#define NUMVARS           31
#define NUMDISCRETESTATES 0

#define NB_REAL_PARAMS        22
#define NB_INTEGER_PARAMS     5
#define NB_TEXT_PARAMS        0
#define NB_FIXED_VAR          0
#define NB_STATE_VAR          5
#define NB_DISCRETE_STATE_VAR 0
#define NB_PARAMS             32

#define NB_SUBMODELNAME       6

static const char* submodelNameArray[NB_SUBMODELNAME] = {
   "SD0000A instance 1"
  ,"MAS002 instance 1"
  ,"SD0000A instance 2"
  ,"XVLC01 instance 1"
  ,"MAS001 instance 1"
  ,"SIN0 instance 1"
};

static ParamInfo ModelParamInfo[ NB_PARAMS ] = {
   {RealParam,        Expression,     0,    -1, PARAMETER_CATEGORY,     0},
   {RealParam,        Expression,     1,    -1, PARAMETER_CATEGORY,     0},
   {RealParam,        Expression,     2,    -1, PARAMETER_CATEGORY,     0},
   {RealParam,        Expression,     3,    -1, PARAMETER_CATEGORY,     0},
   {RealParam,        Expression,     4,    -1, PARAMETER_CATEGORY,     0},
   {RealParam,        Expression,     5,    -1, PARAMETER_CATEGORY,     0},
   {IntegerParam,     Expression,     0,    -1, PARAMETER_CATEGORY,     0},
   {IntegerParam,     Expression,     1,    -1, PARAMETER_CATEGORY,     0},
   {RealParam,        Expression,     6,    -1, PARAMETER_CATEGORY,     1},
   {RealParam,        Expression,     7,    -1, PARAMETER_CATEGORY,     1},
   {StateVar,         Expression,     2,    14, STATE_CATEGORY,         1},
   {StateVar,         Expression,     3,    15, STATE_CATEGORY,         1},
   {RealParam,        Expression,     8,    -1, PARAMETER_CATEGORY,     2},
   {RealParam,        Expression,     9,    -1, PARAMETER_CATEGORY,     2},
   {RealParam,        Expression,    10,    -1, PARAMETER_CATEGORY,     2},
   {RealParam,        Expression,    11,    -1, PARAMETER_CATEGORY,     2},
   {RealParam,        Expression,    12,    -1, PARAMETER_CATEGORY,     2},
   {RealParam,        Expression,    13,    -1, PARAMETER_CATEGORY,     2},
   {IntegerParam,     Expression,     2,    -1, PARAMETER_CATEGORY,     2},
   {IntegerParam,     Expression,     3,    -1, PARAMETER_CATEGORY,     2},
   {RealParam,        Expression,    14,    -1, PARAMETER_CATEGORY,     3},
   {RealParam,        Expression,    15,    -1, PARAMETER_CATEGORY,     3},
   {IntegerParam,     Expression,     4,    -1, PARAMETER_CATEGORY,     3},
   {StateVar,         Expression,     4,    28, STATE_CATEGORY,         3},
   {RealParam,        Expression,    16,    -1, PARAMETER_CATEGORY,     4},
   {RealParam,        Expression,    17,    -1, PARAMETER_CATEGORY,     4},
   {StateVar,         Expression,     0,     4, STATE_CATEGORY,         4},
   {StateVar,         Expression,     1,     5, STATE_CATEGORY,         4},
   {RealParam,        Expression,    18,    -1, PARAMETER_CATEGORY,     5},
   {RealParam,        Expression,    19,    -1, PARAMETER_CATEGORY,     5},
   {RealParam,        Expression,    20,    -1, PARAMETER_CATEGORY,     5},
   {RealParam,        Expression,    21,    -1, PARAMETER_CATEGORY,     5}
};
static double      RealParamArray   [ NB_REAL_PARAMS ];
static int         IntegerParamArray[ NB_INTEGER_PARAMS ];
static       char* TextParamArray   [ 1 ];

/* parameters and stores */
static double RS0[ 2 ];
static int IS0[ 4 ];
static double *RP0 = &RealParamArray[ 0 ];
static int *IP0 = &IntegerParamArray[ 0 ];
static double RS1[ 1 ];
static int IS1[ 3 ];
static double *RP1 = &RealParamArray[ 6 ];
static double RS2[ 2 ];
static int IS2[ 4 ];
static double *RP2 = &RealParamArray[ 8 ];
static int *IP2 = &IntegerParamArray[ 2 ];
static double *RP3 = &RealParamArray[ 14 ];
static int *IP3 = &IntegerParamArray[ 4 ];
static double RS4[ 1 ];
static int IS4[ 3 ];
static double *RP4 = &RealParamArray[ 16 ];
static double RS5[ 2 ];
static double *RP5 = &RealParamArray[ 18 ];


/* **********  Various static declrs for use in this file ***/

static int ameExitStatus; /* A static variable to communicate between AmeExit and ameTerminate */

/*
  Make it possible to vary the AMESim stepsize from the RT platform 
  By creating a data store with type "ImportedExtern" in a Simulink 
  sketch one can access the step ration below.
  The variable will be renamed to contain the AMESim system nam.
*/
#ifdef AMERT
double IL_OpdrachtDeel2AmesimSine_step_ratio=0;
#endif

#define NUMSTATESx13  (NUMSTATES*13)
#define NUMSTATES2    (NUMSTATES*NUMSTATES)

static ParamData ModelData = { ModelParamInfo, NB_PARAMS,
                               IntegerParamArray, NB_INTEGER_PARAMS,
                               RealParamArray, NB_REAL_PARAMS,
                               TextParamArray, NB_TEXT_PARAMS,
                               NULL, NUMVARS,
                               NULL, NB_STATE_VAR,
                               NULL, NUMDISCRETESTATES,
                               submodelNameArray};

/* static variable */
static double static_v[NUMVARS];

#if defined(AMERT) || defined(FMI)
#define v  static_v
#include "OpdrachtDeel2AmesimSine_.export.h"
#include "OpdrachtDeel2AmesimSine_.sim.h"
#undef v
#endif

#ifdef STANDALONESIMULATOR
static AMESIMSYSTEM * SASsetupsystem();
#include "ProviderHookC.h"
static ProviderHookC *resultsDispatcher=0;
#include "outputresults_standard.c"
#include "outputresults_standalonesimulator.c"
DLLEXPORTDEF void _setProviderHookC(ProviderHookC* r)
{
   resultsDispatcher = r;
   if (resultsDispatcher)
   {
      AMESIMSYSTEM *amesys = GetGlobalSystem();
      if(amesys == NULL)
      {
         amesys = SASsetupsystem();
         SetGlobalSystem(amesys);
      }
      amesys->OutputResults = OutputResultsStandaloneSimulator;
   }
}
#include "ChannelsRedirectionC.h"
static ChannelsRedirectionC *channelsRedirector=0;
#include "ameredirectfprintf.c"

#ifdef WIN32
void activatePipeReadingThread(HANDLE fd, int fileno)
#else
void activatePipeReadingThread(int fd, int fileno)
#endif  
{
   if(channelsRedirector != NULL)
   {
      channelsRedirector->activatePipeReadingThread(channelsRedirector, fd, fileno);
   }
}

DLLEXPORTDEF void _setChannelsRedirector(ChannelsRedirectionC *c)
{
   AMESIMSYSTEM *amesys;
   channelsRedirector = c;
   ameInstallFprintf(ameredirectfprintf);

   amesys = GetGlobalSystem();
   if(amesys == NULL)
   {
      amesys = SASsetupsystem();
      SetGlobalSystem(amesys);
   }
   AmeNewDataStore(amesys, "Activate_Pipe_Reading_Thread", 0, activatePipeReadingThread);
}
#endif

DLLEXPORTDEF_OR_STATIC int AMEGetParam_type(int idx, ParamCType* type) {
   return GetParameter_type(&ModelData, idx, type);
}

DLLEXPORTDEF_OR_STATIC int AMEGetParam_int(int idx, int* value) {
   return GetParameter_int(&ModelData, idx, value);
}

DLLEXPORTDEF_OR_STATIC int AMEGetParam_double(int idx, double* value) {
   return GetParameter_double(&ModelData, idx, value);
}

DLLEXPORTDEF_OR_STATIC int AMEGetParam_string(int idx, const char** value) {
   return GetParameter_string(&ModelData, idx, value);
}

DLLEXPORTDEF_OR_STATIC int AMESetParam_int(int idx, int value) {
   ErrorCode ret = SetParameter_int(&ModelData, idx, value);
   if (ret == no_error)
   {
      SetParameterAsUserDefined(&ModelData, idx);
   }
   return ret;
}

DLLEXPORTDEF_OR_STATIC int AMESetParam_double(int idx, double value) {
   ErrorCode ret = SetParameter_double(&ModelData, idx, value);
   if (ret == no_error)
   {
      SetParameterAsUserDefined(&ModelData, idx);
	  SignalInputChange();
   }
   return ret;
}

DLLEXPORTDEF_OR_STATIC int AMESetParam_string(int idx, const char* value) {
   ErrorCode ret = SetParameter_string(&ModelData, idx, value);
   if (ret == no_error)
   {
      SetParameterAsUserDefined(&ModelData, idx);
   }
   return ret;
}

DLLEXPORTDEF_OR_STATIC int AMEFindParamFromVariable (const int paramidx, int * varidx, ParamCategory * category) {
	return FindParamFromVariable(&ModelData, paramidx, varidx, category);
}

DLLEXPORTDEF_OR_STATIC int AMEChangeOrAddRealGlobalParamValue(const char* param, double value) {
   int jump_ret;
   if( (jump_ret = setjmp(jump_env)) == 0) /* try */
   {  
      return ChangeOrAddRealGlobalParamValue(param, value, 1);
   } /* catch */
   return 0;
}

DLLEXPORTDEF_OR_STATIC int AMEChangeOrAddIntGlobalParamValue(const char* param, int value) {
   int jump_ret;
   if( (jump_ret = setjmp(jump_env)) == 0) /* try */
   {  
      return ChangeOrAddIntGlobalParamValue(param, value, 1);
   } /* catch */
   return 0;
}

DLLEXPORTDEF_OR_STATIC int AMEChangeOrAddTextGlobalParamValue(const char* param, const char* value) {
   int jump_ret;
   if( (jump_ret = setjmp(jump_env)) == 0) /* try */
   {  
      return ChangeOrAddTextGlobalParamValue(param, value, 1);
   } /* catch */
   return 0;
}

DLLEXPORTDEF_OR_STATIC int AMELoadParameters() {
   /* Read OpdrachtDeel2AmesimSine_.data to pick up submodel parameters */
   AMESIMSYSTEM *amesys;
   const char* datafilename;
   char errmsg[2048];
   
#ifdef STANDALONESIMULATOR
   int jump_ret;
   /* We will typically be called at least twice - so don't create two AMESIMSYSTEM */
   amesys = GetGlobalSystem();
   if(amesys == NULL)
   {
      amesys = SASsetupsystem();
      SetGlobalSystem(amesys);
   }
   if( (jump_ret = setjmp(jump_env)) == 0) /* try */
   {
#else
   amesys = GetGlobalSystem();
#endif

   assert(amesys);

   datafilename = GetDataFileName(amesys);

   if (datafilename == NULL)
   {
      ConstructFileNames(amesys, "OpdrachtDeel2AmesimSine_", NULL);
      datafilename = GetDataFileName(amesys);
   }
   return LoadParameterFromFile( datafilename, &ModelData, errmsg);
#ifdef STANDALONESIMULATOR
   } /* catch */
   return 0;
#endif
}

DLLEXPORTDEF_OR_STATIC int AMEGetErrorMsg(int error_code, char* msg, int max_msg_len) {
   return GetParamErrorMsg((ErrorCode) error_code, msg, (size_t)max_msg_len);
}

DLLEXPORTDEF_OR_STATIC void AMESignalModelUnload() 
{
   /* AJN - delete the amesystem after unload */
   AMESIMSYSTEM *amesys = GetGlobalSystem();
   AmeSignalModelUnload();
   DeleteGlobalSystem(amesys);
   /* Report 0121694 clean Global system pointer */
   SetGlobalSystem(NULL);
}


DLLEXPORTDEF_OR_STATIC void AMERequestSimulationInterupt ()
{
   AMESIMSYSTEM *amesys = GetGlobalSystem();
	amesys->requestinterrupt = 1;
}

DLLEXPORTDEF_OR_STATIC int AMEGetTemplateVersion () 
{
	/* If method does not exist we assume versions older than Rev 10	*/
	/* Returned number indicates main verion + sub version	            */
	/* 100 = Rev 10*/
	/* 101 = Rev 10 SL 1*/
	return 100;
}

DLLEXPORTDEF_OR_STATIC void AMEDisableOptimizedSolver (int fDisable) 
{
	/* Optimized solver option  */
	/* 0 = optimized solver is enabled             */
	/* 1 = optimized solver is disabled            */
	if (fDisable)
	{
		disableintopt_();
	}
}

DLLEXPORTDEF_OR_STATIC void AMESetStabilizingOptions (int options)
{
   AMESIMSYSTEM *amesys = GetGlobalSystem();
   amesys->simOptions->stabilOption = options;
}

DLLEXPORTDEF_OR_STATIC void AMESetMinDisHanOption (int option)
{
   if (option)
   {
      AMESIMSYSTEM *amesys = GetGlobalSystem();
	  EnableMinDisHan(amesys);
   }
}

/* *******************  Function prototypes ************ */


extern void sd0000ain_(int *n, double *RP, int *IP, double *RS
                 , int *IS, double *y0, double *y1, double *y2
                   , double *y3);
extern void sd0000a_(int *n, double *ve0, double *ve1, double *ve2
                   , double *ve3, double *ve4, double *vi5, double *vi6
                   , double *vi7, double *vi8, double *vidot8
                   , double *vi9, double *vi10, double *vidot10
                   , double *vi11, double *vidot11, double *vi12
                   , double *vi13, double *vidot13, double *RP
                 , int *IP, double *RS, int *IS);
extern void mas002in_(int *n, double *RP, double *RS, int *IS
                 , double *y0, double *y1, double *y2, double *y3
                   , double *y4, double *y5);
extern void mas002_(int *n, double *ve0, double *vedot0, double *ve1
                   , double *vedot1, double *ve2, double *ve3
                   , double *ve4, double *vi5, double *vidot5
                   , double *vi6, double *vi7, double *vidot7
                   , double *vi8, double *vidot8, double *vi9
                   , double *vi10, double *vidot10, double *RP
                 , double *RS, int *IS);
extern void xvlc01in_(int *n, double *RP, int *IP, double *y0
                   );
extern void xvlc01_(int *n, double *ve0, double *ve1, double *ve2
                   , double *vi3, double *vidot3, double *RP, int *IP
                 );
extern void mas001in_(int *n, double *RP, double *RS, int *IS
                 , double *y0, double *y1, double *y2, double *y3
                   , double *y4, double *y5);
extern void mas001_(int *n, double *ve0, double *vedot0, double *ve1
                   , double *vedot1, double *ve2, double *ve3
                   , double *vi4, double *vidot4, double *vi5
                   , double *vi6, double *vidot6, double *vi7
                   , double *vi8, double *vidot8, double *vi9
                   , double *vidot9, double *RP, double *RS, int *IS
                 );
extern void sin0in_(int *n, double *RP, double *RS);
extern void sin0_(int *n, double *ve0, double *RP, double *RS
                 , double *t);



/* ******************    Here comes the functions ************ */


#if defined(AMERT) || defined(FMI)
#include "OpdrachtDeel2AmesimSine_.ssf.h"
static char **getssflist(int *num_items)
{
   *num_items = savestatusflags_length;
   return savestatusflags;
}
#endif



/**************************************************************
*                                                             *
* Function load_subdata_tables reads data for lookuptables    *
* mostly/only used for realtime                               *
*                                                             *
* 0106429                                                     *
* Move the include outside of function. The include now       *
* contains one function per table and a function that calls   *
* them all. This reduces the risk that a compiler crashes due *
* to a huge function.                                         *
**************************************************************/

#if (defined(AMERT) && !defined(FMI)) || defined(FMI_INC_TABLES)
#include "OpdrachtDeel2AmesimSine_.subdata.h"
#endif

static void load_subdata_tables(void)
{
#if (defined(AMERT) && !defined(FMI)) || defined(FMI_INC_TABLES)
   add_all_tables_to_memory();
#endif
}

/**************************************************************
*                                                             *
* Function USER_UpdateParameter:							  *
* update parameter set from Veristand                         *
*                                                             *
**************************************************************/
#ifdef AMEVERISTAND
extern void USER_UpdateParameter(void);
#endif

#if defined(AMERT) || defined(FMI)
#include "OpdrachtDeel2AmesimSine_.globalparams.h"
#endif

static void loadParameters(AMESIMSYSTEM *amesys, double *y)
{
   ErrorCode ret = file_error; /* Force file error in case of no file and real-time */
   char errmsg[PATH_MAX+128];
   int idummy[9];
   double rdummy[8];
   double *v = amesys->v;
   double *Z = amesys->discrete_states;
#if defined(AMERT) || defined(FMI)
   int lineptr=0;
#include "OpdrachtDeel2AmesimSine_.data.h"
#endif

   idummy[0] = 0;
   rdummy[0] = 0.0;
   
#if defined(AMERT) || defined(FMI)

#ifdef AME_RT_CAN_READ_FILE
   ret = LoadParameterFromFile( GetDataFileName(), &ModelData, errmsg);
   
   if (ret != no_error) {
      /* If the file is not there - we say nothing */
      if (ret != file_error) {
         GetParamErrorMsg(ret, errmsg, PATH_MAX+128);
         amefprintf(stderr,"LoadParameterFromFile> %s\n",errmsg);
      }
      ClearGPs();
   }
   else {
      amefprintf(stderr,"Using data from disk (%s)\n",GetDataFileName());
   }
#endif
   if(ret != no_error) {
      ameAddGlobalParamsFromMemory(amesys,errmsg);
#ifdef AMEVERISTAND
      USER_UpdateParameter();
#endif
      ret = LoadParameterFromDataTable(allparams, &ModelData, errmsg);
   }

#else
   ret = LoadParameterFromFile( GetDataFileName(), &ModelData, errmsg);
#endif

   if (ret != no_error) {
      amefprintf(stderr,"%s",errmsg);
      AmeExit(1);
   }

}

/*************************************************************
*                                                            *
*    Function Input reads submodel and simulation data.      *
*                                                            *
*************************************************************/
static void Input(AMESIMSYSTEM *amesys, double *y)
{
   /* Local variables */
   char text1[PATH_MAX+128];
   SIMOptions tmp_sim_options;

   /*   y   is a double array holding state variables.

       The following statement covers the case where there are
       no state variables y. */
#if NB_STATE_VAR == 0
   y[0] = 0.;
#endif

   /* Load data files for submodels */
   load_subdata_tables();
   
   /* Load parameters for submodels */
   loadParameters(amesys,y);


#if !defined(AMERT) && !defined(FMI)

   /* Read OpdrachtDeel2AmesimSine_.sim to pick up simulation parameters */
   if(readsimfile(&tmp_sim_options, GetSimFileName(), text1)!=0)
   {
      /* only read activity index variable */
      ALA_Setparam(tmp_sim_options.autoLA,0,tmp_sim_options.autoLAstep);
      DISCLOG_SetParam(1);
      amesys->simOptions->activityIndex = tmp_sim_options.activityIndex;
   }
   else
   {
      /* We failed to read the file - never mind. */
   }

#endif
}


static void SetInitValues(AMESIMSYSTEM *amesys)
{
   int i=0;
   if(isconrun_() || isusefinval_())
   {
      double *v = amesys->v;
      double *y = amesys->states;
      double *Z = amesys->discrete_states;

   ChangeState(&y[2], v[14]);
   ChangeState(&y[3], v[15]);
   ChangeState(&y[4], v[28]);
   ChangeState(&y[0], v[4]);
   ChangeState(&y[1], v[5]);

   }
}

/******************************************************
*                                                     *
* This function calls the submodel pre-initialization *
* functions/subroutines to check resp, iesp and       *
* state variable values and set the the con and icon  *
* arrays                                              *
*                                                     *
******************************************************/
static void PreInitialize(AMESIMSYSTEM *amesys, double *y)
{
   int n = 0;
   double *v = amesys->v;
   double *Z = amesys->discrete_states;


}

/******************************************************
*                                                     *
* This function calls the submodel initialiation      *
* functions/subroutines to check resp, iesp and       *
* state variable values and set the the con and icon  *
* arrays                                              *
*                                                     *
******************************************************/
static void Initialize(AMESIMSYSTEM *amesys, double *y)
{
   int n;
   double *v = amesys->v;
   double *Z = amesys->discrete_states;


   n = 1;
   sin0in_(&n, RP5, RS5);

   n = 1;
   xvlc01in_(&n, RP3, IP3, &y[4]);

   n = 2;
   sd0000ain_(&n, RP2, IP2, RS2, IS2, NULL, NULL, NULL, NULL);

   n = 1;
   sd0000ain_(&n, RP0, IP0, RS0, IS0, NULL, NULL, NULL, NULL);

   n = 1;
   mas002in_(&n, RP1, RS1, IS1, &y[2], &y[3], NULL, NULL, NULL
      , NULL);

   n = 1;
   mas001in_(&n, RP4, RS4, IS4, &y[0], &y[1], NULL, NULL, NULL
      , NULL);

}

/*********************************************************************** 
*                                                                      *
* FunctionEval calls the submodels in an order that ensures that the   *
* inputs of each are known when it is called. When submodels flag      *
* discontinuities, conflicts are resolved.                             *
*                                                                      *
************************************************************************/

static void localFunctionEval(AMESIMSYSTEM *amesys, double *dot, double *y, double t, int *flag)
{
   int sflag, oflag, n, panic, i=0;
   static double zero = 0.0e0;
   static int *oldflag, *newflag;
   double *v = amesys->v;
   double *Z = amesys->discrete_states;
   double *input = amesys->inputs;
   double *output = amesys->outputs;
   SetGlobalSystem(amesys);

   /* Record old value of flag (oflag) and set 
      flag value for use in submodels (sflag).
      Also get addresses of main discontinuity flags. */

   oflag = *flag;
   sflag = *flag;

   if(amesys->first_call)
   {
      GetFlagAddresses(&oldflag, &newflag);
   }

   /* Initialize everything ready for potential calls to stepdn
      in submodels. */

   panic = 0;
   getredstep();

   if(isstabrun_())
   {
      t = amesys->simOptions->fixedTime;
   }
   else if(*flag == 2)
   {
      /* Record current simulation time for message passing. */
 
      SetSimTime(t);
   }
   /* Record current simulation time for ametim_(). */

   SetTimeAtThisStep(t);

   if (holdinputs_())
   {
      /* We reset artificially the time to the initial value
         to give the illusion of held inputs. */

      t = getstarttime_();
   }
   /* Assign the state variables y[] calculated by the integrator 
      to the appropriate variables v[]. */

   v[14] = y[2];
   v[15] = y[3];
   v[28] = y[4];
   v[4] = y[0];
   v[5] = y[1];

   /* Assign the interface input variables to the appropriate variable v(). */
 

  /* The following call ensures that lsoda does not integrate past
      time amesys->t_end_of_time_slice. This does not matter in a standard AMESim run but is
      very important with cosimulation. */
      
#if !defined(FMIME1) & !defined(AMERT)
   *oldflag = *newflag = sflag;
   sdistim_(&amesys->t_end_of_time_slice);
   AME_POST_SUBMODCALL_WITH_DISCON(flag,&sflag,&oflag,&panic,"_Cosimulation",1);
#endif
	 
	 
   /* Call submodel calculation subroutine in the order 
      that ensures the input requirements of each submodel
      are satisfied. */

   n = 1;
   *oldflag = *newflag = sflag;
   sin0_(&n, &v[27], RP5, RS5, &t);
   AME_POST_SUBMODCALL_WITH_DISCON(flag,&sflag,&oflag,&panic,"SIN0",1);

   n = 1;
   *oldflag = *newflag = sflag;
   xvlc01_(&n, &v[27], &v[19], &v[20], &v[28], &dot[4], RP3, IP3
      );
   AME_POST_SUBMODCALL_WITH_DISCON(flag,&sflag,&oflag,&panic,"XVLC01",1);

   v[0] = -v[14] /* Duplicate variable. */;
   AME_POST_SUBMODCALL_NO_DISCON(flag);

   v[1] = -v[15] /* Duplicate variable. */;
   AME_POST_SUBMODCALL_NO_DISCON(flag);

   n = 2;
   *oldflag = *newflag = sflag;
   sd0000a_(&n, &v[21], &v[19], &v[20], &v[14], &v[15], &v[22]
      , &v[23], &v[24], NULL, NULL, &v[25], NULL, NULL, NULL, NULL
      , &v[26], NULL, NULL, RP2, IP2, RS2, IS2);
   AME_POST_SUBMODCALL_WITH_DISCON(flag,&sflag,&oflag,&panic,"SD0000A",2);

   v[13] = v[21] /* Duplicate variable. */;
   AME_POST_SUBMODCALL_NO_DISCON(flag);

   n = 1;
   *oldflag = *newflag = sflag;
   sd0000a_(&n, &v[3], &v[0], &v[1], &v[4], &v[5], &v[8], &v[9]
      , &v[10], NULL, NULL, &v[11], NULL, NULL, NULL, NULL, &v[12]
      , NULL, NULL, RP0, IP0, RS0, IS0);
   AME_POST_SUBMODCALL_WITH_DISCON(flag,&sflag,&oflag,&panic,"SD0000A",1);

   v[7] = v[3] /* Duplicate variable. */;
   AME_POST_SUBMODCALL_NO_DISCON(flag);

   n = 1;
   *oldflag = *newflag = sflag;
   mas002_(&n, &v[14], &dot[2], &v[15], &dot[3], &v[16], &v[13]
      , &v[3], NULL, NULL, &v[17], NULL, NULL, NULL, NULL, &v[18]
      , NULL, NULL, RP1, RS1, IS1);
   AME_POST_SUBMODCALL_WITH_DISCON(flag,&sflag,&oflag,&panic,"MAS002",1);

   v[2] = -v[16] /* Duplicate variable. */;
   AME_POST_SUBMODCALL_NO_DISCON(flag);

   n = 1;
   *oldflag = *newflag = sflag;
   mas001_(&n, &v[4], &dot[0], &v[5], &dot[1], &v[6], &v[7], NULL
      , NULL, &v[29], NULL, NULL, &v[30], NULL, NULL, NULL, NULL
      , RP4, RS4, IS4);
   AME_POST_SUBMODCALL_WITH_DISCON(flag,&sflag,&oflag,&panic,"MAS001",1);


   /* Set interface outputs here. */


   if(*flag == 0)
   {
      /* It is an initialization call and the user
         is permitted to change the state variables
         and discrete variables. */

      ChangeState(&y[2], v[14]);
      ChangeState(&y[3], v[15]);
      ChangeState(&y[4], v[28]);
      ChangeState(&y[0], v[4]);
      ChangeState(&y[1], v[5]);
   }
   for (i = 0; i < NUMSTATES; i++)
   {
      *(dot+i)*=getStateStatus_(&i);
   }
   UpFECount(amesys);

   amesys->first_call =0;
}


/*********************************************************************** 
*                                                                      *
* JFunctionEval is a "cut down" version of FunctionEval for use when a *
* Jacobian is being evaluated.                                         *
*                                                                      *
************************************************************************/

static void localJFunctionEval(AMESIMSYSTEM *amesys, double *dot, double *y, double t, int col)
{
   int sflag=1; /* Only one flag value is required. */
   int n=1, i=0;
   static double zero = 0.0e0;
   double *v = amesys->v;
   double *vcopy = amesys->vcopy;
   double *input = amesys->inputs;
   double *output = amesys->outputs;
   double *Z = amesys->discrete_states;

   /* Record current simulation time for ametim_(). */

   SetTimeAtThisStep(t);

   if (holdinputs_())
   {
      /* We reset artificially the time to the initial value
         to give the illusion of held inputs. */

      t = getstarttime_();
   }
   memcpy((void *)vcopy, (void *)v, (size_t)(NUMVARS*sizeof(double)));

   /* Assign the state variables y[] calculated by the integrator 
      to the appropriate variables v[] and right calls necessary
      for that state in a case of a switch. */

   switch (col)
   {
      case 0:
         v[4] = y[col];
         n = 1;
         sd0000a_(&n, &v[3], &v[0], &v[1], &v[4], &v[5], &v[8]
            , &v[9], &v[10], NULL, NULL, &v[11], NULL, NULL, NULL
            , NULL, &v[12], NULL, NULL, RP0, IP0, RS0, IS0);
         v[7] = v[3] /* Duplicate variable. */;
         n = 1;
         mas002_(&n, &v[14], &dot[2], &v[15], &dot[3], &v[16]
            , &v[13], &v[3], NULL, NULL, &v[17], NULL, NULL, NULL
            , NULL, &v[18], NULL, NULL, RP1, RS1, IS1);
         v[2] = -v[16] /* Duplicate variable. */;
         n = 1;
         mas001_(&n, &v[4], &dot[0], &v[5], &dot[1], &v[6], &v[7]
            , NULL, NULL, &v[29], NULL, NULL, &v[30], NULL, NULL
            , NULL, NULL, RP4, RS4, IS4);
      break;
      case 1:
         v[5] = y[col];
         n = 1;
         sd0000a_(&n, &v[3], &v[0], &v[1], &v[4], &v[5], &v[8]
            , &v[9], &v[10], NULL, NULL, &v[11], NULL, NULL, NULL
            , NULL, &v[12], NULL, NULL, RP0, IP0, RS0, IS0);
         v[7] = v[3] /* Duplicate variable. */;
         n = 1;
         mas002_(&n, &v[14], &dot[2], &v[15], &dot[3], &v[16]
            , &v[13], &v[3], NULL, NULL, &v[17], NULL, NULL, NULL
            , NULL, &v[18], NULL, NULL, RP1, RS1, IS1);
         v[2] = -v[16] /* Duplicate variable. */;
         n = 1;
         mas001_(&n, &v[4], &dot[0], &v[5], &dot[1], &v[6], &v[7]
            , NULL, NULL, &v[29], NULL, NULL, &v[30], NULL, NULL
            , NULL, NULL, RP4, RS4, IS4);
      break;
      case 2:
         v[14] = y[col];
         v[0] = -v[14] /* Duplicate variable. */;
         n = 2;
         sd0000a_(&n, &v[21], &v[19], &v[20], &v[14], &v[15], &v[22]
            , &v[23], &v[24], NULL, NULL, &v[25], NULL, NULL, NULL
            , NULL, &v[26], NULL, NULL, RP2, IP2, RS2, IS2);
         v[13] = v[21] /* Duplicate variable. */;
         n = 1;
         sd0000a_(&n, &v[3], &v[0], &v[1], &v[4], &v[5], &v[8]
            , &v[9], &v[10], NULL, NULL, &v[11], NULL, NULL, NULL
            , NULL, &v[12], NULL, NULL, RP0, IP0, RS0, IS0);
         v[7] = v[3] /* Duplicate variable. */;
         n = 1;
         mas002_(&n, &v[14], &dot[2], &v[15], &dot[3], &v[16]
            , &v[13], &v[3], NULL, NULL, &v[17], NULL, NULL, NULL
            , NULL, &v[18], NULL, NULL, RP1, RS1, IS1);
         v[2] = -v[16] /* Duplicate variable. */;
         n = 1;
         mas001_(&n, &v[4], &dot[0], &v[5], &dot[1], &v[6], &v[7]
            , NULL, NULL, &v[29], NULL, NULL, &v[30], NULL, NULL
            , NULL, NULL, RP4, RS4, IS4);
      break;
      case 3:
         v[15] = y[col];
         v[1] = -v[15] /* Duplicate variable. */;
         n = 2;
         sd0000a_(&n, &v[21], &v[19], &v[20], &v[14], &v[15], &v[22]
            , &v[23], &v[24], NULL, NULL, &v[25], NULL, NULL, NULL
            , NULL, &v[26], NULL, NULL, RP2, IP2, RS2, IS2);
         v[13] = v[21] /* Duplicate variable. */;
         n = 1;
         sd0000a_(&n, &v[3], &v[0], &v[1], &v[4], &v[5], &v[8]
            , &v[9], &v[10], NULL, NULL, &v[11], NULL, NULL, NULL
            , NULL, &v[12], NULL, NULL, RP0, IP0, RS0, IS0);
         v[7] = v[3] /* Duplicate variable. */;
         n = 1;
         mas002_(&n, &v[14], &dot[2], &v[15], &dot[3], &v[16]
            , &v[13], &v[3], NULL, NULL, &v[17], NULL, NULL, NULL
            , NULL, &v[18], NULL, NULL, RP1, RS1, IS1);
         v[2] = -v[16] /* Duplicate variable. */;
         n = 1;
         mas001_(&n, &v[4], &dot[0], &v[5], &dot[1], &v[6], &v[7]
            , NULL, NULL, &v[29], NULL, NULL, &v[30], NULL, NULL
            , NULL, NULL, RP4, RS4, IS4);
      break;
      case 4:
         v[28] = y[col];
         n = 1;
         xvlc01_(&n, &v[27], &v[19], &v[20], &v[28], &dot[4], RP3
            , IP3);
         n = 2;
         sd0000a_(&n, &v[21], &v[19], &v[20], &v[14], &v[15], &v[22]
            , &v[23], &v[24], NULL, NULL, &v[25], NULL, NULL, NULL
            , NULL, &v[26], NULL, NULL, RP2, IP2, RS2, IS2);
         v[13] = v[21] /* Duplicate variable. */;
         n = 1;
         mas002_(&n, &v[14], &dot[2], &v[15], &dot[3], &v[16]
            , &v[13], &v[3], NULL, NULL, &v[17], NULL, NULL, NULL
            , NULL, &v[18], NULL, NULL, RP1, RS1, IS1);
         v[2] = -v[16] /* Duplicate variable. */;
      break;
   }
   
   memcpy((void *)v, (void *)vcopy, (size_t)(NUMVARS*sizeof(double)));

   UpFECount(amesys);
}

/******************************************************
*                                                     *
* This function calls the submodel terminate          *
* functions/subroutines to allow for resource         *
* liberation etc                                      *
*                                                     *
******************************************************/
static void EndOfSimulation(AMESIMSYSTEM *amesys)
{
   int n=1;
   double *y = amesys->states;
   double *v = amesys->v;
   double *Z = amesys->discrete_states;


}

/******************************************************************
 * An interface neutral function that returns the number of 
 * inputs/outputs and the names of the variables on the ports.
 * 2011-09-12 AJN : avoid crashes when being called for 
 * extracting only inputs or outputs (numinputs or numoutputs is NULL).
 ******************************************************************/

static void getPortNames(int *numinputs, int *numoutputs, char ***arg_inputports, char ***arg_outputports)
{
   int i;
   char **inputports;
   char **outputports;

#if (NUMINPUTS >= 0)
   if(numinputs != NULL)
   {
      *numinputs = NUMINPUTS;
   }
#endif
#if (NUMOUTPUTS >= 0)
   if(numoutputs != NULL)
   {
      *numoutputs = NUMOUTPUTS;
   }
#endif
      
   if(numinputs != NULL)
   {
      inputports = malloc(sizeof(char*)*NUMINPUTS);
   }
   if(numoutputs != NULL)
   {
      outputports = malloc(sizeof(char*)*NUMOUTPUTS);
   }

   /* The generated code will create variables "outputVarTitles" and "inputVarTitles"
      that contains the names of the variables on the ports */


/* START of generated code for 'INTERFACEPORTNAMES_START' */
   {
      int iii;
      #if (NUMOUTPUTS > 0)
      char *outputVarTitles[NUMOUTPUTS];
      #else
      char **outputVarTitles = NULL;
      #endif
      #if (NUMINPUTS > 0)
      char *inputVarTitles[NUMINPUTS];
      #else
      char **inputVarTitles = NULL;
      #endif

/*  END of generated code for 'INTERFACEPORTNAMES_START' */
      
   if(numoutputs != NULL)
   {
      for (i=0; i<NUMOUTPUTS; i++)
      {
         outputports[i] = malloc(strlen(outputVarTitles[i])+1);
         strcpy(outputports[i], outputVarTitles[i]);
      }
   }
   if(numinputs != NULL)
   {
      for (i=0; i<NUMINPUTS; i++)
      {
         inputports[i] = malloc(strlen(inputVarTitles[i])+1);
         strcpy(inputports[i], inputVarTitles[i]);
      }
   }


/*  START of generated code for 'INTERFACEPORTNAMES_END' */

      for (iii=0; iii< NUMOUTPUTS; iii++)
      {
         free(outputVarTitles[iii]);
      }
      for (iii=0; iii< NUMINPUTS; iii++)
      {
         free(inputVarTitles[iii]);
      }
   }
/*  END of generated code for 'INTERFACEPORTNAMES_END' */

   if(numinputs != NULL)
   {
      *arg_inputports = inputports;
   }
   if(numoutputs != NULL)
   {
      *arg_outputports = outputports;
   }
}


/******************************************************
*                                                     *
* reset_real_integer_stores resets the real           *
* and integer stores to zero.                         *
*                                                     *
******************************************************/

static void reset_real_integer_stores()
{

/* START of generated code for 'InsertInterfaceReset C IC' */
  IS0[0] = IS0[1] = IS0[2] = IS0[3] = 0; 
  RS0[0] = RS0[1] = 0.0; 
  IS1[0] = IS1[1] = IS1[2] = 0; 
  RS1[0] = 0.0; 
  IS2[0] = IS2[1] = IS2[2] = IS2[3] = 0; 
  RS2[0] = RS2[1] = 0.0; 
  IS4[0] = IS4[1] = IS4[2] = 0; 
  RS4[0] = 0.0; 
  RS5[0] = RS5[1] = 0.0; 

/*  END of generated code for 'InsertInterfaceReset C IC' */
      ;
}

/* *****************************************************/
/*                                                     */
/*  Below here some cosimulation special functions     */
/*                                                     */
/* *****************************************************/

/*************************************************************************
 * This function will perform local integration using a fixedstep method *
 *
 *	Issue                                                                *
 *************************************************************************/
static void AMEDoAFixedStep(double time)
{
   double timerange;
   double actual_timestep;
   int stepratio;
   int stepratio_ceil;
   int stepratio_floor;
   int i=0;
   int zero=0;
   double tinc;
   int isTimeForPrint=0;
   double next_print_time;
   AMESIMSYSTEM *amesys = GetGlobalSystem();

   assert(amesys);

   if (amesys->first_call)
   {
      SetIsPrinting(amesys);
      amesys->FunctionEval(amesys, amesys->yh,amesys->states,amesys->tlast,&zero);
      ClearIsPrinting(amesys);
      
#if !defined(AMERT) && !defined(FMI) && !defined(STANDALONESIMULATOR)
      amesys->OutputResults(amesys, amesys->tlast);
      amesys->resultFileStructPtr->lastprinttime = amesys->tlast;
#endif
   }

   timerange = time - amesys->tlast;
   amesys->requestinterrupt = 0;
   
   if(timerange <= 0.0)
   {
      return;
   }
   if(amesys->simOptions->fixedH > 0.0)
   {
      stepratio = stepratio_ceil = (int)ceil(timerange/amesys->simOptions->fixedH);
      stepratio_floor = (int)floor(timerange/amesys->simOptions->fixedH);
      
      actual_timestep = timerange/(double)stepratio_ceil;

      if(fabs(actual_timestep-amesys->simOptions->fixedH) > 0.001*amesys->simOptions->fixedH)
      {
         if(stepratio_floor == 0)
         {
#ifdef AMEDEBUG
            amefprintf(stdout,"skipping %14.8e  range= %14.8e  actual_timestep=%14.8e stepratio_ceil=%d   stepratio_floor=%d\n",time, timerange, actual_timestep, stepratio_ceil,stepratio_floor);
#endif
            return;
         }
      
#ifdef AMEDEBUGw
         amefprintf(stdout,"using floor\n");
         amefprintf(stdout,"timerange=%14.8e\n", timerange);
         amefprintf(stdout,"actual_timestep=%14.8e\n", actual_timestep);
         amefprintf(stdout,"amesys->simOptions->fixedH=%14.8e\n", amesys->simOptions->fixedH);
         amefprintf(stdout,"fabs(actual_timestep-amesys->simOptions->fixedH)=%14.8e\n", fabs(actual_timestep-amesys->simOptions->fixedH));
         amefprintf(stdout,"stepratio_floor=%d\n", stepratio_floor);
         amefprintf(stdout,"stepratio_ceil=%d\n", stepratio_ceil);
#endif
         actual_timestep = timerange/(double)stepratio_floor;
         stepratio = stepratio_floor;
      }
      
      if(fabs(actual_timestep-amesys->simOptions->fixedH) > 0.001*amesys->simOptions->fixedH)
      {
#ifdef AMEDEBUGw
         amefprintf(stdout,"Adjusting time step %14.8e => %14.8e\n", amesys->simOptions->fixedH, actual_timestep);
         amefprintf(stdout,"stepratio_floor=%d\n", stepratio_floor);
         amefprintf(stdout,"stepratio_ceil=%d\n", stepratio_ceil);
         amefprintf(stdout,"stepratio=%d\n", stepratio);
         amefprintf(stdout,"timerange=%14.8e\n", timerange);
#endif
      }
   }
   else
   {
      /* just single step from the last point in time when we were called */
      stepratio=1;
      actual_timestep = timerange;
   }

#if !defined(AMERT) && !defined(FMI) && !defined(STANDALONESIMULATOR)
   if(amesys->resultFileStructPtr && !amesys->resultFileStructPtr->outoff)
   {
      next_print_time = GetNextPrintTime(&tinc, amesys->resultFileStructPtr->lastprinttime, amesys->simOptions->tFinal, amesys->simOptions->tInc, actual_timestep);
   }
#endif

#ifdef AMERT
   /* Allow changes of the stepratio - typically on RT platforms. */
   {
      double localstepratio=floor(IL_OpdrachtDeel2AmesimSine_step_ratio);
      if (localstepratio >= 1) 
      {
         actual_timestep=timerange/localstepratio;
         stepratio = (int)localstepratio;
      }   
   }
#endif
#ifdef STANDALONESIMULATOR
   FixedStepIntegrate(amesys,NUMSTATES,amesys->tlast,time,amesys->simOptions->tInc,amesys->states,
         amesys->yh,amesys->simOptions->fixedType,amesys->simOptions->fixedOrder,actual_timestep);
#else 
	for (i=0; (i<stepratio) && (amesys->requestinterrupt == 0);i++)
   {
      /*Integrate one step */
	  if ( amesys->simOptions->fixedType == 1)
	  {
		DoAnABStep(amesys, amesys->numstates, amesys->simOptions->fixedOrder, &amesys->tlast, actual_timestep, amesys->states, amesys->yh);
	  }
	  else
	  {
		DoAnRKStep(amesys, amesys->numstates, amesys->simOptions->fixedOrder, &amesys->tlast, actual_timestep, amesys->states, amesys->yh);
	  }
      
#ifndef AMERT   
      isTimeForPrint = amesys->resultFileStructPtr && (!amesys->resultFileStructPtr->outoff && ((amesys->tlast >= next_print_time) || ((next_print_time-amesys->tlast)/tinc < TIME_ROUNDOFF)));
      if(isTimeForPrint)
      {
         ProcessTime(1);
         SetIsPrinting(amesys);
         amesys->FunctionEval(amesys, amesys->yh,amesys->states,amesys->tlast,&zero);
         ClearIsPrinting(amesys);
         amesys->OutputResults(amesys,amesys->tlast);
         next_print_time = GetNextPrintTime(&tinc, amesys->tlast, amesys->simOptions->tFinal, amesys->simOptions->tInc, actual_timestep);
      }
      else
#endif
      {
         amesys->FunctionEval(amesys, amesys->yh,amesys->states,amesys->tlast,&zero);
      }
   }
#endif  
   amesys->tlast = time;
}



/* This function should be called before running running a simulation
   It sets up start time, end time, save increment, maxstepsize,
   tolerance, error control type, how much info is printed,
   extra dicontinuity printouts, run statistics, run type */

static int ameSetOptions(AMESIMSYSTEM *amesys,
                         double tsaveinc, 
                         double maxstepsize,
                         double tolerance,
                         int errorcontrol,
                         int writelevel,
                         int extradisconprint,
                         int runstats,
                         int theruntype,
                         int thesolvertype)
{
   
   double *hmax, *tinc; 
   
   hmax = (double *)calloc(1,sizeof(double));
   tinc = (double *)calloc(1,sizeof(double));
   
   SetGlobalSystem(amesys);
   amesys->simOptions->outoff  = tsaveinc <= 0.0;  /* Possibility to switch off output (if saveinc <= 0) */
   *hmax = maxstepsize;
   amesys->simOptions->hMax       = maxstepsize;
   amesys->simOptions->iWrite     = writelevel;
   amesys->simOptions->rStrtp     = extradisconprint;
   amesys->simOptions->statistics = runstats;

   amesys->simOptions->tInc       = tsaveinc;
   if(tsaveinc < 0)
   {
      amesys->simOptions->tInc       = -tsaveinc;
   }

   *tinc = amesys->simOptions->tInc;
   ValidateRuntype(theruntype);
   amesys->simOptions->runType = theruntype;
   amesys->simOptions->solverType = thesolvertype;

   amesys->simOptions->tol = tolerance;
#ifdef STANDALONESIMULATOR
	RegisterRunParams(errorcontrol, tolerance,extradisconprint,runstats,0, thesolvertype,theruntype,2,amesys->simOptions->tInc,0.0,0.0,*hmax);
#endif 
 
  
   amesys->simOptions->hMax       = *hmax;

   if(amesys->simOptions->solverType)
   {
      /* It is the cautious option. The maximum time step
         should not exceed the print interval. */

      setmaxstep_(&amesys->simOptions->tInc);
   }

   ameSetupTolerance(amesys->simOptions);

   free(hmax);
   free(tinc);
   return 1;
}
                   
                   
/* This function return the number of inputs, outputs, states and implicits */                  
DLLEXPORTDEF_OR_STATIC int AMEInitSizes(int *numinputs, int *numoutputs, int *numstates, int *numimplicits)
{
#ifdef NUMINPUTS
   *numinputs=NUMINPUTS;
#else
   *numinputs=0;
#endif

#ifdef NUMOUTPUTS
   *numoutputs=NUMOUTPUTS;
#else
   *numoutputs=0;
#endif

   *numstates=NUMSTATES;

#ifdef NEQ
   *numimplicits=NEQ-NUMSTATES;
#else
   *numimplicits=0;
#endif

   return 1;
}
DLLEXPORTDEF_OR_STATIC int AMEGetSizes(int *numinputs, int *numoutputs, int *numstates, int *numimplicits)
{
   return AMEInitSizes(numinputs, numoutputs, numstates, numimplicits);
}


/* Function to set-up initial conditions. 
   it should return the initialvalues on the output variables */
DLLEXPORTDEF_OR_STATIC int AMEInitializeConditions(double time, int numOutputs, double *outputARG)
{
#ifndef AMERT
   AMESIMSYSTEM *amesys = GetGlobalSystem();

   assert(amesys);

   if(numOutputs != amesys->numoutputs)
   {
      DisplayMessage("AMEInitializeConditions> The number of outputs does not correspond\n");
      return 0;
   }
  
   {
   static int num_fixed = 0;
   static int FIXED[1]; /* Never used. */

      SetGlobalSystemFixed(amesys, num_fixed, FIXED);
   }

   /* Initialise some static variables */

   amesys->first_call=1;  /* should this be done or not ?*/
   amesys->needrestart = 1;

   amesys->tlast=TLAST_MIN;

   memset(amesys->ecount,0,amesys->numstates*sizeof(int));
   memset(amesys->dotstates,0,amesys->numstates*sizeof(double));

   /* Call Input to read submodel and simulation parameters. */
   Input(amesys, amesys->states);
   
   /*Register print interval that maybe used bys some submodels*/
	
   recordtinc_(amesys->simOptions->tInc);
  
   setstarttime_(time);
  
   /* Call pre-initialize function */

   PreInitialize(amesys,amesys->states);

#ifndef FMI   
   if( NeedReloadInputFiles() != 0 )
   {
      ClearGPs();
      Input(amesys,amesys->states);
      ClearReloadedFiles();
   }
#endif
   
   /* Call initialize subroutine to set con and icon array members */
   
   Initialize(amesys,amesys->states);

   /* Overwriting initial state values with requests emitted by */
   /* submodels that have a more global view (cf. register.c mechanism) */
   /* Can also fire some callbacks to 'fix' float and integer store */
   OverloadStatesWithRegister(amesys, amesys->states, SVREGISTER_DEFAULT);

   CheckSimParams(&amesys->simOptions->abstol,
                  &amesys->simOptions->reltol,
                  &amesys->simOptions->hMax); 
   
   /*  Open file for results. */
   amesys->AmeReadFile(amesys, &time, amesys->states);
   SetInitValues(amesys);

   /* Read linear analysis specification. */
#ifndef FMI
   SetLADetails(GetLAFileName(), amesys->numstates, amesys->numvars, time,  amesys->simOptions->reltol, amesys->simOptions->abstol, getfinaltime_()-time);

   /* Remove old err file */
   
   remove(GetErrorFileName());
   
   /* Initialize the Performance analyzer */
   if(!isfixedstepsolver_()){
      PerfAnalyzer_Init(amesys, time, getfinaltime_() );
   }
#endif
   if(isconrun_())
      setstarttime_(time);

   /* Set the locked states info */
#if defined(FMI) || defined(AMERT)
   {
#include "OpdrachtDeel2AmesimSine_.lock.h"
      if(0 != SetUpLockedStatesFromMemory(amesys, lockedstates_length, lockedstates_array))
      {
         amefprintf(stderr,"Failed to set the locked states status.\n");
      }
   }
#else
   SetUpLockedStates(GetCircuitName());
#endif

   if (IsAssemblyNecessary_())
   {
      static double dot[NUMSTATES];
      int local_flag;

      /* Perform the assembly. */

      amesys->consflag = 1;
      local_flag = 0;
      amesys->FunctionEval(amesys, dot, amesys->states, time, &local_flag);
      amesys->first_call = 1;
   
      amesys->consflag = 2;
      local_flag = 0;
      amesys->FunctionEval(amesys, dot, amesys->states, time, &local_flag);

      amesys->first_call = 1;
      amesys->consflag = 0;
   }

   /* Initialize, maybe perform an initialising run */

   if(isstabrun_())
   {
      amesys->simOptions->fixedTime     = time;
   }
   amesys->simOptions->stabilOption += 4*amesys->simOptions->solverType;
  
   
   if(!IntegrateInit(amesys, time, time))
   {
      return 0;
   }

#if (NUMOUTPUTS > 0)
   memcpy(outputARG, amesys->outputs,amesys->numoutputs*sizeof(double) );
#endif
   amesys->tlast = time;
   
   return 1;
#else
   char error_message[256];
   sprintf(error_message, "AMEInitializeConditions> Should never be called for real-time simulation\n");      
   DisplayMessage(error_message);
   return 0;
#endif
}

/* Function to return derivatives. NOT USED (YET) */

DLLEXPORTDEF_OR_STATIC void AMEDerivatives(double *dotstates, double *states, double *inputs)
{

}

/* This function sets the inputs for the next time interval */

static int ameInputs(AMESIMSYSTEM *amesys, int numInputs, const double *inputARG)
{
   if(numInputs != amesys->numinputs)
   {
      char error_message[256];
      sprintf(error_message, "AMEInputs> Expected %d inputs but got %d\n", amesys->numinputs, numInputs);      
      DisplayMessage(error_message);
      return 0;
   }
#if (NUMINPUTS > 0)
   memcpy(amesys->inputs, inputARG, amesys->numinputs*sizeof(double) );
#endif
   return 1;
}

DLLEXPORTDEF_OR_STATIC int AMEInputs(int numInputs, const double *inputARG)
{
  AMESIMSYSTEM *amesys = GetGlobalSystem();
  return ameInputs(amesys, numInputs, inputARG);
}

/* Function to return outputs at a given time (must be greater than the last time). */

static int ameOutputs(AMESIMSYSTEM *amesys, double timeARG, int numOutputs, double *outputARG)
{
   int theprintflag=1;
   double *dot;
   double *v;
   
   assert(amesys);

   v = amesys->v;
   dot = amesys->dotstates;

   if(numOutputs != amesys->numoutputs)
   {
      char error_message[256];
      sprintf(error_message, "AMEOutputs> Expected %d outputs but got %d\n", amesys->numoutputs, numOutputs);
      DisplayMessage(error_message);
      AmeExit(1);
   }

   if (amesys->simOptions->runType == 4)
	{
		/* stabilizing has already been processed during Init.*/
		/*Exit */
		return 1;
	}
   if(timeARG < amesys->tlast)
   {
      DisplayMessage("trying to integrate backwards\n");
      return 0;
   }
   setfinaltime_(timeARG);
   amesys->t_end_of_time_slice = timeARG; 

#ifndef AMERT
   if(!isfixedstepsolver_())
   {
      if(!IntegrateStep(amesys, amesys->tlast, timeARG))
      {
         DisplayMessage("IntegrateStep failed");
         return 0;
      }
      amesys->tlast = timeARG;
   }
   else
   {
      AMEDoAFixedStep(timeARG);
   }
#else
   AMEDoAFixedStep(timeARG);
#endif

#if (NUMOUTPUTS > 0)
   memcpy(outputARG, amesys->outputs, amesys->numoutputs*sizeof(double) );
#endif

   return 1;

}

DLLEXPORTDEF_OR_STATIC int AMEOutputs(double timeARG, int numOutputs, double *outputARG)
{
   AMESIMSYSTEM *amesys = GetGlobalSystem();

   return ameOutputs(amesys, timeARG, numOutputs, outputARG);
}

/*
 * AMETerminate - called when the simulation is terminated.
 *
 * In this function, you should perform any actions that are necessary
 * at the termination of a simulation.  For example, if memory was allocated
 * in AMEInitializeConditions, this is the place to free it.
 * This function is also called from the AmeExit (or by the caller of the model)
 *
 * Issue 0128299 : Problem when model calls AmeExit for the case of user_cosim
 *                 add a test on amesys here - to avoid crashes if AmeTerminate 
 *                 is called after AmeExit
 *
 *                 If we get called before setting up the runflags we can crash
 *                 in the call to isfixedstepsolver. Add check on amesys->RunFlags
 */

static void ameTerminate(AMESIMSYSTEM *amesys)
{
   if (amesys == NULL)
   {
      return;
   }
  
   SetGlobalSystem(amesys);
#if !defined(AMEVERISTAND) && (!defined(LABVIEWCOSIM) || !defined(AMERT))
   if(amesys->simOptions ) {
      if(amesys->simOptions->statistics)
      {
         WriteRunStats(amesys);
      }
   }
#endif
   ProcessTime(2);

   /* Save state count, discontinuities and finalize the Performance Analyzer */
#ifndef FMI
   if(!isfixedstepsolver_()){
      PerfAnalyzer_SaveStateCount (amesys);
      PerfAnalyzer_SaveDiscontinuities(amesys);
      PerfAnalyzer_Close(amesys);   
   }
#endif
   /* Simulation complete. Close results files. */

#if !defined(AMERT) && !defined(FMI)
   amesys->CloseResultFile(amesys);
#endif

   /* Do a call to  IntegrateEnd to cleanup the solver */
#if !defined(AMERT) && !defined(FMICS1)
   if(amesys->RunFlags && !isfixedstepsolver_())
   {
      IntegrateEnd(amesys, amesys->tlast, amesys->tlast);
   }
#endif

   EndOfSimulation(amesys);

   AmeCallAtEnd(ameExitStatus);
#ifndef STANDALONESIMULATOR
   AmeSignalModelUnload();
#endif

   reset_real_integer_stores();

#ifndef STANDALONESIMULATOR
   DeleteGlobalSystem(amesys);
   /* Report 0121694 clean Global system pointer */
   SetGlobalSystem(NULL);      
#endif

}


void AMESetRunParam_double(const char* paramname, double value);
void AMESetRunParam_int(const char* paramname, int value);



DLLEXPORTDEF_OR_STATIC void AMETerminate(void)
{
   AMESIMSYSTEM *amesys = GetGlobalSystem();
   ameTerminate(amesys);
}

/* AMEInitModel is the main initialization function */

static void ameInitModel(AMESIMSYSTEM *amesys,
                         double time,
                         double PrintInterval, 
                         double MaxTimeStep,
                         double tolerance,
                         int errCtrl,
                         int writeLevel,
                         int extraDisconPrints,
                         int runStats,
                         int runType,
                         int thesolvertype,
                         int numInputs,
                         int numOutputs,
                         double *theInputs, 
                         double *theOutputs)
{
#ifndef AMERT
   int nResult;

   SetGlobalSystem(amesys);

   amesys->num_steps_taken=0;
   nResult=AMEInputs(numInputs, theInputs);              /* Should check the return value here */
   
   nResult=ameSetOptions(amesys,
                         PrintInterval,    /* Print Interval, negative if no output */
                         MaxTimeStep,      /* Max time step */
                         tolerance,        /* Tolerance */
                         errCtrl,          /* Error control, 0 for mixed, 1 for relative, 2 for absolute */
                         writeLevel,       /* Write level 0 for time, 1 for full and 2 for no output */
                         extraDisconPrints,/* Extra discon prints if 1 */
                         runStats,         /* Runstats if 1 */
                         runType,           /* Run type, 8 for normal run */
                         thesolvertype
                         );

   if(!AMEInitializeConditions(time, numOutputs, theOutputs))
   {
      AmeExit(1);
   }
#else
   char error_message[256];
   ConstructFileNames(amesys,"OpdrachtDeel2AmesimSine_", NULL);
   sprintf(error_message, "AMEInitModel> Should never be called for real-time simulations.\n");      
   DisplayMessage(error_message);
   return;
#endif
}


/***********************************************
 *
 * Setup the fixedstep solver - and the model
 *
 * 2012-03-14 0125135 AJN Add "if (IsAssemblyNecessary_())..."
 *            to deal with assembly also for fixedstep.
 *            Note that this will work fine on a regular PC,
 *            but for a RT target it may fail as it does not 
 *            yet have access to the lock file.
 * 2013-09-06 erz 0134224 continuation run and use old final values with opensimulator
 *            Add a argument to handle the run_type, fixed_step arg still handles the integration method  
 *  
 ***********************************************/
static int ameSetUpFixedStepSolver2(AMESIMSYSTEM *amesys, double start_time, int numinputs, int numoutputs, 
                                    int run_type, int fixed_type, int runge_kutta_order, double fixed_h, double printinterval)
{
   SetGlobalSystem(amesys);
   if (numinputs != NUMINPUTS)
   {
      amefprintf(stderr,"Expected numinputs = %d, got %d - exit!", NUMINPUTS, numinputs);
      return 1;
   }

   if (numoutputs != NUMOUTPUTS)
   {
      amefprintf(stderr,"Expected numoutputs = %d, got %d - exit!", NUMOUTPUTS, numoutputs);
      return 2;
   }


   /* Initialise some static variables */
   
   amesys->first_call=1;  /* should this be done or not ?*/

   amesys->num_steps_taken=0;
   
   amesys->tlast=-1E30;
   amesys->simOptions->iWrite = 2;
   memset(amesys->yh,0,NUMSTATESx13*sizeof(double));


   /* Init solver */
   InitFixedStepIntegrate(amesys);

   /* Call Input to read submodel and simulation parameters. */
   Input(amesys,amesys->states);

   ValidateRuntype(run_type);
	
	/* Ensure that runflag StabilizingRun=0. It might have been set to true */
	/* due to a previous selection for the variable step integrator. */
   ClearStabilizingRun();
   
	amesys->simOptions->fixedOrder = runge_kutta_order;
   amesys->simOptions->fixedStep  = 1; /* Yes - fixed step */
   amesys->simOptions->fixedH     = fixed_h;

   amesys->simOptions->tFinal     = 1e30; /* currently we don't know when to stop */

   if(printinterval > 0)
   {
      amesys->simOptions->tInc =  printinterval;
      amesys->simOptions->outoff = 0;
   }
   else
   {
      amesys->simOptions->outoff = 1;
   }

   amesys->simOptions->fixedType  = fixed_type;
   SetIsUsingFixedSolver(( fixed_type == 1)*100 +  (fixed_type != 1)*200 + runge_kutta_order);
   
   SetFixedTimeStep(fixed_h);

   recordtinc_(amesys->simOptions->tInc);

   setstarttime_(start_time);
   
   /* Call pre-initialize function */

   PreInitialize(amesys,amesys->states);

   if( NeedReloadInputFiles() != 0 )
   {
      ClearGPs();
      Input(amesys,amesys->states);
      ClearReloadedFiles();
   }

   /* Call initialize subroutine to set con and icon array members */
   
   Initialize(amesys,amesys->states);

   /* Overwriting initial state values with requests emitted by */
   /* submodels that have a more global view (cf. register.c mechanism) */
   /* Can also fire some callbacks to 'fix' float and integer store */
   OverloadStatesWithRegister(amesys, amesys->states, SVREGISTER_DEFAULT);

   /*  Open file for results. */
   amesys->AmeReadFile(amesys, &start_time, amesys->states);
   SetInitValues(amesys);

   if(isconrun_())
      setstarttime_(start_time);

   if (IsAssemblyNecessary_())
   {
      static double dot[NUMSTATES];
      int local_flag;

      /* Set the locked states info */
      
#if defined(FMI) || defined(AMERT)
      {
#include "OpdrachtDeel2AmesimSine_.lock.h"
         if(0 != SetUpLockedStatesFromMemory(amesys, lockedstates_length, lockedstates_array))
         {
            amefprintf(stderr,"Failed to set the locked states status.\n");
         }
      }
#else
      SetUpLockedStates(GetCircuitName());
#endif

      /* Perform the assembly. */

      amesys->consflag = 1;
      local_flag = 0;
      amesys->FunctionEval(amesys, dot, amesys->states, start_time, &local_flag);
      amesys->first_call = 1;
   
      amesys->consflag = 2;
      local_flag = 0;
      amesys->FunctionEval(amesys, dot, amesys->states, start_time, &local_flag);

      amesys->first_call = 1;
      amesys->consflag = 0;
   }

   amesys->tlast = start_time;

   /* Initialization of time timers */
   ProcessTime(0);
   
   return 0;
}

static int ameSetUpFixedStepSolver(AMESIMSYSTEM *amesys, double start_time, int numinputs, int numoutputs, 
                                   int fixed_type, int runge_kutta_order, double fixed_h, double printinterval)
{
   int run_type = 8;
   return ameSetUpFixedStepSolver2(amesys, start_time,  numinputs,  numoutputs, 
                                   run_type, fixed_type,  runge_kutta_order,  fixed_h,  printinterval);
}

/***********************************************************************
 * AMEdoAStep performs the actual simulation using AMEOutputs and AMEInputs .
 * The function that does the real work is actually AMEOutputs.
 *
 * AMEInputs only sets up the input vector for the next time increment.
 *
 * Please notice that the outputs at the time t are obtained using the
 * inputs that were set the last time increment.
 * This will introduce a time delay of one sample time.  This
 * is corresponding to a sampled system.
 *
 * See also AMEdoAStep2 where the order of the calls is reversed.
 *
 ************************************************************************/

static int amedoAStep(AMESIMSYSTEM *amesys, double t, int numInputs, int numOutputs, 
                                      const double *theInputs, double *theOutputs)
{
   /* Reset interrupt simulation flag */
   amesys->requestinterrupt = 0;

   if(!AMEOutputs(t, numOutputs, theOutputs))
   {
      return 0;
   }
   CheckIfColdStartNeed(amesys->inputs, theInputs, numInputs, amesys->num_steps_taken, &amesys->needrestart);
   if(!AMEInputs(numInputs, theInputs))
   {
      return 0;
   }
   
   amesys->num_steps_taken++;

   return 1;
}

DLLEXPORTDEF_OR_STATIC int AMEdoAStep(double t, int numInputs, int numOutputs, 
                                      const double *theInputs, double *theOutputs)
{
   AMESIMSYSTEM *amesys = GetGlobalSystem();

   return amedoAStep(amesys, t, numInputs, numOutputs, theInputs, theOutputs);
}

/***********************************************************************
 * AMEdoAStep2 performs the actual simulation using AMEOutputs and
 * AMEInputs. The function that does the real work is actually AMEOutputs.
 *
 * AMEInputs only sets up the input vector for the next time increment.
 *
 * Please notice that the outputs at the time t are calculated using the
 * inputs that have just been received from the master. The master and
 * slave can work in parallel as separate processes until it is time to
 * exchange information.
 *
 * The first stage in this information exchange is that the slave sends
 * its outputs it has just calculated to the master. The master then
 * receives these values (i.e. the master samples the slave). The master
 * then sends it values to the slave and the cycle repeats.
 *
 * See also AMEdoAStep where the order of the calls is reversed.
 *
 ************************************************************************/

static int ameDoAStep2(AMESIMSYSTEM *amesys, double t, int numInputs, int numOutputs, const double *theInputs,
                                       double *theOutputs)
{
   /* Reset interrupt simulation flag */
   amesys->requestinterrupt = 0;

   SetGlobalSystem(amesys);
   if(!AMEInputs(numInputs, theInputs))
   {
      return 0;
   }
   if(!AMEOutputs(t, numOutputs, theOutputs))
   {
      return 0;
   }
   amesys->num_steps_taken++;

   return 1;
}


DLLEXPORTDEF_OR_STATIC int AMEdoAStep2(double t, int numInputs, int numOutputs, const double *theInputs,
                                       double *theOutputs)
{
   AMESIMSYSTEM *amesys = GetGlobalSystem();
   return ameDoAStep2(amesys,  t,  numInputs,  numOutputs, theInputs,theOutputs);
}

/***************************************************************

   Author       : A Jansson

   Created on   : 2007-08-22
   brief Function that returns the "numvalues" values in the v array 
          (set in the "values" array) that is indicated by the "indices" input vector.
   Revisions    : 2011-09-12 Add to cosim templates - for VL motion

***************************************************************/

DLLEXPORTDEF_OR_STATIC int AMEGetDisplayVariableValues(int numvalues, int *indices, double *values)
{
   int i;
   int status=-1;
   AMESIMSYSTEM *amesys = GetGlobalSystem();

   if ( (numvalues > 0) && (indices != NULL) && (values != NULL) ) {
      status = 0;

      for (i=0; i< numvalues; i++) {
         if ( (indices[i] >= 0) && (indices[i] < NUMVARS) ) {
            values[i] = amesys->v[indices[i]];
         }
         else {
            values[i] = 1.0e40;
            status = -1000;
         }
      }
   }

   return(status);
}

/***************************************************************

   Author       : A Jansson

   Description  : return labels for input variables.
   Revisions    :

***************************************************************/

DLLEXPORTDEF_OR_STATIC int AMEModelGetInputVariables(int *numinputs, char ***inputlabels, char ***units)
{
   getPortNames(numinputs, NULL, inputlabels, NULL);
   return 0;
}

/***************************************************************

   Author       : A Jansson

   Description  : return labels for output variables.
   Revisions    :

***************************************************************/
DLLEXPORTDEF_OR_STATIC int AMEModelGetOutputVariables(int *numoutputs, char ***outputlabels, char ***units)
{
   getPortNames(NULL, numoutputs, NULL, outputlabels);
   return 0;
}

/***************************************************************

   Author       : A Jansson

   Description  : free memory allocated by AMEModelGetInputVariables and
                  AMEModelGetOutputVariables
   Revisions    : 2011-09-11 AJN avoid crashes when "labels" or "units" are NULL

***************************************************************/
DLLEXPORTDEF_OR_STATIC void AMEModelFreeVariableLablesAndUnits(int *numvars, char ***labels, char ***units)
{
   int i;
   char **freetheselables = NULL;
   char **freetheseunits  = NULL;

   if(labels != NULL)
   {
      freetheselables = *labels;
   }
   if(units != NULL)
   {
      freetheseunits  = *units;
   }

   if (*numvars > 0)
   {
      for (i=0; i < *numvars ; i++)
      {
         if(freetheselables && freetheselables[i])
         {
            free(freetheselables[i]);
            freetheselables[i] = NULL;
         }
         if(freetheseunits && freetheseunits[i])
         {
            free(freetheseunits[i]);
            freetheseunits[i] = NULL;
         }
      }
      if(freetheselables)
      {
         free(freetheselables);
         freetheselables = NULL;
      }
      if(freetheseunits)
      {
         free(freetheseunits);
         freetheseunits = NULL;
      }
   }
}

/***************************************************************

   Author       : A Jansson

   Created on   : 2011-06-01
   Description  : create a AMESYSTEM structure and fill it
   Revisions    :

***************************************************************/

static AMESIMSYSTEM *setupAmeSystem()
{
   AMESIMSYSTEM *amesys;

#ifndef NEQ
   amesys = InitGlobalSystem(NUMSTATES, NUMVARS, NUMSTATES, NUMDISCRETESTATES);
#else
   amesys = InitGlobalSystem(NUMSTATES, NUMVARS, NEQ, NUMDISCRETESTATES);
#endif
   SetGlobalSystem(amesys);
   ConstructFileNames(amesys,"OpdrachtDeel2AmesimSine_", NULL);
   
#if defined(AMERT) || defined(FMI)
   amesys->getssflist = getssflist;
#endif
   amesys->consflag = 0;

   free(amesys->v);
   amesys->v = static_v;
   amesys->v_is_static = 1;

   amesys->numinputs = NUMINPUTS;
   amesys->numoutputs = NUMOUTPUTS;

#if (NUMINPUTS > 0)
   amesys->inputs = calloc(amesys->numinputs,sizeof(double));
#else
   amesys->inputs = NULL;
#endif

#if (NUMOUTPUTS > 0)  
   amesys->outputs = calloc(amesys->numoutputs,sizeof(double));
#else
   amesys->outputs = NULL;
#endif

   amesys->FunctionEval = localFunctionEval;
   amesys->JFunctionEval = localJFunctionEval;
   amesys->AmeExit = ModelAmeExit;

   ameExitStatus = 0;

   ModelData.varArray =  amesys->v;
   ModelData.varArraySize =  NUMVARS;
   ModelData.stateVarArraySize =  NB_STATE_VAR;
   ModelData.stateVarArray =  amesys->states;
   ModelData.discreteStateVarArray = amesys->discrete_states;
   return  amesys;
}

/***************************************************************

   Created on   : 2012-APR-18

   Description  : Call the LDoLinearAnalysisOnDemand on the
                  current system and return the A,B,C,D matrices
                  of the state-space form of the linearised system.
                  Albeit this function seems useless,
                  we actually need it to call the right
                  linear analysis function w.r.t. to the type
                  of solver (here LSODA).
   Revisions    :

***************************************************************/
#if (defined(FMICS1) || defined(FMICS2)) && (!defined(AMERT))
static int getPartialDerivatives(AMESIMSYSTEM  *amesys,
                                 SPARSE_MATRIX *Amat,
                                 SPARSE_MATRIX *Bmat,
                                 SPARSE_MATRIX *Cmat,
                                 SPARSE_MATRIX *Dmat)
{

   /* Here we assume that the Amat, Bmat, Cmat, Dmat matrices are
      already allocated by the caller with the correct sizes. */
   if (amesys->tlast > TLAST_MIN)
      return LDoLinearAnalysisOnDemand(amesys, amesys->numstates,
                                       amesys->tlast, amesys->states,
                                       Amat, Bmat, Cmat, Dmat);
   else
      return 0;  /* failed: system probably not initialized */
} /* getPartialDerivatives */
#endif

/***************************************************************
 * Instead of having several files with almost the same content
 * all interface dependent code is isolated in the following
 * c files that are included into this file.
 */

#define SYSNME OpdrachtDeel2AmesimSine_

#ifdef AMESIMULINK
#include "ame_simulink_cosim.h"
#elif defined(AMESIMPACK)
#include "ame_simpack_cosim.c"
#elif defined(AMEAMESIM)
#include "ame_amesim_cosim.c"
#elif defined(FMICS1)
#include "ame_fmics1.h"
#elif defined(FMICS2)
#include "ame_fmics2.h"
#elif defined(FMIME1)
#if defined(AMERT)
#error "FMU for real-time for model exchange is not allowed."
#else
#include "ame_fmime1.h"
#endif
#elif defined(FMIX)
#include "ame_user_cosim.c"
#elif defined(AMESTANDALONE)
#include "ame_standalone.c"
#elif defined(AMEADAMS)
#include "ame_adams_cosim.c"
#elif defined(AMERECURDYN)
#include "ame_recurdyn_cosim.c"
#elif defined(STANDALONESIMULATOR)
#include "ame_standalone_simulator.c"
#elif defined(AMEUSERCOSIM)
#include "ame_user_cosim.c"
#elif defined(AMEVERISTAND)
#include "ame_user_cosim.c"
#include "NIV_model.c"
#include "AME_NIVERISTAND_API.c"
#elif defined(AMEMORPHEE)
#include "ame_morphee_cosim.c"
#if defined(AMERT)
#include "OpdrachtDeel2AmesimSine_.morphee_sigmap.h"
#endif
#if defined(AMERT)
#include "OpdrachtDeel2AmesimSine_.morphee_parmap.h"
#endif
#elif defined(DISCRETEPART)
#include "ame_discrete_part.c"
#elif defined(LABVIEWCOSIM)
#include "labview_cosim.c"
#elif defined(AMEMOTIONCOSIM)
#include "ame_motion_cosim.c"
#else
#error "One of AMESIMULINK, AMESIMPACK, AMERECURDYN, AMEUSERCOSIM, AMEVERISTAND, FMICS1, FMICS2, FMIME1, AMEAMESIM, DISCRETEPART or LABVIEWCOSIM must be defined"
#endif
