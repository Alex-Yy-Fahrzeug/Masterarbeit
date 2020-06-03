//////////////////////////////////////////////////////////////////////////
// include files
//////////////////////////////////////////////////////////////////////////
#define _USE_MATH_DEFINES

#include "vcisdk.h"

#include <stdio.h>
#include <conio.h>
#include "SocketSelectDlg.hpp"
#include <time.h>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include <fstream>
#include <string>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>



//////////////////////////////////////////////////////////////////////////
// constant definitions
//////////////////////////////////////////////////////////////////////////
#define MAX_RADAR_OBJECTS 90
#define MAX_MOBILEYE_OBJECTS 10
#define MAX_MEASUREMENT_OBJECTS 40
#define MAX_FUSION_OBJECTS 80
#define MAX_UDP_OBJECTS 40
#define MAX_POINTNUMBER 1000
#define MAX_OBJECTHISTORY 100
#define NUM_BYTES_1401 1453
#define NUM_BYTES_1402 16
#define NUM_BYTES_1403 52

typedef unsigned char       UINT8, * PUINT8;

///////////////////////////////////////////
// Color definition
///////////////////////////////////////////

static cv::Scalar red = cv::Scalar(0, 0, 255);
static cv::Scalar magenta = cv::Scalar(255, 0, 255);
static cv::Scalar blue = cv::Scalar(255, 0, 0);
static cv::Scalar teal = cv::Scalar(255, 255, 0);
static cv::Scalar green = cv::Scalar(0, 255, 0);
static cv::Scalar yellow = cv::Scalar(0, 255, 255);
static cv::Scalar white = cv::Scalar(255, 255, 255);

static int imgWidth_TV = 600;
static int imgHight_TV = 600;
static int VisionWidth = 150;
static int VisionFar = 150;

//////////////////////////////////////////////////////////////////////////
// structure definitions
//////////////////////////////////////////////////////////////////////////
typedef struct _RadarObject {
	UINT8 ID;
	UINT8 Class;
	UINT8 ProbOfExist;
	Eigen::Vector4f x;
	float Width;
	float Length;
	float RCS;
} RadarObject;

typedef struct _ObjectList_Radar {
	UINT8 n;	//number of Objects
	float timestamp;
	float speed;
	RadarObject Objects[MAX_RADAR_OBJECTS];
} ObjectList_Radar;

typedef struct _MobileyeObject {
	UINT8 ID;
	UINT8 Class;
	UINT8 Lane;
	Eigen::Vector4f x;
	float Width;
} MobileyeObject;

typedef struct _ObjectList_Mobileye {
	UINT8 n;	//number of Objects
	float timestamp;
	MobileyeObject Objects[MAX_MOBILEYE_OBJECTS];
} ObjectList_Mobileye;

typedef struct _FusionObject {
	UINT8 ID_Rad;
	UINT8 ID_Mob;
	UINT8 Class;
	UINT8 Lane;
	Eigen::Vector4f xk;
	Eigen::Matrix4f Pk;
	float Width;
	float Length;
	float ProbOfExist;
	float RCS;
	float Relevance;
} FusionObject;

typedef struct _ObjectList_Measurement {
	UINT8 n;	//number of Objects
	FusionObject Objects[MAX_MEASUREMENT_OBJECTS];
} ObjectList_Measurement;

typedef struct _ObjectList_Fusion {
	UINT8 n;	//number of Objects
	float timestamp_Rad;
	float timestamp_Mob;
	float speed;
	FusionObject Objects[MAX_FUSION_OBJECTS + MAX_MEASUREMENT_OBJECTS];
} ObjectList_Fusion;

typedef struct _UDPObject {
	UINT8 ID_Rad;
	UINT8 ID_Mob;
	UINT8 Class;
	UINT8 Lane;
	float PosLongX;
	float PosLatY;
	float VrelLongX;
	float VrelLatY;
	float Width;
	float Length;
	float ProbOfExist;
	float RCS;
} UDPObject;

// typdef from YY for Fahrschlauch //////////////////////////////////
typedef struct _Objecthistory {
	UINT8 ID_Rad;
	UINT8 ID_Mob;
	float timestamp_latest_renew; 
	int   NumofPoint;	 // number of Points
	bool  Detection;
	bool  Selected;
	float distance;
	float SPW;
	float PLA;           // Spurwahrscheinlichkeit
	float Position[2][MAX_POINTNUMBER];
	float curvature[2];
}Objecthistory;

typedef struct _Pufferspeicher {
	int   n;	                            // number of Objects
	float timestamp;
	float last_timestamp;
	Objecthistory Object_History[MAX_OBJECTHISTORY];      // it should not be too huge 
}Pufferspeicher;

typedef struct _Vehicledynamik {
	float Long_Vel;
	float Lat_Vel;
	float Yaw_Rate;
	float Cur_Dyn;
}Vehicledynamik;

typedef struct _AimObject {
	float PLA;
	float SPW;
	bool Exist_of_Aim;
	bool  Detection;
	char  leer[2];
	UDPObject Info;
} AimObject;


//////////////////////////////////////////////////////////////////////////
// global variables
//////////////////////////////////////////////////////////////////////////

//Process control objects
static DWORD				dwRadarReceiveThread;			//thread handles
static DWORD				dwMobileyeReceiveThread;
static DWORD				dwSensorFusionThread;
static DWORD				dwRadarFusionThread;
static DWORD				dwMABReceiveThread;

static HANDLE				hEventListReady[2];				//thread sync events: 0: Radar 1: Mobileye
static HANDLE				hMutex[2];						//thread sync mutexes: 0: Radar 1: Mobileye
static HANDLE				hEventRadarFusionReady;

static LONG					display_enabled = 0;			//display flag for the receive threads

//time objects
static UINT64				system_frequency;
static UINT64				system_frequency_Rad;
static UINT64				system_frequency_Mob;
static UINT64				t_program_start;
static UINT64				t_program_start_Rad;
static UINT64				t_program_start_Mob;
static UINT64				t_cycle_Rad;
static UINT64				t_cycle_Mob;
														
//CAN objects
static IBalObject*			pBalObject_Radar = 0;			// bus access objects
static IBalObject*			pBalObject_Mobileye = 0;

static ICanControl*			pCanControl_Radar = 0;			// control interfaces
static ICanControl*			pCanControl_Mobileye = 0;

static ICanChannel*			pCanChn_Radar = 0;				// channel interfaces
static ICanChannel*			pCanChn_Mobileye = 0;

static HANDLE				hEventReader_Radar = 0;			//CAN reader event handles
static HANDLE				hEventReader_Mobileye = 0;

static PFIFOREADER			pReader_Radar = 0;				//FIFO Readers
static PFIFOREADER			pReader_Mobileye = 0;

static PFIFOWRITER			pWriter_Radar = 0;				//FIFO writers
static PFIFOWRITER			pWriter_Mobileye = 0;

//UDP objects
static SOCKET				UDPSocket;
static SOCKET               UDPSocket_from_MAB;
static SOCKET				UDPSocket_AimInfo;
static SOCKADDR_IN			MABAddr_Port1401;
static SOCKADDR_IN			PCAddr_Port1401;
static SOCKADDR_IN			EBPCAddr_Port1402;
static SOCKADDR_IN			PCAddr_Port1403;
static char					buf_MAB[NUM_BYTES_1401];		//UDP send buffer
static char                 buf_MAB_Receive[NUM_BYTES_1402];  //UDP recive buffer
static char                 buf_AimInfo[NUM_BYTES_1403];

//Radar variables and storage structures
static bool					RadarFilterSet_NofObj = false;
static bool					RadarFilterSet_Azimuth = false;
static bool					RadarFilterSet_X = false;
static bool					RadarFilterSet_Y = false;
static bool					RadarFilterSet_Prob = false;
static bool					RadarFiltersSet = false;
static bool					SpeedWritten = false;

static ObjectList_Radar		rList_Reception;				//new CAN data
static ObjectList_Radar		rList_Exchange;					//for exchange between threads
static ObjectList_Radar		rList_Fusion_Start;				//base for radar filtering
static ObjectList_Radar		rList_Fusion_Candidates;		//base for radar fusion
static ObjectList_Radar		rList_Fusion_Final;				//base for sensor fusion	

static Eigen::Matrix4f		Radar_R;						//Rotation matrix for radar angular correction
static Eigen::Matrix4f		Radar_C;						//Rotation matrix for radar longitudinal factor correction
static Eigen::Vector4f		x_offset;

static UINT8				AscVec_Rad[MAX_RADAR_OBJECTS];	//Radar association vector
static UINT8				Status_Rad[MAX_RADAR_OBJECTS];	//Radar fusion status: 0: copy, 1: delete				

static UINT8				MsgCounter60B = 0;				//Message counters
static UINT8				MsgCounter60C = 0;
static UINT8				MsgCounter60D = 0;

//Mobileye variables and storage strutures
static ObjectList_Mobileye	mList_Reception;				//new CAN data
static ObjectList_Mobileye	mList_Exchange;					//for exchange between threads
static ObjectList_Mobileye	mList_Fusion;					//base for sensor fusion

static UINT8				MsgCounterA = 0;				//Message counters
static UINT8				MsgCounterB = 0;
static UINT8				MsgCounterC = 0;

//Fusion variables and storage structures
static ObjectList_Measurement	fList_Measurement;			//contains fused radar and mobileye data
static ObjectList_Fusion		fList_Fusion;				//contains fusion objects
static UDPObject				UDPObjects[MAX_UDP_OBJECTS];//contains objects to be sent via UDP
static UINT8					NofObjects_UDP;				//contains number of objects to be sent via UDP

static Eigen::Matrix4f			Kalman_P;					//Initial error covariance matrix for kalman filter
static Eigen::Matrix4f			Kalman_A;					//Transition matrix for kalman filter
static Eigen::Matrix4f			Kalman_A_Mobileye;			//Transition matrix for kalman filter
static Eigen::Matrix4f			Kalman_Q;					//Process noise covariance matrix for kalman filter
static Eigen::Matrix4f			Kalman_R;					//Sensor covariance matrix for kalman filter
static Eigen::Matrix4f			Kalman_I;					//Identity matrix for kalman filter

//Parameters as specified by config file
//Process control
static DWORD				FusionEventWaitTime = 130;
static UINT8				sendFusionList = 1;
static UINT8				sendMeasurementList = 0;
static UINT8				sendRadarList = 0;
static UINT8				sendMobileyeList = 0;
static UINT8				sendToMAB = 1;
static UINT8				sendToPC = 1;
static UINT8				simulationMode = 0;
//Radar filter thresholds
static UINT16				Radar_FilterCfg_Min_NofObj = 0;
static UINT16				Radar_FilterCfg_Max_NofObj = 80;
static UINT16				Radar_FilterCfg_Min_Azimuth = 0;
static UINT16				Radar_FilterCfg_Max_Azimuth = 4095;
static UINT16				Radar_FilterCfg_Min_ProbExists = 6;
static UINT16				Radar_FilterCfg_Max_ProbExists = 7;
static UINT16				Radar_FilterCfg_Min_Y = 1948;
static UINT16				Radar_FilterCfg_Max_Y = 2147;
static UINT16				Radar_FilterCfg_Min_X = 2500;
static UINT16				Radar_FilterCfg_Max_X = 3750;
//Radar offsets
static float				gamma_offset = -2.25f;			//degree
static float				c_x_offset = 1.0f;				
static float				p_x_offset = 0.0f;				
static float				p_y_offset = -0.1f;				
//Relevant zone definition for radar objects 
static float				p_y_relzone_Radar = 6.0f;
//Association thresholds for matching radar objects amongst themselves
static float				dist_x_max_Radar = 2.5f;
static float				Delta_p_x_min_Radar = 0.0f;
static float				Delta_p_x_max_Radar = 2.0f;
//Association thresholds for matching radar objects with Mobileye objects
static float				Delta_p_x_min = -5.0f;
static float				Delta_p_x_max_abs = 5.0f;
static float				Delta_p_x_max_percent = 0.2f;
static float				Delta_p_y_max = 2.0f;
//Sensor fusion weights
static float				w_p_x_Radar = 0.9f;
static float				w_p_y_Radar = 0.3f;
static float				w_vrel_x_Radar = 0.9f;
static float				w_Width_Radar = 0.1f;
//Association threshold for matching fusion objects
static float				dist_max = 2.0f;
//Kalman prediction and update weights
static float				w_t_Radar = 0.5f;
static float				w_Width_Hist = 0.5f;
static float				w_Length_Hist = 0.5f;
static float				w_RCS_Hist = 0.5f;
//Kalman filter parameter gains
static float				Q_Gain = 1.0f;
static float				R_Gain = 1.0f;
//ProbOfExist thresholds, values and factors
static float				MaxProbOfExist_RadarObject = 0.6f;
static float				MaxProbOfExist_MobileyeObject = 0.8f;
static float				MaxProbOfExist_FusionObject = 1.0f;
static float				NewObjectPenalty_ProbOfExist = 0.5f;
static float				SendThreshold_ProbOfExist = 0.5f;
static float				DeleteThreshold_ProbOfExist = 0.1f;
static float				NoMatchUpdate_ProbOfExist = 0.5f;

// Vaiable from YY ////////////////////////////////////////////////////////////

static Pufferspeicher       Point_Buffer;
static Vehicledynamik       Fahrdynamik;
static float                cur_ego[41];
static Eigen::ArrayXXf      width_coefficient;
static Eigen::ArrayXXf      d_feld(1, 41);
static Eigen::ArrayXXf      x_feld(17, 41);
static Eigen::ArrayXXf      y_feld(17, 41);
static Eigen::ArrayXXf      SPW_feld(1, 16);
static AimObject            Aim_Info;
static std::mutex           Fahrdynamik_wr_mutex;

// Tunable parameter for Aimselect
static int					removeObjekt_threshold_1_min_Points = 5;
static float				removeObjekt_threshold_1_max_Time = 2;
static float				removeObjekt_threshold_2_max_Time = 5;

static float                Threshold_add_new_point = 0.1;

static float				ridge_regression_lameda = 3;
static float				curve_fit_threshold_max_cur = 0.002;

static float                weight_a_Str_von_Aim = 0.7;

static float				SPW_no_detection = -0.5;

static float				Threashold_Aim_min_SPW = 0.2;
static float				Threashold_No_Aim_min_SPW = 0.4;

//////////////////////////////////////////////////////////////////////////
// function prototypes
//////////////////////////////////////////////////////////////////////////
void	parseConfigFile();
UINT16	convertNofObj(int value);
UINT16	convertAzimuth(float value);
UINT16	convertProbExists(int value);
UINT16	convertX(float value);
UINT16	convertY(float value);

int		InitUDPSocket();

HRESULT SelectDevice();
HRESULT InitSocket(IBalObject* pBalObject, ICanControl** ppCanControl, ICanChannel** ppCanChn, HANDLE* phEventReader, PFIFOREADER* ppReader, PFIFOWRITER* ppWriter);

void    ReceiveThread_Radar(void* Param);
void	ProcessRadarMessage(PCANMSG pCanMsg);
void	SendRadarConfigMessages();
void	SendRadarConfig();
void	SendRadarFilterConfig(BOOLEAN FilterCfg_Valid, BOOLEAN FilterCfg_Active, UINT8 FilterCfg_Index, BOOLEAN FilterCfg_Type, UINT16 FilterCfg_Min, UINT16 FilterCfg_Max);
void	copyRadarObjectList();

void	ReceiveThread_Mobileye(void* Param);
void	ProcessMobileyeMessage(PCANMSG pCanMsg);
void	copyMobileyeObjectList();

UINT8	convertBytesUINT8(UINT8* msgData, int startBit, int len);
UINT16	convertBytesUINT16(UINT8* msgData, int startBit, int len, int endian);
short	convertBytesINT16(UINT8* msgData, int startBit, int len);

void	SensorFusionThread(void* Param);
float	deg2rad(float deg);
void	RadarFusionThread(void* Param);
void	fuseRadarObjects(UINT8 i);
void	createMeasurementList();
void	createFusionObject(UINT8 mobID, UINT8 radID);
UINT8	convertRadClass(UINT8 RadClass);
void	copyMobileyeObjectToMeasurementList(UINT8 mobID);
void	copyRadarObjectToMeasurementList(UINT8 radID);
void	updateFusionList();
void	kalman_update(UINT8 fusID, UINT8 measID);
bool	compareRelevance(const FusionObject& first, const FusionObject& second);
void    send_UDP_MeasurementList();
void	send_UDP_MobileyeList();
void	send_UDP_RadarList();
void    send_UDP_FusionList();
void	send_UDP_ObjectList();

// Function from YY //////////////////////////////////////////////////////
void    Aim_Select_init();
int     Aim_Select();
void    Coordiante_Transmission();
void    Remove_expired();
void    Add_new();
void    Fit_curve();
void    SPW_distribution();
void    PLA_update();
float   interp1(Eigen::ArrayXXf x, Eigen::ArrayXXf y, float x_0);
void    Select_with_PLA();
void    Renew_Aim_Info(Objecthistory* pAim_Object_History);
void    ReceiveThread_MAB(void* Param);
void    Show_the_Frame(cv::Mat frame);
void    Show_the_Topview(cv::Mat& frame_TV,cv::Mat& frame_ObjectInfo);
cv::Point trans_TV(float x, float y);
void    weighted_ego_curve(float cur_aim, float cur_nonaim, float count_nonaim, float cur_vehicle, float v);
void    curve_Interpolation(float cur_road, float cur_vehicle);

//////////////////////////////////////////////////////////////////////////
/**
  Main entry point of the application.
*/
//////////////////////////////////////////////////////////////////////////
void main() {

	long UDPError;

	//parse config file
	parseConfigFile();

	//Initialize UDP
	UDPError = InitUDPSocket();

	if (UDPError == 0) {

		HRESULT hResult = 0;

		//Select Adapter
		hResult = SelectDevice();

		if (VCI_OK == hResult) {

			printf("CAN adapter selected.\n");

			// Initialize CAN 1
			hResult = InitSocket(pBalObject_Radar, &pCanControl_Radar, &pCanChn_Radar, &hEventReader_Radar, &pReader_Radar, &pWriter_Radar);

			if (VCI_OK == hResult) {

				printf("CAN 1 connected at 500kBaud/s.\n");

				// Initialize CAN 2
				hResult = InitSocket(pBalObject_Mobileye, &pCanControl_Mobileye, &pCanChn_Mobileye, &hEventReader_Mobileye, &pReader_Mobileye, &pWriter_Mobileye);
				
				if (VCI_OK == hResult) {

					printf("CAN 2 connected at 500kBaud/s.\n");

					//Initialize Multithreading events
					hEventListReady[0] = CreateEvent(NULL, FALSE, FALSE, NULL);
					hEventListReady[1] = CreateEvent(NULL, FALSE, FALSE, NULL);
					hEventRadarFusionReady = CreateEvent(NULL, TRUE, FALSE, NULL);
					
					//Initialize Mutexes
					hMutex[0] = CreateMutex(NULL, FALSE, NULL);
					hMutex[1] = CreateMutex(NULL, FALSE, NULL);
					
					//start time measurement
					QueryPerformanceCounter((LARGE_INTEGER*)&t_program_start);
					memcpy(&t_program_start_Rad, &t_program_start, sizeof(t_program_start));
					memcpy(&t_program_start_Mob, &t_program_start, sizeof(t_program_start));
					QueryPerformanceFrequency((LARGE_INTEGER*)&system_frequency);
					memcpy(&system_frequency_Rad, &system_frequency, sizeof(system_frequency));
					memcpy(&system_frequency_Mob, &system_frequency, sizeof(system_frequency));

					//
					// start the receive threads
					//
					CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)ReceiveThread_Radar, 0, NULL, &dwRadarReceiveThread);
					CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)ReceiveThread_Mobileye, 0, NULL, &dwMobileyeReceiveThread);
					CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)SensorFusionThread, 0, NULL, &dwSensorFusionThread);
					CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)ReceiveThread_MAB, 0, NULL, &dwMABReceiveThread);

					while (1)
					{
						Sleep(1);
					}
				}
			}
		}
	}
}

//////////////////////////////////////////////////////////////////////////
/**

Parses Config file

*/
//////////////////////////////////////////////////////////////////////////
void parseConfigFile() {
	std::ifstream is_File("config.txt");
	std::string line, key, value;
	
	printf("Parsing config file...\n");
	if (is_File.is_open()) {
		while (std::getline(is_File, line)) {
			std::istringstream is_line(line);
			if (std::getline(is_line, key, '=')) {
				if (std::getline(is_line, value)) {
					if (key == "Radar_FilterCfg_Min_NofObj")			Radar_FilterCfg_Min_NofObj = convertNofObj(stoi(value));
					else if (key == "Radar_FilterCfg_Max_NofObj")		Radar_FilterCfg_Max_NofObj = convertNofObj(stoi(value));
					else if (key == "Radar_FilterCfg_Min_Azimuth")		Radar_FilterCfg_Min_Azimuth = convertAzimuth(stof(value));
					else if (key == "Radar_FilterCfg_Max_Azimuth")		Radar_FilterCfg_Max_Azimuth = convertAzimuth(stof(value));
					else if (key == "Radar_FilterCfg_Min_ProbExists")	Radar_FilterCfg_Min_ProbExists = convertProbExists(stoi(value));
					else if (key == "Radar_FilterCfg_Max_ProbExists")	Radar_FilterCfg_Max_ProbExists = convertProbExists(stoi(value));
					else if (key == "Radar_FilterCfg_Min_Y")			Radar_FilterCfg_Min_Y = convertY(stof(value));
					else if (key == "Radar_FilterCfg_Max_Y")			Radar_FilterCfg_Max_Y = convertY(stof(value));
					else if (key == "Radar_FilterCfg_Min_X")			Radar_FilterCfg_Min_X = convertX(stof(value));
					else if (key == "Radar_FilterCfg_Max_X")			Radar_FilterCfg_Max_X = convertX(stof(value));
					else if (key == "p_y_relzone_Radar")				p_y_relzone_Radar = stof(value);
					else if (key == "gamma_offset")						gamma_offset = stof(value);
					else if (key == "c_x_offset")						c_x_offset = stof(value);
					else if (key == "p_x_offset")						p_x_offset = stof(value);
					else if (key == "p_y_offset")						p_y_offset = stof(value);
					else if (key == "Delta_p_x_max_abs")				Delta_p_x_max_abs = stof(value);
					else if (key == "Delta_p_x_max_percent")			Delta_p_x_max_percent = stof(value);
					else if (key == "Delta_p_x_min")					Delta_p_x_min = stof(value);
					else if (key == "Delta_p_y_max")					Delta_p_y_max = stof(value);
					else if (key == "dist_x_max_Radar")					dist_x_max_Radar = stof(value);
					else if (key == "Delta_p_x_min_Radar")				Delta_p_x_min_Radar = stof(value);
					else if (key == "Delta_p_x_max_Radar")				Delta_p_x_max_Radar = stof(value);
					else if (key == "dist_max")							dist_max = stof(value);
					else if (key == "w_t_Radar")						w_t_Radar = stof(value);
					else if (key == "w_p_x_Radar")						w_p_x_Radar = stof(value);
					else if (key == "w_p_y_Radar")						w_p_y_Radar = stof(value);
					else if (key == "w_vrel_x_Radar")					w_vrel_x_Radar = stof(value);
					else if (key == "w_Width_Radar")					w_Width_Radar = stof(value);
					else if (key == "Q_Gain")							Q_Gain = stof(value);
					else if (key == "R_Gain")							R_Gain = stof(value);
					else if (key == "MaxProbOfExist_RadarObject")		MaxProbOfExist_RadarObject = stof(value);
					else if (key == "MaxProbOfExist_MobileyeObject")	MaxProbOfExist_MobileyeObject = stof(value);
					else if (key == "MaxProbOfExist_FusionObject")		MaxProbOfExist_FusionObject = stof(value);
					else if (key == "NewObjectPenalty_ProbOfExist")		NewObjectPenalty_ProbOfExist = stof(value);
					else if (key == "SendThreshold_ProbOfExist")		SendThreshold_ProbOfExist = stof(value);
					else if (key == "DeleteThreshold_ProbOfExist")		DeleteThreshold_ProbOfExist = stof(value);
					else if (key == "NoMatchUpdate_ProbOfExist")		NoMatchUpdate_ProbOfExist = stof(value);
					else if (key == "w_Width_Hist")						w_Width_Hist = stof(value);
					else if (key == "w_Length_Hist")					w_Length_Hist = stof(value);
					else if (key == "w_RCS_Hist")						w_RCS_Hist = stof(value);
					else if (key == "FusionEventWaitTime")				FusionEventWaitTime = stoi(value);
					else if (key == "sendFusionList")					sendFusionList = stoi(value);
					else if (key == "sendRadarList")					sendRadarList = stoi(value);
					else if (key == "sendMeasurementList")				sendMeasurementList = stoi(value);
					else if (key == "sendMobileyeList")					sendMobileyeList = stoi(value);
					else if (key == "sendToMAB")						sendToMAB = stoi(value);
					else if (key == "sendToPC")							sendToPC = stoi(value);
					else if (key == "simulationMode")					simulationMode = stoi(value);
					else if (key == "removeObjekt_threshold_1_min_Points")			removeObjekt_threshold_1_min_Points = stof(value);
					else if (key == "removeObjekt_threshold_1_max_Time")			removeObjekt_threshold_1_max_Time = stof(value);
					else if (key == "removeObjekt_threshold_2_max_Time ")			removeObjekt_threshold_2_max_Time = stof(value);
					else if (key == "Threshold_add_new_point")			Threshold_add_new_point = stof(value);
					else if (key == "ridge_regression_lameda")			ridge_regression_lameda = stof(value);
					else if (key == "curve_fit_threshold_max_cur")		curve_fit_threshold_max_cur = stof(value);
					else if (key == "weight_a_Str_von_Aim")				weight_a_Str_von_Aim = stof(value);
					else if (key == "SPW_no_detection")					SPW_no_detection = stof(value);
					else if (key == "Threashold_Aim_min_SPW")			Threashold_Aim_min_SPW = stof(value);
					else if (key == "Threashold_No_Aim_min_SPW")		Threashold_No_Aim_min_SPW = stof(value);
					else printf("Could not set key: \"%s\"\n", key.c_str());
				}
			}
		}
		printf("Config file parsed.\n");
	}
	else {
		printf("Config file not found.\n");
	}

}

UINT16 convertNofObj(int value) {
	UINT16 NofObj;
	if (value > 4095) NofObj = 4095;
	else if (value < 0) NofObj = 0;
	else NofObj = value;
	return NofObj;
}

UINT16 convertAzimuth(float value) {
	UINT16 Azimuth;
	if (value > 52.375f) Azimuth = 4095;
	else if (value < -50.0f) Azimuth = 0;
	else Azimuth = (UINT16) roundf((value + 50.0f) / 0.025f);
	return Azimuth;
}

UINT16 convertProbExists(int value) {
	UINT16 ProbExists;
	if (value > 7) ProbExists = 7;
	else if (value < 0) ProbExists = 0;
	else ProbExists = value;
	return ProbExists;
}

UINT16 convertX(float value) {
	UINT16 X;
	if (value > 1138.2f) X = 8191;
	else if (value < -500.0f) X = 0;
	else X = (UINT16) roundf((value + 500.0f) / 0.2f);
	return X;
}

UINT16 convertY(float value) {
	UINT16 Y;
	if (value > 409.5f) Y = 4095;
	else if (value < -409.5f) Y = 0;
	else Y = (UINT16) roundf((value + 409.5f) / 0.2f);
	return Y;
}

//////////////////////////////////////////////////////////////////////////
/**

Sets up UDP Socket

*/
//////////////////////////////////////////////////////////////////////////
int InitUDPSocket() {
	long UDPError = 0;
	WSADATA wsa;

	//Start WinSock
	UDPError = WSAStartup(MAKEWORD(2, 0), &wsa);
	if (UDPError != 0) {
		printf("error:  WSAStartup, error code; %d\n", UDPError);
		return 1;
	}
	else
	{
		printf("Winsock started.\n");

	}


	//Setup and connect Socket 1
	UDPSocket = socket(AF_INET, SOCK_DGRAM, 0);
	if (UDPSocket == INVALID_SOCKET) {
		printf("error: Der Socket konnte nicht erstellt werden, error code: %d\n", WSAGetLastError());
		return 1;
	}
	else {
		printf("UDP socket created.\n");
	}

	UDPSocket_from_MAB = socket(AF_INET, SOCK_DGRAM, 0);
	if (UDPSocket_from_MAB == INVALID_SOCKET) {
		printf("error: Der Socket konnte nicht erstellt werden, error code: %d\n", WSAGetLastError());
		return 1;
	}
	else {
		printf("UDP socket created.\n");
	}
	
	//Setup and connect Socket 1
	UDPSocket_AimInfo = socket(AF_INET, SOCK_DGRAM, 0);
	if (UDPSocket == INVALID_SOCKET) {
		printf("error: Der Socket konnte nicht erstellt werden, error code: %d\n", WSAGetLastError());
		return 1;
	}
	else {
		printf("UDP socket created.\n");
	}
	//prepare MAB Ports
	MABAddr_Port1401.sin_family = AF_INET;
	MABAddr_Port1401.sin_port = htons(1401);
	MABAddr_Port1401.sin_addr.s_addr = inet_addr("192.168.140.4");

	PCAddr_Port1401.sin_family = AF_INET;
	PCAddr_Port1401.sin_port = htons(1401);
	PCAddr_Port1401.sin_addr.s_addr = inet_addr("192.168.140.2");

	EBPCAddr_Port1402.sin_family = AF_INET;
	EBPCAddr_Port1402.sin_port = htons(1402);
	EBPCAddr_Port1402.sin_addr.s_addr = inet_addr("192.168.140.3");

	PCAddr_Port1403.sin_family = AF_INET;
	PCAddr_Port1403.sin_port = htons(1403);
	PCAddr_Port1403.sin_addr.s_addr = inet_addr("192.168.140.2");

	//Bind the UDPSocket_from_MAB with local portal
	if (bind(UDPSocket_from_MAB, (struct sockaddr*) & EBPCAddr_Port1402, sizeof(EBPCAddr_Port1402)) < 0)
	{
		printf("error: Der Vernimdung mit Portal 1402 im MAB konnte nicht aufgebaut werden. errorcode: %d\n", WSAGetLastError());
		return 1;
	}
	else {
		printf("UDP socket UDPSocket_from_MAB connected.\n");
	}

	return 0;
}

//////////////////////////////////////////////////////////////////////////
/**
  Selects the first CAN adapter.

  @param fUserSelect
	If this parameter is set to TRUE the functions display a dialog box which
	allows the user to select the device.

  @return
	VCI_OK on success, otherwise an Error code
*/
//////////////////////////////////////////////////////////////////////////
HRESULT SelectDevice()
{
	HRESULT hResult; // error code

	IVciDeviceManager*  pDevMgr = 0;    // device manager
	IVciEnumDevice*     pEnum = 0;    // enumerator handle
	VCIDEVICEINFO       sInfo, sInfo2;          // device info
	IVciDevice*			pDevice;

	hResult = VciGetDeviceManager(&pDevMgr);
	if (hResult == VCI_OK)
	{
		do {
			hResult = pDevMgr->EnumDevices(&pEnum);

			//
			// retrieve information about the first two
			// devices within the device list
			//
			if (hResult == VCI_OK)
			{

				hResult = pEnum->Next(1, &sInfo, NULL);
				hResult = pEnum->Next(1, &sInfo2, NULL); 

				//
				// close the device list (no longer needed)
				//
				pEnum->Release();
				pEnum = NULL;
			}
		} while (hResult != VCI_OK);
	}

	//
	// open the first device via device manager and get the bal object
	//
	if (hResult == VCI_OK)
	{
		hResult = pDevMgr->OpenDevice(sInfo.VciObjectId, &pDevice);

		if (hResult == VCI_OK)
		{

			if (strcmp("USB-to-CAN compact", sInfo.Description) == 0) {
				hResult = pDevice->OpenComponent(CLSID_VCIBAL, IID_IBalObject, (void**)&pBalObject_Radar);
			} 
			else if (strcmp("USB-to-CAN V2 compact", sInfo.Description) == 0) {
				hResult = pDevice->OpenComponent(CLSID_VCIBAL, IID_IBalObject, (void**)&pBalObject_Mobileye);
			}

			pDevice->Release();
		}
	}

	//
	// open the second device via device manager and get the bal object
	//
	if (hResult == VCI_OK)
	{
		hResult = pDevMgr->OpenDevice(sInfo2.VciObjectId, &pDevice);

		if (hResult == VCI_OK)
		{
			if (strcmp("USB-to-CAN compact", sInfo2.Description) == 0) {
				hResult = pDevice->OpenComponent(CLSID_VCIBAL, IID_IBalObject, (void**)&pBalObject_Radar);
			}
			else if (strcmp("USB-to-CAN V2 compact", sInfo2.Description) == 0) {
				hResult = pDevice->OpenComponent(CLSID_VCIBAL, IID_IBalObject, (void**)&pBalObject_Mobileye);
			}

			pDevice->Release();
		}
	}

	//
	// close device manager
	//
	if (pDevMgr)
	{
		pDevMgr->Release();
		pDevMgr = NULL;
	}

	DisplayError(NULL, hResult);
	return hResult;
}


//////////////////////////////////////////////////////////////////////////
/**
  Opens the specified socket, creates a message channel, initializes
  and starts the CAN controller.

  @param dwCanNo
	Number of the CAN controller to open.

  @return
	VCI_OK on success, otherwise an Error code

  @note
	If <dwCanNo> is set to 0xFFFFFFFF, the function shows a dialog box
	which allows the user to select the VCI device and CAN controller.
*/
//////////////////////////////////////////////////////////////////////////
HRESULT InitSocket(IBalObject* pBalObject, ICanControl** ppCanControl, ICanChannel** ppCanChn, HANDLE* phEventReader, PFIFOREADER* ppReader, PFIFOWRITER* ppWriter)
{
	HRESULT hResult = E_FAIL;

	if (pBalObject != NULL)
	{
		//
		// check controller capabilities create a message channel
		//
		LONG lCtrlNo = 0;
		ICanSocket* pCanSocket = 0;
		ICanControl* pCanControl;
		ICanChannel* pCanChn;
		PFIFOREADER pReader;
		PFIFOWRITER pWriter;
		hResult = pBalObject->OpenSocket(lCtrlNo, IID_ICanSocket, (void**)&pCanSocket);
		if (hResult == VCI_OK)
		{
			// check capabilities
			CANCAPABILITIES capabilities = { 0 };
			hResult = pCanSocket->GetCapabilities(&capabilities);
			if (VCI_OK == hResult)
			{
				//
				// This sample expects that standard and extended mode are
				// supported simultaneously. See use of
				// CAN_OPMODE_STANDARD | CAN_OPMODE_EXTENDED in InitLine() below
				//
				if (capabilities.dwFeatures & CAN_FEATURE_STDANDEXT)
				{
					// supports simultaneous standard and extended -> ok
				}
				else
				{
					printf("Simultaneous standard and extended mode feature not supported !\n");
					hResult = VCI_E_NOT_SUPPORTED;
				}
			}
			else
			{
				// should not occurr
				printf(" pCanSocket->GetCapabilities failed: 0x%08lX !\n", hResult);
			}

			//
			// create a message channel
			//
			if (VCI_OK == hResult)
			{
				hResult = pCanSocket->CreateChannel(FALSE, &pCanChn);
			}

			pCanSocket->Release();
		}

		//
		// initialize the message channel
		//
		if (hResult == VCI_OK)
		{
			UINT16 wRxFifoSize = 1024;
			UINT16 wRxThreshold = 1;
			UINT16 wTxFifoSize = 1;
			UINT16 wTxThreshold = 1;

			hResult = pCanChn->Initialize(wRxFifoSize, wTxFifoSize);
			if (hResult == VCI_OK)
			{
				hResult = pCanChn->GetReader(&pReader);
				if (hResult == VCI_OK)
				{
					pReader->SetThreshold(wRxThreshold);

					*phEventReader = CreateEvent(NULL, FALSE, FALSE, NULL);
					pReader->AssignEvent(*phEventReader);
				}
			}

			if (hResult == VCI_OK)
			{
				hResult = pCanChn->GetWriter(&pWriter);
				if (hResult == VCI_OK)
				{
					pWriter->SetThreshold(wTxThreshold);
				}
			}
		}

		//
		// activate the CAN channel
		//
		if (hResult == VCI_OK)
		{
			hResult = pCanChn->Activate();
		}

		//
		// Open the CAN control interface
		//
		// During the programs lifetime we have multiple options:
		// 1) Open the control interface and keep it open
		//     -> No other programm is able to get the control interface and change the line settings
		// 2) Try to get the control interface and change the settings only when we get it
		//     -> Other programs can change the settings by getting the control interface
		//
		if (hResult == VCI_OK)
		{
			hResult = pBalObject->OpenSocket(lCtrlNo, IID_ICanControl, (void**)&pCanControl);

			//
			// initialize the CAN controller
			//
			if (hResult == VCI_OK)
			{
				CANINITLINE init = {
				  CAN_OPMODE_STANDARD |
				  CAN_OPMODE_EXTENDED | CAN_OPMODE_ERRFRAME,      // opmode
				  0,                                              // bReserved
				  CAN_BT0_500KB, CAN_BT1_500KB                    // bt0, bt1
				};

				hResult = pCanControl->InitLine(&init);
				if (hResult != VCI_OK)
				{
					printf(" pCanControl->InitLine failed: 0x%08lX !\n", hResult);
				}

				//
				// set the acceptance filter
				//
				if (hResult == VCI_OK)
				{
					hResult = pCanControl->SetAccFilter(CAN_FILTER_STD, CAN_ACC_CODE_ALL, CAN_ACC_MASK_ALL);

					//
					// set the acceptance filter
					//
					if (hResult == VCI_OK)
					{
						hResult = pCanControl->SetAccFilter(CAN_FILTER_EXT, CAN_ACC_CODE_ALL, CAN_ACC_MASK_ALL);
					}

					if (VCI_OK != hResult)
					{
						printf(" pCanControl->SetAccFilter failed: 0x%08lX !\n", hResult);
					}

					//
					// SetAccFilter() returns VCI_E_INVALID_STATE if already controller is started. 
					// We ignore this because the controller could already be started
					// by another application.
					//
					if (VCI_E_INVALID_STATE == hResult)
					{
						hResult = VCI_OK;
					}
				}

				//
				// start the CAN controller
				//
				if (hResult == VCI_OK)
				{
					hResult = pCanControl->StartLine();
					if (hResult == VCI_OK)
					{
						*ppCanControl = pCanControl;
						*ppCanChn = pCanChn;
						*ppReader = pReader;
						*ppWriter = pWriter;
					}
					else 
					{
						printf(" pCanControl->StartLine failed: 0x%08lX !\n", hResult);
					}
				}

				printf(" Got Control interface. Settings applied !\n");
			}
			else
			{
				//
				// If we can't get the control interface it is occupied by another application.
				// This means the application is in charge of the controller parameters.
				// We live with it and move on.
				// 
				printf(" Control interface occupied. Settings not applied: 0x%08lX !\n", hResult);
				hResult = S_OK;
			}
		}
	}
	else
	{
		hResult = VCI_E_INVHANDLE;
	}

	DisplayError(NULL, hResult);
	return hResult;
}

//////////////////////////////////////////////////////////////////////////
/**
  Receive thread (Radar).

  @param Param
	ptr on a user defined information
*/
//////////////////////////////////////////////////////////////////////////
void ReceiveThread_Radar(void* Param)
{
	UNREFERENCED_PARAMETER(Param);

	BOOL receiveSignaled = FALSE;
	BOOL moreMsgAvail = FALSE;

	PCANMSG pCanMsg;
	UINT16  wCount = 0;
	UINT16 iter;

	while (true)
	{
		// if no more messages available wait 100msec for reader event
		if (!moreMsgAvail)
		{
			receiveSignaled = (WAIT_OBJECT_0 == WaitForSingleObject(hEventReader_Radar, 100));
		}

		// process messages while messages are available
		if (receiveSignaled || moreMsgAvail)
		{
			moreMsgAvail = S_FALSE;

			if (pReader_Radar->AcquireRead((PVOID*)&pCanMsg, &wCount) == VCI_OK)
			{
				iter = wCount;
				while (iter)
				{
					ProcessRadarMessage(pCanMsg);
					iter--;
					pCanMsg++;
				}
				pReader_Radar->ReleaseRead(wCount);

				moreMsgAvail = S_OK;
			}
		}
	}
}

//////////////////////////////////////////////////////////////////////////
/**

  Process a Radar message

*/
//////////////////////////////////////////////////////////////////////////
void ProcessRadarMessage(PCANMSG pCanMsg)
{
	if (pCanMsg->uMsgInfo.Bytes.bType == CAN_MSGTYPE_DATA)
	{
		//show data frames
		if (pCanMsg->uMsgInfo.Bits.rtr == 0)
		{
			if (RadarFiltersSet)
			{
				switch (pCanMsg->dwMsgId)
				{
				case 0x300:
					if (SpeedWritten == false) {
						SpeedWritten = true;
						rList_Reception.speed = 0.02f* convertBytesUINT16(pCanMsg->abData, 8, 13, -1);
					}
					break;
				case 0x60A: //Save number of Objects in current cycle
					QueryPerformanceCounter((LARGE_INTEGER*)&t_cycle_Rad);
					rList_Reception.n = pCanMsg->abData[0];
					rList_Reception.timestamp = (t_cycle_Rad - t_program_start_Rad)*1.0f / system_frequency_Rad;
					MsgCounter60B = 0;
					MsgCounter60C = 0;
					MsgCounter60D = 0;
					SpeedWritten = false;
					if (0 == rList_Reception.n) copyRadarObjectList();
					break;
				case 0x60B:
					rList_Reception.Objects[MsgCounter60B].ID = convertBytesUINT8(pCanMsg->abData, 0, 8);
					rList_Reception.Objects[MsgCounter60B].x(0) = 0.2f * convertBytesUINT16(pCanMsg->abData, 19, 13, -1) - 500.0f;
					rList_Reception.Objects[MsgCounter60B].x(1) = 0.2f * convertBytesUINT16(pCanMsg->abData, 24, 11, -1) - 204.6f;
					rList_Reception.Objects[MsgCounter60B].x(2) = 0.25f * convertBytesUINT16(pCanMsg->abData, 46, 10, -1) - 128.0f;
					rList_Reception.Objects[MsgCounter60B].x(3) = 0.25f * convertBytesUINT16(pCanMsg->abData, 53, 9, -1) - 64.0f;
					rList_Reception.Objects[MsgCounter60B].RCS = 0.5f * convertBytesUINT8(pCanMsg->abData, 56, 8) - 64.0f;
					MsgCounter60B++;
					break;
				case 0x60C:
					rList_Reception.Objects[MsgCounter60C].ProbOfExist = convertBytesUINT8(pCanMsg->abData, 53, 3);
					MsgCounter60C++;
					break;
				case 0x60D:
					rList_Reception.Objects[MsgCounter60D].Class = convertBytesUINT8(pCanMsg->abData, 24, 3);
					//rList_Reception.Objects[MsgCounter60D].ArelLongX = 0.01f * convertBytesUINT16(pCanMsg->abData, 21, 11, -1) - 10.0f;
					rList_Reception.Objects[MsgCounter60D].Length = 0.2f * convertBytesUINT8(pCanMsg->abData, 48, 8);
					rList_Reception.Objects[MsgCounter60D].Width = 0.2f * convertBytesUINT8(pCanMsg->abData, 56, 8);
					MsgCounter60D++;
					if (MsgCounter60D >= rList_Reception.n) copyRadarObjectList(); //if all messages of current cycle have been received
					break;
				}
			}
			else {
				switch (pCanMsg->dwMsgId)
				{
				case 0x201:
					SendRadarConfigMessages();
					printf("Radar Config Sent.\n");
					if (simulationMode == 1) RadarFiltersSet = true;
					break;
				case 0x204:
					UINT16 iMin, iMax;
					float fMin, fMax;
					switch (convertBytesUINT8(pCanMsg->abData, 3, 4))
					{
					case 0:
						RadarFilterSet_NofObj = true;
						iMin = convertBytesUINT16(pCanMsg->abData, 16, 12, -1);
						iMax = convertBytesUINT16(pCanMsg->abData, 32, 12, -1);
						printf("NofObj Filter Set. Min: %d Max: %d\n", iMin, iMax);
						break;
					case 2:
						RadarFilterSet_Azimuth = true;
						fMin = 0.025f * convertBytesUINT16(pCanMsg->abData, 16, 12, -1) - 50.0f;
						fMax = 0.025f * convertBytesUINT16(pCanMsg->abData, 32, 12, -1) - 50.0f;
						printf("Azimuth Filter Set. Min: %f Max: %f\n", fMin, fMax);
						break;
					case 8:
						RadarFilterSet_Prob = true;
						iMin = convertBytesUINT16(pCanMsg->abData, 16, 12, -1);
						iMax = convertBytesUINT16(pCanMsg->abData, 32, 12, -1);
						printf("ProbOfExist Filter Set. Min: %d Max: %d\n", iMin, iMax);
						break;
					case 9:
						RadarFilterSet_Y = true;
						fMin = 0.2f * convertBytesUINT16(pCanMsg->abData, 16, 12, -1) - 409.5f;
						fMax = 0.2f * convertBytesUINT16(pCanMsg->abData, 32, 12, -1) - 409.5f;
						printf("Y Filter Set. Min: %f Max: %f\n", fMin, fMax);
						break;
					case 10:
						RadarFilterSet_X = true;
						fMin = 0.2f * convertBytesUINT16(pCanMsg->abData, 16, 13, -1) - 500.0f;
						fMax = 0.2f * convertBytesUINT16(pCanMsg->abData, 32, 13, -1) - 500.0f;
						printf("X Filter Set. Min: %f Max: %f\n", fMin, fMax);
						break;
					}
					if (RadarFilterSet_NofObj && RadarFilterSet_Azimuth && RadarFilterSet_Prob && RadarFilterSet_Y && RadarFilterSet_X) {
						RadarFiltersSet = true;
					}
					break;
				}
			}
		}
		else
		{
			printf("Time: %10u ID: %3X  DLC: %1u  Remote Frame\n",
				pCanMsg->dwTime,
				pCanMsg->dwMsgId,
				pCanMsg->uMsgInfo.Bits.dlc);
		}
	}
	else if (pCanMsg->uMsgInfo.Bytes.bType == CAN_MSGTYPE_INFO)
	{
		//
		// show informational frames
		//
		switch (pCanMsg->abData[0])
		{
		case CAN_INFO_START: printf("CAN started...\n"); break;
		case CAN_INFO_STOP: printf("CAN stopped...\n"); break;
		case CAN_INFO_RESET: printf("CAN reseted...\n"); break;
		}
	}
	else if (pCanMsg->uMsgInfo.Bytes.bType == CAN_MSGTYPE_ERROR)
	{
		//
		// show error frames
		//
		switch (pCanMsg->abData[0])
		{
		case CAN_ERROR_STUFF: printf("stuff error...\n");          break;
		case CAN_ERROR_FORM: printf("form error...\n");           break;
		case CAN_ERROR_ACK: printf("acknowledgment error...\n"); break;
		case CAN_ERROR_BIT: printf("bit error...\n");            break;
		case CAN_ERROR_CRC: printf("CRC error...\n");            break;
		case CAN_ERROR_OTHER:
		default: /*printf("other error...");   */       break;
		}
	}
}

void SendRadarConfigMessages()
{
	SendRadarConfig();
	Sleep(10);
	SendRadarFilterConfig(1, 1, 0x0, 1, Radar_FilterCfg_Min_NofObj, Radar_FilterCfg_Max_NofObj);
	Sleep(10);
	SendRadarFilterConfig(1, 1, 0x2, 1, Radar_FilterCfg_Min_Azimuth, Radar_FilterCfg_Max_Azimuth);
	Sleep(10);
	SendRadarFilterConfig(1, 1, 0x8, 1, Radar_FilterCfg_Min_ProbExists, Radar_FilterCfg_Max_ProbExists);
	Sleep(10);
	SendRadarFilterConfig(1, 1, 0x9, 1, Radar_FilterCfg_Min_Y, Radar_FilterCfg_Max_Y);
	Sleep(10);
	SendRadarFilterConfig(1, 1, 0xA, 1, Radar_FilterCfg_Min_X, Radar_FilterCfg_Max_X);
}

void SendRadarConfig()
{
	UINT16  count = 0;
	PCANMSG pMsg;

	// length of message payload
	UINT payloadLen = 8;

	// aquire write access to FIFO
	HRESULT hr = pWriter_Radar->AcquireWrite((void**)&pMsg, &count);
	if (VCI_OK == hr)
	{
		// number of written messages needed for ReleaseWrite
		UINT16 written = 0;

		if (count > 0)
		{
			pMsg->dwTime = 0;
			pMsg->dwMsgId = 0x200;

			pMsg->uMsgInfo.Bytes.bType = CAN_MSGTYPE_DATA;
			// Flags:
			// srr = 1
			pMsg->uMsgInfo.Bytes.bFlags = CAN_MAKE_MSGFLAGS(CAN_LEN_TO_SDLC(payloadLen), 0, 1, 0, 0);
			// Flags2:
			// Set bFlags2 to 0 because FIFO memory will not be initialized by AquireWrite
			pMsg->uMsgInfo.Bytes.bFlags2 = CAN_MAKE_MSGFLAGS2(0, 0, 0, 0, 0);

			UINT8 RadarCfg_MaxDistance_valid = 0;
			UINT8 RadarCfg_SensorID_valid = 0;
			UINT8 RadarCfg_RadarPower_valid = 0;
			UINT8 RadarCfg_OutputType_valid = 1;
			UINT8 RadarCfg_SendQuality_valid = 1;
			UINT8 RadarCfg_SendExtInfo_valid = 1;
			UINT8 RadarCfg_SortIndex_valid = 1;
			UINT8 RadarCfg_StoreInNVM_valid = 0;
			UINT8 RadarCfg_MaxDistance = 0;
			UINT8 RadarCfg_SensorID = 0;
			UINT8 RadarCfg_OutputType = 1;
			UINT8 RadarCfg_RadarPower = 0;
			UINT8 RadarCfg_CtrlRelay_valid = 0;
			UINT8 RadarCfg_CtrlRelay = 0;
			UINT8 RadarCfg_SendQuality = 1;
			UINT8 RadarCfg_SendExtInfo = 1;
			UINT8 RadarCfg_SortIndex = 1;
			UINT8 RadarCfg_StoreInNVM = 0;
			UINT8 RadarCfg_RCS_Threshold_valid = 0;
			UINT8 RadarCfg_RCS_Threshold = 0;

			pMsg->abData[0] = 
				RadarCfg_MaxDistance_valid +
				(RadarCfg_SensorID_valid << 1) +
				(RadarCfg_RadarPower_valid << 2) +
				(RadarCfg_OutputType_valid << 3) +
				(RadarCfg_SendQuality_valid << 4) +
				(RadarCfg_SendExtInfo_valid << 5) +
				(RadarCfg_SortIndex_valid << 6) +
				(RadarCfg_StoreInNVM_valid << 7);
			pMsg->abData[1] = RadarCfg_MaxDistance >> 2;
			pMsg->abData[2] = RadarCfg_MaxDistance << 6;
			pMsg->abData[3] = 0;
			pMsg->abData[4] =
				RadarCfg_SensorID +
				(RadarCfg_OutputType << 3) +
				(RadarCfg_RadarPower << 5);
			pMsg->abData[5] =
				RadarCfg_CtrlRelay_valid +
				(RadarCfg_CtrlRelay << 1) +
				(RadarCfg_SendQuality << 2) +
				(RadarCfg_SendExtInfo << 3) +
				(RadarCfg_SortIndex << 4) +
				(RadarCfg_StoreInNVM << 7);
			pMsg->abData[6] =
				RadarCfg_RCS_Threshold_valid +
				(RadarCfg_RCS_Threshold << 1);
			pMsg->abData[7] = 0;


			written = 1;
		}

		// release write access to FIFO
		hr = pWriter_Radar->ReleaseWrite(written);
		if (VCI_OK != hr)
		{
			printf("ReleaseWrite failed: 0x%08lX\n", hr);
		}
	}
	else
	{
		printf("AcquireWrite failed: 0x%08lX\n", hr);
	}
}

//////////////////////////////////////////////////////////////////////////
/**

	Sends a Filter Configuration Message to the Radar
	@param FilterCfg_Valid		0 - Invalid, 1 - Valid
	@param FilterCfg_Active		0 - Inactive, 1 - Active
	@param FilterCfg_Index		Filter criteria specified in Radar Doc
	@param FilterCfg_Type		0 - Cluster, 1 - Object
	@param FilterCfg_Min		Depends on selected Criteria
	@param FilterCfg_Max		Depends on selected Criteria

*/
//////////////////////////////////////////////////////////////////////////
void SendRadarFilterConfig(BOOLEAN FilterCfg_Valid, BOOLEAN FilterCfg_Active, UINT8 FilterCfg_Index, BOOLEAN FilterCfg_Type, UINT16 FilterCfg_Min, UINT16 FilterCfg_Max)
{
	UINT16  count = 0;
	PCANMSG pMsg;

	// length of message payload
	UINT payloadLen = 5;

	// aquire write access to FIFO
	HRESULT hr = pWriter_Radar->AcquireWrite((void**)&pMsg, &count);
	if (VCI_OK == hr)
	{
		// number of written messages needed for ReleaseWrite
		UINT16 written = 0;

		if (count > 0)
		{
			pMsg->dwTime = 0;
			pMsg->dwMsgId = 0x202;

			pMsg->uMsgInfo.Bytes.bType = CAN_MSGTYPE_DATA;
			// Flags:
			// srr = 1
			pMsg->uMsgInfo.Bytes.bFlags = CAN_MAKE_MSGFLAGS(CAN_LEN_TO_SDLC(payloadLen), 0, 1, 0, 0);
			// Flags2:
			// Set bFlags2 to 0 because FIFO memory will not be initialized by AquireWrite
			pMsg->uMsgInfo.Bytes.bFlags2 = CAN_MAKE_MSGFLAGS2(0, 0, 0, 0, 0);

			pMsg->abData[0] =
				(FilterCfg_Valid<<1) +
				(FilterCfg_Active << 2) +
				(FilterCfg_Index << 3) +
				(FilterCfg_Type << 7);
			pMsg->abData[1] = (UINT8)(FilterCfg_Min >> 8);
			pMsg->abData[2] = (UINT8)(FilterCfg_Min);
			pMsg->abData[3] = (UINT8)(FilterCfg_Max >> 8);
			pMsg->abData[4] = (UINT8)(FilterCfg_Max);


			written = 1;
		}

		// release write access to FIFO
		hr = pWriter_Radar->ReleaseWrite(written);
		if (VCI_OK != hr)
		{
			printf("ReleaseWrite failed: 0x%08lX\n", hr);
		}
	}
	else
	{
		printf("AcquireWrite failed: 0x%08lX\n", hr);
	}
}



//////////////////////////////////////////////////////////////////////////
/**

  Safely copy new Radar object list from receive storage array to
  exchange storage array. Notify sensor fusion thread. Then reset receive
  storage array.

*/
//////////////////////////////////////////////////////////////////////////
void copyRadarObjectList()
{
	if (WAIT_OBJECT_0 == WaitForSingleObject(hMutex[0], 10)) { //request mutex access
		memcpy(&rList_Exchange, &rList_Reception, sizeof(ObjectList_Radar)); //copy reception array to mutex protected array
		ReleaseMutex(hMutex[0]);
		SetEvent(hEventListReady[0]);
	}
	memset(&rList_Reception, 0, sizeof(ObjectList_Radar)); //Reset reception array
}

//////////////////////////////////////////////////////////////////////////
/**
  Receive thread (Mobileye).

  Note:
	Here console output in the receive thread is used for demonstration purposes.
	Using console outout in the receive thread involves Asynchronous
	Local Procedure Calls (ALPC) with the console host application (conhost.exe).
	So expect console output to be slow.
	Slow output can stall receive queue handling and finally lead
	to controller overruns on some CAN interfaces, even with moderate busloads
	(moderate = 1000 kBit/s, dlc=8, busload >= 30%).

  @param Param
	ptr on a user defined information
*/
//////////////////////////////////////////////////////////////////////////
void ReceiveThread_Mobileye(void* Param)
{
	UNREFERENCED_PARAMETER(Param);

	BOOL receiveSignaled = FALSE;
	BOOL moreMsgAvail = FALSE;

	PCANMSG pCanMsg;
	UINT16  wCount = 0;
	UINT16 iter;

	while (true)
	{
		// if no more messages available wait 100msec for reader event
		if (!moreMsgAvail)
		{
			receiveSignaled = (WAIT_OBJECT_0 == WaitForSingleObject(hEventReader_Mobileye, 100));
		}

		// process messages while messages are available
		if (receiveSignaled || moreMsgAvail)
		{
			moreMsgAvail = S_FALSE;

			if (pReader_Mobileye->AcquireRead((PVOID*)&pCanMsg, &wCount) == VCI_OK)
			{
				iter = wCount;
				while (iter)
				{
					ProcessMobileyeMessage(pCanMsg);
					iter--;
					pCanMsg++;
				}
				pReader_Mobileye->ReleaseRead(wCount);

				moreMsgAvail = S_OK;
			}
		}
	}
}

//////////////////////////////////////////////////////////////////////////
/**

  Process a Mobileye message

*/
//////////////////////////////////////////////////////////////////////////
void ProcessMobileyeMessage(PCANMSG pCanMsg)
{
	if (pCanMsg->uMsgInfo.Bytes.bType == CAN_MSGTYPE_DATA)
	{
		//show data frames
		if (pCanMsg->uMsgInfo.Bits.rtr == 0)
		{
			switch (pCanMsg->dwMsgId)
			{
			case 0x738: //Save number of Objects in current cycle and trigger sensor fusion if no Objects
				QueryPerformanceCounter((LARGE_INTEGER*)&t_cycle_Mob);
				mList_Reception.n = pCanMsg->abData[0];
				mList_Reception.timestamp = (t_cycle_Mob - t_program_start_Mob)*1.0f / system_frequency_Mob;
				MsgCounterA = 0;
				MsgCounterB = 0;
				MsgCounterC = 0;
				if (0 == mList_Reception.n) copyMobileyeObjectList();
				break;
			case 0x739: //Write ObstacleDataA to storage
			case 0x73C:
			case 0x73F:
			case 0x742:
			case 0x745:
			case 0x748:
			case 0x74B:
			case 0x74E:
			case 0x751:
			case 0x754:
				mList_Reception.Objects[MsgCounterA].ID = convertBytesUINT8(pCanMsg->abData, 0, 8);
				mList_Reception.Objects[MsgCounterA].x(0) = 0.0625f * convertBytesUINT16(pCanMsg->abData, 8, 12, 1);
				mList_Reception.Objects[MsgCounterA].x(1) = 0.0625f * convertBytesINT16(pCanMsg->abData, 24, 10);
				mList_Reception.Objects[MsgCounterA].x(2) = 0.0625f * convertBytesINT16(pCanMsg->abData, 40, 12);
				mList_Reception.Objects[MsgCounterA].Class = convertBytesUINT8(pCanMsg->abData, 52, 3);
				MsgCounterA++;
				break;
			case 0x73A: //Write ObstacleDataB to storage and trigger sensor fusion after last message received
			case 0x73D:
			case 0x740:
			case 0x743:
			case 0x746:
			case 0x749:
			case 0x74C:
			case 0x74F:
			case 0x752:
			case 0x755:
				mList_Reception.Objects[MsgCounterB].Width = 0.05f * convertBytesUINT8(pCanMsg->abData, 8, 8);
				mList_Reception.Objects[MsgCounterB].Lane = convertBytesUINT8(pCanMsg->abData, 24, 2);
				MsgCounterB++;
				if (MsgCounterB >= mList_Reception.n) copyMobileyeObjectList();
				break;
				//case 0x73B: //Write ObstacleDataC to storage and trigger sensor fusion after last message received
				//case 0x73E: 
				//case 0x741: 
				//case 0x744: 
				//case 0x747: 
				//case 0x74A: 
				//case 0x74D: 
				//case 0x750: 
				//case 0x753: 
				//case 0x756:
					//mList_Reception.Objects[MsgCounterB].ArelLongX = 0.03f * convertBytesINT16(pCanMsg->abData, 32, 10);
					//MsgCounterC++;
					//if (MsgCounterC >= mList_Reception.n) copyMobileyeObjectList();
					//break;
			}
		}
		else
		{
			printf("Time: %10u ID: %3X  DLC: %1u  Remote Frame\n",
				pCanMsg->dwTime,
				pCanMsg->dwMsgId,
				pCanMsg->uMsgInfo.Bits.dlc);
		}
	}
	else if (pCanMsg->uMsgInfo.Bytes.bType == CAN_MSGTYPE_INFO)
	{
		//
		// show informational frames
		//
		switch (pCanMsg->abData[0])
		{
		case CAN_INFO_START: printf("CAN started...\n"); break;
		case CAN_INFO_STOP: printf("CAN stopped...\n"); break;
		case CAN_INFO_RESET: printf("CAN reseted...\n"); break;
		}
	}
	else if (pCanMsg->uMsgInfo.Bytes.bType == CAN_MSGTYPE_ERROR)
	{
		//
		// show error frames
		//
		switch (pCanMsg->abData[0])
		{
		case CAN_ERROR_STUFF: printf("stuff error...\n");          break;
		case CAN_ERROR_FORM: printf("form error...\n");           break;
		case CAN_ERROR_ACK: printf("acknowledgment error...\n"); break;
		case CAN_ERROR_BIT: printf("bit error...\n");            break;
		case CAN_ERROR_CRC: printf("CRC error...\n");            break;
		case CAN_ERROR_OTHER:
		default: printf("other error...\n");          break;
		}
	}
}

//////////////////////////////////////////////////////////////////////////
/**

  Safely copy new Mobileye object list from receive storage array to 
  exchange storage array. Notify sensor fusion thread. Then reset receive 
  storage array.

*/
//////////////////////////////////////////////////////////////////////////
void copyMobileyeObjectList()
{
	if (WAIT_OBJECT_0 == WaitForSingleObject(hMutex[1], 10)) { //request mutex access
		memcpy(&mList_Exchange, &mList_Reception, sizeof(ObjectList_Mobileye)); //copy reception array to mutex protected array
		ReleaseMutex(hMutex[1]);
		SetEvent(hEventListReady[1]);
	}
	memset(&mList_Reception, 0, sizeof(ObjectList_Mobileye)); //Reset storage array
}

//////////////////////////////////////////////////////////////////////////
/**

 Returns the data of a single signal in a CAN message as a UINT8.
 Signal must be within one Byte

 @param msgData: Pointer to msgData block from CAN message
 @param startBit: least significant bit as defined in dbc file
 @param len: DLC of signal

*/
//////////////////////////////////////////////////////////////////////////
UINT8 convertBytesUINT8(UINT8* msgData, int startBit, int len) {

	int lsB = startBit / 8; //determine least significant Byte (lsB)
	int bitsDone = 8 - (startBit % 8); //determine number of significant bits in lsB
	UINT8 newInt = msgData[lsB] >> (startBit % 8); //crop lsB bits to the right of startBit
	newInt &= (1 << len) - 1; //crop insignificant bits to the left

	return newInt;
}

//////////////////////////////////////////////////////////////////////////
/**

 Returns the data of a single signal in a CAN message as a UINT16
 using Byte order specified by endian parameter

 @param msgData: Pointer to msgData block from CAN message
 @param startBit: least significant bit as defined in dbc file
 @param len: DLC of signal
 @param endian: 1: Intel, -1: Motorola

*/
//////////////////////////////////////////////////////////////////////////
UINT16 convertBytesUINT16(UINT8* msgData, int startBit, int len, int endian) {

	int lsB = startBit / 8; //determine least significant Byte (lsB)
	int bitsDone = 8 - (startBit % 8); //determine number of significant bits in lsB
	UINT16 newInt = msgData[lsB] >> (startBit % 8); //crop lsB bits to the right of startBit
	newInt += (msgData[lsB + endian] << bitsDone); //add msB to newInt
	newInt &= (1 << len) - 1; //crop insignificant bits to the left

	return newInt;
}

//////////////////////////////////////////////////////////////////////////
/**

 Returns the data of a single signal in a Mobileye CAN message as a short int
 using Intel Byte order

 @param msgData: Pointer to msgData block from CAN message
 @param startBit: least significant bit as defined in dbc file
 @param len: DLC of signal

*/
//////////////////////////////////////////////////////////////////////////
short convertBytesINT16(UINT8* msgData, int startBit, int len) {

	int lsB = startBit / 8; //determine least significant Byte (lsB)
	int bitsDone = 8 - (startBit % 8); //determine number of significant bits in lsB
	short newInt = msgData[lsB] >> (startBit % 8); //crop lsB bits to the right of startBit
	newInt += (msgData[lsB + 1] << bitsDone); //add msB to newInt
	newInt &= (1 << len) - 1; //crop insignificant bits to the left
	if (newInt >> (len - 1) == 1) {//if the number is negative (sign bit = 1)
		newInt |= -1 << (len - 1); //set all bits left of significant portion to 1 to preserve value
	}
	return newInt;
}

//////////////////////////////////////////////////////////////////////////
/**
  Sensor fusion thread.

  @param Param
	ptr on a user defined information
*/
//////////////////////////////////////////////////////////////////////////
void SensorFusionThread(void* Param)
{
	UNREFERENCED_PARAMETER(Param);

	//Variables for Prediction step
	float timestamp_Rad_prev;
	float timestamp_Mob_prev;
	float dt;
	float sig_pos;
	float sig_vel;
	float sig_posvel;

	//Variables for time measurement
	UINT64 n = 0;
	UINT64 t_end_previous = 0;
	UINT64 t_start_wait = 0;
	UINT64 t_fusion_start = 0;
	UINT64 t_end_new = 0;
	double t_delta_mean = 0;
	double t_delta_sdev = 0;
	double t_wait_mean = 0;
	double t_wait_sdev = 0;
	double t_fusion_mean = 0;
	double t_fusion_sdev = 0;

	//Initialize rotation matrix
	Radar_C <<
		c_x_offset, 0.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 1.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f;
	Radar_R <<
		cos(deg2rad(gamma_offset)), -sin(deg2rad(gamma_offset)), 0.0f, 0.0f,
		sin(deg2rad(gamma_offset)), cos(deg2rad(gamma_offset)), 0.0f, 0.0f,
		0.0f, 0.0f, cos(deg2rad(gamma_offset)), -sin(deg2rad(gamma_offset)),
		0.0f, 0.0f, sin(deg2rad(gamma_offset)), cos(deg2rad(gamma_offset));
	x_offset <<
		p_x_offset,
		p_y_offset,
		0.0f,
		0.0f;

	//Initialize FusionList and Kalman Parameters
	memset(&fList_Fusion, 0, sizeof(ObjectList_Fusion));
	Kalman_P <<
		5.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 2.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 5.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 5.0f;
	Kalman_R <<
		5.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 2.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 5.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 5.0f;
	Kalman_R *= R_Gain;
	Kalman_I = Eigen::Matrix4f::Identity();

	Aim_Select_init();

	printf("\nStarting Sensor Fusion thread.\n");
	//cv::VideoCapture capture(0);
	cv::Mat frame;
	cv::Mat frame_TV;
	cv::Mat frame_ObjectInfo;

	while (true) {

		//QueryPerformanceCounter((LARGE_INTEGER*)&t_start_wait);
		if (WAIT_OBJECT_0 == WaitForMultipleObjects(2, hEventListReady, TRUE, FusionEventWaitTime)) {
			if (WAIT_OBJECT_0 == WaitForMultipleObjects(2, hMutex, TRUE, INFINITE)) {
				//QueryPerformanceCounter((LARGE_INTEGER*)&t_fusion_start);
				//copy exchange array data to fusion arrays
				memcpy(&rList_Fusion_Start, &rList_Exchange, sizeof(ObjectList_Radar));
				ResetEvent(hEventListReady[0]);
				ReleaseMutex(hMutex[0]);
				memcpy(&mList_Fusion, &mList_Exchange, sizeof(ObjectList_Mobileye));
				ResetEvent(hEventListReady[1]);
				ReleaseMutex(hMutex[1]);
			}
		}
		else { 
			DWORD dwEvent = WaitForMultipleObjects(2, hEventListReady, FALSE, INFINITE);
			switch (dwEvent) {
			case WAIT_OBJECT_0 + 0: //Only Radar List received
				if (WAIT_OBJECT_0 == WaitForSingleObject(hMutex[0], INFINITE)) {
					//QueryPerformanceCounter((LARGE_INTEGER*)&t_fusion_start);
					//copy exchange array data to fusion arrays
					memcpy(&rList_Fusion_Start, &rList_Exchange, sizeof(ObjectList_Radar));
					ResetEvent(hEventListReady[0]);
					ReleaseMutex(hMutex[0]);
					memset(&mList_Fusion, 0, sizeof(ObjectList_Mobileye));
					ResetEvent(hEventListReady[1]);
				}
				break;
			case WAIT_OBJECT_0 + 1: //Only Mobileye List received
				if (WAIT_OBJECT_0 == WaitForSingleObject(hMutex[1], INFINITE)) {
					//QueryPerformanceCounter((LARGE_INTEGER*)&t_fusion_start);
					//copy exchange array data to fusion arrays
					memset(&rList_Fusion_Start, 0, sizeof(ObjectList_Radar));
					ResetEvent(hEventListReady[0]);
					memcpy(&mList_Fusion, &mList_Exchange, sizeof(ObjectList_Mobileye));
					ResetEvent(hEventListReady[1]);
					ReleaseMutex(hMutex[1]);
				}
				break;
			}	
		}

		//setup fusion arrays
		//reset rList_Fusion_Candidates and copy new timestamps 
		memset(&rList_Fusion_Candidates, 0, sizeof(ObjectList_Radar));
		rList_Fusion_Candidates.timestamp = rList_Fusion_Start.timestamp;
		rList_Fusion_Candidates.speed = rList_Fusion_Start.speed;
		//reset rList_Fusion_Final and copy new timestamps 
		memset(&rList_Fusion_Final, 0, sizeof(ObjectList_Radar));
		rList_Fusion_Final.timestamp = rList_Fusion_Start.timestamp;
		rList_Fusion_Final.speed = rList_Fusion_Start.speed;
		//reset fList_Measurement
		memset(&fList_Measurement, 0, sizeof(ObjectList_Measurement));
		//update fList_Fusion
		timestamp_Rad_prev = fList_Fusion.timestamp_Rad;
		timestamp_Mob_prev = fList_Fusion.timestamp_Mob;
		fList_Fusion.timestamp_Rad = rList_Fusion_Start.timestamp;
		fList_Fusion.timestamp_Mob = mList_Fusion.timestamp;
		fList_Fusion.speed = rList_Fusion_Start.speed;
		//reset UDP_Objects
		memset(&UDPObjects, 0, sizeof(UDPObjects));

		////////////////////////////////////////////////////////////////////
		//start of sensor fusion
		////////////////////////////////////////////////////////////////////
		//Perform sensor fusion of Mobileye and radar data using a Kalman filter
		//reduce size of ObjectList_Radar by removing irrelevant Objects
		CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)RadarFusionThread, 0, NULL, &dwRadarReceiveThread);

		//perform prediction step for fusion Object List (converting delta in millis to seconds)
		if (fList_Fusion.timestamp_Rad == 0) {
			dt = fList_Fusion.timestamp_Mob - timestamp_Mob_prev;
		}
		else if(fList_Fusion.timestamp_Mob == 0) {
			dt = fList_Fusion.timestamp_Rad - timestamp_Rad_prev;
		}
		else {
			dt = w_t_Radar * (fList_Fusion.timestamp_Rad - timestamp_Rad_prev)
				+ (1.0f - w_t_Radar)*(fList_Fusion.timestamp_Mob - timestamp_Mob_prev);
		}
		sig_pos = pow(dt, 4) / 4;
		sig_vel = pow(dt, 2);
		sig_posvel = pow(dt, 3) / 2;
		Kalman_A <<
			1.0f, 0.0f, dt, 0.0f,
			0.0f, 1.0f, 0.0f, dt,
			0.0f, 0.0f, 1.0f, 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f;
		Kalman_Q <<
			sig_pos, sig_pos, sig_posvel, sig_posvel,
			sig_pos, sig_pos, sig_posvel, sig_posvel,
			sig_posvel, sig_posvel, sig_vel, sig_vel,
			sig_posvel, sig_posvel, sig_vel, sig_vel;
		Kalman_Q *= Q_Gain;

		for (UINT8 i = 0; i < fList_Fusion.n; i++)
		{	
			fList_Fusion.Objects[i].xk = Kalman_A * fList_Fusion.Objects[i].xk;
			fList_Fusion.Objects[i].Pk = Kalman_A * fList_Fusion.Objects[i].Pk * Kalman_A.transpose() + Kalman_Q;
			fList_Fusion.Objects[i].ID_Rad = -1; //Delete Radar ID
			fList_Fusion.Objects[i].ID_Mob = -1; //Delete Mobileye ID
			fList_Fusion.Objects[i].ProbOfExist *= NoMatchUpdate_ProbOfExist; //Update ProbofExist (will be undone if confirmed later)
		}

		//wait for radar fusion thread to end
		if (WAIT_OBJECT_0 == WaitForSingleObject(hEventRadarFusionReady, INFINITE)) {
			ResetEvent(hEventRadarFusionReady);
		}

		//perform sensor fusion
		//match radar and Mobileye objects to create new fusion objects
		createMeasurementList();

		UINT8 sendDelay = 0;
		if (sendFusionList == 1) {
			//match FusionList members with new measurement objects using a kalman filter
			updateFusionList();
			// with the fusionlist the Aimobject is selected
			Aim_Select();
			//send list via UDP
			sendDelay = 20;
		}
		if (sendMeasurementList == 1) {
			Sleep(sendDelay);
			send_UDP_MeasurementList();
			sendDelay = 20;
		}
		if (sendMobileyeList == 1) {
			Sleep(sendDelay);
			send_UDP_MobileyeList();
			sendDelay = 20;
		}
		if (sendRadarList == 1) {
			Sleep(sendDelay);
			send_UDP_RadarList();
			sendDelay = 20;
		}

		printf("\rTimestamp Radar: %f\tTimestamp Mobileye: %f\t", rList_Fusion_Start.timestamp, mList_Fusion.timestamp);
		//capture.read(frame);

		//Show_the_Frame(frame);
		Show_the_Topview(frame_TV,frame_ObjectInfo);

		//t_end_previous = t_end_new;
		//QueryPerformanceCounter((LARGE_INTEGER*)&t_end_new);
		//if (t_end_previous > 0) {
		//	n++;
		//	t_delta_sdev = n > 1 ? ((n - 2)*t_delta_sdev / (n - 1)) + (pow((t_end_new - t_end_previous - t_delta_mean), 2) / n) : 0;
		//	t_delta_mean = ((n - 1)*t_delta_mean + (t_end_new - t_end_previous)) / n;
		//	t_wait_sdev = n > 1 ? ((n - 2)*t_wait_sdev / (n - 1)) + (pow((t_fusion_start - t_start_wait - t_wait_mean), 2) / n) : 0;
		//	t_wait_mean = ((n - 1)*t_wait_mean + (t_fusion_start - t_start_wait)) / n;
		//	t_fusion_sdev = n > 1 ? ((n - 2)*t_fusion_sdev / (n - 1)) + (pow((t_end_new - t_fusion_start - t_fusion_mean), 2) / n) : 0;
		//	t_fusion_mean = ((n - 1)*t_fusion_mean + (t_end_new - t_fusion_start)) / n;
		//	printf("\nNumber of Radar Objects raw:        %d", rList_Fusion_Start.n);
		//	printf("\nNumber of Radar Objects candidates: %d", rList_Fusion_Candidates.n);
		//	printf("\nNumber of Radar Objects final:      %d", rList_Fusion_Final.n);
		//	printf("\nNumber of Mobileye Objects:         %d", mList_Fusion.n);
		//	printf("\nNumber of Measurement objects:      %d", fList_Measurement.n);
		//	printf("\nNumber of Fusion objects:           %d", fList_Fusion.n);
		//	printf("\nNumber of Sent objects:             %d", NofObjects_UDP);
		//	printf("\nTime between Fusions.         Mean: %f \t SDev: %f", t_delta_mean*1000.0 / system_frequency, sqrt(t_delta_sdev)*1000.0 / system_frequency);
		//	printf("\nTime spent waiting for Mutex. Mean: %f \t SDev: %f", t_wait_mean*1000.0 / system_frequency, sqrt(t_wait_sdev) * 1000.0 / system_frequency);
		//	printf("\nTime for sensor fusion.       Mean: %f \t SDev: %f", t_fusion_mean*1000.0 / system_frequency, sqrt(t_fusion_sdev)*1000.0 / system_frequency);
		//	printf("\nFusionCounter: %ld, NaNCounter: %ld", fusioncounter, nancounter);
		//	//printf("\nTimestamp Radar: %f", rList_Fusion_Start.timestamp);
		//	//printf("\nTimestamp Mobileye: %f", mList_Fusion.timestamp);
		//	printf("\n======================================");
		//}
	}
}

float deg2rad(float deg) {
	return (float)M_PI * deg / 180.0f;
}

//////////////////////////////////////////////////////////////////////////
/**
  Radar fusion thread. Removes irrelevant and redundant Objects from rList_Fusion_Start.

  @param Param
	ptr on a user defined information
*/
//////////////////////////////////////////////////////////////////////////
void RadarFusionThread(void* Param)
{
	//temporary variables
	float dX;
	float dX_max;
	float dY;
	float distX;
	float distX_min;
	
	//correct radar calibration
	for (UINT8 i = 0; i < rList_Fusion_Start.n; i++) {
		rList_Fusion_Start.Objects[i].x = ((Radar_C * Radar_R) * rList_Fusion_Start.Objects[i].x) +  x_offset;
	}
	//copy Objects that are either fusion candidates with ObjectList_Mobileye or within the relevant zone to candidate array
	for (UINT8 i = 0; i < rList_Fusion_Start.n; i++) {
		if (rList_Fusion_Start.Objects[i].ProbOfExist == 7) {
			if (abs(rList_Fusion_Start.Objects[i].x(1)) < p_y_relzone_Radar) { //add RadarObjects within the relevant zone
				rList_Fusion_Candidates.Objects[rList_Fusion_Candidates.n] = rList_Fusion_Start.Objects[i];
				rList_Fusion_Candidates.n++;
			}
			else { //add fusion candidates
				for (UINT8 j = 0; j < mList_Fusion.n; j++) {
					dX = rList_Fusion_Start.Objects[i].x(0) - mList_Fusion.Objects[j].x(0);
					dX_max = std::max(Delta_p_x_max_abs, Delta_p_x_max_percent * mList_Fusion.Objects[j].x(0));
					dY = rList_Fusion_Start.Objects[i].x(1) - mList_Fusion.Objects[j].x(1);
					if (dX > Delta_p_x_min && dX < dX_max && abs(dY) < Delta_p_y_max) {
						rList_Fusion_Candidates.Objects[rList_Fusion_Candidates.n] = rList_Fusion_Start.Objects[i];
						rList_Fusion_Candidates.n++;
						break;
					}
				}
			}
		}
	}

	//further decrease list size by removing redundant objects
	//find redundant objects within RadarObjectList_candidates
	memset(&Status_Rad, 0, sizeof(Status_Rad)); //set all objects as "copy to final array" first
	for (UINT8 i = 0; i < rList_Fusion_Candidates.n; i++) {
		distX_min = INFINITY;
		AscVec_Rad[i] = -1; //reset all references to "infinite"
		for (UINT8 j = 0; j < rList_Fusion_Candidates.n; j++) {
			if (i != j) {
				dX = rList_Fusion_Candidates.Objects[j].x(0) - rList_Fusion_Candidates.Objects[i].x(0);
				distX = dX - rList_Fusion_Candidates.Objects[i].Length; //calculate distance between front and back of two Objects
				dY = rList_Fusion_Candidates.Objects[j].x(1) - rList_Fusion_Candidates.Objects[i].x(1);
				//if distX is smaller than threshold, mark them for fusion, if it is the closest object
				if (dX > Delta_p_x_min_Radar && distX < dist_x_max_Radar && abs(dY) < Delta_p_x_max_Radar && distX < distX_min) {
					AscVec_Rad[i] = j; //set j as reference object for i
					distX_min = distX;
				}
			}
		}
		if (AscVec_Rad[i] < (UINT8)-1) {
			Status_Rad[AscVec_Rad[i]] = 1; //set minimal j as "delete after fusion"
		}
	}
	//fuse redundant objects
	for (UINT8 i = 0; i < rList_Fusion_Candidates.n; i++) {
		if (Status_Rad[i] == 0) { //if object is not a reference object itself, copy it to final array
			if (AscVec_Rad[i] < (UINT8)-1) { //if object has (a) reference object(s), fuse them recursively
				fuseRadarObjects(i);
			}
			rList_Fusion_Final.Objects[rList_Fusion_Final.n] = rList_Fusion_Candidates.Objects[i];
			rList_Fusion_Final.n++;
		}
	}
	SetEvent(hEventRadarFusionReady);
}

void fuseRadarObjects(UINT8 i)
{
	UINT8 j = AscVec_Rad[i]; //array position of reference object
	if (AscVec_Rad[j] < (UINT8)-1) { //if the reference object has a reference object itself, fuse them first
		fuseRadarObjects(j);
	}

	RadarObject* RadObj_i = &rList_Fusion_Candidates.Objects[i]; //pointer to RadarObject i 
	RadarObject* RadObj_j = &rList_Fusion_Candidates.Objects[j]; //pointer to RadarObject j

	if (RadObj_i->Width < RadObj_j->Width) {
		RadObj_i->Class = RadObj_j->Class;
	}
	RadObj_i->ProbOfExist = std::max(RadObj_i->ProbOfExist, RadObj_j->ProbOfExist);
	RadObj_i->x(1) = (RadObj_i->x(1) + RadObj_j->x(1)) / 2;
	RadObj_i->x(2) = (RadObj_i->x(2) + RadObj_j->x(2)) / 2;
	RadObj_i->x(3) = (RadObj_i->x(3) + RadObj_j->x(3)) / 2;
	//RadObj_i->ArelLongX = (RadObj_i->ArelLongX + RadObj_j->ArelLongX) / 2;
	RadObj_i->Width = std::max(RadObj_i->Width, RadObj_j->Width);
	RadObj_i->Length = RadObj_j->x(0) - RadObj_i->x(0) + RadObj_j->Length;
	RadObj_i->RCS = std::max(RadObj_i->RCS, RadObj_j->RCS);
}

void createMeasurementList() {

	//temporary variables
	UINT8 radID; //iterator for Radar objects
	UINT8 mobID; //iterator for Mobileye objects
	UINT8 nN;
	UINT8 fusionStatus[MAX_RADAR_OBJECTS];
	float dX;
	float dX_max;
	float dX_min;
	float dY;
	float distMat[MAX_RADAR_OBJECTS][MAX_MOBILEYE_OBJECTS];

	//1. find nearest eligible Mobileye object of each Radar Object
	for (radID = 0; radID < rList_Fusion_Final.n; radID++) {
		nN = -1;
		dX_min = INFINITY;
		fusionStatus[radID] = 0;
		for (mobID = 0; mobID < mList_Fusion.n; mobID++) {
			dX = rList_Fusion_Final.Objects[radID].x(0) - mList_Fusion.Objects[mobID].x(0);
			dX_max = std::max(Delta_p_x_max_percent * mList_Fusion.Objects[mobID].x(0), Delta_p_x_max_abs);
			dY = rList_Fusion_Final.Objects[radID].x(1) - mList_Fusion.Objects[mobID].x(1);
			distMat[radID][mobID] = INFINITY; //initialise distance matrix with inf
			//if a (better) match is found, update nearest neighbor and distance
			if (dX > Delta_p_x_min && dX < dX_max && abs(dY) < Delta_p_y_max && dX < dX_min) {
				nN = mobID;
				dX_min = dX;    
			}
		}
		if (nN != (UINT8)-1) { //if a minimal match is found, insert distance in distmat
			distMat[radID][nN] = dX_min;
		}
	}
	//2. In the resulting matrix, find the nearest radar object for each Mobileye object and fuse them, insert others into measurement list
	for (mobID = 0; mobID < mList_Fusion.n; mobID++) {
		nN = -1;
		dX_min = INFINITY;
		for (radID = 0; radID < rList_Fusion_Final.n; radID++) {
			if (distMat[radID][mobID] < dX_min) { //if a better match is found, update association object and min distance
				dX_min = distMat[radID][mobID];
				nN = radID;
			}
		}
		if (nN != (UINT8)-1) { //if match is found, fuse objects and insert into measurement object list and mark radar object as fused
			createFusionObject(mobID, nN);
			fusionStatus[nN] = 1;
		}
		else { //insert mobileye object into measurement object list
			copyMobileyeObjectToMeasurementList(mobID);
		}
	}
	//Add all radar objects that have not been fused to measurement object list
	for (radID = 0; radID < rList_Fusion_Final.n; radID++) {
		if (fList_Measurement.n >= MAX_MEASUREMENT_OBJECTS) break; //cancel loop if measurement list is full
		if (fusionStatus[radID] == 0) {
			copyRadarObjectToMeasurementList(radID);
		}
	}
}

void createFusionObject(UINT8 mobID, UINT8 radID) {
	FusionObject* pFusObj = &fList_Measurement.Objects[fList_Measurement.n];
	RadarObject* pRadObj = &rList_Fusion_Final.Objects[radID];
	MobileyeObject *pMobObj = &mList_Fusion.Objects[mobID];

	pFusObj->ID_Rad = pRadObj->ID;
	pFusObj->ID_Mob = pMobObj->ID;
	pFusObj->Class = pMobObj->Class;
	pFusObj->Lane = pMobObj->Lane;
	pFusObj->xk <<
		w_p_x_Radar * pRadObj->x(0) + (1.0f - w_p_x_Radar)*pMobObj->x(0),
		w_p_y_Radar * pRadObj->x(1) + (1.0f - w_p_y_Radar)*pMobObj->x(1),
		w_vrel_x_Radar * pRadObj->x(2) + (1.0f - w_vrel_x_Radar)*pMobObj->x(2),
		pRadObj->x(3);
	pFusObj->Width = w_Width_Radar * pRadObj->Width + (1.0f - w_Width_Radar)*pMobObj->Width;
	pFusObj->Length = pRadObj->Length;
	pFusObj->ProbOfExist = MaxProbOfExist_FusionObject;
	pFusObj->RCS = pRadObj->RCS;

	fList_Measurement.n++;
}

UINT8 convertRadClass(UINT8 RadClass) {
	switch (RadClass) {
	case 0: return 5;
	case 1: return 0;
	case 2: return 1;
	case 4: return 2;
	case 5: return 4;
	case 6: return 6;
	default: return -1;
	}
}

void copyMobileyeObjectToMeasurementList(UINT8 mobID) {
	FusionObject *pFusObj = &fList_Measurement.Objects[fList_Measurement.n];
	MobileyeObject *pMobObj = &mList_Fusion.Objects[mobID];

	pFusObj->ID_Rad = -1;
	pFusObj->ID_Mob = pMobObj->ID;
	pFusObj->Class = pMobObj->Class;
	pFusObj->Lane = pMobObj->Lane;
	pFusObj->xk = pMobObj->x;
	pFusObj->Width = pMobObj->Width;
	pFusObj->ProbOfExist = MaxProbOfExist_MobileyeObject;

	fList_Measurement.n++;
}

void copyRadarObjectToMeasurementList(UINT8 radID) {
	FusionObject* pFusObj = &fList_Measurement.Objects[fList_Measurement.n];
	RadarObject* pRadObj = &rList_Fusion_Final.Objects[radID];

	pFusObj->ID_Rad = pRadObj->ID;
	pFusObj->ID_Mob = -1;
	pFusObj->Class = convertRadClass(pRadObj->Class);
	pFusObj->xk = pRadObj->x;
	pFusObj->Width = pRadObj->Width;
	pFusObj->Length = pRadObj->Length;
	pFusObj->ProbOfExist = MaxProbOfExist_RadarObject;
	pFusObj->RCS = pRadObj->RCS;

	fList_Measurement.n++;
}

void updateFusionList() {

	//temporary variables
	UINT8 fusID; //iterator for fusion objects
	UINT8 measID; //iterator for measurement objects
	UINT8 nN; //nearest Neighbor ID
	UINT8 fusionStatus[MAX_MEASUREMENT_OBJECTS];
	float dX;
	float dY;
	float dist;
	float dist_min;
	float distMat[MAX_FUSION_OBJECTS][MAX_MEASUREMENT_OBJECTS];

	//find nearest neighbors of fusion objects in measurement list
	for (fusID = 0; fusID < fList_Fusion.n; fusID++) {
		nN = -1;
		dist_min = INFINITY;
		for (measID = 0; measID < fList_Measurement.n; measID++) {
			distMat[fusID][measID] = INFINITY; //initialise distance matrix with inf
			dX = fList_Fusion.Objects[fusID].xk(0) - fList_Measurement.Objects[measID].xk(0);
			dY = fList_Fusion.Objects[fusID].xk(1) - fList_Measurement.Objects[measID].xk(1);
			dist = sqrt(dX * dX + dY * dY);
			if (dist < dist_max && dist < dist_min) { //if a (better) match is found, update nN and dist_min
				nN = measID;
				dist_min = dist;
			}
		}
		if (nN != (UINT8)-1) { //if a minimal match is found, insert distance in distmat
			distMat[fusID][nN] = dist_min;
		}
	}
	//fuse with nearest match
	for (measID = 0; measID < fList_Measurement.n; measID++) {
		nN = -1;
		dist_min = INFINITY;
		fusionStatus[measID] = 0;
		for (fusID = 0; fusID < fList_Fusion.n; fusID++) {
			if (distMat[fusID][measID] < dist_min) { //if a better match is found, update association object and min distance
				dist_min = distMat[fusID][measID];
				nN = fusID;
			}
		}
		if (nN != (UINT8)-1) { //if match is found, fuse objects and update fusion list
			kalman_update(nN, measID);
			fusionStatus[measID] = 1;
		}
	}
	//insert measurement objects that are not fused into fusion list as new objects with reduced probability (needs to be confirmed in next steps)
	for (measID = 0; measID < fList_Measurement.n; measID++) { 
		if (fusionStatus[measID] == 0) {
			fList_Fusion.Objects[fList_Fusion.n] = fList_Measurement.Objects[measID];
			fList_Fusion.Objects[fList_Fusion.n].ProbOfExist *= NewObjectPenalty_ProbOfExist;
			fList_Fusion.n++;
		}
	}
	//calculate relevance for objects above probability threshold, set relevance of objects below threshold to 0
	for (fusID = 0; fusID < fList_Fusion.n; fusID++) {
		if (fList_Fusion.Objects[fusID].ProbOfExist < DeleteThreshold_ProbOfExist) {
			fList_Fusion.Objects[fusID].Relevance = 0.0f;
		}
		else {
			fList_Fusion.Objects[fusID].Relevance = 1.0f /
				((fList_Fusion.Objects[fusID].xk(0) * fList_Fusion.Objects[fusID].xk(0) / 100.0f)
					+ (fList_Fusion.Objects[fusID].xk(1) * fList_Fusion.Objects[fusID].xk(1)));
		}
	}
	//sort resulting list by Relevance
	std::sort(fList_Fusion.Objects, fList_Fusion.Objects + fList_Fusion.n, compareRelevance);
	//sort resulting list by ProbofExist
	//std::sort(fList_Fusion.Objects, fList_Fusion.Objects + fList_Fusion.n, compareProbOfExist);
	//crop list to maximum number of fusion objects
	if (fList_Fusion.n > MAX_FUSION_OBJECTS) fList_Fusion.n = MAX_FUSION_OBJECTS;
	//crop list to exclude objects with 0 relevance and add objects above send threshold to UDP send list
	NofObjects_UDP = 0;
	for (fusID = 0; fusID < fList_Fusion.n; fusID++) {
		if (fList_Fusion.Objects[fusID].Relevance == 0.0f) {
			fList_Fusion.n = fusID;
			break;
		}
	}	
}

void kalman_update(UINT8 fusID, UINT8 measID) {
	FusionObject* pMeasObj = &fList_Measurement.Objects[measID];
	FusionObject* pFusObj = &fList_Fusion.Objects[fusID];
	Eigen::Matrix4f S;	//innovation covariance
	Eigen::Matrix4f K;	//Kalman gain
	
	if (pMeasObj->ID_Rad == (UINT8)-1) pMeasObj->xk(3) = pFusObj->xk(3);
	S = pFusObj->Pk + Kalman_R;
	K = S.colPivHouseholderQr().solve(pFusObj->Pk);
	if (K.hasNaN()) { //If equation cant be solved, replace fusion object with measurement object
		fList_Fusion.Objects[fusID] = fList_Measurement.Objects[measID];
	}
	else {
		pFusObj->xk = pFusObj->xk + K * (pMeasObj->xk - pFusObj->xk);
		pFusObj->Pk = (Kalman_I - K)*pFusObj->Pk;
		pFusObj->ID_Mob = pMeasObj->ID_Mob;
		pFusObj->ID_Rad = pMeasObj->ID_Rad;
		if (pMeasObj->ID_Mob != (UINT8)-1) pFusObj->Class = pMeasObj->Class;
		pFusObj->Lane = pMeasObj->Lane;
		pFusObj->Width = (w_Width_Hist*pFusObj->Width) + ((1.0f - w_Width_Hist) * pMeasObj->Width);
		pFusObj->Length = (w_Length_Hist * pFusObj->Length) + ((1.0f - w_Length_Hist) * pMeasObj->Length);
		pFusObj->ProbOfExist = (0.5f*pFusObj->ProbOfExist / NoMatchUpdate_ProbOfExist) + (0.5f * pMeasObj->ProbOfExist);
		pFusObj->RCS = (w_RCS_Hist * pFusObj->RCS) + ((1.0f - w_RCS_Hist) * pMeasObj->RCS);
	}
}

//bool compareProbOfExist(const FusionObject& first, const FusionObject& second) {
//	return first.ProbOfExist > second.ProbOfExist;
//}

bool compareRelevance(const FusionObject& first, const FusionObject& second) {
	return first.Relevance > second.Relevance;
}

void send_UDP_FusionList()
{
	memset(&UDPObjects, 0, sizeof(UDPObjects));

	////////////////////////////////////////
	memcpy(&fList_Fusion, &fList_Measurement, sizeof(fList_Fusion));
	///////////////////////////////////////



	NofObjects_UDP = std::min(MAX_UDP_OBJECTS, (int)fList_Fusion.n);
	for (UINT8 fusID = 0; fusID < NofObjects_UDP; fusID++)
	{
		if (fList_Fusion.Objects[fusID].ProbOfExist > SendThreshold_ProbOfExist) {
			UDPObjects[fusID].ID_Rad = fList_Fusion.Objects[fusID].ID_Rad;
			UDPObjects[fusID].ID_Mob = fList_Fusion.Objects[fusID].ID_Mob;
			UDPObjects[fusID].Class = fList_Fusion.Objects[fusID].Class;
			UDPObjects[fusID].Lane = fList_Fusion.Objects[fusID].Lane;
			UDPObjects[fusID].PosLongX = fList_Fusion.Objects[fusID].xk(0);
			UDPObjects[fusID].PosLatY = fList_Fusion.Objects[fusID].xk(1);
			UDPObjects[fusID].VrelLongX = fList_Fusion.Objects[fusID].xk(2);
			UDPObjects[fusID].VrelLatY = fList_Fusion.Objects[fusID].xk(3);
			UDPObjects[fusID].Width = fList_Fusion.Objects[fusID].Width;
			UDPObjects[fusID].Length = fList_Fusion.Objects[fusID].Length;
			UDPObjects[fusID].ProbOfExist = fList_Fusion.Objects[fusID].ProbOfExist;
			UDPObjects[fusID].RCS = fList_Fusion.Objects[fusID].RCS;
		}
		
	}
	send_UDP_ObjectList();
}

void send_UDP_MeasurementList() {
	memset(&UDPObjects, 0, sizeof(UDPObjects));
	NofObjects_UDP = std::min(MAX_UDP_OBJECTS, (int)fList_Measurement.n);
	for (UINT8 measID = 0; measID < NofObjects_UDP; measID++) {
		UDPObjects[measID].ID_Rad = fList_Measurement.Objects[measID].ID_Rad;
		UDPObjects[measID].ID_Mob = fList_Measurement.Objects[measID].ID_Mob;
		UDPObjects[measID].Class = fList_Measurement.Objects[measID].Class;
		UDPObjects[measID].Lane = fList_Measurement.Objects[measID].Lane;
		UDPObjects[measID].PosLongX = fList_Measurement.Objects[measID].xk(0);
		UDPObjects[measID].PosLatY = fList_Measurement.Objects[measID].xk(1);
		UDPObjects[measID].VrelLongX = fList_Measurement.Objects[measID].xk(2);
		UDPObjects[measID].VrelLatY = fList_Measurement.Objects[measID].xk(3);
		UDPObjects[measID].Width = fList_Measurement.Objects[measID].Width;
		UDPObjects[measID].Length = fList_Measurement.Objects[measID].Length;
		UDPObjects[measID].ProbOfExist = fList_Measurement.Objects[measID].ProbOfExist;
		UDPObjects[measID].RCS = fList_Measurement.Objects[measID].RCS;
	}
	send_UDP_ObjectList();
}

void send_UDP_MobileyeList() {
	memset(&UDPObjects, 0, sizeof(UDPObjects));
	NofObjects_UDP = std::min(MAX_UDP_OBJECTS, (int)mList_Fusion.n);
	for (UINT8 mobID = 0; mobID < NofObjects_UDP; mobID++) {
		UDPObjects[mobID].ID_Rad = -1;
		UDPObjects[mobID].ID_Mob = mList_Fusion.Objects[mobID].ID;
		UDPObjects[mobID].Class = mList_Fusion.Objects[mobID].Class;
		UDPObjects[mobID].Lane = mList_Fusion.Objects[mobID].Lane;
		UDPObjects[mobID].PosLongX = mList_Fusion.Objects[mobID].x(0);
		UDPObjects[mobID].PosLatY = mList_Fusion.Objects[mobID].x(1);
		UDPObjects[mobID].VrelLongX = mList_Fusion.Objects[mobID].x(2);
		UDPObjects[mobID].Width = mList_Fusion.Objects[mobID].Width;
		UDPObjects[mobID].ProbOfExist = MaxProbOfExist_MobileyeObject;
	}
	send_UDP_ObjectList();
}

void send_UDP_RadarList() {
	memset(&UDPObjects, 0, sizeof(UDPObjects));
	NofObjects_UDP = std::min(MAX_UDP_OBJECTS, (int)rList_Fusion_Start.n);
	for (UINT8 radID = 0; radID < NofObjects_UDP; radID++) {
		UDPObjects[radID].ID_Rad = rList_Fusion_Start.Objects[radID].ID;
		UDPObjects[radID].ID_Mob = -1;
		UDPObjects[radID].Class = convertRadClass(rList_Fusion_Start.Objects[radID].Class);
		UDPObjects[radID].PosLongX = rList_Fusion_Start.Objects[radID].x(0);
		UDPObjects[radID].PosLatY = rList_Fusion_Start.Objects[radID].x(1);
		UDPObjects[radID].VrelLongX = rList_Fusion_Start.Objects[radID].x(2);
		UDPObjects[radID].VrelLatY = rList_Fusion_Start.Objects[radID].x(3);
		UDPObjects[radID].Width = rList_Fusion_Start.Objects[radID].Width;
		UDPObjects[radID].Length = rList_Fusion_Start.Objects[radID].Length;
		UDPObjects[radID].ProbOfExist = MaxProbOfExist_RadarObject;
		UDPObjects[radID].RCS = rList_Fusion_Start.Objects[radID].RCS;
	}
	send_UDP_ObjectList();
}

//////////////////////////////////////////////////////////////////////////
/**

 Sends accumulated data via UDP

*/
//////////////////////////////////////////////////////////////////////////
void send_UDP_ObjectList() {
	long UDPError = 0;

	NofObjects_UDP = std::min(fList_Fusion.n,(UINT8)40);
	memcpy(buf_MAB, &NofObjects_UDP, 1);
	memcpy(buf_MAB + 1, &fList_Fusion.timestamp_Rad, 4);
	memcpy(buf_MAB + 5, &fList_Fusion.timestamp_Mob, 4);
	memcpy(buf_MAB + 9, &fList_Fusion.speed, 4);
	memcpy(buf_MAB + 13, &UDPObjects, 1440);

	memcpy(buf_AimInfo, &Aim_Info, 48);
	memcpy(buf_AimInfo + 48, &cur_ego, 4);

	if (sendToMAB == 1) {
		UDPError = sendto(UDPSocket, buf_MAB, NUM_BYTES_1401, 0, (SOCKADDR*)&MABAddr_Port1401, sizeof(SOCKADDR_IN));
		Sleep(20);
		UDPError = sendto(UDPSocket_AimInfo, buf_AimInfo, NUM_BYTES_1403, 0, (SOCKADDR*)&PCAddr_Port1401, sizeof(SOCKADDR_IN));
	}
	if (sendToPC == 1) {
		UDPError = sendto(UDPSocket, buf_MAB, NUM_BYTES_1401, 0, (SOCKADDR*)&PCAddr_Port1401, sizeof(SOCKADDR_IN));
		Sleep(20);
		UDPError = sendto(UDPSocket_AimInfo, buf_AimInfo, NUM_BYTES_1403, 0, (SOCKADDR*)&PCAddr_Port1403, sizeof(SOCKADDR_IN));
	}
	if (UDPError == SOCKET_ERROR)
	{
		printf("error: send, error code: %d\n", WSAGetLastError());
	}
}


int Aim_Select() {

	// The current FusionList is fList_Fusion
	// First Step: Coordinat Transmission
	Coordiante_Transmission();

	// Remove the expired date
	Remove_expired();

	// Adding the new date.
	Add_new();

	// Fitting the curves.
	Fit_curve();

	// spw (spurwahrscheinlichkeit) distribution
	SPW_distribution();

	// Update the PLA with SPW-Disstribution
	PLA_update();

	// Select the Aim according to the PLA
	Select_with_PLA();

	// Sending the result
	send_UDP_FusionList();

	return 1;
}

// ----------------------------------------------------------------------------------- //
//  In this subfunction....                                                            //
//                                                                                     //
//                                                                                     //
// ----------------------------------------------------------------------------------- //
void Aim_Select_init()
{
	// Temple, later it  should be deleted
	//memset(&Fahrdynamik, 0, sizeof(Fahrdynamik));

	// Initializing the system with no aim.
	Aim_Info.Exist_of_Aim = 0;
	Aim_Info.PLA = 0;
	Aim_Info.Detection = 0;

	// Initializing the width of SPW distribution.
	width_coefficient = Eigen::ArrayXXf::Zero(8, 3);
	Eigen::ArrayXXf width_suooprtpoint = Eigen::ArrayXXf::Zero(8, 4);
	width_suooprtpoint.col(1) << 1.0f, 1.05f, 1.10f, 1.15f, 1.20f, 1.25f, 1.30f, 1.35f;
	width_suooprtpoint.col(2) << 40.0f, 60.0f, 70.0f, 90.0f, 120.0f, 140.0f, 150.0f, 160.0f;
	width_suooprtpoint.col(3) << 1.5f, 1.7f, 1.8f, 1.9f, 2.2f, 2.3f, 2.4f, 2.6f;

	// Initializing the Value of SPW distribution.
	SPW_feld << -0.75f, -0.5f, -0.25f, 0.0f, 0.25f, 0.5f, 0.75f, 1.0f, 1.0f, 0.75f, 0.5f, 0.25f, 0.0f, -0.25f, -0.5f, -0.75f;

	for (int i = 0; i < 8; i++)
	{
		width_coefficient(i, 0) = (width_suooprtpoint(i, 1) - width_suooprtpoint(i, 3)) / pow(width_suooprtpoint(i, 2), 2);
		width_coefficient(i, 1) = 2.0f * (width_suooprtpoint(i, 3) - width_suooprtpoint(i, 1)) / width_suooprtpoint(i, 2);
		width_coefficient(i, 2) = width_suooprtpoint(i, 1);
	}

	// Initializing the x coordinates of center line.
	for (float i = 0; i < 41; i++)
	{
		d_feld(8, (int)i) = 5.0f * i;
	}
	UDPSocket = socket(AF_INET, SOCK_DGRAM, 0);
}

// ----------------------------------------------------------------------------------- //
//  In this subfunction....                                                            //
//                                                                                     //
//                                                                                     //
// ----------------------------------------------------------------------------------- //
void Coordiante_Transmission()
{
	// Firstly the timestamp will be here updated.
	Point_Buffer.last_timestamp = Point_Buffer.timestamp;
	float count_timestamp = 0.0;
	if (fList_Fusion.timestamp_Mob > 0) ++count_timestamp;
	if (fList_Fusion.timestamp_Rad > 0) ++count_timestamp;
	Point_Buffer.timestamp = (fList_Fusion.timestamp_Mob + fList_Fusion.timestamp_Rad) / count_timestamp;

	// The mutex will be locked, that the writing thread can not write. 
	/*std::lock_guard<std::mutex> lock(Fahrdynamik.Fahrdynamik_mutex);*/
	Fahrdynamik_wr_mutex.lock();
	// Then we conculate the displacement.
	float v = sqrt(pow(Fahrdynamik.Long_Vel, 2.0f) + pow(Fahrdynamik.Lat_Vel, 2.0f));
	float delta_t = Point_Buffer.timestamp - Point_Buffer.last_timestamp;     //Hier ist zu beobachten, wie man die echte Zeit rechnen soll.
	//float delta_x = Fahrdynamik.Yaw_Rate / Fahrdynamik.Yaw_Rate * sin(Fahrdynamik.Yaw_Rate * delta_t);
	//float delta_y = Fahrdynamik.Long_Vel / Fahrdynamik.Yaw_Rate * (1 - cos(Fahrdynamik.Yaw_Rate * delta_t));
	float delta_x;
	float delta_y;
	if (Fahrdynamik.Yaw_Rate == 0)
	{
		delta_x = v * delta_t;
		delta_y = 0;
	}
	else
	{
		delta_x = v / Fahrdynamik.Yaw_Rate * sin(Fahrdynamik.Yaw_Rate * delta_t);
		delta_y = v / Fahrdynamik.Yaw_Rate * (1 - cos(Fahrdynamik.Yaw_Rate * delta_t));
	}
	// At last all the saved points  should be transfered.
	for (int i = 0; i < Point_Buffer.n; ++i)
	{
		for (int j = 0; j < Point_Buffer.Object_History[i].NumofPoint; ++j)
		{
			float x = Point_Buffer.Object_History[i].Position[0][j];
			float y = Point_Buffer.Object_History[i].Position[1][j];
			Point_Buffer.Object_History[i].Position[0][j] = (x - delta_x) * cos(Fahrdynamik.Yaw_Rate * delta_t) + (y - delta_y) * sin(Fahrdynamik.Yaw_Rate * delta_t);
			Point_Buffer.Object_History[i].Position[1][j] = -(x - delta_x) * sin(Fahrdynamik.Yaw_Rate * delta_t) + (y - delta_y) * cos(Fahrdynamik.Yaw_Rate * delta_t);
		}
	}
	Fahrdynamik_wr_mutex.unlock();
}


// ----------------------------------------------------------------------------------- //
//  In this subfunction....                                                            //
//                                                                                     //
//                                                                                     //
// ----------------------------------------------------------------------------------- //
void Remove_expired()
{
	float count_timestamp = 0.0;
	if (fList_Fusion.timestamp_Mob > 0) ++count_timestamp;
	if (fList_Fusion.timestamp_Rad > 0) ++count_timestamp;
	float timestamp_now = (fList_Fusion.timestamp_Mob + fList_Fusion.timestamp_Rad) / count_timestamp;
	//Here all the expired data should be deleted.
	for (int i = 0; i < Point_Buffer.n; ++i)
	{
		while (Point_Buffer.Object_History[i].NumofPoint > 0)
		{
			bool Nominus_in_X_Position = 1;
			for (int j = 0; j < Point_Buffer.Object_History[i].NumofPoint; ++j)
			{
				if ((Point_Buffer.Object_History[i].Position[0][j] <= 0) && (Point_Buffer.Object_History[i].NumofPoint > 0))
				{
					for (int k = 0; k < Point_Buffer.Object_History[i].NumofPoint - 1; ++k)
					{
						Point_Buffer.Object_History[i].Position[0][k] = Point_Buffer.Object_History[i].Position[0][k + 1];
						Point_Buffer.Object_History[i].Position[1][k] = Point_Buffer.Object_History[i].Position[1][k + 1];
					}
					--Point_Buffer.Object_History[i].NumofPoint;
					--j;
				}
			}
			for (int j = 0; j < Point_Buffer.Object_History[i].NumofPoint; ++j)
			{
				if (Point_Buffer.Object_History[i].Position[0][j] <= 0) Nominus_in_X_Position = 0;
			}
			if (Nominus_in_X_Position) break;
		}
		bool  expiration_1 = Point_Buffer.Object_History[i].NumofPoint == 0;
		bool  expiration_2 = (Point_Buffer.Object_History[i].NumofPoint <= removeObjekt_threshold_1_min_Points) && ((timestamp_now - Point_Buffer.Object_History[i].timestamp_latest_renew) > removeObjekt_threshold_1_max_Time);
		bool  expiration_3 = (timestamp_now - Point_Buffer.Object_History[i].timestamp_latest_renew) > removeObjekt_threshold_2_max_Time;
		if (expiration_1 || expiration_2 || expiration_3)
		{
			for (int j = i; j < Point_Buffer.n - 1; ++j)
			{
				memcpy(&Point_Buffer.Object_History[j], &Point_Buffer.Object_History[j + 1], sizeof(Point_Buffer.Object_History[j + 1]));
			}
			--Point_Buffer.n;
		}
	}
}


// ----------------------------------------------------------------------------------- //
//  In this subfunction....                                                            //
//                                                                                     //
//                                                                                     //
// ----------------------------------------------------------------------------------- //
void Add_new()
{
	int flag_assigned;

	// All the "detection" will be reset to 0;
	for (int i = 0; i < Point_Buffer.n; ++i)
	{
		Point_Buffer.Object_History[i].Detection = 0;
	}
	float count_timestamp = 0.0;
	if (fList_Fusion.timestamp_Mob > 0) ++count_timestamp;
	if (fList_Fusion.timestamp_Rad > 0) ++count_timestamp;
	float timestamp_now = (fList_Fusion.timestamp_Mob + fList_Fusion.timestamp_Rad) / count_timestamp;
	// All the detected objects will be attchted to the "History".
	for (int i = 0; i < fList_Fusion.n; ++i)
	{
		if (fList_Fusion.Objects[i].ID_Mob < 255 || fList_Fusion.Objects[i].ID_Rad < 255)
		{
			if (fList_Fusion.Objects[i].xk[0] > 0) // Only valid data
			{
				flag_assigned = 0;
				for (int j = 0; j < Point_Buffer.n; ++j)
				{
					// if the data comes from an object with history.
					if ((fList_Fusion.Objects[i].ID_Mob == Point_Buffer.Object_History[j].ID_Mob) && (fList_Fusion.Objects[i].ID_Rad == Point_Buffer.Object_History[j].ID_Rad))
					{
						Point_Buffer.Object_History[j].Detection = 1;
						Point_Buffer.Object_History[j].timestamp_latest_renew = timestamp_now;
						bool repeat_point = false;
						for (int k = 0; k < Point_Buffer.Object_History[j].NumofPoint; ++k)
						{
							float delta_x = Point_Buffer.Object_History[j].Position[0][k] - fList_Fusion.Objects[i].xk[0];
							float delta_y = Point_Buffer.Object_History[j].Position[1][k] - fList_Fusion.Objects[i].xk[1];
							double distance = sqrt(pow(double(delta_x), 2) + pow(double(delta_y), 2));
							if (distance < Threshold_add_new_point)
							{
								repeat_point = true;
								break;
							}
						}
						if (!repeat_point)
						{
							if (Point_Buffer.Object_History[j].NumofPoint < MAX_POINTNUMBER)
							{
								Point_Buffer.Object_History[j].Position[0][Point_Buffer.Object_History[j].NumofPoint] = fList_Fusion.Objects[i].xk[0];
								Point_Buffer.Object_History[j].Position[1][Point_Buffer.Object_History[j].NumofPoint] = fList_Fusion.Objects[i].xk[1];
								Point_Buffer.Object_History[j].NumofPoint += 1;
							}
							else
							{
								// Find the Point with smallest X, and replace it.
								float smallest_x = 200;
								int Index_Replace = 0;
								for (int k = 0; k < Point_Buffer.Object_History[j].NumofPoint; ++k)
								{
									if (Point_Buffer.Object_History[j].Position[0][k] < smallest_x)
									{
										smallest_x = Point_Buffer.Object_History[j].Position[0][k];
										Index_Replace = k;
									}
								}
								Point_Buffer.Object_History[j].Position[0][Index_Replace] = fList_Fusion.Objects[i].xk[0];
								Point_Buffer.Object_History[j].Position[1][Index_Replace] = fList_Fusion.Objects[i].xk[1];
							}
						}

						Point_Buffer.Object_History[j].distance = sqrt(pow(fList_Fusion.Objects[i].xk[0], 2.0f) + pow(fList_Fusion.Objects[i].xk[1], 2.0f));
						flag_assigned = 1;
					}
				}
				// if the data comes from an new object.
				if (flag_assigned == 0)
				{
					UINT8 n = Point_Buffer.n;
					if (n < MAX_OBJECTHISTORY)
					{
						Point_Buffer.Object_History[n].Detection = 1;
						Point_Buffer.Object_History[n].ID_Mob = fList_Fusion.Objects[i].ID_Mob;
						Point_Buffer.Object_History[n].ID_Rad = fList_Fusion.Objects[i].ID_Rad;
						Point_Buffer.Object_History[n].Position[0][0] = fList_Fusion.Objects[i].xk[0];
						Point_Buffer.Object_History[n].Position[1][0] = fList_Fusion.Objects[i].xk[1];
						Point_Buffer.Object_History[n].NumofPoint = 1;
						Point_Buffer.Object_History[n].PLA = 0;
						Point_Buffer.Object_History[n].distance = sqrt(pow(Point_Buffer.Object_History[n].Position[0][0], 2.0f) + pow(Point_Buffer.Object_History[n].Position[1][0], 2.0f));
						Point_Buffer.Object_History[n].Selected = 0;
						Point_Buffer.Object_History[n].timestamp_latest_renew = timestamp_now;
						++Point_Buffer.n;
					}
					else
					{
						int min_pointnumber = MAX_POINTNUMBER;
						int Index_Replace = 0;
						for (int j = 0; j < n; j++)
						{
							if (Point_Buffer.Object_History[j].Detection > 0) continue;
							if (Point_Buffer.Object_History[j].NumofPoint < min_pointnumber)
							{
								min_pointnumber = Point_Buffer.Object_History[j].NumofPoint;
								Index_Replace = j;
							}
						}
						Point_Buffer.Object_History[Index_Replace].Detection = 1;
						Point_Buffer.Object_History[Index_Replace].ID_Mob = fList_Fusion.Objects[i].ID_Mob;
						Point_Buffer.Object_History[Index_Replace].ID_Rad = fList_Fusion.Objects[i].ID_Rad;
						Point_Buffer.Object_History[Index_Replace].Position[0][0] = fList_Fusion.Objects[i].xk[0];
						Point_Buffer.Object_History[Index_Replace].Position[1][0] = fList_Fusion.Objects[i].xk[1];
						Point_Buffer.Object_History[Index_Replace].NumofPoint = 1;
						Point_Buffer.Object_History[Index_Replace].PLA = 0;
						Point_Buffer.Object_History[Index_Replace].distance = sqrt(pow(Point_Buffer.Object_History[n].Position[0][0], 2.0f) + pow(Point_Buffer.Object_History[n].Position[1][0], 2.0f));
						Point_Buffer.Object_History[Index_Replace].Selected = 0;
						Point_Buffer.Object_History[Index_Replace].timestamp_latest_renew = timestamp_now;
					}
				}
			}
		}
	}
}


// ----------------------------------------------------------------------------------- //
//  In this subfunction....                                                            //
//                                                                                     //
//                                                                                     //
// ----------------------------------------------------------------------------------- //
void Fit_curve()
{
	float count_nonaim = 0;
	float cur_nonaim = 0;
	float cur_aim = NAN;
	Fahrdynamik_wr_mutex.lock();
	float v = sqrt(pow(Fahrdynamik.Long_Vel, 2.0f) + pow(Fahrdynamik.Lat_Vel, 2.0f));
	float cur_vehicle = Fahrdynamik.Cur_Dyn;     //to be readed from MAB
	Fahrdynamik_wr_mutex.unlock();

	for (int i = 0; i < Point_Buffer.n; ++i)
	{
		if (Point_Buffer.Object_History[i].NumofPoint > 5)
		{
			float sum_of_d_2p = 0;   // Summe of square of x
			float sum_of_d_4p = 0;   // Summe of 4th power of x
			float sum_of_y = 0;      // Summe of y
			float sum_of_y_d_2p = 0; // Summe of square of x multiply y
			float n = 0;

			for (int j = 0; j < Point_Buffer.Object_History[i].NumofPoint; ++j)
			{
				++n;
				sum_of_d_2p += pow(Point_Buffer.Object_History[i].Position[0][j], 2.0f) + pow(Point_Buffer.Object_History[i].Position[1][j], 2.0f);
				sum_of_d_4p += pow(pow(Point_Buffer.Object_History[i].Position[0][j], 2.0f) + pow(Point_Buffer.Object_History[i].Position[1][j], 2.0f), 2.0f);
				sum_of_y += Point_Buffer.Object_History[i].Position[1][j];
				sum_of_y_d_2p += Point_Buffer.Object_History[i].Position[1][j] * (pow(Point_Buffer.Object_History[i].Position[0][j], 2.0f) + pow(Point_Buffer.Object_History[i].Position[1][j], 2.0f));
			}
			Eigen::Matrix2f A;
			Eigen::Vector2f B;
			A << sum_of_d_4p, sum_of_d_2p,
				sum_of_d_2p, n;
			B << sum_of_y_d_2p,
				sum_of_y;
			Eigen::Matrix2f C;
			C << 1, 0,
				0, 0;
			Eigen::Vector2f cur = ((A + ridge_regression_lameda * C).inverse() * B);
			Point_Buffer.Object_History[i].curvature[0] = cur[0];
			Point_Buffer.Object_History[i].curvature[1] = cur[1];
			if (std::abs(Point_Buffer.Object_History[i].curvature[0]) > curve_fit_threshold_max_cur) Point_Buffer.Object_History[i].curvature[0] = INFINITY;
			if (Point_Buffer.Object_History[i].Selected == 1)
			{
				cur_aim = Point_Buffer.Object_History[i].curvature[0];
			}
			else
			{
				if (!isnan(Point_Buffer.Object_History[i].curvature[0]) && !isinf(Point_Buffer.Object_History[i].curvature[0]))
				{
					double distance_max = 0;
					for (int k = 0; k < Point_Buffer.Object_History[i].NumofPoint; ++k)
					{
						distance_max = max(distance_max, sqrt(pow(double(Point_Buffer.Object_History[i].Position[0][k]), 2) + pow(double(Point_Buffer.Object_History[i].Position[1][k]), 2)));
					}
					if (distance_max > 10)
					{
						cur_nonaim += Point_Buffer.Object_History[i].curvature[0];
						++count_nonaim;
					}
				}
			}
		}
	}

	// Then the curvature of the perdicted trajectory will be coculated with weight.
	weighted_ego_curve(cur_aim, cur_nonaim, count_nonaim, cur_vehicle, v);
}

// ----------------------------------------------------------------------------------- //
//  In this subfunction....                                                            //
//                                                                                     //
//                                                                                     //
// ----------------------------------------------------------------------------------- //
void weighted_ego_curve(float cur_aim, float cur_nonaim, float count_nonaim, float cur_vehicle, float v)
{
	float cur_tem[41];
	memcpy(cur_tem, cur_ego, sizeof(cur_ego));

	float cur_road;
	if (!isnan(cur_aim) && !isinf(cur_aim))
	{
		if (count_nonaim != 0)
		{
			cur_nonaim = cur_nonaim / count_nonaim;
			cur_road = weight_a_Str_von_Aim * cur_aim + (1 - weight_a_Str_von_Aim) * cur_nonaim;
			curve_Interpolation(cur_road, cur_vehicle, v);
		}
		else
		{
			cur_road = cur_aim;
			curve_Interpolation(cur_road, cur_vehicle, v);
		}
	}
	else
	{
		if (count_nonaim != 0)
		{
			cur_nonaim = cur_nonaim / count_nonaim;
			cur_road = cur_nonaim;
			curve_Interpolation(cur_road, cur_vehicle, v);
		}
		else
		{
			memset(cur_ego, cur_vehicle, sizeof(cur_ego));
		}
	}
}

// ----------------------------------------------------------------------------------- //
//  In this subfunction....                                                            //
//                                                                                     //
//                                                                                     //
// ----------------------------------------------------------------------------------- //
void curve_Interpolation(float cur_road, float cur_vehicle)
{
	for (int i = 0; i < 51; i += 5)
	{
		cur_ego[(int)(i / 5)] = cur_vehicle;
	}
	for (int i = 55; i < 151; i += 5)
	{
		cur_ego[(int)(i / 5)] = (150.0 - (float)i) / 100.0 * cur_vehicle + ((float)i - 50.0) / 100.0 * cur_road;
	}
	for (int i = 155; i < 206; i += 5)
	{
		cur_ego[(int)(i / 5)] = cur_road;
	}
}


// ----------------------------------------------------------------------------------- //
//  In this subfunction....                                                            //
//                                                                                     //
//                                                                                     //
// ----------------------------------------------------------------------------------- //
void SPW_distribution()
{
	for (int i = 0; i < 41; ++i)
	{
		y_feld(8, i) = d_feld(0, i) * d_feld(0, i) * cur_ego[i];
		x_feld(8, i) = sqrt(pow(d_feld(0, i), 2.0) - pow(y_feld(8, i), 2.0));
	}

	for (int i = 0; i < 8; i++)
	{
		Eigen::ArrayXXf width(1, 41);
		width = width_coefficient((Eigen::Index)7 - i, 0) * d_feld.square() + width_coefficient((Eigen::Index)7 - i, 1) * d_feld + width_coefficient((Eigen::Index)7 - i, 2);
		for (int j = 0; j < 41; ++j)
		{
			if (cur_ego[j] == 0)
			{
				x_feld(i, j) = x_feld(8, j);
				y_feld(i, j) = y_feld(8, j) + width(0, j);
				x_feld((Eigen::Index)16 - i, j) = x_feld(8, j);
				y_feld((Eigen::Index)16 - i, j) = y_feld(8, j) - width(0, j);
			}
			else
			{
				if (cur_ego[j] > 0.0)
				{
					x_feld(i, j) = x_feld(8, j) - width(0, j) * x_feld(8, j) / sqrt(pow(x_feld(8, j), 2) + pow((2.0 / cur_ego[j] - y_feld(8, j)), 2));
					y_feld(i, j) = y_feld(8, j) + width(0, j) * (2.0 / cur_ego[j] - y_feld(8, j)) / sqrt(pow(x_feld(8, j), 2) + pow((2.0 / cur_ego[j] - y_feld(8, j)), 2));
					x_feld((Eigen::Index)16 - i, j) = x_feld(8, j) + width(0, j) * x_feld(8, j) / sqrt(pow(x_feld(8, j), 2) + pow((2.0 / cur_ego[j] - y_feld(8, j)), 2));
					y_feld((Eigen::Index)16 - i, j) = y_feld(8, j) - width(0, j) * (2.0 / cur_ego[j] - y_feld(8, j)) / sqrt(pow(x_feld(8, j), 2) + pow((2.0 / cur_ego[j] - y_feld(8, j)), 2));
				}
				else
				{
					x_feld(i, j) = x_feld(8, j) + width(0, j) * x_feld(8, j) / sqrt(pow(x_feld(8, j), 2) + pow((2.0 / cur_ego[j] - y_feld(8, j)), 2));
					y_feld(i, j) = y_feld(8, j) - width(0, j) * (2.0 / cur_ego[j] - y_feld(8, j)) / sqrt(pow(x_feld(8, j), 2) + pow((2.0 / cur_ego[j] - y_feld(8, j)), 2));
					x_feld((Eigen::Index)16 - i, j) = x_feld(8, j) - width(0, j) * x_feld(8, j) / sqrt(pow(x_feld(8, j), 2) + pow((2.0 / cur_ego[j] - y_feld(8, j)), 2));
					y_feld((Eigen::Index)16 - i, j) = y_feld(8, j) + width(0, j) * (2.0 / cur_ego[j] - y_feld(8, j)) / sqrt(pow(x_feld(8, j), 2) + pow((2.0 / cur_ego[j] - y_feld(8, j)), 2));
				}
			}
		}
	}
}


// ----------------------------------------------------------------------------------- //
//  In this subfunction....                                                            //
//                                                                                     //
//                                                                                     //
// ----------------------------------------------------------------------------------- //
void PLA_update()
{
	float delta_t = Point_Buffer.timestamp - Point_Buffer.last_timestamp;

	for (int i = 0; i < Point_Buffer.n; ++i)
	{
		if (Point_Buffer.Object_History[i].Detection == 1)
		{
			float x = Point_Buffer.Object_History[i].Position[0][Point_Buffer.Object_History[i].NumofPoint - 1];
			float y = Point_Buffer.Object_History[i].Position[1][Point_Buffer.Object_History[i].NumofPoint - 1];
			Eigen::ArrayXXf y_stufe(1, 17);
			for (int j = 0; j < 17; ++j)
			{
				y_stufe(0, j) = interp1(x_feld.row(j), y_feld.row(j), x);
			}
			if (y <= y_stufe(0, 0) && y >= y_stufe(0, 16))
			{
				for (int j = 0; j < ((17 - 1) / 2 + 1); ++j)
				{
					if ((y <= y_stufe(0, j) && y >= y_stufe(0, j + 1)) || (y >= y_stufe(0, 16 - j) && y < y_stufe(0, 16 - j - 1)))
					{
						Point_Buffer.Object_History[i].SPW = SPW_feld(0, j);
						break;
					}
				}
			}
			else
			{
				Point_Buffer.Object_History[i].SPW = -1.0f;
			}
		}
		else
		{
			Point_Buffer.Object_History[i].SPW = -0.5f;
		}
		Point_Buffer.Object_History[i].PLA += Point_Buffer.Object_History[i].SPW * delta_t;
		Point_Buffer.Object_History[i].PLA = std::min(1.0f, Point_Buffer.Object_History[i].PLA);
		Point_Buffer.Object_History[i].PLA = std::max(0.0f, Point_Buffer.Object_History[i].PLA);
		if (Point_Buffer.Object_History[i].ID_Mob == Aim_Info.Info.ID_Mob && Point_Buffer.Object_History[i].ID_Rad == Aim_Info.Info.ID_Rad)
		{
			Aim_Info.PLA = Point_Buffer.Object_History[i].PLA;
			Aim_Info.SPW = Point_Buffer.Object_History[i].SPW;
		}
	}

}


// ----------------------------------------------------------------------------------- //
//  In this subfunction....                                                            //
//                                                                                     //
//                                                                                     //
// ----------------------------------------------------------------------------------- //
float   interp1(Eigen::ArrayXXf x, Eigen::ArrayXXf y, float x_0)
{
	int k = (int)x.cols() - 1;
	// with the loop we test, until which point is the series monotonous.
	for (UINT8 i = 0; i < (x.cols() - 1); ++i)
	{
		if ((x(0, i) - x(0, i + 1)) >= 0)
		{
			k = (int)i;
			break;
		}
	}

	//conculate the interpolation, linear, exter
	if ((x_0 >= x(0, 0)) && (x_0 <= x(0, k)))
	{
		for (UINT8 i = 0; i < k - 1; ++i)
		{
			if (x_0 >= x(0, i) && x_0 <= x(0, i + 1))
			{
				return ((y(0, i + 1) - y(0, i)) * (x_0 - x(0, i)) / (x(0, i + 1) - x(0, i)) + y(0, i));
			}
		}
		return NAN;
	}
	else
	{
		if (x_0 > x(0, k))
		{
			return ((y(0, k) - y(0, k - 1)) * (x_0 - x(0, k)) / (x(0, k) - x(0, k - 1)) + y(0, k));
		}
		else
		{
			return (-(y(0, 1) - y(0, 0)) * (x(0, 0) - x_0) / (x(0, 1) - x(0, 0)) + y(0, 0));
		}
	}
}


// ----------------------------------------------------------------------------------- //
//  In this subfunction....                                                            //
//                                                                                     //
//                                                                                     //
// ----------------------------------------------------------------------------------- //
void Select_with_PLA()
{
	// Listing all the event, that activate the process selecting new Aim.
	bool tic1 = (Aim_Info.Exist_of_Aim == 0);
	bool tic2 = ((Aim_Info.Exist_of_Aim != 0) && Aim_Info.PLA < Threashold_Aim_min_SPW);

	float PLA_max = 0;
	for (int i = 0; i < Point_Buffer.n; ++i)
	{
		if (!(Point_Buffer.Object_History[i].ID_Mob == Aim_Info.Info.ID_Mob && Point_Buffer.Object_History[i].ID_Rad == Aim_Info.Info.ID_Rad))
		{
			PLA_max = max(PLA_max, Point_Buffer.Object_History[i].PLA);
		}
	}
	bool tic3 = PLA_max >= Aim_Info.PLA;

	int count_PLA = 0;
	for (int i = 0; i < Point_Buffer.n; ++i)
	{
		if (Point_Buffer.Object_History[i].PLA == 1)
		{
			++count_PLA;
		}
	}
	bool tic4 = count_PLA > 1;

	// Reset the bool selected in Point_Buffer
	for (int i = 0; i < Point_Buffer.n; ++i)
	{
		Point_Buffer.Object_History[i].Selected = 0;
	}

	// If any of rhe events is activated, try to renew the choice of Aim.
	if (tic1 || tic2 || tic3 || tic4)
	{
		for (int i = 0; i < Point_Buffer.n; ++i)
		{
			PLA_max = max(PLA_max, Point_Buffer.Object_History[i].PLA);
		}
		if (PLA_max >= Threashold_No_Aim_min_SPW)
		{
			float distance = 200;
			for (int i = 0; i < Point_Buffer.n; ++i)
			{
				if (PLA_max == Point_Buffer.Object_History[i].PLA)
				{
					if (Point_Buffer.Object_History[i].distance < distance)
					{
						Renew_Aim_Info(&Point_Buffer.Object_History[i]);
					}
				}
			}
		}
		else
		{
			if (Aim_Info.Exist_of_Aim == 1 && Aim_Info.PLA > Threashold_Aim_min_SPW)
			{
				for (int i = 0; i < Point_Buffer.n; ++i)
				{
					if (Point_Buffer.Object_History[i].NumofPoint > 3 && Point_Buffer.Object_History[i].ID_Mob == Aim_Info.Info.ID_Mob && Point_Buffer.Object_History[i].ID_Rad == Aim_Info.Info.ID_Rad)
					{
						Renew_Aim_Info(&Point_Buffer.Object_History[i]);
						break;
					}
				}
			}
			else
			{
				Aim_Info.Exist_of_Aim = 0;
				Aim_Info.PLA = 0.0f;
				Aim_Info.SPW = 0.0f;
				Aim_Info.Info.ID_Mob = 0;
				Aim_Info.Info.ID_Rad = 0;
			}
		}
	}
	else
	{
		if (Aim_Info.Exist_of_Aim == 1 && Aim_Info.PLA > Threashold_Aim_min_SPW)
		{
			for (int i = 0; i < Point_Buffer.n; ++i)
			{
				if (Point_Buffer.Object_History[i].ID_Mob == Aim_Info.Info.ID_Mob && Point_Buffer.Object_History[i].ID_Rad == Aim_Info.Info.ID_Rad) //Point_Buffer.Object_History[i].NumofPoint > 3 && 
				{
					Renew_Aim_Info(&Point_Buffer.Object_History[i]);
					break;
				}
			}
		}
		else
		{
			Aim_Info.Exist_of_Aim = 0;
			Aim_Info.PLA = 0.0f;
			Aim_Info.SPW = 0.0f;
			Aim_Info.Info.ID_Mob = 0;
			Aim_Info.Info.ID_Rad = 0;
		}
	}
	Aim_Info.Detection = 0;
	for (int i = 0; i <= fList_Fusion.n; ++i)
	{
		if ((fList_Fusion.Objects[i].ID_Mob < 255 && 0 < fList_Fusion.Objects[i].ID_Mob) || (fList_Fusion.Objects[i].ID_Rad < 255 && 0 < fList_Fusion.Objects[i].ID_Rad))
		{
			if (Aim_Info.Info.ID_Mob == fList_Fusion.Objects[i].ID_Mob && Aim_Info.Info.ID_Rad == fList_Fusion.Objects[i].ID_Rad) Aim_Info.Detection = 1;
		}
	}
}


// ----------------------------------------------------------------------------------- //
//  In this subfunction....                                                            //
//                                                                                     //
//                                                                                     //
// ----------------------------------------------------------------------------------- //
void Renew_Aim_Info(Objecthistory* pAim_Object_History)
{
	Aim_Info.Exist_of_Aim = 1;
	Aim_Info.PLA = (*pAim_Object_History).PLA;
	Aim_Info.SPW = (*pAim_Object_History).SPW;
	for (int j = 0; j < fList_Fusion.n; ++j)
	{
		if ((*pAim_Object_History).ID_Mob == fList_Fusion.Objects[j].ID_Mob && (*pAim_Object_History).ID_Rad == fList_Fusion.Objects[j].ID_Rad)
		{
			(*pAim_Object_History).Selected = 1;
			Aim_Info.Info.ID_Rad = fList_Fusion.Objects[j].ID_Rad;
			Aim_Info.Info.ID_Mob = fList_Fusion.Objects[j].ID_Mob;
			Aim_Info.Info.Class = fList_Fusion.Objects[j].Class;
			Aim_Info.Info.Lane = fList_Fusion.Objects[j].Lane;
			Aim_Info.Info.PosLongX = fList_Fusion.Objects[j].xk(0);
			Aim_Info.Info.PosLatY = fList_Fusion.Objects[j].xk(1);
			Aim_Info.Info.VrelLongX = fList_Fusion.Objects[j].xk(2);
			Aim_Info.Info.VrelLatY = fList_Fusion.Objects[j].xk(3);
			Aim_Info.Info.Width = fList_Fusion.Objects[j].Width;
			Aim_Info.Info.Length = fList_Fusion.Objects[j].Length;
			Aim_Info.Info.ProbOfExist = fList_Fusion.Objects[j].ProbOfExist;
			Aim_Info.Info.RCS = fList_Fusion.Objects[j].RCS;
		}
	}
}

// In this thread the massage from MAB will be recived and processed///////////

void ReceiveThread_MAB(void* Param)
{
	UNREFERENCED_PARAMETER(Param);
	int UDP_Recivcount = 0;
	int len = sizeof(EBPCAddr_Port1402);
	printf("Embedded PC starts to wait for the information of vehicle dynamik. \n");

	while (true) {
		UDP_Recivcount = recvfrom(UDPSocket_from_MAB, buf_MAB_Receive, sizeof(buf_MAB_Receive), 0, (struct sockaddr*) & EBPCAddr_Port1402, (int *)&len);
		if (UDP_Recivcount < 0)
		{
			printf("Error in UDPSocket_from_MAB reciving the message from MAB. \n");
			continue;
		}
		else {
			/*std::lock_guard<std::mutex> lock(Fahrdynamik.Fahrdynamik_mutex);*/
			Fahrdynamik_wr_mutex.lock();
			memcpy(&Fahrdynamik.Long_Vel, buf_MAB_Receive, 4);
			memcpy(&Fahrdynamik.Lat_Vel, buf_MAB_Receive+4, 4);
			memcpy(&Fahrdynamik.Yaw_Rate, buf_MAB_Receive+8, 4);
			memcpy(&Fahrdynamik.Cur_Dyn, buf_MAB_Receive + 12, 4);
			//printf("Fahrdynakik: Vx = %f, Vy = %f, omega = %f, Cur = %f \r", Fahrdynamik.Long_Vel, Fahrdynamik.Lat_Vel, Fahrdynamik.Yaw_Rate, Fahrdynamik.Cur_Dyn);
			Fahrdynamik_wr_mutex.unlock();
		}
	} 
}
void Show_the_Frame(cv::Mat frame)
{
	std::string str;
	str = "t_Rad: " + std::to_string(fList_Fusion.timestamp_Rad);
	cv::putText(frame, str, cv::Point(0, 430), cv::FONT_HERSHEY_SIMPLEX, 0.5, teal);
	str = "t_Mob: " + std::to_string(fList_Fusion.timestamp_Mob);
	cv::putText(frame, str, cv::Point(0, 445), cv::FONT_HERSHEY_SIMPLEX, 0.5, teal);
	str = "curvaure: " + std::to_string(cur_ego[0]);
	cv::putText(frame, str, cv::Point(0, 415), cv::FONT_HERSHEY_SIMPLEX, 0.5, teal);

	cv::imshow("video", frame);
}

void Show_the_Topview(cv::Mat& frame_TV, cv::Mat& frame_ObjectInfo)
{
	frame_TV = cv::Mat(imgHight_TV, imgWidth_TV, CV_8UC3, cv::Scalar(0, 0, 0)); // Weiss
	frame_ObjectInfo = cv::Mat(imgHight_TV, imgWidth_TV, CV_8UC3, cv::Scalar(0, 0, 0));
	//cv::Mat::ones(cv::Size(imgWidth_TV, imgHight_TV), CV_8SC3) * 255; // Schwarz

	// Zeichnen alle Punkte der Objekte, die Prognose der Objekttrajektorie, Schreib die Info der Objekte auf linker Seite
	for (int i = 0; i < Point_Buffer.n; ++i)
	{
		if (Point_Buffer.Object_History[i].NumofPoint > 1)
		{
			for (int j = 1; j < Point_Buffer.Object_History[i].NumofPoint; ++j)
			{
				cv::Point pos = trans_TV(Point_Buffer.Object_History[i].Position[0][j], Point_Buffer.Object_History[i].Position[1][j]);
				cv::Point pos_last = trans_TV(Point_Buffer.Object_History[i].Position[0][j - 1], Point_Buffer.Object_History[i].Position[1][j - 1]);
				cv::line(frame_TV, pos, pos_last, teal, 1);
			}
			std::string str = "ID-Rad/Mob:" + std::to_string(Point_Buffer.Object_History[i].ID_Rad) + '/' + std::to_string(Point_Buffer.Object_History[i].ID_Mob)
				+ " " + std::to_string(Point_Buffer.Object_History[i].NumofPoint) + " Points, Cur:   " + std::to_string(Point_Buffer.Object_History[i].curvature[0])
				+ ", PLA: " + std::to_string(Point_Buffer.Object_History[i].PLA);
			cv::putText(frame_ObjectInfo, str, cv::Point(15, 15 + 15 * i), cv::FONT_HERSHEY_SIMPLEX, 0.4, white);

			if ((Point_Buffer.Object_History[i].NumofPoint > 5) && !isnan(Point_Buffer.Object_History[i].curvature[0]) && !isinf(Point_Buffer.Object_History[i].curvature[0]))
			{
				for (int j = 0; j < 100; ++j)
				{
					cv::Point pos = trans_TV(2 * j, Point_Buffer.Object_History[i].curvature[0] * 4 * j * j + Point_Buffer.Object_History[i].curvature[1]);
					cv::Point pos_next = trans_TV(2 * j + 1, Point_Buffer.Object_History[i].curvature[0] * (2 * j + 1) * (2 * j + 1) + Point_Buffer.Object_History[i].curvature[1]);
					cv::line(frame_TV, pos, pos_next, magenta, 1);
				}
			}
		}
	}

	// Zeichne die Aktuelle Lage der Objekte 
	for (int i = 0; i < fList_Fusion.n; ++i)
	{
		if (fList_Fusion.Objects[i].ID_Mob < 255 || fList_Fusion.Objects[i].ID_Rad < 255)
		{
			if (fList_Fusion.Objects[i].xk[0] > 0)
			{
				cv::Point pos = trans_TV(fList_Fusion.Objects[i].xk[0], fList_Fusion.Objects[i].xk[1]);
				cv::circle(frame_TV, pos, 5, teal, -1);
				std::string str = "ID Rad/Mob: " + std::to_string(fList_Fusion.Objects[i].ID_Rad) + "/" + std::to_string(fList_Fusion.Objects[i].ID_Mob);
				pos.x += 5;
				pos.y -= 5;
				cv::putText(frame_TV, str, pos, cv::FONT_HERSHEY_SIMPLEX, 0.5, teal);
			}
		}
	}

	if (Aim_Info.Exist_of_Aim && Aim_Info.Detection)
	{
		cv::Point pos = trans_TV(Aim_Info.Info.PosLongX, Aim_Info.Info.PosLatY);
		cv::circle(frame_TV, pos, 5, red, -1);
		std::string str = "ID Rad/Mob: " + std::to_string(Aim_Info.Info.ID_Rad) + "/" + std::to_string(Aim_Info.Info.ID_Mob);
		pos.x += 5;
		pos.y -= 5;
		cv::putText(frame_TV, str, pos, cv::FONT_HERSHEY_SIMPLEX, 0.5, red, 2);
		str = "PLA/SPW : " + std::to_string(Aim_Info.PLA) + "/" + std::to_string(Aim_Info.SPW);
		pos.y -= 15;
		cv::putText(frame_TV, str, pos, cv::FONT_HERSHEY_SIMPLEX, 0.5, red, 2);
	}

	cv::Point pts_l[41];
	cv::Point pts_r[41];
	int npt[] = { 41 };
	for (int i = 0; i < 8; ++i)
	{
		for (int j = 0; j < 41; ++j)
		{
			float x_l = x_feld(i, j);
			float x_r = x_feld(16 - i, j);
			float y_l = y_feld(i, j);
			float y_r = y_feld(16 - i, j);
			pts_l[int(j)] = trans_TV(x_l, y_l);
			pts_r[int(j)] = trans_TV(x_r, y_r);
		}
		cv::Point* ppts_l[] = { pts_l };
		cv::Point* ppts_r[] = { pts_r };
		cv::polylines(frame_TV, ppts_l, npt, 1, false, yellow, 1, 8);
		cv::polylines(frame_TV, ppts_r, npt, 1, false, yellow, 1, 8);
	}

	cv::imshow("Topview", frame_TV);
	cv::imshow("ObjectInfo", frame_ObjectInfo);
	cv::waitKey(1);
}


cv::Point trans_TV(float x, float y)
{
	int y_TV = int(imgHight_TV * (1 - x / VisionFar) - 50);
	int x_TV = int(-imgWidth_TV * y / VisionWidth + imgWidth_TV / 2);
	return cv::Point(x_TV, y_TV);
}