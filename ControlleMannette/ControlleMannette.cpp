// ControlleMannette.cpp : Ce fichier contient la fonction 'main'. L'exécution du programme commence et se termine à cet endroit.
//

#include "pch.h"
#include <iostream>
#include "headers/openvr.h"
#include <string>
#include <WS2tcpip.h>
#include "headers/cylinder.h"
#include "headers/cpTime.h"
#include "headers/Matrices.h"
#include "headers/LighthouseTracking.h"
#include <sstream>
//#include "headers/LighthouseTracking.cpp"

#pragma comment(lib, "ws2_32.lib")

using namespace vr;

IVRSystem* vr_pointer = NULL;
IVRSystem* vr_context;

void printPositionalData();
void RunProcedure2(SOCKET socket);
void RunProcedure1();
void iterateAssignIds();

std::string Message;
char buf[4096];
int nDevice = 0;

int sHand = 0;
int deviceId_Left = -1;
int deviceId_Right = -1;
int idtrigger = 0;
int idpad = 0;

void main()
{
	int controllerIndex; //The index of the controllers[] array that corresponds with the controller that had a buttonEvent
	EVRInitError eError = VRInitError_None;
	vr_pointer = VR_Init(&eError, VRApplication_Background);

	std::string ipAddress = "86.XXX.XXX.XXX";// IP Address Public of Paul server
	int port = 2600;//54000;						// Listening port # on the server

	// Initialize WinSock
	WSAData data;
	WORD ver = MAKEWORD(2, 2);
	int wsResult = WSAStartup(ver, &data);
	if (wsResult != 0)
	{
		std::cout << "Can't start Winsock, Err #" << wsResult << std::endl;
		return;
	}

	// Create socket
	SOCKET sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (sock == INVALID_SOCKET)
	{
		std::cout << "Can't create socket, Err #" << WSAGetLastError() << std::endl;
		WSACleanup();
		return;
	}

	// Fill in a hint structure
	sockaddr_in hint;
	hint.sin_family = AF_INET;
	hint.sin_port = htons(port);
	inet_pton(AF_INET, ipAddress.c_str(), &hint.sin_addr);

	// Connect to server
	int connResult = connect(sock, (sockaddr*)&hint, sizeof(hint));
	if (connResult == SOCKET_ERROR)
	{
		std::cout << "Can't connect to server, Err #" << WSAGetLastError() << std::endl;
		closesocket(sock);
		WSACleanup();
		return;
	}

	if (eError != VRInitError_None)
	{
		vr_pointer = NULL;
		printf("Unable to init VR runtime: %s \n",
			VR_GetVRInitErrorAsEnglishDescription(eError));
		exit(EXIT_FAILURE);
	}
	std::cout << "Projet Drone Terrestre\n";

	Message = "";
	iterateAssignIds();
	while (1) {
		RunProcedure2(sock);
	}
}
/////////////////////////////////////////////////
// Exécuter le programme : Ctrl+F5 ou menu Déboguer > Exécuter sans débogage
// Déboguer le programme : F5 ou menu Déboguer > Démarrer le débogage
// Conseils pour bien démarrer : 
//   1. Utilisez la fenêtre Explorateur de solutions pour ajouter des fichiers et les gérer.
//   2. Utilisez la fenêtre Team Explorer pour vous connecter au contrôle de code source.
//   3. Utilisez la fenêtre Sortie pour voir la sortie de la génération et d'autres messages.
//   4. Utilisez la fenêtre Liste d'erreurs pour voir les erreurs.
//   5. Accédez à Projet > Ajouter un nouvel élément pour créer des fichiers de code, ou à Projet > Ajouter un élément existant pour ajouter des fichiers de code existants au projet.
//   6. Pour rouvrir ce projet plus tard, accédez à Fichier > Ouvrir > Projet et sélectionnez le fichier .sln.

// Purpose: Returns true if the action is active and had a rising edge
void RunProcedure1() {
	VREvent_t event1;
	if (vr_pointer->PollNextEvent(&event1, sizeof(event1)))
	{
		switch (event1.eventType)
		{
		case VREvent_TrackedDeviceActivated:
			printf("EVENT (OpenVR) Device : %d attached\n",
				event1.trackedDeviceIndex);
			break;
			//and so on, can test for any amount of vr events
		default:
			printf("EVENT--(OpenVR) Event: %d\n", event1.eventType);
		}
	}
}

void printDevicePositionalData(const char* deviceName, vr::HmdMatrix34_t posMatrix, vr::HmdVector3_t position, vr::HmdQuaternion_t quaternion)
{
	LARGE_INTEGER qpc; // Query Performance Counter for Acquiring high-resolution time stamps.
					   // From MSDN: "QPC is typically the best method to use to time-stamp events and 
					   // measure small time intervals that occur on the same system or virtual machine.
	QueryPerformanceCounter(&qpc);
	// Print position and quaternion (rotation).
	//std::cout << qpc.QuadPart <<"/"<< deviceName << "/" << position.v[0] << "/" << position.v[1] << "/" << position.v[2] << "/" << quaternion.w << "/" << quaternion.x << "/" << quaternion.y << "/" << quaternion.z;
	//std::cout << qpc.QuadPart << "/" << deviceName  << std::endl;
	std::cout << "Position/" << position.v[0] << "/" << position.v[1] << "/" << position.v[2] << std::endl;
	std::cout << "Quaternion / W = " << quaternion.w << "/ X = " << quaternion.x << "/ Y = " << quaternion.y << "/ Z = " << quaternion.z << std::endl;

}

HmdQuaternion_t GetRotation(vr::HmdMatrix34_t matrix) {
	vr::HmdQuaternion_t q;

	q.w = sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
	q.x = sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
	q.y = sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
	q.z = sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;
	q.x = copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
	q.y = copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
	q.z = copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);
	return q;
}
//-----------------------------------------------------------------------------
// Purpose: Extracts position (x,y,z).
// from: https://github.com/Omnifinity/OpenVR-Tracking-Example/blob/master/HTC%20Lighthouse%20Tracking%20Example/LighthouseTracking.cpp
//-----------------------------------------------------------------------------

vr::HmdVector3_t GetPosition(vr::HmdMatrix34_t matrix) {
	vr::HmdVector3_t vector;

	vector.v[0] = matrix.m[0][3];
	vector.v[1] = matrix.m[1][3];
	vector.v[2] = matrix.m[2][3];

	return vector;
}

IVRSystem* m_pHMD;
void printPositionalData()
{
	// Process SteamVR device states
	for (TrackedDeviceIndex_t unDevice = 0; unDevice < vr::k_unMaxTrackedDeviceCount; unDevice++)
	{
		//if (!m_pHMD->IsTrackedDeviceConnected(unDevice))
			//continue;

		VRControllerState_t state;
		//if (m_pHMD->GetControllerState(unDevice, &state, sizeof(state)))
		//{
		//m_pHMD->GetControllerState(unDevice, &state, sizeof(state));
		TrackedDevicePose_t trackedDevicePose;
		TrackedDevicePose_t trackedControllerPose;
		VRControllerState_t controllerState;
		HmdMatrix34_t poseMatrix;
		HmdVector3_t position;
		HmdQuaternion_t quaternion;
		ETrackedDeviceClass trackedDeviceClass = VRSystem()->GetTrackedDeviceClass(unDevice);

		switch (trackedDeviceClass) {
		case ETrackedDeviceClass::TrackedDeviceClass_HMD:
			VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, &trackedDevicePose, 1);
			// print positiona data for the HMD.
			poseMatrix = trackedDevicePose.mDeviceToAbsoluteTracking; // This matrix contains all positional and rotational data.
			position = GetPosition(trackedDevicePose.mDeviceToAbsoluteTracking);
			quaternion = GetRotation(trackedDevicePose.mDeviceToAbsoluteTracking);

			LARGE_INTEGER qpc; // Query Performance Counter for Acquiring high-resolution time stamps.
				   // From MSDN: "QPC is typically the best method to use to time-stamp events and 
				   // measure small time intervals that occur on the same system or virtual machine.
			QueryPerformanceCounter(&qpc);

			//printDevicePositionalData("HMD", poseMatrix, position, quaternion);
			//std::cout << qpc.QuadPart << "HMD" << position.v[0] << position.v[1] << position.v[2] << quaternion.w << quaternion.x << quaternion.y << quaternion.z;

			break;

		case ETrackedDeviceClass::TrackedDeviceClass_GenericTracker:
			VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, &trackedDevicePose, 1);
			// print positiona data for a general vive tracker.
			break;

		case ETrackedDeviceClass::TrackedDeviceClass_Controller:
			VRSystem()->GetControllerStateWithPose(vr::TrackingUniverseStanding, unDevice, &controllerState,
				sizeof(controllerState), &trackedControllerPose);
			poseMatrix = trackedControllerPose.mDeviceToAbsoluteTracking; // This matrix contains all positional and rotational data.
			position = GetPosition(trackedControllerPose.mDeviceToAbsoluteTracking);
			quaternion = GetRotation(trackedControllerPose.mDeviceToAbsoluteTracking);

			auto trackedControllerRole = VRSystem()->GetControllerRoleForTrackedDeviceIndex(unDevice);
			std::string whichHand = "";
			switch (trackedControllerRole)
			{
			case TrackedControllerRole_Invalid:
				// invalid
				break;

			case TrackedControllerRole_LeftHand:
			case TrackedControllerRole_RightHand:
				printDevicePositionalData(whichHand.c_str(), poseMatrix, position, quaternion);
				break;
			}
			break;
		}
	}
}

InitFlags flags;
int hmdDeviceId = -1;
LighthouseTracking::_ControllerData ControllerData;
LighthouseTracking::_ControllerData controllers[2];

void getid()
{
	for (int x = 0; x < k_unControllerStateAxisCount; x++)
	{
		int prop = vr_pointer->GetInt32TrackedDeviceProperty(ControllerData.deviceId,
			(ETrackedDeviceProperty)(Prop_Axis0Type_Int32 + x));

		if (prop == k_eControllerAxis_Trigger) {
			ControllerData.idtrigger = x;
			idtrigger = x;
		}
		else if (prop == k_eControllerAxis_TrackPad) {
			ControllerData.idpad = x;
			idpad = x;
		}
	}
}

void iterateAssignIds()
{
	controllers[0].deviceId = -1;
	controllers[1].deviceId = -1;

	int numTrackersInitialized = 0;
	int numControllersInitialized = 0;

	for (unsigned int i = 0; i < k_unMaxTrackedDeviceCount; i++)  // Iterates across all of the potential device indicies
	{
		if (!vr_pointer->IsTrackedDeviceConnected(i))
			continue; //Doesn't use the id if the device isn't connected

		//vr_pointer points to the VRSystem that was in init'ed in the constructor.
		ETrackedDeviceClass trackedDeviceClass = vr_pointer->GetTrackedDeviceClass(i);

		//Finding the type of device
		if (trackedDeviceClass == ETrackedDeviceClass::TrackedDeviceClass_HMD)
		{
			hmdDeviceId = i;
			if (flags.printSetIds)
				printf("\n SETID--Assigned hmdDeviceId=%d", hmdDeviceId);
		}
		else if (trackedDeviceClass == ETrackedDeviceClass::TrackedDeviceClass_Controller && numControllersInitialized < 2)
		{
			//ControllerData* pC = &(controllers[numControllersInitialized]);
			int sHand = -1;

			ETrackedControllerRole role = vr_pointer->GetControllerRoleForTrackedDeviceIndex(i);
			if (role == TrackedControllerRole_Invalid) //Invalid hand is actually very common, always need to test for invalid hand (lighthouses have lost tracking)
				sHand = 0;
			else if (role == TrackedControllerRole_LeftHand)
				sHand = 1;
			else if (role == TrackedControllerRole_RightHand)
				sHand = 2;
			

			//Used to get/store property ids for the xy of the pad and the analog reading of the trigger
			getid();
			
			if (flags.printSetIds) {
				printf("\n SETID--Assigned controllers[%d] .hand=%d .deviceId=%d .idtrigger=%d .idpad=%d", numControllersInitialized, sHand, i, idtrigger, idpad);
				ControllerData.deviceId = i;
				
				if (controllers[numControllersInitialized].deviceId == -1) {
					controllers[numControllersInitialized].deviceId = i;
				}
				std::cout << "\n controllers :" << controllers[numControllersInitialized].deviceId;
				deviceId_Right = controllers[0].deviceId;
				deviceId_Left = controllers[1].deviceId;
			}
			numControllersInitialized++; //Increment this count so that the other controller gets initialized after initializing this one
		}
		ControllerData.hand = sHand;
		
	}
	
	printf("\n");
}

void PrintDevices() {

	char buf[1024];
	sprintf_s(buf, sizeof(buf), "\nDevice list:\n---------------------------\n");
	printf_s(buf);

	// Loop over all conntected devices and print some stuff about them
	for (vr::TrackedDeviceIndex_t unDevice = 0; unDevice < vr::k_unMaxTrackedDeviceCount; unDevice++)
	{
		// if no HMD is connected in slot 0 just skip the rest of the code
		// since a HMD must be present.
		if (!m_pHMD->IsTrackedDeviceConnected(unDevice))
			continue;

		// Get what type of device it is and work with its data
		ETrackedDeviceClass trackedDeviceClass = VRSystem()->GetTrackedDeviceClass(unDevice);
		switch (trackedDeviceClass) {
		case ETrackedDeviceClass::TrackedDeviceClass_HMD:
			// print stuff for the HMD here, see controller stuff in next case block

			char buf[1024];
			sprintf_s(buf, sizeof(buf), "Device %d: Class: [HMD]", unDevice);
			printf_s(buf);
			break;

		case ETrackedDeviceClass::TrackedDeviceClass_Controller:
			// do stuff for a controller here
			sprintf_s(buf, sizeof(buf), "Device %d: Class: [Controller]", unDevice);
			printf_s(buf);

			break;

		case ETrackedDeviceClass::TrackedDeviceClass_GenericTracker:
			// do stuff for a generic tracker here
			sprintf_s(buf, sizeof(buf), "Device %d: Class: [GenericTracker]", unDevice);
			printf_s(buf);
			break;

		case ETrackedDeviceClass::TrackedDeviceClass_TrackingReference:
			/// do stuff for a tracking reference here

			sprintf_s(buf, sizeof(buf), "Device %d: Class: [TrackingReference]", unDevice);
			printf_s(buf);
			break;

		case ETrackedDeviceClass::TrackedDeviceClass_DisplayRedirect:
			/// do stuff for a display redirect here

			sprintf_s(buf, sizeof(buf), "Device %d: Class: [DisplayRedirect]", unDevice);
			printf_s(buf);
			break;

		case ETrackedDeviceClass::TrackedDeviceClass_Invalid:
			// do stuff for an invalid class

			sprintf_s(buf, sizeof(buf), "Device %d: Class: [Invalid]", unDevice);
			printf_s(buf);
			break;
		}

		// print some of the meta data for the device
		char manufacturer[1024];
		VRSystem()->GetStringTrackedDeviceProperty(unDevice, ETrackedDeviceProperty::Prop_ManufacturerName_String, manufacturer, sizeof(manufacturer));

		char modelnumber[1024];
		VRSystem()->GetStringTrackedDeviceProperty(unDevice, ETrackedDeviceProperty::Prop_ModelNumber_String, modelnumber, sizeof(modelnumber));

		char serialnumber[1024];
		VRSystem()->GetStringTrackedDeviceProperty(unDevice, ETrackedDeviceProperty::Prop_SerialNumber_String, serialnumber, sizeof(serialnumber));

		bool canPowerOff = VRSystem()->GetBoolTrackedDeviceProperty(unDevice, ETrackedDeviceProperty::Prop_DeviceCanPowerOff_Bool);

		sprintf_s(buf, sizeof(buf), " %s - %s [%s] can power off: %d\n", manufacturer, modelnumber, serialnumber, canPowerOff);
		printf_s(buf);
	}
	sprintf_s(buf, sizeof(buf), "---------------------------\nEnd of device list\n\n");
	printf_s(buf);
}

vr::VRActionSetHandle_t m_actionSetDemo = vr::k_ulInvalidActionSetHandle;
const char* actionSetDemoPath = "/actions/demo";
vr::VRActionHandle_t m_actionAnalogInput = vr::k_ulInvalidActionHandle;
const char* actionDemoAnalogInputPath = "/actions/demo/in/AnalogInput";
vr::VRActionHandle_t m_actionHideCubes = vr::k_ulInvalidActionHandle;
const char* actionDemoHideCubesPath = "/actions/demo/in/HideCubes";

vr::VRActionHandle_t m_actionClick = vr::k_ulInvalidActionHandle;
const char* actionDemoClickPath = "/actions/demo/in/ClickAction";

vr::VRActionHandle_t m_actionTouch = vr::k_ulInvalidActionHandle;
const char* actionDemoTouchPath = "/actions/demo/in/TouchAction";

vr::VRActionHandle_t m_actionDemoHandLeft = vr::k_ulInvalidActionHandle;
const char* actionDemoHandLeftPath = "/actions/demo/in/Hand_Left";

vr::VRActionHandle_t m_actionDemoHandRight = vr::k_ulInvalidActionHandle;
const char* actionDemoHandRightPath = "/actions/demo/in/Hand_Right";

vr::VRInputValueHandle_t m_inputHandLeftPath = vr::k_ulInvalidInputValueHandle;
const char* inputHandLeftPath = "/user/hand/left";

vr::VRInputValueHandle_t m_inputHandRightPath = vr::k_ulInvalidInputValueHandle;
const char* inputHandRightPath = "/user/hand/right";

EVRInputError inputError;

void ParseTrackingFrame(int filterIndex) {

	char buf[1024];

	sprintf_s(buf, sizeof(buf), "\n");
	printf_s(buf);

	// Process SteamVR action state
	// UpdateActionState is called each frame to update the state of the actions themselves. The application
	// controls which action sets are active with the provided array of VRActiveActionSet_t structs.
	VRActiveActionSet_t actionSet = { 0 };
	/*actionSet.ulActionSet = m_actionSetDemo;
	inputError = vr::VRInput()->UpdateActionState(&actionSet, sizeof(actionSet), 1);
	if (inputError == vr::VRInputError_None) {
		sprintf_s(buf, sizeof(buf), "%s | UpdateActionState(): Ok\n", actionSetDemoPath);
		printf_s(buf);
	}
	else {
		sprintf_s(buf, sizeof(buf), "%s | UpdateActionState(): Error: %d\n", actionSetDemoPath, inputError);
		printf_s(buf);
	}*/

	// Get analog data
	vr::InputAnalogActionData_t analogData;
	//inputError = 
	vr::VRInput()->GetAnalogActionData(m_actionAnalogInput, &analogData, sizeof(analogData), vr::k_ulInvalidInputValueHandle);
	//if (inputError == vr::VRInputError_None)
	//{
		sprintf_s(buf, sizeof(buf), "%s | GetAnalogActionData() Ok\n", actionDemoAnalogInputPath);
		printf_s(buf);

		//if (analogData.bActive) {
			float m_vAnalogValue0 = analogData.x;
			float m_vAnalogValue1 = analogData.y;
			sprintf_s(buf, sizeof(buf), "%s | x: %f  y:%f\n", actionDemoAnalogInputPath, m_vAnalogValue0, m_vAnalogValue1);
			printf_s(buf);
		//}
		/*else {
			sprintf_s(buf, sizeof(buf), "%s | action not avail to be bound\n", actionDemoAnalogInputPath);
			printf_s(buf);
		}*/
	//}
	/*else {
		sprintf_s(buf, sizeof(buf), "%s | GetAnalogActionData() Not Ok. Error: %d\n", actionDemoAnalogInputPath, inputError);
		printf_s(buf);
	}*/

	/*
	// Get digital data
	vr::InputDigitalActionData_t digitalData;
	inputError = vr::VRInput()->GetDigitalActionData(m_actionHideCubes, &digitalData, sizeof(digitalData), vr::k_ulInvalidInputValueHandle);
	if (inputError == vr::VRInputError_None)
	{
		sprintf_s(buf, sizeof(buf), "%s | GetDigitalActionData() Ok\n", actionDemoHideCubesPath);
		printf_s(buf);

		if (digitalData.bActive) {
			bool m_vDigitalValue0 = digitalData.bState;
			sprintf_s(buf, sizeof(buf), "%s | State: %d\n", actionDemoHideCubesPath, m_vDigitalValue0);
			printf_s(buf);

			// check from which device the action came
			vr::InputOriginInfo_t originInfo;
			if (vr::VRInputError_None == vr::VRInput()->GetOriginTrackedDeviceInfo(digitalData.activeOrigin, &originInfo, sizeof(originInfo)))
			{
				if (originInfo.devicePath == m_inputHandLeftPath) {
					sprintf_s(buf, sizeof(buf), "Action comes from left hand\n");
					printf_s(buf);
				}
				else if (originInfo.devicePath == m_inputHandRightPath) {
					sprintf_s(buf, sizeof(buf), "Action comes from right hand\n");
					printf_s(buf);
				}
			}

		}
		else {
			sprintf_s(buf, sizeof(buf), "%s | action not avail to be bound\n", actionDemoHideCubesPath);
			printf_s(buf);
		}
	}
	else {
		sprintf_s(buf, sizeof(buf), "%s | GetDigitalActionData() Not Ok. Error: %d\n", actionDemoHideCubesPath, inputError);
		printf_s(buf);
	}

	// Get digital data of a "Touch Action"
	vr::InputDigitalActionData_t digitalDataTouch;
	inputError = vr::VRInput()->GetDigitalActionData(m_actionTouch, &digitalDataTouch, sizeof(digitalDataTouch), vr::k_ulInvalidInputValueHandle);
	if (inputError == vr::VRInputError_None)
	{
		sprintf_s(buf, sizeof(buf), "%s | GetDigitalActionData() Ok\n", actionDemoTouchPath);
		printf_s(buf);

		if (digitalDataTouch.bActive) {
			bool m_vDigitalValue0 = digitalDataTouch.bState;
			sprintf_s(buf, sizeof(buf), "%s | State: %d\n", actionDemoTouchPath, m_vDigitalValue0);
			printf_s(buf);

			// check from which device the action came
			vr::InputOriginInfo_t originInfo;
			if (vr::VRInputError_None == vr::VRInput()->GetOriginTrackedDeviceInfo(digitalDataTouch.activeOrigin, &originInfo, sizeof(originInfo)))
			{
				if (originInfo.devicePath == m_inputHandLeftPath) {
					sprintf_s(buf, sizeof(buf), "Action comes from left hand\n");
					printf_s(buf);
				}
				else if (originInfo.devicePath == m_inputHandRightPath) {
					sprintf_s(buf, sizeof(buf), "Action comes from right hand\n");
					printf_s(buf);
				}
			}

		}
		else {
			sprintf_s(buf, sizeof(buf), "%s | action not avail to be bound\n", actionDemoTouchPath);
			printf_s(buf);
		}
	}
	else {
		sprintf_s(buf, sizeof(buf), "%s | GetDigitalActionData() Not Ok. Error: %d\n", actionDemoTouchPath, inputError);
		printf_s(buf);
	}

	// Get digital data of a "Click Action"
	vr::InputDigitalActionData_t digitalDataClick;
	inputError = vr::VRInput()->GetDigitalActionData(m_actionClick, &digitalDataClick, sizeof(digitalDataClick), vr::k_ulInvalidInputValueHandle);
	if (inputError == vr::VRInputError_None)
	{
		sprintf_s(buf, sizeof(buf), "%s | GetDigitalActionData() Ok\n", actionDemoClickPath);
		printf_s(buf);

		if (digitalDataClick.bActive) {
			bool m_vDigitalValue0 = digitalDataClick.bState;
			sprintf_s(buf, sizeof(buf), "%s | State: %d\n", actionDemoClickPath, m_vDigitalValue0);
			printf_s(buf);

			// check from which device the action came
			vr::InputOriginInfo_t originInfo;
			if (vr::VRInputError_None == vr::VRInput()->GetOriginTrackedDeviceInfo(digitalDataClick.activeOrigin, &originInfo, sizeof(originInfo)))
			{
				if (originInfo.devicePath == m_inputHandLeftPath) {
					sprintf_s(buf, sizeof(buf), "Action comes from left hand\n");
					printf_s(buf);
				}
				else if (originInfo.devicePath == m_inputHandRightPath) {
					sprintf_s(buf, sizeof(buf), "Action comes from right hand\n");
					printf_s(buf);
				}
			}

		}
		else {
			sprintf_s(buf, sizeof(buf), "%s | action not avail to be bound\n", actionDemoClickPath);
			printf_s(buf);
		}
	}
	else {
		sprintf_s(buf, sizeof(buf), "%s | GetDigitalActionData() Not Ok. Error: %d\n", actionDemoClickPath, inputError);
		printf_s(buf);
	}


	// get pose data
	vr::InputPoseActionData_t poseData;
	inputError = vr::VRInput()->GetPoseActionDataForNextFrame(m_actionDemoHandLeft, vr::TrackingUniverseStanding, &poseData, sizeof(poseData), vr::k_ulInvalidInputValueHandle);
	if (inputError == vr::VRInputError_None) {
		sprintf_s(buf, sizeof(buf), "%s | GetPoseActionData() Ok\n", actionDemoHandLeftPath);
		printf_s(buf);

		if (poseData.bActive) {
			vr::VRInputValueHandle_t activeOrigin = poseData.activeOrigin;
			bool bPoseIsValid = poseData.pose.bPoseIsValid;
			bool bDeviceIsConnected = poseData.pose.bDeviceIsConnected;
			sprintf_s(buf, sizeof(buf), "Origin: %d Validity: %d DeviceIsConnected: %d\n", activeOrigin, bPoseIsValid, bDeviceIsConnected);
			printf_s(buf);


			// Code below is old ---> 
			vr::HmdVector3_t position;
			vr::HmdQuaternion_t quaternion;

			// get the position and rotation
			position = GetPosition(poseData.pose.mDeviceToAbsoluteTracking);
			quaternion = GetRotation(poseData.pose.mDeviceToAbsoluteTracking);

			// print the tracking data
			//if (printHmdTrackingData) {
			sprintf_s(buf, sizeof(buf), "\n%s Pose\nx: %.2f y: %.2f z: %.2f\n", actionDemoHandLeftPath, position.v[0], position.v[1], position.v[2]);
			printf_s(buf);
			sprintf_s(buf, sizeof(buf), "qw: %.2f qx: %.2f qy: %.2f qz: %.2f\n", quaternion.w, quaternion.x, quaternion.y, quaternion.z);
			printf_s(buf);
			//}
		// <--- End of old code 


		}
		else {
			sprintf_s(buf, sizeof(buf), "%s | action not avail to be bound\n", actionDemoHandLeftPath);
			printf_s(buf);
		}
	}
	else {
		sprintf_s(buf, sizeof(buf), "%s | GetPoseActionData() Call Not Ok. Error: %d\n", actionDemoHandLeftPath, inputError);
		printf_s(buf);
	}*/
}

void RunProcedure2(SOCKET socket)
{
	VREvent_t event;
	LighthouseTracking::_ControllerData ControllerData;
	TrackedDeviceIndex_t unControllerDeviceIndex;
	TrackedDevicePose_t trackedDevicePose;
	VRControllerState_t controllerState;

	TrackedDevicePose_t trackedControllerPose;
	HmdMatrix34_t poseMatrix;
	HmdVector3_t position;
	HmdQuaternion_t quaternion;

	//InputDigitalActionData_t digitalDataClick;
	//VRActionHandle_t m_actionClick = k_ulInvalidActionHandle;
	//const char* actionDemoClickPath = "/actions/demo/in/ClickAction";
	//EVRInitError eError = VRInitError_None;
	//m_pHMD = VR_Init(&eError, VRApplication_Background);
	//VRActionHandle_t m_actionTouch = k_ulInvalidActionHandle;
	//const char* actionDemoTouchPath = "/actions/demo/in/TouchAction";
	
	//VRActionHandle_t m_actionAnalogInput = k_ulInvalidActionHandle;
	//const char* actionDemoAnalogInputPath = "/actions/demo/in/AnalogInput";

	//VRInputValueHandle_t m_inputHandLeftPath = k_ulInvalidInputValueHandle;
	//const char* inputHandLeftPath = "/user/hand/left";

	//VRInputValueHandle_t m_inputHandRightPath = k_ulInvalidInputValueHandle;
	//const char* inputHandRightPath = "/user/hand/right";

	//VRActionSetHandle_t m_actionSetDemo = k_ulInvalidActionSetHandle;
	//const char* actionSetDemoPath = "/actions/demo";

	//InputDigitalActionData_t digitalDataTouch;
	//VRInput()->GetDigitalActionData(m_actionTouch, &digitalDataTouch, sizeof(digitalDataTouch), k_ulInvalidInputValueHandle);
	
	if (vr_pointer->PollNextEvent(&event, sizeof(event)))
	{
		ETrackedControllerRole role = vr_pointer->GetControllerRoleForTrackedDeviceIndex(event.trackedDeviceIndex);
		switch (event.data.controller.button) {
		case k_EButton_ProximitySensor:
			std::cout << "ProximitySensor \n";
			VRSystem()->GetDeviceToAbsoluteTrackingPose(TrackingUniverseStanding, 0, &trackedDevicePose, 1);
			// print positiona data for the HMD.
			poseMatrix = trackedDevicePose.mDeviceToAbsoluteTracking; // This matrix contains all positional and rotational data.
			position = GetPosition(trackedDevicePose.mDeviceToAbsoluteTracking);
			quaternion = GetRotation(trackedDevicePose.mDeviceToAbsoluteTracking);

			printDevicePositionalData("HMD", poseMatrix, position, quaternion);
			break;
		case k_EButton_Grip:
			switch (event.eventType) {
			case VREvent_ButtonPress:
				std::cout << "Grip press\n";
				//Message = "Grip press";
				if (role == TrackedControllerRole_Invalid) {
					std::cout << "error\n";
					//Message += " error";
				}
				else if (role == TrackedControllerRole_LeftHand) {
					std::cout << "Left Haptic diviceid \n" << deviceId_Left;
					vr_pointer->GetControllerStateWithPose(TrackingUniverseStanding, deviceId_Left, &controllerState, sizeof(controllerState), &trackedDevicePose);
					for (int i = 0;i < 5;i++) {
						vr_pointer->TriggerHapticPulse(deviceId_Left, 0, 3999);
					}
					Message = "3";
				}
				else if (role == TrackedControllerRole_RightHand) {
					std::cout << "Right Haptic diviceid \n" << controllers[0].deviceId;
					vr_pointer->GetControllerStateWithPose(TrackingUniverseStanding, deviceId_Right, &controllerState, sizeof(controllerState), &trackedDevicePose);
					for (int i = 0;i < 5;i++) {
						vr_pointer->TriggerHapticPulse(deviceId_Right, 0, 3999);
					}
					Message = "4";
				}
				if (Message.size() > 0)		// Make sure the user has typed in something
				{
					// Send the text
					int sendResult = send(socket, Message.c_str(), Message.size() + 1, 0);
				}
				Message = "";
				break;
			case VREvent_ButtonUnpress:
				std::cout << "Grip unpress\n";
				//Message = "Grip unpres";

				if (role == TrackedControllerRole_Invalid) {
					std::cout << "error\n";
					//Message += " error";
				}
				else if (role == TrackedControllerRole_LeftHand) {

					std::cout << "Left\n ";
					//Message += " Left";
				}
				else if (role == TrackedControllerRole_RightHand) {

					std::cout << "right\n";
					//Message += " right";
				}
				Message = "0";
				if (Message.size() > 0)	// Make sure the user has typed in something
				{
					// Send the text
					int sendResult = send(socket, Message.c_str(), Message.size() + 1, 0);
				}
				Message = "";
				break;
			}
			break;
		case k_EButton_SteamVR_Trigger:
			switch (event.eventType) {
			case VREvent_ButtonPress:
				std::cout << "Trigger press\n";
				//Message = "Trigger press";
				if (role == TrackedControllerRole_Invalid) {
					std::cout << "error\n";
					//Message += " error";
				}
				else if (role == TrackedControllerRole_LeftHand) {
					std::cout << "deviceId: " << deviceId_Left;
					vr_pointer->GetControllerStateWithPose(TrackingUniverseStanding, deviceId_Left, &controllerState, sizeof(controllerState), &trackedDevicePose);
					int t = 1;
					while (controllerState.rAxis[t].x != 0) {
						vr_pointer->GetControllerStateWithPose(TrackingUniverseStanding, deviceId_Left, &controllerState, sizeof(controllerState), &trackedDevicePose);
						int t = 1;
						double Left_AnalogValue = 1 + controllerState.rAxis[t].x;
						std::cout << "Left AnalogValue :" << Left_AnalogValue << "\n";
						std::ostringstream oss;
						oss << Left_AnalogValue;
						Message = oss.str() + "\n";
						int sendResult = send(socket, Message.c_str(), Message.size() + 1, 0);
					}
					printPositionalData();
				}
				else if (role == TrackedControllerRole_RightHand) {
					std::cout << "deviceId: " << deviceId_Right;
					vr_pointer->GetControllerStateWithPose(TrackingUniverseStanding, deviceId_Right, &controllerState, sizeof(controllerState), &trackedDevicePose);
					int t = 1;
					while (controllerState.rAxis[t].x != 0) {
						vr_pointer->GetControllerStateWithPose(TrackingUniverseStanding, deviceId_Right, &controllerState, sizeof(controllerState), &trackedDevicePose);
						int t = 1;
						double Right_AnalogValue = controllerState.rAxis[t].x;
						std::cout << "Right AnalogValue :" << Right_AnalogValue << "\n";
						std::ostringstream oss;
						oss << Right_AnalogValue;
						Message = oss.str() + "\n";
						int sendResult = send(socket, Message.c_str(), Message.size() + 1, 0);
					}
					printPositionalData();
				}
				break;

			case VREvent_ButtonUnpress:
				std::cout << "Trigger unpress \n";
				//Message = "Trigger unpress";
				if (role == TrackedControllerRole_Invalid) {
					std::cout << "error\n";
					//Message += " error";
				}
				else if (role == TrackedControllerRole_LeftHand) {
					std::cout << "Left unpress \n";
					//Message += " Left";
				}
				else if (role == TrackedControllerRole_RightHand)
				{
					std::cout << "Right unpress \n";
					//Message += " right";
				}
				Message = "0";
				if (Message.size() > 0)		// Make sure the user has typed in something
				{
					// Send the text
					int sendResult = send(socket, Message.c_str(), Message.size() + 1, 0);
				}
				Message = "";
				break;
			}
			break;
		case k_EButton_SteamVR_Touchpad:
			switch (event.eventType) {
			case VREvent_ButtonPress:
				std::cout << "Touchpad press\n";
				if (role == TrackedControllerRole_Invalid) {
					std::cout << "error\n";
				}
				else if (role == TrackedControllerRole_LeftHand) {
					std::cout << "Left\n";
				}
				else if (role == TrackedControllerRole_RightHand) {
					std::cout << "right\n";	
				}
				break;
			case VREvent_ButtonUnpress:

				std::cout << "Touchpad press\n";
				if (role == TrackedControllerRole_Invalid) {
					std::cout << "error\n";
				}
				else if (role == TrackedControllerRole_LeftHand) {
					std::cout << "Left\n";
				}
				else if (role == TrackedControllerRole_RightHand)
				{
					std::cout << "right\n";
				}
				break;

			case VREvent_ButtonTouch:
				std::cout << "ButtonTouch press\n";
				//Message = "ButtonTouch press";
				if (role == TrackedControllerRole_Invalid) {
					std::cout << "error\n";
					//Message += " error";
				}
				else if (role == TrackedControllerRole_LeftHand) {
					vr_pointer->GetControllerStateWithPose(TrackingUniverseStanding, deviceId_Left, &controllerState, sizeof(controllerState), &trackedDevicePose);
					int p = idpad;
					float x = controllerState.rAxis[p].x;
					float y = controllerState.rAxis[p].y;
					std::cout << "Left\n" << " X :" << x /*controllerState.rAxis[0].x */<< " Y :" << y /*controllerState.rAxis[0].y */<< "\n";
					std::ostringstream oss, osx , osy;
					oss << 6;
					osx << x;
					osy << y;
					Message = oss.str() + "\n" + " X:" + osx.str() + " Y:" + osy.str() + "\n";
					int sendResult = send(socket, Message.c_str(), Message.size() + 1, 0);
					//Message = "6";
				}
				else if (role == TrackedControllerRole_RightHand){
					vr_pointer->GetControllerStateWithPose(TrackingUniverseStanding, deviceId_Right, &controllerState, sizeof(controllerState), &trackedDevicePose);
					int p = idpad;
					float x = controllerState.rAxis[p].x;
					float y = controllerState.rAxis[p].y;
					std::cout << "right\n" << " X :" << x /*controllerState.rAxis[0].x */ << " Y :" << y /*controllerState.rAxis[0].y */ << "\n";
					std::ostringstream oss, osx, osy;
					oss << 5;
					osx << x;
					osy << y;
					Message = oss.str()+"\n" + " X:" + osx.str() + " Y:" + osy.str() + "\n";
					int sendResult = send(socket, Message.c_str(), Message.size() + 1, 0);
					//Message = "5";
				}

				/*if (Message.size() > 0)		// Make sure the user has typed in something
				{
					// Send the text
					int sendResult = send(socket, Message.c_str(), Message.size() + 1, 0);
				}
				Message = "";*/
				break;

			case VREvent_ButtonUntouch:

				std::cout << "ButtonTouch unpress\n";
				//Message = "ButtonTouch unpress";

				if (role == TrackedControllerRole_Invalid) {
					std::cout << "error\n";
					//Message += " error";
				}
				else if (role == TrackedControllerRole_LeftHand) {
					std::cout << "Left\n";
					//Message += " Left";
				}
				else if (role == TrackedControllerRole_RightHand){
					std::cout << "right\n";
					//Message += " right";
				}
				Message = "0 \n";
				if (Message.size() > 0)		// Make sure the user has typed in something
				{
					// Send the text
					int sendResult = send(socket, Message.c_str(), Message.size() + 1, 0);
				}
				Message = "";
				break;
			}
			break;
		case k_EButton_ApplicationMenu:
			switch (event.eventType) {
			case VREvent_ButtonPress:

				std::cout << "ApplicationMenu press\n";
				if (role == TrackedControllerRole_Invalid) {
					std::cout << "error\n";
				}
				else if (role == TrackedControllerRole_LeftHand) {
					std::cout << "Left\n";
				}
				else if (role == TrackedControllerRole_RightHand)
				{
					std::cout << "right\n";
				}
				break;

			case VREvent_ButtonUnpress:

				std::cout << "ApplicationMenu unpress\n";
				if (role == TrackedControllerRole_Invalid) {
					std::cout << "error\n";
				}
				else if (role == TrackedControllerRole_LeftHand) {
					std::cout << "Left\n";
				}
				else if (role == TrackedControllerRole_RightHand){
					std::cout << "right\n";
				}
				break;
			}
			break;
		}
	}
}
//Output tracking data or do other things

