#ifdef USE_JETSON

/********************************************************************************
 * @file    target_tracking.cpp
 * @author  Cameron Rose
 * @date    12/27/2023
 * @brief   All methods needed to initialize and create a detection network and 
			choose a target.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "target_tracking.h"

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/
detectNet* net = NULL;
detectNet::Detection* detections = NULL;
int numDetections = 0;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: Target
 * Description: Class constructor
 ********************************************************************************/
Target::Target(void){};

/********************************************************************************
 * Function: ~Target
 * Description: Class destructor
 ********************************************************************************/
Target::~Target(void){};

/********************************************************************************
 * Function: create_detection_network
 * Description: Initialize the network used for object detection.
 ********************************************************************************/
int create_detection_network(void)
{
	//net = detectNet::Create("SSD_Inception_V2", 0.5, 4);
	net = detectNet::Create("SSD_Mobilenet_V2", 0.5, 4);
	net->SetTracker(objectTrackerIOU::Create(3, 100, 0.5f));
	
	if( !net )
	{
		LogError("detectnet:  failed to load detectNet model\n");
		return 1;
	}
}

/********************************************************************************
 * Function: detect_objects
 * Description: Initialize the network used for object detection.
 ********************************************************************************/
void detect_objects(void)
{
	uint32_t overlay_flags = 0;
	
	//overlay_flags = overlay_flags | detectNet::OVERLAY_BOX | detectNet::OVERLAY_LABEL | detectNet::OVERLAY_CONFIDENCE | detectNet::OVERLAY_TRACKING | detectNet::OVERLAY_LINES;
	
	if (overlay_flags > 0)
	{
		numDetections = net->Detect(image, input->GetWidth(), input->GetHeight(), &detections, overlay_flags);
	}
	else
	{
		numDetections = net->Detect(image, input->GetWidth(), input->GetHeight(), &detections);
	}
}

/********************************************************************************
 * Function: get_object_info
 * Description: Obtain info about detected objects.
 ********************************************************************************/
void get_object_info(void)
{
	if( numDetections > 0 )
	{
		//LogVerbose("%i objects detected\n", numDetections);
	
		for( int n=0; n < numDetections; n++ )
		{
			float boxWidth = detections[n].Width();
			float boxHeight = detections[n].Height();
		}
	}	
}

/********************************************************************************
 * Function: print_object_info
 * Description: Print info about detected objects.
 ********************************************************************************/
void print_object_info(void)
{
	if( numDetections > 0 )
	{
	LogVerbose("%i objects detected\n", numDetections);

		for( int n=0; n < numDetections; n++ )
		{
			if( detections[n].TrackID >= 0 ) // is this a tracked object?
			{			
				if (detections[n].ClassID == 1 && detections[n].Confidence > 0.5)
				{
					LogVerbose("\ndetected obj %i  class #%u (%s)  confidence=%f\n", n, detections[n].ClassID, net->GetClassDesc(detections[n].ClassID), detections[n].Confidence);
					LogVerbose("bounding box %i  (%.2f, %.2f)  (%.2f, %.2f)  w=%.2f  h=%.2f\n", n, detections[n].Left, detections[n].Top, detections[n].Right, detections[n].Bottom, detections[n].Width(), detections[n].Height());
					LogVerbose("tracking  ID %i  status=%i  frames=%i  lost=%i\n", detections[n].TrackID, detections[n].TrackStatus, detections[n].TrackFrames, detections[n].TrackLost);
					LogVerbose("Object %i Edges (Left,Right,Top,Bottom)=(%.2f, %.2f, %.2f, %.2f)\n",n,detections[n].Left, detections[n].Right, detections[n].Top, detections[n].Bottom);
					LogVerbose("video width, video height: (%.2f, %.2f)\n", input_video_width, input_video_height);
					LogVerbose("box width, box height: (%.2f, %.2f)\n", detections[n].Width(), detections[n].Height());
				}
			}
		}
	}
}

/********************************************************************************
 * Function: print_print_performance_statsobject_info
 * Description: Print info about detection network performance.
 ********************************************************************************/
void print_performance_stats(void)
{
	// print out timing info
	net->PrintProfilerTimes();
}

/********************************************************************************
 * Function: delete_tracking_net
 * Description: Delete detection network to free up resources.
 ********************************************************************************/
void delete_tracking_net(void)
{
	SAFE_DELETE(net);
}

#endif // USE_JETSON