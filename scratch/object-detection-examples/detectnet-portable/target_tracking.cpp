#include "target_tracking.h"

detectNet* net;
detectNet::Detection* detections;

int create_detection_network(const commandLine& cmdLine)
{
	/*
	 * create detection network
	 */
	net = detectNet::Create(cmdLine);
	net->SetTracker(objectTrackerIOU::Create(3, 500, 0.5f));
	
	if( !net )
	{
		LogError("detectnet:  failed to load detectNet model\n");
		return 1;
	}
}

void track_target(const commandLine& cmdLine)
{
	// parse overlay flags
	const uint32_t overlayFlags = detectNet::OverlayFlagsFromStr(cmdLine.GetString("overlay", "box,labels,conf"));
	
	// detect objects in the frame
	detections = NULL;

	const int numDetections = net->Detect(image, input->GetWidth(), input->GetHeight(), &detections, overlayFlags);
	
	if( numDetections > 0 )
	{
		LogVerbose("%i objects detected\n", numDetections);
	
		for( int n=0; n < numDetections; n++ )
		{
			LogVerbose("\ndetected obj %i  class #%u (%s)  confidence=%f\n", n, detections[n].ClassID, net->GetClassDesc(detections[n].ClassID), detections[n].Confidence);
			LogVerbose("bounding box %i  (%.2f, %.2f)  (%.2f, %.2f)  w=%.2f  h=%.2f\n", n, detections[n].Left, detections[n].Top, detections[n].Right, detections[n].Bottom, detections[n].Width(), detections[n].Height()); 
		
			if( detections[n].TrackID >= 0 ) // is this a tracked object?
				LogVerbose("tracking  ID %i  status=%i  frames=%i  lost=%i\n", detections[n].TrackID, detections[n].TrackStatus, detections[n].TrackFrames, detections[n].TrackLost);
		}
	}	
}

void print_performance_stats(void)
{
	// print out timing info
	net->PrintProfilerTimes();
}

void delete_tracking_net(void)
{
	SAFE_DELETE(net);
}

