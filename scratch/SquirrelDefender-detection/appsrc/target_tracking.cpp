#include "target_tracking.h"

detectNet* net = NULL;
detectNet::Detection* detections = NULL;
int numDetections = 0;

int create_detection_network(void)
{
	/*
	 * create detection network
	 */
	//net = detectNet::Create("SSD_Inception_V2", 0.5, 4);
	net = detectNet::Create("SSD_Mobilenet_V2", 0.5, 4);
	net->SetTracker(objectTrackerIOU::Create(3, 100, 0.5f));
	
	if( !net )
	{
		LogError("detectnet:  failed to load detectNet model\n");
		return 1;
	}
}

void detect_objects(void)
{
	uint32_t overlay_flags = 0;
	
	//overlay_flags = overlay_flags | detectNet::OVERLAY_BOX | detectNet::OVERLAY_LABEL | detectNet::OVERLAY_CONFIDENCE | detectNet::OVERLAY_TRACKING | detectNet::OVERLAY_LINES;

	// detect objects in the frame
	detections = NULL;
	
	if (overlay_flags > 0)
	{
		numDetections = net->Detect(image, input->GetWidth(), input->GetHeight(), &detections, overlay_flags);
	}
	else
	{
		numDetections = net->Detect(image, input->GetWidth(), input->GetHeight(), &detections);
	}
}

void get_object_info(void)
{
	if( numDetections > 0 )
	{
		//LogVerbose("%i objects detected\n", numDetections);
	
		for( int n=0; n < numDetections; n++ )
		{
			// Calculate corner positions relative to top-left corner of video feed
            uint32_t videoWidth = input->GetWidth();
            uint32_t videoHeight = input->GetHeight();
			
			float boxWidth = detections[n].Width();
			float boxHeight = detections[n].Height();

			//LogVerbose("box width, box height: (%.2f, %.2f)\n", boxWidth, boxHeight);
			//LogVerbose("video width, video height: (%.2f, %.2f)\n", videoWidth, videoHeight);
		}
	}	
}

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
					std::cout << "Left: " << detections[n].Left << std::endl;
					std::cout << "Right: " << detections[n].Right << std::endl;
					std::cout << "Top: " << detections[n].Top << std::endl;
					std::cout << "Bottom: " << detections[n].Bottom << std::endl;
				}
			}
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

