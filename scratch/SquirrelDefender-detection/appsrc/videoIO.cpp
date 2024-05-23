/********************************************************************************
 * @file    videoIO.cpp
 * @author  Cameron Rose
 * @date    5/22/2024
 * @brief   Configure and start video streams, and output video for other
			software components.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "videoIO.h"

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/
videoSource* input;
videoOutput* output;
uchar3* image;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
* Function: Video
* Description: Constructor of the Video class.
********************************************************************************/
Video::Video(void){}

/********************************************************************************
* Function: Video
* Description: Constructor of the Video class.
********************************************************************************/
Video::~Video(void){}

/********************************************************************************
* Function: create_input_video_stream
* Description: Create an input video stream from the attached cameras.
********************************************************************************/
int Video::create_input_video_stream(const commandLine& cmdLine, int positionArg)
{
	/*
	 * create input stream
	 */
	input = videoSource::Create(cmdLine, positionArg);
    /*videoOptions options;

    // Set the video options
    options.resource = URI("csi://0");
    options.width = 1280;
    options.height = 720;
    options.frameRate = 30;
    options.numBuffers = 4;
    options.flipMethod = videoOptions::FlipMethod::FLIP_ROTATE_180;
    
	input = videoSource::Create(options);*/
	
	if( !input )
	{
		LogError("detectnet:  failed to create input stream\n");
		return 1;
	}
}

/********************************************************************************
* Function: create_output_video
* Description: Create an output video stream from the input video stream
*			   for displaying and passing to other software components.
********************************************************************************/
int Video::create_output_video_stream(const commandLine& cmdLine, int positionArg)
{
	/*
	 * create output stream
	 */
	output = videoOutput::Create(cmdLine, positionArg);
	
	if( !output )
	{
		LogError("detectnet:  failed to create output stream\n");	
		return 1;
	}
}

/********************************************************************************
* Function: capture_image
* Description: Capture an image from the input video stream.
********************************************************************************/
bool Video::capture_image(void)
{
	image = NULL; // is this needed between loops?
	int status = 0;
	
	if( !input->Capture(&image, &status) )
	{
		if( status != videoSource::TIMEOUT )
		{
			return false;
		}
	}
	return true;
}

/********************************************************************************
* Function: render_output
* Description: Display the image on the screen.
********************************************************************************/
bool Video::render_output(void)
{
	// render outputs
	if( output != NULL )
	{
		output->Render(image, input->GetWidth(), input->GetHeight());

		// update the status bar
		char str[256];
		sprintf(str, "TensorRT %i.%i.%i | %s | Network %.0f FPS", NV_TENSORRT_MAJOR, NV_TENSORRT_MINOR, NV_TENSORRT_PATCH, precisionTypeToStr(net->GetPrecision()), net->GetNetworkFPS());
		output->SetStatus(str);

		// check if the user quit
		if( !output->IsStreaming() )
		{
			return false;
		}
	}
	
	return true;
}

/********************************************************************************
* Function: delete_input_video_stream
* Description: Delete the input to stop using resources.
********************************************************************************/
void Video::delete_input_video_stream(void)
{
	SAFE_DELETE(input);
}

/********************************************************************************
* Function: delete_output_video_stream
* Description: Delete the input to stop using resources.
********************************************************************************/
void Video::delete_output_video_stream(void)
{
	SAFE_DELETE(output);
}
