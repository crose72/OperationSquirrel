#include "videoIO.h"

videoSource* input;
videoOutput* output;
uchar3* image;

int input_video(const commandLine& cmdLine, int positionArg)
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

int output_video(const commandLine& cmdLine, int positionArg)
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

bool capture_image(void)
{
	// capture next image
	image = NULL;
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

bool render_output(void)
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

void delete_input(void)
{
	SAFE_DELETE(input);
}

void delete_output(void)
{
	SAFE_DELETE(output);
}
