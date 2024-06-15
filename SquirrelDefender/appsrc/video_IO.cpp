#ifdef USE_JETSON

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
#include "video_IO.h"

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/
bool valid_image_rcvd;
videoSource* input;
videoOutput* output;
uchar3* image;
uint32_t input_video_width;
uint32_t input_video_height;
GstElement* pipeline;
GstBus* bus;


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
bool Video::create_input_video_stream(const commandLine& cmdLine, int positionArg)
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
	
	if(!input)
	{
		LogError("detectnet:  failed to create input stream\n");
		return false;
	}

    return true;
}

/********************************************************************************
* Function: create_output_video
* Description: Create an output video stream from the input video stream
*			   for displaying and passing to other software components.
********************************************************************************/
bool Video::create_output_video_stream(const commandLine& cmdLine, int positionArg)
{
	/*
	 * create output stream
	 */
	output = videoOutput::Create(cmdLine, positionArg);
	
	if(!output)
	{
		LogError("detectnet:  failed to create output stream\n");	
		return false;
	}

    return true;
}

/********************************************************************************
* Function: capture_image
* Description: Capture an image from the input video stream.
********************************************************************************/
bool Video::capture_image(void)
{
    DebugTerm VideoDebug("/dev/pts/6");

	image = NULL;
	int status = 0;
	
	if(!input->Capture(&image, &status))
	{
		if(status != videoSource::TIMEOUT)
		{
			return false;
		}
	}

    if (image == NULL)
    {
        valid_image_rcvd = false;
        return false; // Return false if the image is not valid
    }

    valid_image_rcvd = true;

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
* Function: calc_video_res
* Description: Delete the input to stop using resources.
********************************************************************************/
void Video::calc_video_res(void)
{
	input_video_width = input->GetWidth();
	input_video_height = input->GetHeight();
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

/********************************************************************************
* Function: video_init
* Description: Code to initialize video streams to run onces at the start of the 
*              program.
********************************************************************************/
bool Video::video_init(const commandLine& cmdLine, int positionArg)
{
    valid_image_rcvd = true;

    if (!create_input_video_stream(cmdLine, ARG_POSITION(0)) || 
        !create_output_video_stream(cmdLine, ARG_POSITION(1)))
    {
        return false;
    }
           
    calc_video_res();

    /* Access the pipeline from the gstCamera instance */
    /*
    DebugTerm GstInfo("/dev/pts/5");
    gstCamera* camera = dynamic_cast<gstCamera*>(input);
    if (camera) 
    {
        GstInfo.cpp_cout("Cast input to gstCamera");
        pipeline = camera->GetPipeline();
        if (pipeline) 
        {
            GstInfo.cpp_cout("Grabbed pipeline");
            bus = gst_element_get_bus(pipeline);
            if (bus)
            {
                gst_bus_add_watch(bus, static_bus_callback, pipeline);
                GstInfo.cpp_cout("Bus watch added");
            }
        }
    }
    */
   
    return true;
}

/********************************************************************************
* Function: video_proc_loop
* Description: Main video processing loop.
********************************************************************************/
void Video::video_proc_loop(void)
{  
    capture_image();
}

/********************************************************************************
* Function: video_output_loop
* Description: Code needed to run each loop to  provide the video output.
********************************************************************************/
void Video::video_proc_loop(void)
{  
    capture_image();
    render_output();
    
    /* Code for trying to access the gstreamer pipeline state */
    /*
    GstMessage* msg = gst_bus_timed_pop_filtered(bus, GST_CLOCK_TIME_NONE, 
                                                     (GstMessageType)(GST_MESSAGE_STATE_CHANGED | GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
    if (msg != nullptr) 
    {
        static_bus_callback(bus, msg, this);
        gst_message_unref(msg);
    }
    */
}

/********************************************************************************
 * Function: shutdown
 * Description: Shutdown detection network
 ********************************************************************************/
void Video::shutdown(void)
{
	LogVerbose("video:  shutting down...\n");
	Video::delete_input_video_stream();
	Video::delete_output_video_stream();
	LogVerbose("video:  shutdown complete.\n");
}

#endif // USE_JETSON