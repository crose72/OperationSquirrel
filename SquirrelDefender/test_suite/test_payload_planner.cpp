#include "test_utils/test_csv_utils.h"
#include "localize_target.h"
#include "time_calc.h"

/********************************************************************************
 * Setup
 ********************************************************************************/
std::string filename = "../test_data/data_1.csv";
std::vector<float> g_app_elapsed_time_arr;
std::vector<float> g_mav_veh_pitch_arr;
std::vector<float> g_target_valid_arr;
std::vector<float> g_target_detection_id_arr;
std::vector<float> g_target_track_id_arr;
std::vector<float> g_target_cntr_offset_x_arr;
std::vector<float> g_target_cntr_offset_y_arr;
std::vector<float> g_target_height_arr;
std::vector<float> g_target_width_arr;
std::vector<float> g_target_aspect_arr;
std::vector<float> g_target_left_arr;
std::vector<float> g_target_right_arr;
std::vector<float> g_target_top_arr;
std::vector<float> g_target_bottom_arr;
float time_prv;
bool g_use_video_playback;
std::string input_video_path;
bool g_stop_program;

void setup(void)
{
    // Create array of input signals for playback
    g_app_elapsed_time_arr = get_column_data<float>(filename, "g_app_elapsed_time");
    g_mav_veh_pitch_arr = get_column_data<float>(filename, "g_mav_veh_pitch");
    g_target_valid_arr = get_column_data<float>(filename, "g_target_valid");
    g_target_detection_id_arr = get_column_data<float>(filename, "g_target_detection_id");
    g_target_track_id_arr = get_column_data<float>(filename, "g_target_track_id");
    g_target_cntr_offset_x_arr = get_column_data<float>(filename, "g_target_cntr_offset_x");
    g_target_cntr_offset_y_arr = get_column_data<float>(filename, "g_target_cntr_offset_y");
    g_target_height_arr = get_column_data<float>(filename, "g_target_height");
    g_target_width_arr = get_column_data<float>(filename, "g_target_width");
    g_target_aspect_arr = get_column_data<float>(filename, "g_target_aspect");
    g_target_left_arr = get_column_data<float>(filename, "g_target_left");
    g_target_right_arr = get_column_data<float>(filename, "g_target_right");
    g_target_top_arr = get_column_data<float>(filename, "g_target_top");
    g_target_bottom_arr = get_column_data<float>(filename, "g_target_bottom");

    // Check if any input signal arrays are empty
    if (g_app_elapsed_time_arr.empty() || g_mav_veh_pitch_arr.empty() || g_target_valid_arr.empty() ||
        g_target_detection_id_arr.empty() || g_target_track_id_arr.empty() || g_target_cntr_offset_x_arr.empty() ||
        g_target_cntr_offset_y_arr.empty() || g_target_height_arr.empty() || g_target_width_arr.empty() ||
        g_target_aspect_arr.empty() || g_target_left_arr.empty() || g_target_right_arr.empty() ||
        g_target_top_arr.empty() || g_target_bottom_arr.empty()) 
    {
        std::cerr << "Error: No signal data found!" << std::endl;
        return;
    }

    // Initialize the software component  
    Localize::init();

    // Additional initializations
    time_prv = 0.0;
}

void run(void)
{
    // Create csv file to save outputs to
    std::ofstream outputFile("localize_output.csv");
    if (!outputFile.is_open()) 
    {
        std::cerr << "Error opening output file!" << std::endl;
        return;
    }

    // Write csv Header
    outputFile << "g_app_elapsed_time,"
                << "g_mav_veh_pitch,"
                << "g_target_valid,"
                << "g_target_detection_id,"
                << "g_target_track_id,"
                << "g_target_cntr_offset_x,"
                << "g_target_cntr_offset_y,"
                << "g_target_height,"
                << "g_target_width,"
                << "g_target_aspect,"
                << "g_target_left,"
                << "g_target_right,"
                << "g_target_top,"
                << "g_target_bottom,"
                << "d_target_h,"
                << "d_target_w,"
                << "g_x_target,"
                << "g_y_target,"
                << "g_z_target,"
                << "d_target,"
                << "g_x_error,"
                << "g_y_error,"
                << "g_delta_angle,"
                << "g_camera_tilt_angle,"
                << "g_delta_d_x,"
                << "g_delta_d_z,"
                << "g_camera_comp_angle,"
                << "g_x_target_ekf,"
                << "g_y_target_ekf,"
                << "g_vx_target_ekf,"
                << "g_vy_target_ekf,"
                << "g_ax_target_ekf,"
                << "g_ay_target_ekf\n";

    // Get max number of rows to ensure all data is included
    size_t num_rows = g_app_elapsed_time_arr.size();

    for (size_t i = 0; i < num_rows; ++i) 
    {
        if (g_app_elapsed_time_arr[i] != NULL)
        {
            // Set inputs
            g_app_elapsed_time = g_app_elapsed_time_arr[i];
            g_mav_veh_pitch = g_mav_veh_pitch_arr[i];
            g_target_valid = g_target_valid_arr[i];
            g_target_detection_id = g_target_detection_id_arr[i];
            g_target_track_id = g_target_track_id_arr[i];
            g_target_cntr_offset_x = g_target_cntr_offset_x_arr[i];
            g_target_cntr_offset_y = g_target_cntr_offset_y_arr[i];
            g_target_height = g_target_height_arr[i];
            g_target_width = g_target_width_arr[i];
            g_target_aspect = g_target_aspect_arr[i];
            g_target_left = g_target_left_arr[i];
            g_target_right = g_target_right_arr[i];
            g_target_top = g_target_top_arr[i];
            g_target_bottom = g_target_bottom_arr[i];

            // Calculate timestep manually since this is not running on the drone in real time
            g_dt = g_app_elapsed_time - time_prv;
            time_prv = g_app_elapsed_time;

            // Have to calculate the bounding box for the localize algo since
            // it's calculated in the track_target function (or update the inputs
            // to this test case and call Track::loop)
            g_target_center_y = (g_target_left + g_target_right) / 2.0f;
            g_target_center_x = (g_target_bottom + g_target_top) / 2.0f;

            // Call Localize::loop() to calculate it's outputs
            Localize::loop();
            
            // Write the latest values of global variables directly to the CSV
            outputFile << g_app_elapsed_time << ","
                       << g_mav_veh_pitch << ","
                       << g_target_valid << ","
                       << g_target_detection_id << ","
                       << g_target_track_id << ","
                       << g_target_cntr_offset_x << ","
                       << g_target_cntr_offset_y << ","
                       << g_target_height << ","
                       << g_target_width << ","
                       << g_target_aspect << ","
                       << g_target_left << ","
                       << g_target_right << ","
                       << g_target_top << ","
                       << g_target_bottom << ","
                       << d_target_h << ","
                       << d_target_w << ","
                       << g_x_target << ","
                       << g_y_target << ","
                       << g_z_target << ","
                       << d_target << ","
                       << g_x_error << ","
                       << g_y_error << ","
                       << g_delta_angle << ","
                       << g_camera_tilt_angle << ","
                       << g_delta_d_x << ","
                       << g_delta_d_z << ","
                       << g_camera_comp_angle << ","
                       << g_x_target_ekf << ","
                       << g_y_target_ekf << ","
                       << g_vx_target_ekf << ","
                       << g_vy_target_ekf << ","
                       << g_ax_target_ekf << ","
                       << g_ay_target_ekf << "\n";
        }
    }

    // Close file after writing to it
    outputFile.close();
    std::cout << "Processing complete. Results saved to output_signals.csv\n";
}

/********************************************************************************
 * Test group name
 ********************************************************************************/
int main() 
{
    // Read in data from log file
    setup();
    
    // Run the inputs through the function
    run();

    return 0;
}