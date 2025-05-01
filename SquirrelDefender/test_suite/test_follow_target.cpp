#include "test_utils/test_csv_utils.h"
#include "localize_target.h"
#include "path_planner.h"
#include "time_calc.h"
#include "param_reader.h"

/********************************************************************************
 * Setup
 ********************************************************************************/
std::string test_inputs;
std::string test_output_path;
std::string test_output_file_name;
std::string test_unique_output_file_name;
std::ofstream test_output_file;
size_t num_test_steps;
float time_prv;

bool g_use_video_playback;
std::string input_video_path;
bool g_stop_program;
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

void init_test_inputs (void)
{
    test_inputs = "../test_data/follow_2.csv";

    // Create array of input signals for playback
    g_app_elapsed_time_arr = get_column_data<float>(test_inputs, "g_app_elapsed_time");
    g_mav_veh_pitch_arr = get_column_data<float>(test_inputs, "g_mav_veh_pitch");
    g_target_valid_arr = get_column_data<float>(test_inputs, "g_target_valid");
    g_target_detection_id_arr = get_column_data<float>(test_inputs, "g_target_detection_id");
    g_target_track_id_arr = get_column_data<float>(test_inputs, "g_target_track_id");
    g_target_cntr_offset_x_arr = get_column_data<float>(test_inputs, "g_target_cntr_offset_x");
    g_target_cntr_offset_y_arr = get_column_data<float>(test_inputs, "g_target_cntr_offset_y");
    g_target_height_arr = get_column_data<float>(test_inputs, "g_target_height");
    g_target_width_arr = get_column_data<float>(test_inputs, "g_target_width");
    g_target_aspect_arr = get_column_data<float>(test_inputs, "g_target_aspect");
    g_target_left_arr = get_column_data<float>(test_inputs, "g_target_left");
    g_target_right_arr = get_column_data<float>(test_inputs, "g_target_right");
    g_target_top_arr = get_column_data<float>(test_inputs, "g_target_top");
    g_target_bottom_arr = get_column_data<float>(test_inputs, "g_target_bottom");

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

    // Additional initializations
    time_prv = 0.0;
    num_test_steps = g_app_elapsed_time_arr.size();
}

void init_test_output (void)
{
    test_output_path = "./";
    test_output_file_name = "follow_output";
    test_unique_output_file_name = generate_unique_filename(test_output_path, test_output_file_name);

    // Create csv file to save outputs to
    test_output_file.open(test_unique_output_file_name);
    
    if (!test_output_file.is_open()) 
    {
        std::cerr << "Error opening output file!" << std::endl;
        return;
    }

    // Write csv Header
    test_output_file << "g_app_elapsed_time,"
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
                << "g_delta_angle,"
                << "g_camera_tilt_angle,"
                << "g_delta_d_x,"
                << "g_delta_d_z,"
                << "g_camera_comp_angle,"
                << "g_target_too_close,"
                << "g_x_error,"
                << "g_y_error,"
                << "g_vx_adjust,"
                << "g_vy_adjust,"
                << "g_vz_adjust,"
                << "g_x_target_ekf,"
                << "g_y_target_ekf,"
                << "g_vx_target_ekf,"
                << "g_vy_target_ekf,"
                << "g_ax_target_ekf,"
                << "g_ay_target_ekf\n";
}

void init_software_components (void)
{

    // Initialize all inputs to the software component  
    Localize::init();
    PathPlanner::init();
}

void get_test_inputs (size_t data_index)
{
    // Set inputs
    g_app_elapsed_time = g_app_elapsed_time_arr[data_index];
    g_mav_veh_pitch = g_mav_veh_pitch_arr[data_index];
    g_target_valid = g_target_valid_arr[data_index];
    g_target_detection_id = g_target_detection_id_arr[data_index];
    g_target_track_id = g_target_track_id_arr[data_index];
    g_target_cntr_offset_x = g_target_cntr_offset_x_arr[data_index];
    g_target_cntr_offset_y = g_target_cntr_offset_y_arr[data_index];
    g_target_height = g_target_height_arr[data_index];
    g_target_width = g_target_width_arr[data_index];
    g_target_aspect = g_target_aspect_arr[data_index];
    g_target_left = g_target_left_arr[data_index];
    g_target_right = g_target_right_arr[data_index];
    g_target_top = g_target_top_arr[data_index];
    g_target_bottom = g_target_bottom_arr[data_index];
    
    // Calculate timestep manually since this is not running on the drone in real time
    g_dt = g_app_elapsed_time - time_prv;
    time_prv = g_app_elapsed_time;

    // Have to calculate the bounding box for the localize algo since
    // it's calculated in the track_target function (or update the inputs
    // to this test case and call Track::loop)
    g_target_center_y = (g_target_left + g_target_right) / 2.0f;
    g_target_center_x = (g_target_bottom + g_target_top) / 2.0f;
}

void write_test_outputs (void)
{
    // Write the latest values of global variables directly to the CSV
    test_output_file << g_app_elapsed_time << ","
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
                << g_delta_angle << ","
                << g_camera_tilt_angle << ","
                << g_delta_d_x << ","
                << g_delta_d_z << ","
                << g_camera_comp_angle << ","
                << g_target_too_close << ","
                << g_x_error << ","
                << g_y_error << ","
                << g_vx_adjust << ","
                << g_vy_adjust << ","
                << g_vz_adjust << ","
                << g_x_target_ekf << ","
                << g_y_target_ekf << ","
                << g_vx_target_ekf << ","
                << g_vy_target_ekf << ","
                << g_ax_target_ekf << ","
                << g_ay_target_ekf << "\n";
}

void save_test_output (void)
{
    // Close file after writing to it
    test_output_file.close();
    std::cout << "Processing complete. Results saved to" << test_unique_output_file_name << ".csv\n";
}

void run_loops (void)
{
    Localize::loop();
    PathPlanner::loop();
}

void setup(void)
{
    init_test_inputs();
    init_test_output();
    init_software_components();
}

void run(void)
{
    for (size_t i = 0; i < num_test_steps; ++i) 
    {
        if (g_app_elapsed_time_arr[i] != NULL)
        {
            // Input signals at the start of the loopp
            get_test_inputs(i);

            // Calculate outputs using desired functions
            run_loops();
            
            // Save results to csv file
            write_test_outputs();
        }
    }

    //save_test_output();
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