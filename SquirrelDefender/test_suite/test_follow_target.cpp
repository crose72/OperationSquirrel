#include "test_utils/test_csv_utils.h"
#include "global_objects.h"
#include "localize_target.h"
#include "delivery_planner.h"
#include "time_calc.h"
#include "datalog.h"
#include "param_reader.h"

/********************************************************************************
 * Setup
 ********************************************************************************/
ParamReader test_params("../test_suite/test_params.json");
std::string test_inputs;
std::string test_output_path;
std::string test_output_file_name;
std::string test_unique_output_file_name;
std::ofstream test_output_file;
size_t num_test_steps;
float time_prv;

std::vector<float> g_app_elapsed_time_arr;
std::vector<float> g_target_valid_arr;
std::vector<float> g_target_detection_num_arr;
std::vector<float> g_target_track_id_arr;
std::vector<float> g_detection_class_arr;
std::vector<float> g_target_detection_conf_arr;
std::vector<float> g_target_cntr_offset_x_arr;
std::vector<float> g_target_cntr_offset_y_arr;
std::vector<float> g_target_cntr_offset_x_filt_arr;
std::vector<float> g_target_cntr_offset_y_filt_arr;
std::vector<float> g_target_height_arr;
std::vector<float> g_target_width_arr;
std::vector<float> g_target_aspect_arr;
std::vector<float> g_target_left_arr;
std::vector<float> g_target_right_arr;
std::vector<float> g_target_top_arr;
std::vector<float> g_target_bottom_arr;
std::vector<float> g_mav_veh_local_ned_vx_arr;
std::vector<float> g_mav_veh_local_ned_vy_arr;
std::vector<float> g_mav_veh_imu_ax_arr;
std::vector<float> g_mav_veh_imu_ay_arr;
std::vector<float> g_mav_veh_imu_az_arr;
std::vector<float> g_mav_veh_yaw_arr;
std::vector<float> g_mav_veh_pitch_arr;
std::vector<float> g_mav_veh_local_ned_z_arr;
std::vector<float> g_yaw_target_arr;

void init_test_inputs(void)
{
    test_inputs = test_params.get_string_param("Input_File", "File_Name");

    // Create array of input signals for playback
    g_app_elapsed_time_arr = get_column_data<float>(test_inputs, "g_app_elapsed_time");
    g_target_valid_arr = get_column_data<float>(test_inputs, "g_target_valid");
    g_target_detection_num_arr = get_column_data<float>(test_inputs, "g_target_detection_num");
    g_target_track_id_arr = get_column_data<float>(test_inputs, "g_target_track_id");
    g_detection_class_arr = get_column_data<float>(test_inputs, "g_detection_class");
    g_target_detection_conf_arr = get_column_data<float>(test_inputs, "g_target_detection_conf");
    g_target_cntr_offset_x_arr = get_column_data<float>(test_inputs, "g_target_cntr_offset_x");
    g_target_cntr_offset_y_arr = get_column_data<float>(test_inputs, "g_target_cntr_offset_y");
    g_target_cntr_offset_x_filt_arr = get_column_data<float>(test_inputs, "g_target_cntr_offset_x_filt");
    g_target_cntr_offset_y_filt_arr = get_column_data<float>(test_inputs, "g_target_cntr_offset_y_filt");
    g_target_height_arr = get_column_data<float>(test_inputs, "g_target_height");
    g_target_width_arr = get_column_data<float>(test_inputs, "g_target_width");
    g_target_aspect_arr = get_column_data<float>(test_inputs, "g_target_aspect");
    g_target_left_arr = get_column_data<float>(test_inputs, "g_target_left");
    g_target_right_arr = get_column_data<float>(test_inputs, "g_target_right");
    g_target_top_arr = get_column_data<float>(test_inputs, "g_target_top");
    g_target_bottom_arr = get_column_data<float>(test_inputs, "g_target_bottom");
    g_mav_veh_local_ned_vx_arr = get_column_data<float>(test_inputs, "g_mav_veh_local_ned_vx");
    g_mav_veh_local_ned_vy_arr = get_column_data<float>(test_inputs, "g_mav_veh_local_ned_vy");
    g_mav_veh_imu_ax_arr = get_column_data<float>(test_inputs, "g_mav_veh_imu_ax");
    g_mav_veh_imu_ay_arr = get_column_data<float>(test_inputs, "g_mav_veh_imu_ay");
    g_mav_veh_imu_az_arr = get_column_data<float>(test_inputs, "g_mav_veh_imu_az");
    g_mav_veh_yaw_arr = get_column_data<float>(test_inputs, "g_mav_veh_yaw");
    g_mav_veh_pitch_arr = get_column_data<float>(test_inputs, "g_mav_veh_pitch");
    g_mav_veh_local_ned_z_arr = get_column_data<float>(test_inputs, "g_mav_veh_local_ned_z");
    g_yaw_target_arr = get_column_data<float>(test_inputs, "g_yaw_target");

    // Check if any input signal arrays are empty
    if (g_app_elapsed_time_arr.empty() ||
        g_target_valid_arr.empty() ||
        g_target_detection_num_arr.empty() ||
        g_target_track_id_arr.empty() ||
        g_target_cntr_offset_x_arr.empty() ||
        g_target_cntr_offset_y_arr.empty() ||
        g_target_cntr_offset_x_filt_arr.empty() ||
        g_target_cntr_offset_y_filt_arr.empty() ||
        g_target_height_arr.empty() ||
        g_target_width_arr.empty() ||
        g_target_aspect_arr.empty() ||
        g_target_left_arr.empty() ||
        g_target_right_arr.empty() ||
        g_target_top_arr.empty() ||
        g_target_bottom_arr.empty() ||
        g_mav_veh_local_ned_vx_arr.empty() ||
        g_mav_veh_local_ned_vy_arr.empty() ||
        g_mav_veh_imu_ax_arr.empty() ||
        g_mav_veh_imu_ay_arr.empty() ||
        g_mav_veh_imu_az_arr.empty() ||
        g_mav_veh_yaw_arr.empty() ||
        g_mav_veh_pitch_arr.empty() ||
        g_yaw_target_arr.empty())
    {
        std::cerr << "Error: No signal data found!" << std::endl;
        return;
    }

    // Additional initializations
    time_prv = 0.0;
    num_test_steps = g_app_elapsed_time_arr.size();
}

void init_test_output(void)
{
    test_output_path = test_params.get_string_param("Output_File", "File_Path");
    test_output_file_name = test_params.get_string_param("Output_File", "File_Name");
    test_unique_output_file_name = generate_unique_filename(test_output_path, test_output_file_name);

    // Create csv file to save outputs to
    test_output_file.open(test_unique_output_file_name);

    if (!test_output_file.is_open())
    {
        std::cerr << "Error opening output file!" << std::endl;
        return;
    }

    // Write csv Header
    test_output_file
        << "g_app_elapsed_time,"
        /* Start Detection Info */
        << "g_target_detection_num,"
        << "g_target_track_id,"
        << "g_detection_class,"
        << "g_target_detection_conf,"
        /* End Detection Info */
        /* Start Tracking Info */
        << "g_target_valid,"
        << "g_target_cntr_offset_x,"
        << "g_target_cntr_offset_y,"
        << "g_target_height,"
        << "g_target_width,"
        << "g_target_aspect,"
        << "g_target_left,"
        << "g_target_right,"
        << "g_target_top,"
        << "g_target_bottom,"
        /* End Tracking Info */
        /* Start Localization Info */
        << "g_d_target_h,"
        << "g_d_target_w,"
        << "g_x_target,"
        << "g_y_target,"
        << "g_z_target,"
        << "g_d_target,"
        << "g_delta_angle,"
        << "g_camera_tilt_angle,"
        << "g_delta_d_x,"
        << "g_delta_d_z,"
        << "g_camera_comp_angle,"
        << "g_target_too_close,"
        << "g_fov_height,"
        << "g_x_target_ekf,"
        << "g_y_target_ekf,"
        << "g_vx_target_ekf,"
        << "g_vy_target_ekf,"
        << "g_ax_target_ekf,"
        << "g_ay_target_ekf,"
        << "g_target_cntr_offset_x_m,"
        << "g_meter_per_pix,"
        << "g_target_cntr_offset_x_mov_avg,"
        << "g_target_cntr_offset_y_mov_avg,"
        << "g_target_data_useful,"
        << "g_line_of_sight,"
        /* End Localization Info */
        /* Start Vehicle Controls */
        << "g_x_error,"
        << "g_y_error,"
        << "g_vx_adjust,"
        << "g_vy_adjust,"
        << "g_vz_adjust,"
        << "g_yaw_target,"
        /* End Vehicle Controls */
        /* Start Mavlink data */
        << "g_mav_veh_local_ned_vx,"
        << "g_mav_veh_local_ned_vy,"
        << "g_mav_veh_imu_ax,"
        << "g_mav_veh_imu_ay,"
        << "g_mav_veh_imu_az,"
        << "g_mav_veh_yaw,"
        << "g_mav_veh_pitch"
        /* End Mavlink data */
        << "\n";
}

void init_software_components(void)
{
    // Initialize all inputs to the software component
    Localize::init();
    PathPlanner::init();
}

void get_test_inputs(size_t data_index)
{
    // Set inputs
    g_app_elapsed_time = g_app_elapsed_time_arr[data_index];
    g_target_valid = g_target_valid_arr[data_index];
    g_target_detection_num = g_target_detection_num_arr[data_index];
    g_target_track_id = g_target_track_id_arr[data_index];
    g_detection_class = g_detection_class_arr[data_index];
    g_target_detection_conf = g_target_detection_conf_arr[data_index];
    g_target_cntr_offset_x = g_target_cntr_offset_x_arr[data_index];
    g_target_cntr_offset_y = g_target_cntr_offset_y_arr[data_index];
    g_target_cntr_offset_x_filt = g_target_cntr_offset_x_filt_arr[data_index];
    g_target_cntr_offset_y_filt = g_target_cntr_offset_y_filt_arr[data_index];
    g_target_height = g_target_height_arr[data_index];
    g_target_width = g_target_width_arr[data_index];
    g_target_aspect = g_target_aspect_arr[data_index];
    g_target_left = g_target_left_arr[data_index];
    g_target_right = g_target_right_arr[data_index];
    g_target_top = g_target_top_arr[data_index];
    g_target_bottom = g_target_bottom_arr[data_index];
    g_mav_veh_local_ned_vx = g_mav_veh_local_ned_vx_arr[data_index];
    g_mav_veh_local_ned_vy = g_mav_veh_local_ned_vy_arr[data_index];
    g_mav_veh_imu_ax = g_mav_veh_imu_ax_arr[data_index];
    g_mav_veh_imu_ay = g_mav_veh_imu_ay_arr[data_index];
    g_mav_veh_imu_az = g_mav_veh_imu_az_arr[data_index];
    g_mav_veh_yaw = g_mav_veh_yaw_arr[data_index];
    g_mav_veh_pitch = g_mav_veh_pitch_arr[data_index];
    g_mav_veh_local_ned_z = g_mav_veh_local_ned_z_arr[data_index];
    g_yaw_target = g_yaw_target_arr[data_index];

    // Calculate timestep manually since this is not running on the drone in real time
    g_dt = g_app_elapsed_time - time_prv;
    time_prv = g_app_elapsed_time;

    // Have to calculate the bounding box for the localize algo since
    // it's calculated in the track_target function (or update the inputs
    // to this test case and call Track::loop)
    g_target_center_y = (g_target_left + g_target_right) / 2.0f;
    g_target_center_x = (g_target_bottom + g_target_top) / 2.0f;
}

void run_loops(void)
{
    Localize::loop();
    PathPlanner::loop();
}

void write_test_outputs(void)
{
    // Write the latest values of global variables directly to the CSV
    test_output_file
        << g_app_elapsed_time << ","
        /* Start Detection Info */
        << g_target_detection_num << ","
        << g_target_track_id << ","
        << g_detection_class << ","
        << g_target_detection_conf << ","
        /* End Detection Info */
        /* Start Tracking Info */
        << g_target_valid << ","
        << g_target_cntr_offset_x << ","
        << g_target_cntr_offset_y << ","
        << g_target_height << ","
        << g_target_width << ","
        << g_target_aspect << ","
        << g_target_left << ","
        << g_target_right << ","
        << g_target_top << ","
        << g_target_bottom << ","
        /* End Tracking Info */
        /* Start Localization Info */
        << g_d_target_h << ","
        << g_d_target_w << ","
        << g_x_target << ","
        << g_y_target << ","
        << g_z_target << ","
        << g_d_target << ","
        << g_delta_angle << ","
        << g_camera_tilt_angle << ","
        << g_delta_d_x << ","
        << g_delta_d_z << ","
        << g_camera_comp_angle << ","
        << g_target_too_close << ","
        << g_fov_height << ","
        << g_x_target_ekf << ","
        << g_y_target_ekf << ","
        << g_vx_target_ekf << ","
        << g_vy_target_ekf << ","
        << g_ax_target_ekf << ","
        << g_ay_target_ekf << ","
        << g_target_cntr_offset_x_m << ","
        << g_meter_per_pix << ","
        << g_target_cntr_offset_x_mov_avg << ","
        << g_target_cntr_offset_y_mov_avg << ","
        << g_target_data_useful << ","
        << g_line_of_sight << ","
        /* End Localization Info */
        /* Start Vehicle Controls */
        << g_x_error << ","
        << g_y_error << ","
        << g_vx_adjust << ","
        << g_vy_adjust << ","
        << g_vz_adjust << ","
        << g_yaw_target << ","
        /* End Vehicle Controls */
        /* Start Mavlink data */
        << g_mav_veh_local_ned_vx << ","
        << g_mav_veh_local_ned_vy << ","
        << g_mav_veh_imu_ax << ","
        << g_mav_veh_imu_ay << ","
        << g_mav_veh_imu_az << ","
        << g_mav_veh_yaw << ","
        << g_mav_veh_pitch
        /* End Mavlink data */
        << "\n";
}

void save_test_output(void)
{
    // Close file after writing to it
    test_output_file.close();
    std::cout << "Processing complete. Results saved to" << test_unique_output_file_name << "\n";
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

    save_test_output();
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