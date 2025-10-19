#include "test_utils/test_csv_utils.h"
#include "test_utils/csv_reader.h"
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
size_t num_test_steps = 0;
float time_prv = 0.0f;

// NEW: single CSV loader, no more per-column vectors
std::unique_ptr<CSVReader> g_csv;

void init_test_inputs(void)
{
    test_inputs = test_params.get_string_param("Input_File", "File_Name");

    // Load the CSV once
    g_csv = std::make_unique<CSVReader>(test_inputs, ',');

    // Basic sanity checks for required columns
    const char *required_cols[] = {
        "g_app_elapsed_time",
        "g_target_valid",
        "g_target_detection_num",
        "g_target_track_id",
        "g_detection_class",
        "g_target_detection_conf",
        "g_target_cntr_offset_x",
        "g_target_cntr_offset_y",
        "g_target_cntr_offset_x_filt",
        "g_target_cntr_offset_y_filt",
        "g_target_height",
        "g_target_width",
        "g_target_aspect",
        "g_target_left",
        "g_target_right",
        "g_target_top",
        "g_target_bottom",
        "g_mav_veh_local_ned_vx",
        "g_mav_veh_local_ned_vy",
        "g_mav_veh_imu_ax",
        "g_mav_veh_imu_ay",
        "g_mav_veh_imu_az",
        "g_mav_veh_yaw",
        "g_mav_veh_pitch",
        "g_mav_veh_local_ned_z",
        "g_yaw_target"};

    for (const char *col : required_cols)
    {
        if (!g_csv->hasColumn(col))
        {
            throw std::runtime_error(std::string("Missing required column in CSV: ") + col);
        }
    }

    num_test_steps = g_csv->rows();
    if (num_test_steps == 0)
    {
        std::cerr << "Error: No signal data rows found!\n";
    }

    time_prv = 0.0f;
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
    // Read inputs directly from CSV by header at the row index
    g_app_elapsed_time = (float)(*g_csv)("g_app_elapsed_time", data_index);
    g_target_valid = (float)(*g_csv)("g_target_valid", data_index);
    g_target_detection_num = (float)(*g_csv)("g_target_detection_num", data_index);
    g_target_track_id = (float)(*g_csv)("g_target_track_id", data_index);
    g_detection_class = (float)(*g_csv)("g_detection_class", data_index);
    g_target_detection_conf = (float)(*g_csv)("g_target_detection_conf", data_index);
    g_target_cntr_offset_x = (float)(*g_csv)("g_target_cntr_offset_x", data_index);
    g_target_cntr_offset_y = (float)(*g_csv)("g_target_cntr_offset_y", data_index);
    g_target_cntr_offset_x_filt = (float)(*g_csv)("g_target_cntr_offset_x_filt", data_index);
    g_target_cntr_offset_y_filt = (float)(*g_csv)("g_target_cntr_offset_y_filt", data_index);
    g_target_height = (float)(*g_csv)("g_target_height", data_index);
    g_target_width = (float)(*g_csv)("g_target_width", data_index);
    g_target_aspect = (float)(*g_csv)("g_target_aspect", data_index);
    g_target_left = (float)(*g_csv)("g_target_left", data_index);
    g_target_right = (float)(*g_csv)("g_target_right", data_index);
    g_target_top = (float)(*g_csv)("g_target_top", data_index);
    g_target_bottom = (float)(*g_csv)("g_target_bottom", data_index);
    g_mav_veh_local_ned_vx = (float)(*g_csv)("g_mav_veh_local_ned_vx", data_index);
    g_mav_veh_local_ned_vy = (float)(*g_csv)("g_mav_veh_local_ned_vy", data_index);
    g_mav_veh_imu_ax = (float)(*g_csv)("g_mav_veh_imu_ax", data_index);
    g_mav_veh_imu_ay = (float)(*g_csv)("g_mav_veh_imu_ay", data_index);
    g_mav_veh_imu_az = (float)(*g_csv)("g_mav_veh_imu_az", data_index);
    g_mav_veh_yaw = (float)(*g_csv)("g_mav_veh_yaw", data_index);
    g_mav_veh_pitch = (float)(*g_csv)("g_mav_veh_pitch", data_index);
    g_mav_veh_local_ned_z = (float)(*g_csv)("g_mav_veh_local_ned_z", data_index);
    g_yaw_target = (float)(*g_csv)("g_yaw_target", data_index);

    // Calculate timestep manually since this is not running on the drone in real time
    g_dt = g_app_elapsed_time - time_prv;
    time_prv = g_app_elapsed_time;

    // Derive bbox center for the localize algo (unchanged logic)
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
    std::cout << "Processing complete. Results saved to " << test_unique_output_file_name << "\n";
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
        // Using CSV row count as source of truth; no NULL checks needed
        get_test_inputs(i);
        run_loops();
        write_test_outputs();
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
