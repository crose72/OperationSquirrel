#include "test_utils/test_harness.h"
#include "test_utils/csv_reader.h"
#include "test_utils/test_utils.h"

// MODIFY FOR YOUR TEST ONLY - EVERYTHING ELSE STAYS THE SAME
#include "global_objects.h"
#include "param_reader.h"
#include "datalog.h"
#include "video_io.h"
#include "system_controller.h"
#include "mav_data_hub.h"
#include "mav_utils.h"
#include "vehicle_controller.h"
#include "target_detection.h"
#include "target_tracking.h"
#include "target_localization.h"
#include "velocity_controller.h"
#include "time_calc.h"
#include "timer.h"
#include "velocity_controller.h"

/********************************************************************************
 * Globals
 ********************************************************************************/
ParamReader test_params("../test_suite/test_params.json");
std::string test_inputs;
std::string test_output_path;
std::string test_output_file_name;
std::string test_unique_output_file_name;
std::ofstream test_output_file;
size_t num_test_steps = 0;
float time_prv = 0.0f;
std::unique_ptr<CSVReader> g_csv;
TestHarness g_h;

/********************************************************************************
 * Inputs/outputs list
 ********************************************************************************/

// MODIFY FOR YOUR TEST ONLY - EVERYTHING ELSE STAYS THE SAME
// Input variables
#define INPUT_VARS(X)          \
    X(g_app_time_s)            \
    X(g_tgt_valid)             \
    X(g_tgt_detect_id)         \
    X(g_tgt_track_id)          \
    X(g_tgt_class_id)          \
    X(g_tgt_conf)              \
    X(g_tgt_cntr_offset_y_pix) \
    X(g_tgt_cntr_offset_x_pix) \
    X(g_tgt_height_pix)        \
    X(g_tgt_width_pix)         \
    X(g_tgt_aspect_ratio)      \
    X(g_tgt_left_px)           \
    X(g_tgt_right_px)          \
    X(g_tgt_top_px)            \
    X(g_tgt_bottom_px)         \
    X(g_mav_veh_vel_ned_x)     \
    X(g_mav_veh_vel_ned_y)     \
    X(g_mav_imu_accel_x)       \
    X(g_mav_imu_accel_y)       \
    X(g_mav_imu_accel_z)       \
    X(g_mav_veh_yaw_rad)       \
    X(g_mav_veh_pitch_rad)     \
    X(g_mav_veh_pos_ned_z)

// MODIFY FOR YOUR TEST ONLY - EVERYTHING ELSE STAYS THE SAME
// Computed-only outputs (written only; if you also want an input recorded,
// you can list it here too—dedup is automatic in the header/row)
#define OUTPUT_VARS(X)                                                     \
    X(g_tgt_los_dist_from_pix_height)                                      \
    X(g_tgt_los_dist_from_pix_width)                                       \
    X(g_tgt_pos_x_meas)                                                    \
    X(g_tgt_pos_y_meas)                                                    \
    X(g_tgt_pos_z_meas)                                                    \
    X(g_tgt_los_dist_meas)                                                 \
    X(g_cam0_delta_angle_rad)                                              \
    X(g_cam0_angle_rad)                                                    \
    X(g_tgt_pos_x_delta)                                                   \
    X(g_tgt_pos_z_delta)                                                   \
    X(g_cam0_comp_angle_rad)                                               \
    X(g_tgt_too_close)                                                     \
    X(g_cam0_fov_height)                                                   \
    X(g_tgt_pos_x_est)                                                     \
    X(g_tgt_pos_y_est)                                                     \
    X(g_tgt_vel_x_est)                                                     \
    X(g_tgt_vel_y_est)                                                     \
    X(g_tgt_acc_x_est)                                                     \
    X(g_tgt_acc_y_est)                                                     \
    X(g_tgt_cntr_offset_x_m)                                               \
    X(g_cam0_m_per_pix)                                                    \
    X(g_tgt_cntr_offset_x_pix_filt)                                        \
    X(g_tgt_cntr_offset_y_pix_filt)                                        \
    X(g_tgt_meas_valid)                                                    \
    X(g_cam0_los_m)                                                        \
    /* Vehicle Controls */                                                 \
    X(g_pos_err_x)                                                         \
    X(g_pos_err_y)                                                         \
    X(g_ctrl_vel_x_cmd)                                                    \
    X(g_ctrl_vel_y_cmd)                                                    \
    X(g_ctrl_vel_z_cmd)                                                    \
    X(g_ctrl_yaw_tgt)                                                      \
    /* Optionally repeat inputs you also want in output CSV (dedup OK): */ \
    X(g_app_time_s)                                                        \
    X(g_tgt_detect_id)                                                     \
    X(g_tgt_track_id)                                                      \
    X(g_tgt_class_id)                                                      \
    X(g_tgt_conf)                                                          \
    X(g_tgt_valid)                                                         \
    X(g_tgt_cntr_offset_y_pix)                                             \
    X(g_tgt_cntr_offset_x_pix)                                             \
    X(g_tgt_height_pix)                                                    \
    X(g_tgt_width_pix)                                                     \
    X(g_tgt_aspect_ratio)                                                  \
    X(g_tgt_left_px)                                                       \
    X(g_tgt_right_px)                                                      \
    X(g_tgt_top_px)                                                        \
    X(g_tgt_bottom_px)                                                     \
    X(g_mav_veh_vel_ned_x)                                                 \
    X(g_mav_veh_vel_ned_y)                                                 \
    X(g_mav_imu_accel_x)                                                   \
    X(g_mav_imu_accel_y)                                                   \
    X(g_mav_imu_accel_z)                                                   \
    X(g_mav_veh_yaw_rad)                                                   \
    X(g_mav_veh_pitch_rad)                                                 \
    X(g_mav_veh_pos_ned_z)

/********************************************************************************
 * Load input/output lists and all data from the csv input file
 ********************************************************************************/

void init_test_inputs(void)
{
    // Load the CSV input file
    std::string test_input_file_path = test_params.get_string_param("Input_File", "File_Path");
    std::string test_input_file_name = test_params.get_string_param("Input_File", "File_Name");
    test_inputs = test_input_file_path + test_input_file_name;
    g_csv = std::make_unique<CSVReader>(test_inputs, ',');

    // Attach CSV to the test harness and assign inputs to be set
    // and outputs to be written
    g_h.attachCSV(*g_csv);
    INPUT_VARS(TESTHARNESS_REGISTER_INPUT);
    OUTPUT_VARS(TESTHARNESS_REGISTER_OUTPUT);

    // Verify input lists successfully loaded
    g_h.validateInputs();

    // Make sure the test isn't empty
    num_test_steps = g_h.rows();

    if (num_test_steps == 0)
    {
        std::cerr << "Error: No inputs found!\n";
    }
}

void init_test_outputs(void)
{
    test_output_path = test_params.get_string_param("Output_File", "File_Path");
    test_output_file_name = test_params.get_string_param("Output_File", "File_Name");
    test_unique_output_file_name = generate_unique_filename(test_output_path, test_output_file_name);

    test_output_file.open(test_unique_output_file_name);
    if (!test_output_file.is_open())
    {
        std::cerr << "Error: opening output file!" << std::endl;
        return;
    }

    // headers: inputs first, then outputs-not-in-inputs (dedup)
    g_h.writeHeader(test_output_file);
}

void get_test_inputs(size_t data_index)
{
    // Set inputs from CSV
    g_h.setInputs(data_index);

    // MODIFY FOR YOUR TEST ONLY - EVERYTHING ELSE STAYS THE SAME
    // Manual calculations as needed
    // Manual calculate dt since it's not logged
    g_app_dt = g_app_time_s - time_prv;
    time_prv = g_app_time_s;

    // Calculate target bbox center manually - not logged originally
    g_tgt_center_x_px = (g_tgt_left_px + g_tgt_right_px) / 2.0f;
    g_tgt_center_y_px = (g_tgt_bottom_px + g_tgt_top_px) / 2.0f;
}

void write_test_outputs(void)
{
    g_h.writeRow(test_output_file);
}

void save_test_output(void)
{
    test_output_file.close();
    std::cout << "Processing complete. Results saved to " << test_unique_output_file_name << "\n";
}

void init_software_components(void)
{
    // MODIFY FOR YOUR TEST ONLY - EVERYTHING ELSE STAYS THE SAME
    // Initialize all inputs to the software component
    TargetLocalization::init();
    VelocityController::init();
}

void run_loops(void)
{
    // MODIFY FOR YOUR TEST ONLY - EVERYTHING ELSE STAYS THE SAME
    TargetLocalization::loop();
    VelocityController::loop();
}

void setup(void)
{
    init_test_inputs();
    init_test_outputs();
    init_software_components();
}

void run(void)
{
    for (size_t i = 0; i < num_test_steps; ++i)
    {
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

#undef INPUT_VARS
#undef OUTPUT_VARS