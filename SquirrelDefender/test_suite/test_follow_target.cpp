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
#include "detect_target.h"
#include "track_target.h"
#include "localize_target.h"
#include "path_planner.h"
#include "time_calc.h"
#include "timer.h"
#include "path_planner.h"

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
    X(g_app_elapsed_time)      \
    X(g_target_valid)          \
    X(g_target_detection_id)   \
    X(g_target_track_id)       \
    X(g_detection_class)       \
    X(g_target_detection_conf) \
    X(g_target_cntr_offset_x)  \
    X(g_target_cntr_offset_y)  \
    X(g_target_height)         \
    X(g_target_width)          \
    X(g_target_aspect)         \
    X(g_target_left)           \
    X(g_target_right)          \
    X(g_target_top)            \
    X(g_target_bottom)         \
    X(g_mav_veh_local_ned_vx)  \
    X(g_mav_veh_local_ned_vy)  \
    X(g_mav_veh_imu_ax)        \
    X(g_mav_veh_imu_ay)        \
    X(g_mav_veh_imu_az)        \
    X(g_mav_veh_yaw)           \
    X(g_mav_veh_pitch)         \
    X(g_mav_veh_local_ned_z)

// MODIFY FOR YOUR TEST ONLY - EVERYTHING ELSE STAYS THE SAME
// Computed-only outputs (written only; if you also want an input recorded,
// you can list it here too—dedup is automatic in the header/row)
#define OUTPUT_VARS(X)                                                     \
    X(g_d_target_h)                                                        \
    X(g_d_target_w)                                                        \
    X(g_x_target)                                                          \
    X(g_y_target)                                                          \
    X(g_z_target)                                                          \
    X(g_d_target)                                                          \
    X(g_delta_angle)                                                       \
    X(g_camera_tilt_angle)                                                 \
    X(g_delta_d_x)                                                         \
    X(g_delta_d_z)                                                         \
    X(g_camera_comp_angle)                                                 \
    X(g_target_too_close)                                                  \
    X(g_fov_height)                                                        \
    X(g_x_target_ekf)                                                      \
    X(g_y_target_ekf)                                                      \
    X(g_vx_target_ekf)                                                     \
    X(g_vy_target_ekf)                                                     \
    X(g_ax_target_ekf)                                                     \
    X(g_ay_target_ekf)                                                     \
    X(g_target_cntr_offset_x_m)                                            \
    X(g_meter_per_pix)                                                     \
    X(g_target_cntr_offset_x_mov_avg)                                      \
    X(g_target_cntr_offset_y_mov_avg)                                      \
    X(g_target_data_useful)                                                \
    X(g_line_of_sight)                                                     \
    /* Vehicle Controls */                                                 \
    X(g_x_error)                                                           \
    X(g_y_error)                                                           \
    X(g_vx_adjust)                                                         \
    X(g_vy_adjust)                                                         \
    X(g_vz_adjust)                                                         \
    X(g_yaw_target)                                                        \
    /* Optionally repeat inputs you also want in output CSV (dedup OK): */ \
    X(g_app_elapsed_time)                                                  \
    X(g_target_detection_id)                                               \
    X(g_target_track_id)                                                   \
    X(g_detection_class)                                                   \
    X(g_target_detection_conf)                                             \
    X(g_target_valid)                                                      \
    X(g_target_cntr_offset_x)                                              \
    X(g_target_cntr_offset_y)                                              \
    X(g_target_height)                                                     \
    X(g_target_width)                                                      \
    X(g_target_aspect)                                                     \
    X(g_target_left)                                                       \
    X(g_target_right)                                                      \
    X(g_target_top)                                                        \
    X(g_target_bottom)                                                     \
    X(g_mav_veh_local_ned_vx)                                              \
    X(g_mav_veh_local_ned_vy)                                              \
    X(g_mav_veh_imu_ax)                                                    \
    X(g_mav_veh_imu_ay)                                                    \
    X(g_mav_veh_imu_az)                                                    \
    X(g_mav_veh_yaw)                                                       \
    X(g_mav_veh_pitch)                                                     \
    X(g_mav_veh_local_ned_z)

/********************************************************************************
 * Load input/output lists and all data from the csv input file
 ********************************************************************************/

void init_test_inputs(void)
{
    // Load the CSV input file
    test_inputs = test_params.get_string_param("Input_File", "File_Name");
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
    g_dt = g_app_elapsed_time - time_prv;
    time_prv = g_app_elapsed_time;

    // Calculate target bbox center manually - not logged originally
    g_target_center_y = (g_target_left + g_target_right) / 2.0f;
    g_target_center_x = (g_target_bottom + g_target_top) / 2.0f;
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
    Localize::init();
    PathPlanner::init();
}

void run_loops(void)
{
    // MODIFY FOR YOUR TEST ONLY - EVERYTHING ELSE STAYS THE SAME
    Localize::loop();
    PathPlanner::loop();
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