#include "test_utils/test_csv_utils.h"
#include "test_utils/csv_reader.h"
#include "global_objects.h"
#include "localize_target.h"
#include "delivery_planner.h"
#include "time_calc.h"
#include "datalog.h"
#include "param_reader.h"

#include <unordered_map>
#include <functional>
#include <memory>
#include <fstream>
#include <iostream>
#include <type_traits>

/********************************************************************************
 * ONE-LIST CONFIG (edit these only)
 ********************************************************************************/

// Inputs you want to read from the CSV each step
#define INPUT_VARS(X)              \
    X(g_app_elapsed_time)          \
    X(g_target_valid)              \
    X(g_target_detection_num)      \
    X(g_target_track_id)           \
    X(g_detection_class)           \
    X(g_target_detection_conf)     \
    X(g_target_cntr_offset_x)      \
    X(g_target_cntr_offset_y)      \
    X(g_target_cntr_offset_x_filt) \
    X(g_target_cntr_offset_y_filt) \
    X(g_target_height)             \
    X(g_target_width)              \
    X(g_target_aspect)             \
    X(g_target_left)               \
    X(g_target_right)              \
    X(g_target_top)                \
    X(g_target_bottom)             \
    X(g_mav_veh_local_ned_vx)      \
    X(g_mav_veh_local_ned_vy)      \
    X(g_mav_veh_imu_ax)            \
    X(g_mav_veh_imu_ay)            \
    X(g_mav_veh_imu_az)            \
    X(g_mav_veh_yaw)               \
    X(g_mav_veh_pitch)             \
    X(g_mav_veh_local_ned_z)       \
    X(g_yaw_target)

// Everything you want to write each step (usually inputs + computed vars)
#define OUTPUT_VARS(X)                \
    X(g_app_elapsed_time)             \
    /* Detection Info */              \
    X(g_target_detection_num)         \
    X(g_target_track_id)              \
    X(g_detection_class)              \
    X(g_target_detection_conf)        \
    /* Tracking Info */               \
    X(g_target_valid)                 \
    X(g_target_cntr_offset_x)         \
    X(g_target_cntr_offset_y)         \
    X(g_target_height)                \
    X(g_target_width)                 \
    X(g_target_aspect)                \
    X(g_target_left)                  \
    X(g_target_right)                 \
    X(g_target_top)                   \
    X(g_target_bottom)                \
    /* Localization Info */           \
    X(g_d_target_h)                   \
    X(g_d_target_w)                   \
    X(g_x_target)                     \
    X(g_y_target)                     \
    X(g_z_target)                     \
    X(g_d_target)                     \
    X(g_delta_angle)                  \
    X(g_camera_tilt_angle)            \
    X(g_delta_d_x)                    \
    X(g_delta_d_z)                    \
    X(g_camera_comp_angle)            \
    X(g_target_too_close)             \
    X(g_fov_height)                   \
    X(g_x_target_ekf)                 \
    X(g_y_target_ekf)                 \
    X(g_vx_target_ekf)                \
    X(g_vy_target_ekf)                \
    X(g_ax_target_ekf)                \
    X(g_ay_target_ekf)                \
    X(g_target_cntr_offset_x_m)       \
    X(g_meter_per_pix)                \
    X(g_target_cntr_offset_x_mov_avg) \
    X(g_target_cntr_offset_y_mov_avg) \
    X(g_target_data_useful)           \
    X(g_line_of_sight)                \
    /* Vehicle Controls */            \
    X(g_x_error)                      \
    X(g_y_error)                      \
    X(g_vx_adjust)                    \
    X(g_vy_adjust)                    \
    X(g_vz_adjust)                    \
    X(g_yaw_target)                   \
    /* Mavlink data */                \
    X(g_mav_veh_local_ned_vx)         \
    X(g_mav_veh_local_ned_vy)         \
    X(g_mav_veh_imu_ax)               \
    X(g_mav_veh_imu_ay)               \
    X(g_mav_veh_imu_az)               \
    X(g_mav_veh_yaw)                  \
    X(g_mav_veh_pitch)

/********************************************************************************
 * Setup / globals
 ********************************************************************************/
ParamReader test_params("../test_suite/test_params.json");
std::string test_inputs;
std::string test_output_path;
std::string test_output_file_name;
std::string test_unique_output_file_name;
std::ofstream test_output_file;
size_t num_test_steps = 0;
float time_prv = 0.0f;

// Single CSV loader
std::unique_ptr<CSVReader> g_csv;

// derive arrays of names from the macros
#define TO_STR(name) #name,
static const char *kInputNames[] = {INPUT_VARS(TO_STR)};
static const char *kOutputNames[] = {OUTPUT_VARS(TO_STR)};
static constexpr size_t kInputCount = sizeof(kInputNames) / sizeof(kInputNames[0]);
static constexpr size_t kOutputCount = sizeof(kOutputNames) / sizeof(kOutputNames[0]);
#undef TO_STR

// registries: name -> setter / writer (typed lambdas)
static std::unordered_map<std::string, std::function<void(size_t)>> g_setters;
static std::unordered_map<std::string, std::function<void(std::ostream &)>> g_writers;

/********************************************************************************
 * Registry helpers (auto-typed via decltype(var))
 ********************************************************************************/

template <typename T>
static void reg(const char *name, T &var)
{
    g_setters[name] = [name, &var](size_t row)
    {
        if constexpr (std::is_same_v<T, int16_t>)
        {
            var = static_cast<int16_t>((int)(*g_csv)(name, row));
        }
        else if constexpr (std::is_same_v<T, bool>)
        {
            var = (bool)(*g_csv)(name, row);
        }
        else if constexpr (std::is_enum_v<T>)
        {
            using U = std::underlying_type_t<T>;
            var = static_cast<T>((U)(*g_csv)(name, row));
        }
        else
        {
            var = (T)(*g_csv)(name, row);
        }
    };
    g_writers[name] = [&, name](std::ostream &os)
    {
        if constexpr (std::is_same_v<T, bool>)
        {
            os << (var ? 1 : 0);
        }
        else if constexpr (std::is_same_v<T, int16_t>)
        {
            os << static_cast<int>(var);
        }
        else if constexpr (std::is_enum_v<T>)
        {
            os << static_cast<std::underlying_type_t<T>>(var);
        }
        else
        {
            os << var;
        }
    };
}

// Build the registries automatically FROM THE TWO LISTS (no manual duplication)
static void build_registry()
{
    g_setters.clear();
    g_writers.clear();

// setters/writers for everything in INPUT_VARS and OUTPUT_VARS
// (duplicates are fine; later reg overwrites with identical lambda)
#define REG_LINE(name) reg(#name, name);
    INPUT_VARS(REG_LINE)
    OUTPUT_VARS(REG_LINE)
#undef REG_LINE

    // If you ever want to include extra derived signals for writing only,
    // add them to OUTPUT_VARS instead of touching this function.
}

/********************************************************************************
 * Macro-driven helpers
 ********************************************************************************/

template <size_t N>
static void write_header(std::ostream &os, const char *const (&names)[N])
{
    for (size_t i = 0; i < N; ++i)
    {
        os << names[i];
        if (i + 1 < N)
            os << ",";
    }
    os << "\n";
}

template <size_t N>
static void write_row(std::ostream &os, const char *const (&names)[N])
{
    for (size_t i = 0; i < N; ++i)
    {
        g_writers[names[i]](os);
        if (i + 1 < N)
            os << ",";
    }
    os << "\n";
}

template <size_t N>
static void set_inputs_from_csv(size_t row, const char *const (&names)[N])
{
    for (size_t i = 0; i < N; ++i)
    {
        g_setters[names[i]](row);
    }
}

/********************************************************************************
 * Original flow (unchanged shape)
 ********************************************************************************/

void init_test_inputs(void)
{
    test_inputs = test_params.get_string_param("Input_File", "File_Name");

    // Load the CSV once
    g_csv = std::make_unique<CSVReader>(test_inputs, ',');

    // Build registry of typed setters/writers directly from the two lists
    build_registry();

    // Validate presence and registration
    for (size_t i = 0; i < kInputCount; ++i)
    {
        const char *col = kInputNames[i];
        if (!g_csv->hasColumn(col))
        {
            throw std::runtime_error(std::string("Missing required column in CSV: ") + col);
        }
        if (!g_setters.count(col))
        {
            throw std::runtime_error(std::string("No setter registered for: ") + col);
        }
    }
    for (size_t i = 0; i < kOutputCount; ++i)
    {
        const char *name = kOutputNames[i];
        if (!g_writers.count(name))
        {
            throw std::runtime_error(std::string("No writer registered for: ") + name);
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

    test_output_file.open(test_unique_output_file_name);
    if (!test_output_file.is_open())
    {
        std::cerr << "Error opening output file!" << std::endl;
        return;
    }

    // headers from OUTPUT_VARS
    write_header(test_output_file, kOutputNames);
}

void init_software_components(void)
{
    Localize::init();
    PathPlanner::init();
}

void get_test_inputs(size_t data_index)
{
    // set inputs by name from CSV
    set_inputs_from_csv(data_index, kInputNames);

    // manual dt (not realtime)
    g_dt = g_app_elapsed_time - time_prv;
    time_prv = g_app_elapsed_time;

    // derive bbox center (not in lists—no registry needed)
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
    // row from OUTPUT_VARS
    write_row(test_output_file, kOutputNames);
}

void save_test_output(void)
{
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
    setup();
    run();
    return 0;
}
