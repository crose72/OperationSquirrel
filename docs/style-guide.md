# Operation Squirrel C++ Style Guide

This style guide defines the coding conventions for **Squirreldefender**, **OS-Remote bindings**, and all C++ modules in the Operation Squirrel ecosystem. It enforces clarity, maintainability, and an embedded-friendly structure, while supporting cross-platform builds (Jetson, WSL, Windows).

---

# 1. Project Architecture Principles

Operation Squirrel uses a predictable embedded-style lifecycle:

```
init() → loop() → shutdown()
```

All modules that behave like embedded components must expose this 3-function interface.

Modules should be structured as:

```
ClassName::init()
ClassName::loop()
ClassName::shutdown()
```

This keeps all subsystems consistent:

- MAVLink
- Video I/O
- YOLO / SSD detectors
- Status I/O
- Gamepad / command handler
- Flight logic, etc.

---

# 2. Naming Conventions

## 2.1 Classes

**PascalCase**

```cpp
class StatusIO;
class DetectorYOLO;
class DetectorSSD;
class TargetDetection;
class MavMsg;
```

Classes always start with a noun describing what they are or manage.

---

## 2.2 Functions / Methods

**lowerCamelCase**

```cpp
bool init();
void loop();
void shutdown();
void detectTargets();
void saveVideoButtonState();
```

Functions describe what they *do*.

---

## 2.3 Variables

### Local variables → `lower_snake_case`

```cpp
int button_state;
float yaw_rate;
bool save_pressed;
```

### Global variables → `g_` prefix + `lower_snake_case`

```cpp
bool g_app_stop;
bool g_save_button_press;
cv::Mat g_cam0_img_cpu;
```

---

## 2.4 Constants / Calibration Values

**lower\_snake\_case** (runtime constants)

```cpp
const int red_led_pin = 18;
const float target_det_conf_thresh = 0.45f;
const uint8_t sender_sys_id = 0;
```

These are variables stored in memory and treated like configuration, **not macros**.

---

## 2.5 Compile-Time Preprocessor Flags

**ALL\_CAPS\_WITH\_UNDERSCORES**

```cpp
#define ENABLE_CV
#define BLD_JETSON_ORIN_NANO
#define BLD_WIN
```

Never use lowercase for macros.

---

## 2.6 Enums

Enums use **PascalCase type name** + **ALL\_CAPS members**

```cpp
enum class ControlMode {
    MANUAL,
    AUTO,
    FAILSAFE
};
```

---

# 3. File Naming Conventions

| Purpose             | Filename Example                        |
| ------------------- | --------------------------------------- |
| YOLO/SSD detectors  | `detector_yolo.cpp`, `detector_ssd.cpp` |
| Model arbitrator    | `target_detection.cpp`                  |
| MAVLink subsystem   | `mav_data_hub.cpp`                      |
| Global calibrations | `global_calibrations.cpp`               |
| Global objects      | `global_objects.cpp`                    |
| GPIO / status I/O   | `status_io.cpp`                         |
| Video I/O           | `video_io.cpp`                          |

**Rules:**

- lowercase with underscores
- filenames describe modules, not logic
- only one module/class per file

---

# 4. Code Layout & Structure

Every `.cpp` file must follow this ordering:

```
File header comment
Includes
Typedefs
Private macros
Object definitions
Calibration definitions
Function definitions
```

This keeps the project consistent and navigable.

---

# 5. Commenting Conventions

### Section headers

Use long star-block headers:

```cpp
/********************************************************************************/
```

### Function headers (Doxygen-style)

```cpp
/********************************************************************************
 * Function: status_good
 * Description: Indicate system health with the green LED.
 ********************************************************************************/
```

---

# 6. Embedded Lifecycle Rules

Modules must:

- Perform allocation/config in `init()`
- Perform per-loop work in `loop()`
- Free resources in `shutdown()`
- Never allocate in `loop()`
- Never block unnecessarily in real-time sections

---

# 7. Platform Selection Rules

Platform selection should only occur at module boundaries:

```cpp
#if defined(BLD_JETSON_ORIN_NANO)
// TensorRT backend
#elif defined(BLD_WIN)
// OpenCV DNN backend
#endif
```

Never mix platform code inside core algorithmic functions.

---

# 8. Expected Folder Structure

```
squirreldefender/
    detectors/
        detector_yolo.*
        detector_ssd.*
        target_detection.*
    mavlink/
        mav_data_hub.*
        mav_utils.*
    io/
        video_io.*
        status_io.*
    globals/
        global_calibrations.*
        global_objects.*
    control/
        pid.*
        commander.*
        actuator_mapper.*
    main.cpp
```

---

# 9. JSON Configuration Rules

JSON parameters **must match internal naming** using lowercase snake\_case:

```json
{
  "target_det_params": {
    "conf_thresh": 0.45,
    "max_batch_size": 4
  }
}
```

Which maps to:

```cpp
float conf_thresh;
int max_batch_size;
```

---

# 10. Things to Avoid

- Don’t put macros inside namespaces
- Don’t uppercase runtime constants
- Don’t mix naming conventions
- Don’t use camelCase for filenames
- Don’t bury platform `#ifdef`s inside deep logic

---

# 11. Example Module Template

```cpp
/********************************************************************************
 * @file    module_name.cpp
 * @brief   One sentence summary.
 ********************************************************************************/

#include "module_name.h"

// Object definitions
static int module_state = 0;

// Calibration
const float update_rate_hz = 40.0f;

bool ModuleName::init() { return true; }
void ModuleName::loop() {}
void ModuleName::shutdown() {}
```

---

# 12. Final Notes

This guide reflects the direction Operation Squirrel is already moving toward: clean, modern embedded C++ with predictable structure.

Following it will:

- Make modules interchangeable
- Keep refactors painless
- Ensure long-term maintainability
- Enable future automated documentation

If you'd like, a matching **clang-format config** can be generated next.

