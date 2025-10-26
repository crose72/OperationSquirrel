#ifdef ENABLE_CV

/********************************************************************************
 * @file    track_target.h
 * @author  Cameron Rose
 * @date    1/22/2025
 ********************************************************************************/
#ifndef TRACK_TARGET_H
#define TRACK_TARGET_H

/********************************************************************************
 * Includes
 ********************************************************************************/

/********************************************************************************
 * Exported objects
 ********************************************************************************/
extern bool g_target_valid;
extern int g_target_detection_id;
extern int g_target_track_id;
extern float g_target_cntr_offset_x;
extern float g_target_cntr_offset_y;
extern float g_target_height;
extern float g_target_width;
extern float g_target_aspect;
extern float g_target_left;
extern float g_target_right;
extern float g_target_top;
extern float g_target_bottom;
extern float g_target_center_x;
extern float g_target_center_y;
extern float g_detection_class;
extern float g_target_detection_conf;
extern float g_target_cntr_offset_x_filt;
extern float g_target_cntr_offset_y_filt;

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/
class Tracking
{
public:
    Tracking();
    ~Tracking();

    static bool init(void);
    static void loop(void);
    static void shutdown(void);

private:
};

#endif // TRACK_TARGET_H

#endif // ENABLE_CV
