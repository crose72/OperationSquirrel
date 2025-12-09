#ifdef ENABLE_CV

/********************************************************************************
 * @file    target_tracking.h
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   Extracts and maintains per-frame tracking information for the target,
 *          including bounding-box geometry, pixel offsets, class ID, and
 *          confidence. Acts as a lightweight tracking layer between detection
 *          and localization.
 ********************************************************************************/
#ifndef TRACK_TARGET_H
#define TRACK_TARGET_H

/********************************************************************************
 * Includes
 ********************************************************************************/

/********************************************************************************
 * Exported objects
 ********************************************************************************/
extern bool g_tgt_valid;
extern int g_tgt_detect_id;
extern int g_tgt_track_id;
extern float g_tgt_cntr_offset_y_pix;
extern float g_tgt_cntr_offset_x_pix;
extern float g_tgt_height_pix;
extern float g_tgt_width_pix;
extern float g_tgt_aspect_ratio;
extern float g_tgt_left_px;
extern float g_tgt_right_px;
extern float g_tgt_top_px;
extern float g_tgt_bottom_px;
extern float g_tgt_cntr_y_px;
extern float g_tgt_cntr_x_px;
extern float g_tgt_class_id;
extern float g_tgt_conf;

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/
class TargetTracking
{
public:
    TargetTracking();
    ~TargetTracking();

    static bool init(void);
    static void loop(void);
    static void shutdown(void);

private:
};

#endif // TRACK_TARGET_H

#endif // ENABLE_CV
