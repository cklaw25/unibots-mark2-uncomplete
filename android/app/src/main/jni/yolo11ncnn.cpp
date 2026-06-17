// Tencent is pleased to support the open source community by making ncnn available.
//
// Copyright (C) 2025 THL A29 Limited, a Tencent company. All rights reserved.
//
// Licensed under the BSD 3-Clause License (the "License"); you may not use this file except
// in compliance with the License. You may obtain a copy of the License at
//
// https://opensource.org/licenses/BSD-3-Clause
//
// Unless required by applicable law or agreed to in writing, software distributed
// under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

#include <android/asset_manager_jni.h>
#include <android/native_window_jni.h>
#include <android/native_window.h>

#include <android/log.h>

#include <jni.h>

#include <string>
#include <vector>
#include <cstdlib>
#include <ctime>

#include <platform.h>
#include <benchmark.h>

#include "yolo11.h"
#include "udp_sender.h"

#include <apriltag.h>
#include <tag36h11.h>

#include "ndkcamera.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#if __ARM_NEON
#include <arm_neon.h>
#endif // __ARM_NEON

static int draw_unsupported(cv::Mat& rgb)
{
    const char text[] = "unsupported";

    int baseLine = 0;
    cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 1.0, 1, &baseLine);

    int y = (rgb.rows - label_size.height) / 2;
    int x = (rgb.cols - label_size.width) / 2;

    cv::rectangle(rgb, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + baseLine)),
                    cv::Scalar(255, 255, 255), -1);

    cv::putText(rgb, text, cv::Point(x, y + label_size.height),
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 0));

    return 0;
}

static int draw_fps(cv::Mat& rgb)
{
    // resolve moving average
    float avg_fps = 0.f;
    {
        static double t0 = 0.f;
        static float fps_history[10] = {0.f};

        double t1 = ncnn::get_current_time();
        if (t0 == 0.f)
        {
            t0 = t1;
            return 0;
        }

        float fps = 1000.f / (t1 - t0);
        t0 = t1;

        for (int i = 9; i >= 1; i--)
        {
            fps_history[i] = fps_history[i - 1];
        }
        fps_history[0] = fps;

        if (fps_history[9] == 0.f)
        {
            return 0;
        }

        for (int i = 0; i < 10; i++)
        {
            avg_fps += fps_history[i];
        }
        avg_fps /= 10.f;
    }

    char text[32];
    sprintf(text, "FPS=%.2f", avg_fps);

    int baseLine = 0;
    cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

    int y = 0;
    int x = rgb.cols - label_size.width;

    cv::rectangle(rgb, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + baseLine)),
                    cv::Scalar(255, 255, 255), -1);

    cv::putText(rgb, text, cv::Point(x, y + label_size.height),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));

    return 0;
}

static YOLO11* g_yolo11 = 0;
static ncnn::Mutex lock;
static int g_current_task = 0;
static UdpSender* g_udp_sender = 0;
static uint32_t g_frame_seq = 0;

// USB serial: JNI references to call Java from C++ camera thread
static JavaVM* g_jvm = 0;
static jclass g_yolo11ncnn_cls = 0;
static jmethodID g_send_usb_mid = 0;

// AprilTag detector (task mode 6)
static apriltag_detector_t* g_at_detector = 0;
static apriltag_family_t*   g_at_family   = 0;
static bool g_part2_reset_requested = false; // set true when Task 6 is loaded to reset FSM
static bool g_part1_reset_requested = false; // set true when Task 5 is loaded to reset Part 1 timer
static bool g_request_part1_switch  = false; // set true when Part 2 delivery complete → Java polls this

// Ball tracking state
static const int CENTER_RECT_HEIGHT = 120;  // vertical height of centre rectangle (short strip)
static const float SELECT_INTERVAL = 0.1f; // seconds

// Target COCO class indices: sports ball=32, apple=47, orange=49
static const int TARGET_CLASSES[] = {32, 47, 49};
static const int TARGET_CLASSES_COUNT = 3;

static const char* ball_class_names[] = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
    "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
    "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush"
};

static bool is_target_class(int label)
{
    for (int i = 0; i < TARGET_CLASSES_COUNT; i++)
    {
        if (label == TARGET_CLASSES[i])
            return true;
    }
    return false;
}

static bool box_in_center(const cv::Rect_<float>& rect, int cx1, int cy1, int cx2, int cy2)
{
    float bx_center = rect.x + rect.width / 2.f;
    float by_center = rect.y + rect.height / 2.f;
    return bx_center >= cx1 && bx_center <= cx2 && by_center >= cy1 && by_center <= cy2;
}

static void draw_ball_tracking(cv::Mat& rgb, const std::vector<Object>& objects)
{
    // Persistent state across frames
    static int primary_label = -1;
    static cv::Rect_<float> primary_rect;
    static float primary_prob = 0.f;
    static bool has_primary = false;
    static double last_select_time = 0.0;
    static bool seeded = false;

    // Mirrored robot FSM state (for on-screen debug display)
    enum RobotState { ROTATING, MOVING_FORWARD, COASTING };
    static RobotState robot_state = ROTATING;
    static double coast_start_ms = 0.0;

    // Part 1 phase timer
    static double part1_start_ms  = 0.0;
    static bool   part1_timer_set = false;

    if (!part1_timer_set || g_part1_reset_requested)
    {
        part1_start_ms    = ncnn::get_current_time();
        part1_timer_set   = true;
        g_part1_reset_requested = false;
    }

    if (!seeded)
    {
        srand((unsigned int)time(NULL));
        seeded = true;
    }

    int h = rgb.rows;
    int w = rgb.cols;
    int cx_frame = w / 2;
    int cy_frame = h / 2;

    // Centre rectangle: rotates with phone orientation
    // Portrait  (h > w): full width,  short height strip
    // Landscape (w > h): full height, short width strip
    int cx1, cy1, cx2, cy2;
    if (w >= h)
    {
        cx1 = cx_frame - CENTER_RECT_HEIGHT / 2;  cy1 = 0;
        cx2 = cx_frame + CENTER_RECT_HEIGHT / 2;  cy2 = h;
    }
    else
    {
        cx1 = 0;                              cy1 = cy_frame - CENTER_RECT_HEIGHT / 2;
        cx2 = w;                              cy2 = cy_frame + CENTER_RECT_HEIGHT / 2;
    }

    // Draw centre rectangle (cyan)
    cv::rectangle(rgb, cv::Point(cx1, cy1), cv::Point(cx2, cy2), cv::Scalar(0, 255, 255), 2);

    // Filter to target classes only
    std::vector<int> target_indices;
    for (size_t i = 0; i < objects.size(); i++)
    {
        if (is_target_class(objects[i].label))
        {
            target_indices.push_back((int)i);
        }
    }

    // Check if primary object is still detected
    if (has_primary)
    {
        bool still_exists = false;
        for (size_t i = 0; i < target_indices.size(); i++)
        {
            const Object& obj = objects[target_indices[i]];
            if (obj.label == primary_label &&
                std::abs(obj.rect.x - primary_rect.x) < 50 &&
                std::abs(obj.rect.y - primary_rect.y) < 50)
            {
                // Update primary rect and prob to track movement
                primary_rect = obj.rect;
                primary_prob = obj.prob;
                still_exists = true;
                break;
            }
        }
        if (!still_exists)
        {
            has_primary = false;
        }
    }

    // Select new primary if none exists (with interval throttle)
    double current_time = ncnn::get_current_time() / 1000.0; // ms to seconds
    if (!has_primary && current_time - last_select_time >= SELECT_INTERVAL)
    {
        if (!target_indices.empty())
        {
            int chosen = rand() % target_indices.size();
            const Object& obj = objects[target_indices[chosen]];
            primary_label = obj.label;
            primary_rect = obj.rect;
            primary_prob = obj.prob;
            has_primary = true;
        }
        last_select_time = current_time;
    }

    // Draw detections
    for (size_t i = 0; i < target_indices.size(); i++)
    {
        const Object& obj = objects[target_indices[i]];

        bool is_primary = has_primary &&
                          obj.label == primary_label &&
                          std::abs(obj.rect.x - primary_rect.x) < 50 &&
                          std::abs(obj.rect.y - primary_rect.y) < 50;

        cv::Scalar color;
        int thickness;
        if (is_primary)
        {
            color = cv::Scalar(255, 0, 0); // Red in RGB
            thickness = 3;
        }
        else
        {
            color = cv::Scalar(0, 255, 0); // Green in RGB
            thickness = 2;
        }

        cv::rectangle(rgb, obj.rect, color, thickness);

        char text[256];
        sprintf(text, "%s %.1f%%", ball_class_names[obj.label], obj.prob * 100);

        int baseLine = 0;
        cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

        int x = (int)obj.rect.x;
        int y = (int)obj.rect.y - label_size.height - baseLine;
        if (y < 0) y = 0;
        if (x + label_size.width > rgb.cols) x = rgb.cols - label_size.width;

        cv::rectangle(rgb, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + baseLine)),
                      cv::Scalar(255, 255, 255), -1);

        cv::putText(rgb, text, cv::Point(x, y + label_size.height),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, color);
    }

    // Check if primary is in center
    int is_centered = 0;
    if (has_primary && box_in_center(primary_rect, cx1, cy1, cx2, cy2))
    {
        is_centered = 1;
        __android_log_print(ANDROID_LOG_INFO, "BallTracker", "Primary object is in the centre rectangle");
    }

    // Mirror robot FSM state
    double now_ms = ncnn::get_current_time();
    switch (robot_state)
    {
        case ROTATING:
            if (has_primary && is_centered)
                robot_state = MOVING_FORWARD;
            break;
        case MOVING_FORWARD:
            if (!is_centered)
            {
                coast_start_ms = now_ms;
                robot_state = COASTING;
            }
            break;
        case COASTING:
            if (now_ms - coast_start_ms >= 2500.0)
            {
                if (is_centered)
                    robot_state = MOVING_FORWARD;
                else
                    robot_state = ROTATING;
            }
            break;
    }

    // Draw centre status (top of screen)
    if (has_primary)
    {
        if (is_centered)
            cv::putText(rgb, "IN CENTRE", cv::Point(50, 50),
                        cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 3);
        else
            cv::putText(rgb, "NOT IN CENTRE", cv::Point(50, 50),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
    }
    else
    {
        cv::putText(rgb, "NO OBJECT", cv::Point(50, 50),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(128, 128, 128), 2);
    }

    // Draw robot FSM state (below centre status)
    const char* state_str = "";
    cv::Scalar state_color;
    switch (robot_state)
    {
        case ROTATING:        state_str = "ROBOT: ROTATING";        state_color = cv::Scalar(255, 165, 0);   break;
        case MOVING_FORWARD:  state_str = "ROBOT: MOVING FORWARD";  state_color = cv::Scalar(0, 255, 0);     break;
        case COASTING:        state_str = "ROBOT: COASTING (1s)";   state_color = cv::Scalar(255, 255, 0);   break;
    }
    cv::putText(rgb, state_str, cv::Point(50, 90),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, state_color, 2);

    // Part 1 countdown to Part 2
    {
        double elapsed_s   = (ncnn::get_current_time() - part1_start_ms) / 1000.0;
        double remaining_s = 90.0 - elapsed_s;
        if (remaining_s < 0.0) remaining_s = 0.0;
        char p1timer[48];
        snprintf(p1timer, sizeof(p1timer), "P1 TIME: %.0fs remaining", remaining_s);
        cv::putText(rgb, p1timer, cv::Point(20, 130),
                    cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(200, 200, 200), 2);
    }

    // Format and send UDP packet
    if (g_udp_sender && g_udp_sender->is_running())
    {
        char pkt[512];
        int err_x = 0, err_y = 0;
        if (has_primary)
        {
            err_x = (int)(primary_rect.x + primary_rect.width / 2.f) - cx_frame;
            err_y = (int)(primary_rect.y + primary_rect.height / 2.f) - cy_frame;
        }

        int off = snprintf(pkt, sizeof(pkt), "%u,%u,%d,%d,%d,%d,",
            g_frame_seq, (uint32_t)(current_time * 1000),
            w, h, (int)target_indices.size(), has_primary ? 1 : 0);

        if (has_primary)
        {
            off += snprintf(pkt + off, sizeof(pkt) - off, "%d,%d,%d,%d,%d,%d,%d,%d,",
                (int)primary_rect.x, (int)primary_rect.y,
                (int)primary_rect.width, (int)primary_rect.height,
                (int)(primary_prob * 100), is_centered, err_x, err_y);
        }
        else
        {
            off += snprintf(pkt + off, sizeof(pkt) - off, "0,0,0,0,0,0,0,0,");
        }

        for (size_t i = 0; i < target_indices.size() && off < (int)sizeof(pkt) - 30; i++)
        {
            const Object& obj = objects[target_indices[i]];
            off += snprintf(pkt + off, sizeof(pkt) - off, "%d,%d,%d,%d,%d,",
                (int)obj.rect.x, (int)obj.rect.y,
                (int)obj.rect.width, (int)obj.rect.height,
                (int)(obj.prob * 100));
        }

        if (off > 0 && pkt[off - 1] == ',')
            pkt[off - 1] = '\n';

        g_udp_sender->send_packet(pkt, off);
        g_frame_seq++;

        // Also send via USB serial if connected — call Java YOLO11Ncnn.sendPacketUsb()
        if (g_jvm && g_yolo11ncnn_cls && g_send_usb_mid)
        {
            JNIEnv* env = NULL;
            g_jvm->AttachCurrentThread(&env, NULL);
            if (env)
            {
                jstring jpkt = env->NewStringUTF(pkt);
                env->CallStaticVoidMethod(g_yolo11ncnn_cls, g_send_usb_mid, jpkt);
                env->DeleteLocalRef(jpkt);
            }
        }

        // Draw UDP status
        cv::putText(rgb, "UDP: ON", cv::Point(10, rgb.rows - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
    }
    else if (g_current_task == 5)
    {
        cv::putText(rgb, "UDP: OFF", cv::Point(10, rgb.rows - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(128, 128, 128));
    }
}

// ---- Part 2 helpers ----
static float p2_tag_side_px(apriltag_detection_t* det)
{
    double dx = det->p[1][0] - det->p[0][0];
    double dy = det->p[1][1] - det->p[0][1];
    return (float)sqrt(dx * dx + dy * dy);
}

static void p2_send_cmd(const char* cmd)
{
    if (!g_jvm || !g_yolo11ncnn_cls || !g_send_usb_mid) return;
    JNIEnv* env = NULL;
    g_jvm->AttachCurrentThread(&env, NULL);
    if (!env) return;
    char pkt[64];
    snprintf(pkt, sizeof(pkt), "CMD,%s\n", cmd);
    jstring jpkt = env->NewStringUTF(pkt);
    env->CallStaticVoidMethod(g_yolo11ncnn_cls, g_send_usb_mid, jpkt);
    env->DeleteLocalRef(jpkt);
}

static void draw_apriltag_mode(cv::Mat& rgb)
{
    if (!g_at_detector)
    {
        g_at_family   = tag36h11_create();
        g_at_detector = apriltag_detector_create();
        apriltag_detector_add_family(g_at_detector, g_at_family);
        g_at_detector->quad_decimate     = 2.0f;
        g_at_detector->nthreads          = 1;
        g_at_detector->quad_sigma        = 0.0f;
        g_at_detector->refine_edges      = 1;
        g_at_detector->decode_sharpening = 0.25;
    }

    // ---- Part 2 FSM ----
    enum P2State {
        P2_SCAN, P2_SCAN_PAUSE, P2_ALIGN, P2_APPROACH, P2_FIND_NEXT,
        P2_TURN_180, P2_REVERSE, P2_LIFT_UP, P2_LIFT_DOWN,
        P2_FORWARD_EXIT, P2_DONE
    };
    static P2State p2_state    = P2_SCAN;
    static double  state_ms    = 0.0;  // when current state started
    static double  phase_ms    = 0.0;  // when Part 2 phase started
    static int     tracking_id = -1;   // tag we are currently aligning/approaching
    static int     at_tag_id   = -1;   // tag we just arrived at (for FIND_NEXT)
    static bool    p2_init     = false;

    static const int    TARGET_TAG      = 1;    // final destination tag
    static const int    NUM_TAGS        = 24;
    static const int    ALIGN_THRESH    = 40;   // px — tag centre must be within this of screen centre
    static const float  CLOSE_ENOUGH    = 150.f;// tag apparent side px — "at" a non-target tag
    static const float  TARGET_RANGE_PX = 70.f; // tag apparent side px — trigger delivery when tag this big
    static const double TURN_180_MS     = 1500.0;// ms to spin 180° (tune based on motor speed)
    static const double PHASE_SECS      = 90.0;
    static const double SCAN_STEP_MS    = 400.0; // ms to rotate ~45 degrees per scan step — tune
    static const double SCAN_PAUSE_MS   = 1500.0;// ms to pause still and look for tags

    int w = rgb.cols, h = rgb.rows;
    int cx = w / 2,   cy = h / 2;
    double now_ms = ncnn::get_current_time();

    // Reset FSM on fresh load of Task 6
    if (!p2_init || g_part2_reset_requested)
    {
        p2_state    = P2_SCAN;
        state_ms    = now_ms;
        phase_ms    = now_ms;
        tracking_id = -1;
        at_tag_id   = -1;
        p2_init     = true;
        g_part2_reset_requested = false;
    }

    // Centre rectangle (same strip as Part 1)
    int cx1, cy1, cx2, cy2;
    if (w >= h) { cx1 = cx - CENTER_RECT_HEIGHT/2; cy1 = 0; cx2 = cx + CENTER_RECT_HEIGHT/2; cy2 = h; }
    else        { cx1 = 0; cy1 = cy - CENTER_RECT_HEIGHT/2; cx2 = w; cy2 = cy + CENTER_RECT_HEIGHT/2; }
    cv::rectangle(rgb, cv::Point(cx1,cy1), cv::Point(cx2,cy2), cv::Scalar(0,255,255), 2);

    // Detect tags
    cv::Mat gray;
    cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);
    image_u8_t im = { (int32_t)gray.cols, (int32_t)gray.rows, (int32_t)gray.step[0], gray.data };
    zarray_t* dets = apriltag_detector_detect(g_at_detector, &im);
    int num_det = zarray_size(dets);

    // Find target tag and best non-target tag (fewest hops to target)
    apriltag_detection_t* target_det = nullptr;
    apriltag_detection_t* best_det   = nullptr;
    int best_dist = 100;
    for (int i = 0; i < num_det; i++)
    {
        apriltag_detection_t* d;
        zarray_get(dets, i, &d);
        if (d->id == TARGET_TAG) { target_det = d; }
        else
        {
            int cw   = (TARGET_TAG - d->id + NUM_TAGS) % NUM_TAGS;
            int ccw  = (d->id - TARGET_TAG + NUM_TAGS) % NUM_TAGS;
            int dist = (cw < ccw) ? cw : ccw;
            if (dist < best_dist) { best_dist = dist; best_det = d; }
        }
    }
    // Active tag: target if visible, else closest-to-target available
    apriltag_detection_t* active    = target_det ? target_det : best_det;
    int                   active_id = active ? active->id : -1;

    double phase_remaining = PHASE_SECS - (now_ms - phase_ms) / 1000.0;
    if (phase_remaining < 0.0) phase_remaining = 0.0;
    double state_elapsed = now_ms - state_ms;

    // 90-second phase timeout
    if (p2_state != P2_DONE && phase_remaining <= 0.0)
    {
        p2_send_cmd("STOP");
        p2_state = P2_DONE;
        state_ms = now_ms;
    }

    // ---- FSM ----
    char status_line1[128] = "";
    char status_line2[128] = "";
    cv::Scalar s_color = cv::Scalar(255, 255, 255);

    switch (p2_state)
    {
        case P2_SCAN:
        {
            snprintf(status_line1, sizeof(status_line1), "P2: ROTATING — Scanning step (~45 deg)");
            snprintf(status_line2, sizeof(status_line2), "Rotating CW for %.0fms then pausing...", SCAN_STEP_MS);
            s_color = cv::Scalar(255, 255, 0);
            p2_send_cmd("ROTATE_CW");

            if (state_elapsed >= SCAN_STEP_MS)
            {
                p2_send_cmd("STOP");
                p2_state = P2_SCAN_PAUSE;
                state_ms = now_ms;
            }
            break;
        }

        case P2_SCAN_PAUSE:
        {
            snprintf(status_line1, sizeof(status_line1), "P2: PAUSED — Looking for tags");
            snprintf(status_line2, sizeof(status_line2), "Still for %.0fms — scanning...", SCAN_PAUSE_MS);
            s_color = cv::Scalar(255, 255, 0);
            p2_send_cmd("STOP");

            if (active != nullptr)
            {
                // Tag found during pause — go align to it
                tracking_id = active_id;
                p2_state    = P2_ALIGN;
                state_ms    = now_ms;
            }
            else if (state_elapsed >= SCAN_PAUSE_MS)
            {
                // No tag found — take another step
                p2_state = P2_SCAN;
                state_ms = now_ms;
            }
            break;
        }

        case P2_ALIGN:
        {
            snprintf(status_line1, sizeof(status_line1),
                     tracking_id == TARGET_TAG ? "P2: ROTATING — Aligning to TARGET tag %d" : "P2: ROTATING — Aligning to tag %d",
                     tracking_id);
            s_color = (tracking_id == TARGET_TAG) ? cv::Scalar(0,255,0) : cv::Scalar(0,200,255);

            // Find the tag we're aligning to; if lost, switch to any visible
            apriltag_detection_t* align_det = nullptr;
            if (tracking_id == TARGET_TAG && target_det) align_det = target_det;
            else if (active && active->id == tracking_id) align_det = active;
            if (!align_det && active) { tracking_id = active_id; align_det = active; }

            if (align_det)
            {
                int err_x = (int)align_det->c[0] - cx;
                snprintf(status_line2, sizeof(status_line2),
                         "Tag %d visible | err_x=%d | %s", align_det->id, err_x,
                         abs(err_x) <= ALIGN_THRESH ? "CENTRED — moving in!" :
                         (err_x > 0 ? "rotating right..." : "rotating left..."));

                if (abs(err_x) <= ALIGN_THRESH)
                {
                    p2_send_cmd("STOP");
                    p2_state = P2_APPROACH;
                    state_ms = now_ms;
                }
                else
                {
                    p2_send_cmd(err_x > 0 ? "ROTATE_CCW" : "ROTATE_CW");
                }
            }
            else
            {
                snprintf(status_line2, sizeof(status_line2), "Tag %d lost — rescanning", tracking_id);
                if (state_elapsed > 2000.0) { p2_state = P2_SCAN; state_ms = now_ms; }
            }
            break;
        }

        case P2_APPROACH:
        {
            snprintf(status_line1, sizeof(status_line1), "P2: MOVING FORWARD — to tag %d", tracking_id);
            s_color = cv::Scalar(0, 255, 120);

            apriltag_detection_t* app_det = nullptr;
            if (tracking_id == TARGET_TAG && target_det) app_det = target_det;
            else if (active && active->id == tracking_id) app_det = active;
            if (!app_det) app_det = active;

            if (app_det)
            {
                float sz       = p2_tag_side_px(app_det);
                bool  is_tgt   = (app_det->id == TARGET_TAG);
                float threshold = is_tgt ? TARGET_RANGE_PX : CLOSE_ENOUGH;

                snprintf(status_line2, sizeof(status_line2),
                         "Tag %d | size=%.0fpx | %s", app_det->id, sz,
                         sz >= threshold ? (is_tgt ? "IN RANGE — starting delivery!" : "close — finding next tag")
                                         : "moving forward...");

                if (sz >= threshold)
                {
                    p2_send_cmd("STOP");
                    at_tag_id = app_det->id;
                    // Target tag in range → full delivery sequence
                    // Any other tag close → navigate to next tag
                    p2_state  = (at_tag_id == TARGET_TAG) ? P2_TURN_180 : P2_FIND_NEXT;
                    state_ms  = now_ms;
                }
                else { p2_send_cmd("FORWARD"); }
            }
            else
            {
                snprintf(status_line2, sizeof(status_line2), "Tag lost while moving — rescanning");
                p2_send_cmd("STOP");
                p2_state = P2_SCAN;
                state_ms = now_ms;
            }
            break;
        }

        case P2_FIND_NEXT:
        {
            int cw_dist  = (TARGET_TAG - at_tag_id + NUM_TAGS) % NUM_TAGS;
            int ccw_dist = (at_tag_id - TARGET_TAG + NUM_TAGS) % NUM_TAGS;
            bool go_cw   = (cw_dist <= ccw_dist);
            int  hops    = go_cw ? cw_dist : ccw_dist;

            snprintf(status_line1, sizeof(status_line1),
                     "P2: ROTATING — Finding tag %d (%d hops away)", TARGET_TAG, hops);
            snprintf(status_line2, sizeof(status_line2),
                     "Rotating %s to find next tag...", go_cw ? "right (CW)" : "left (CCW)");
            s_color = cv::Scalar(255, 165, 0);

            p2_send_cmd(go_cw ? "ROTATE_CW" : "ROTATE_CCW");

            // New tag in view (different from the one we just arrived at) → align to it
            if (active != nullptr && active->id != at_tag_id)
            {
                tracking_id = active_id;
                p2_state    = P2_ALIGN;
                state_ms    = now_ms;
            }
            // Safety: if stuck for 10s, rescan
            if (state_elapsed > 10000.0) { p2_state = P2_SCAN; state_ms = now_ms; }
            break;
        }

        case P2_TURN_180:
        {
            snprintf(status_line1, sizeof(status_line1), "P2: ROTATING 180 DEGREES");
            snprintf(status_line2, sizeof(status_line2), "Spinning to face back toward Tag %d wall...", TARGET_TAG);
            s_color = cv::Scalar(255, 200, 0);

            if (state_elapsed < TURN_180_MS)
            {
                p2_send_cmd("ROTATE_CW");
            }
            else
            {
                p2_send_cmd("STOP");
                p2_state = P2_REVERSE;
                state_ms = now_ms;
            }
            break;
        }

        case P2_REVERSE:
        {
            snprintf(status_line1, sizeof(status_line1), "P2: PARKING BACKWARDS");
            snprintf(status_line2, sizeof(status_line2), "Moving backward toward Tag %d wall for 2 seconds...", TARGET_TAG);
            s_color = cv::Scalar(0, 200, 255);

            if (state_elapsed < 2000.0)
            {
                p2_send_cmd("BACKWARD");
            }
            else
            {
                p2_send_cmd("STOP");
                p2_state = P2_LIFT_UP;
                state_ms = now_ms;
            }
            break;
        }

        case P2_LIFT_UP:
        {
            snprintf(status_line1, sizeof(status_line1), "P2: LIFTING MOTOR UP");
            snprintf(status_line2, sizeof(status_line2), "Motor rising — holding for 3 seconds...");
            s_color = cv::Scalar(0, 255, 0);

            if (state_elapsed < 3000.0)
            {
                p2_send_cmd("LIFT_UP");
            }
            else
            {
                p2_state = P2_LIFT_DOWN;
                state_ms = now_ms;
            }
            break;
        }

        case P2_LIFT_DOWN:
        {
            snprintf(status_line1, sizeof(status_line1), "P2: LIFTING MOTOR DOWN");
            snprintf(status_line2, sizeof(status_line2), "Motor lowering — 1 second...");
            s_color = cv::Scalar(0, 180, 0);

            if (state_elapsed < 1000.0)
            {
                p2_send_cmd("LIFT_DOWN");
            }
            else
            {
                p2_send_cmd("STOP");
                p2_state = P2_FORWARD_EXIT;
                state_ms = now_ms;
            }
            break;
        }

        case P2_FORWARD_EXIT:
        {
            snprintf(status_line1, sizeof(status_line1), "P2: MOVING FORWARD — Exiting wall");
            snprintf(status_line2, sizeof(status_line2), "Moving forward for 2 seconds...");
            s_color = cv::Scalar(0, 255, 120);

            if (state_elapsed < 2000.0)
            {
                p2_send_cmd("FORWARD");
            }
            else
            {
                p2_send_cmd("STOP");
                g_request_part1_switch = true; // signal Java to switch back to Part 1
                p2_state = P2_DONE;
                state_ms = now_ms;
            }
            break;
        }

        case P2_DONE:
        {
            p2_send_cmd("STOP");
            if (phase_remaining <= 0.0)
            {
                snprintf(status_line1, sizeof(status_line1), "P2: TIME UP — RETURNING TO PART 1");
                s_color = cv::Scalar(255, 100, 100);
            }
            else
            {
                snprintf(status_line1, sizeof(status_line1), "P2: DELIVERY COMPLETE!");
                snprintf(status_line2, sizeof(status_line2), "Tag %d reached. Waiting for timer...", TARGET_TAG);
                s_color = cv::Scalar(0, 255, 0);
            }
            break;
        }
    }

    // ---- Draw all detected tags ----
    for (int i = 0; i < num_det; i++)
    {
        apriltag_detection_t* d;
        zarray_get(dets, i, &d);
        bool is_target   = (d->id == TARGET_TAG);
        bool is_tracking = (d->id == tracking_id);
        cv::Scalar col = is_target   ? cv::Scalar(0,255,0) :
                         is_tracking ? cv::Scalar(0,200,255) : cv::Scalar(255,255,255);
        int thk = (is_target || is_tracking) ? 3 : 2;

        cv::Point pts[4];
        for (int j = 0; j < 4; j++) pts[j] = cv::Point((int)d->p[j][0], (int)d->p[j][1]);
        for (int j = 0; j < 4; j++) cv::line(rgb, pts[j], pts[(j+1)%4], col, thk);
        cv::circle(rgb, cv::Point((int)d->c[0], (int)d->c[1]), 5, col, -1);

        char lbl[40];
        sprintf(lbl, is_target ? "ID:%d [TARGET]" : "ID:%d", d->id);
        int bl = 0;
        cv::Size lsz = cv::getTextSize(lbl, cv::FONT_HERSHEY_SIMPLEX, 0.6, 1, &bl);
        int lx = (int)d->c[0] - lsz.width/2, ly = (int)d->c[1] - 14;
        if (lx < 0) lx = 0;
        if (ly < lsz.height) ly = lsz.height;
        cv::rectangle(rgb, cv::Rect(cv::Point(lx, ly-lsz.height), cv::Size(lsz.width, lsz.height+bl)),
                      cv::Scalar(0,0,0), -1);
        cv::putText(rgb, lbl, cv::Point(lx, ly), cv::FONT_HERSHEY_SIMPLEX, 0.6, col, 1);
        __android_log_print(ANDROID_LOG_INFO, "Part2", "ID=%d size=%.0fpx", d->id, p2_tag_side_px(d));
    }

    // ---- On-screen display ----
    // Status line 1 (main state)
    cv::putText(rgb, status_line1, cv::Point(20, 50),  cv::FONT_HERSHEY_SIMPLEX, 0.7, s_color, 2);
    // Status line 2 (detail)
    cv::putText(rgb, status_line2, cv::Point(20, 88),  cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(200,200,200), 1);
    // Part 2 countdown
    char p2timer[48];
    snprintf(p2timer, sizeof(p2timer), "P2 TIME: %.0fs remaining", phase_remaining);
    cv::putText(rgb, p2timer, cv::Point(20, 125), cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(200,200,200), 2);
    // No-tag notice (shown below other text so it doesn't overlap state label)
    if (num_det == 0)
        cv::putText(rgb, "NO TAGS IN VIEW", cv::Point(20, 162), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(128,128,128), 2);

    apriltag_detections_destroy(dets);
}

class MyNdkCamera : public NdkCameraWindow
{
public:
    virtual void on_image_render(cv::Mat& rgb) const;
};

void MyNdkCamera::on_image_render(cv::Mat& rgb) const
{
    // yolo11
    {
        ncnn::MutexLockGuard g(lock);

        if (g_current_task == 6)
        {
            // AprilTag detection mode — no YOLO needed
            draw_apriltag_mode(rgb);
        }
        else if (g_yolo11)
        {
            std::vector<Object> objects;
            g_yolo11->detect(rgb, objects);

            if (g_current_task == 5)
            {
                // Ball tracking mode
                draw_ball_tracking(rgb, objects);
            }
            else
            {
                g_yolo11->draw(rgb, objects);
            }
        }
        else
        {
            draw_unsupported(rgb);
        }
    }

    draw_fps(rgb);
}

static MyNdkCamera* g_camera = 0;

extern "C" {

JNIEXPORT jint JNI_OnLoad(JavaVM* vm, void* reserved)
{
    __android_log_print(ANDROID_LOG_DEBUG, "ncnn", "JNI_OnLoad");

    g_camera = new MyNdkCamera;

    ncnn::create_gpu_instance();

    // Cache JVM + YOLO11Ncnn.sendPacketUsb() for USB serial forwarding
    g_jvm = vm;
    JNIEnv* env = NULL;
    if (vm->GetEnv((void**)&env, JNI_VERSION_1_4) == JNI_OK && env)
    {
        jclass cls = env->FindClass("com/tencent/yolo11ncnn/YOLO11Ncnn");
        if (cls)
        {
            g_yolo11ncnn_cls = (jclass)env->NewGlobalRef(cls);
            g_send_usb_mid = env->GetStaticMethodID(g_yolo11ncnn_cls, "sendPacketUsb", "(Ljava/lang/String;)V");
        }
    }

    return JNI_VERSION_1_4;
}

JNIEXPORT void JNI_OnUnload(JavaVM* vm, void* reserved)
{
    __android_log_print(ANDROID_LOG_DEBUG, "ncnn", "JNI_OnUnload");

    {
        ncnn::MutexLockGuard g(lock);

        delete g_yolo11;
        g_yolo11 = 0;
    }

    if (g_udp_sender)
    {
        g_udp_sender->stop();
        delete g_udp_sender;
        g_udp_sender = 0;
    }

    if (g_at_detector)
    {
        apriltag_detector_destroy(g_at_detector);
        g_at_detector = 0;
    }
    if (g_at_family)
    {
        tag36h11_destroy(g_at_family);
        g_at_family = 0;
    }

    ncnn::destroy_gpu_instance();

    delete g_camera;
    g_camera = 0;
}

// public native boolean loadModel(AssetManager mgr, int taskid, int modelid, int cpugpu);
JNIEXPORT jboolean JNICALL Java_com_tencent_yolo11ncnn_YOLO11Ncnn_loadModel(JNIEnv* env, jobject thiz, jobject assetManager, jint taskid, jint modelid, jint cpugpu)
{
    if (taskid < 0 || taskid > 6 || modelid < 0 || modelid > 8 || cpugpu < 0 || cpugpu > 2)
    {
        return JNI_FALSE;
    }

    // AprilTag mode (task 6) needs no YOLO model
    if (taskid == 6)
    {
        ncnn::MutexLockGuard g(lock);
        g_current_task = 6;
        g_part2_reset_requested = true; // reset Part 2 FSM on next frame
        return JNI_TRUE;
    }

    AAssetManager* mgr = AAssetManager_fromJava(env, assetManager);

    __android_log_print(ANDROID_LOG_DEBUG, "ncnn", "loadModel %p", mgr);

    g_current_task = taskid;

    if (taskid == 5)
        g_part1_reset_requested = true; // reset Part 1 timer on fresh load

    // taskid 5 (ball) uses detection model (same as taskid 0)
    int model_taskid = (taskid == 5) ? 0 : taskid;

    const char* tasknames[5] =
    {
        "",
        "_seg",
        "_pose",
        "_cls",
        "_obb"
    };

    const char* modeltypes[9] =
    {
        "n",
        "s",
        "m",
        "n",
        "s",
        "m",
        "n",
        "s",
        "m"
    };

    std::string parampath = std::string("yolo11") + modeltypes[(int)modelid] + tasknames[model_taskid] + ".ncnn.param";
    std::string modelpath = std::string("yolo11") + modeltypes[(int)modelid] + tasknames[model_taskid] + ".ncnn.bin";
    bool use_gpu = (int)cpugpu == 1;
    bool use_turnip = (int)cpugpu == 2;

    // reload
    {
        ncnn::MutexLockGuard g(lock);

        {
            static int old_taskid = 0;
            static int old_modelid = 0;
            static int old_cpugpu = 0;
            if (model_taskid != old_taskid || (modelid % 3) != old_modelid || cpugpu != old_cpugpu)
            {
                // taskid or model or cpugpu changed
                delete g_yolo11;
                g_yolo11 = 0;
            }
            old_taskid = model_taskid;
            old_modelid = modelid % 3;
            old_cpugpu = cpugpu;

            ncnn::destroy_gpu_instance();

            if (use_turnip)
            {
                ncnn::create_gpu_instance("libvulkan_freedreno.so");
            }
            else if (use_gpu)
            {
                ncnn::create_gpu_instance();
            }

            if (!g_yolo11)
            {
                if (model_taskid == 0) g_yolo11 = new YOLO11_det;
                if (model_taskid == 1) g_yolo11 = new YOLO11_seg;
                if (model_taskid == 2) g_yolo11 = new YOLO11_pose;
                if (model_taskid == 3) g_yolo11 = new YOLO11_cls;
                if (model_taskid == 4) g_yolo11 = new YOLO11_obb;

                g_yolo11->load(mgr, parampath.c_str(), modelpath.c_str(), use_gpu || use_turnip);
            }
            int target_size = 320;
            if ((int)modelid >= 3)
                target_size = 480;
            if ((int)modelid >= 6)
                target_size = 640;
            g_yolo11->set_det_target_size(target_size);
        }
    }

    return JNI_TRUE;
}

// public native boolean openCamera(int facing);
JNIEXPORT jboolean JNICALL Java_com_tencent_yolo11ncnn_YOLO11Ncnn_openCamera(JNIEnv* env, jobject thiz, jint facing)
{
    if (facing < 0 || facing > 1)
        return JNI_FALSE;

    __android_log_print(ANDROID_LOG_DEBUG, "ncnn", "openCamera %d", facing);

    g_camera->open((int)facing);

    return JNI_TRUE;
}

// public native boolean closeCamera();
JNIEXPORT jboolean JNICALL Java_com_tencent_yolo11ncnn_YOLO11Ncnn_closeCamera(JNIEnv* env, jobject thiz)
{
    __android_log_print(ANDROID_LOG_DEBUG, "ncnn", "closeCamera");

    g_camera->close();

    return JNI_TRUE;
}

// public native boolean setOutputWindow(Surface surface);
JNIEXPORT jboolean JNICALL Java_com_tencent_yolo11ncnn_YOLO11Ncnn_setOutputWindow(JNIEnv* env, jobject thiz, jobject surface)
{
    ANativeWindow* win = ANativeWindow_fromSurface(env, surface);

    __android_log_print(ANDROID_LOG_DEBUG, "ncnn", "setOutputWindow %p", win);

    g_camera->set_window(win);

    return JNI_TRUE;
}

// public native boolean startUdp(String ip, int port);
JNIEXPORT jboolean JNICALL Java_com_tencent_yolo11ncnn_YOLO11Ncnn_startUdp(JNIEnv* env, jobject thiz, jstring ip, jint port)
{
    const char* ip_str = env->GetStringUTFChars(ip, NULL);

    if (!g_udp_sender)
        g_udp_sender = new UdpSender();

    bool ok = g_udp_sender->start(ip_str, (int)port);

    env->ReleaseStringUTFChars(ip, ip_str);

    return ok ? JNI_TRUE : JNI_FALSE;
}

// public native void stopUdp();
JNIEXPORT void JNICALL Java_com_tencent_yolo11ncnn_YOLO11Ncnn_stopUdp(JNIEnv* env, jobject thiz)
{
    if (g_udp_sender)
    {
        g_udp_sender->stop();
        delete g_udp_sender;
        g_udp_sender = 0;
    }
}

// public native boolean checkAndClearPart1Switch();
// Returns true once when Part 2 delivery is complete, then resets to false.
JNIEXPORT jboolean JNICALL Java_com_tencent_yolo11ncnn_YOLO11Ncnn_checkAndClearPart1Switch(JNIEnv* env, jobject thiz)
{
    if (g_request_part1_switch)
    {
        g_request_part1_switch = false;
        return JNI_TRUE;
    }
    return JNI_FALSE;
}

}
