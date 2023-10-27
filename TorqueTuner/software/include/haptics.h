#ifndef __HAPTICS_H__
#define  __HAPTICS_H__

#include <stdint.h>
#include <stdio.h>
#include <iostream>
#include <functional>
#include <cmath>
#include <vector>
#include "tf_magnet.h"
#include "tf_click.h"
#include "tf_exp_spring.h"
#include "tf_sin.h"
#include "tf_click_2.h"
#include "pressure_table_values.h"
#include "Arduino.h"
#include "variants.h"
#include <driver/uart.h>
#include <soc/uart_struct.h>

const int TABLE_RESOLUTION = 65535;
const int MAX_TORQUE = 180;
const int WALL_TORQUE = 180;
const float MAX_VELOCITY = 500;

const int SERIAL_BUFSIZE = 1024;

class Mode
{
public:
    Mode( float scale_ = MAX_TORQUE / 2, int stretch_ = 1, float min_ = 0, float max_ = 3600, float damping_ = 0.0)
        : min(min_), max(max_), scale_default(scale_), stretch_default(stretch_), damping(damping_) 
        {
          #ifdef MOTEUS
          damping = 0.17;
          #endif
        };
    virtual void set_active(bool is_active) {} // Used for any init/de-init per-mode
    virtual int16_t calc(void* ptr) = 0;
    int16_t calc_index(void* ptr);
    void reset(int16_t angle_);

    float scale_default, stretch_default, min, max, damping, target_velocity_default;
    int offset = 0;

    bool wrap_output = false;
    bool wrap_haptics = false;
    char pid_mode = 't';
    int idx = 0;
    int state = 0;
};

class Reed_Basic: public Mode
{
public:
    Reed_Basic() : Mode(){}
    int16_t calc(void* ptr);
};


class TorqueTuner
{
public:
    enum MODE {
        REED_BASIC = 0
    };
    const int trigger_interval = 50000;  // 10 ms

    TorqueTuner();

    void update();
    void update_angle();
    void update_trig();
    float calc_acceleration(float velocity_);
    float filter(float x);
    float gate(float val, float threshold, float floor);
    int32_t getTime();

    void set_mode(MODE mode_);
    void set_mode(int mode_idx);
    void print_mode(MODE mode_);
    void set_defaults(Mode * mode);
    void set_stretch(float stretch_);
    void reset(Mode * mode_);

    MODE mode = REED_BASIC;
    int16_t angle = 0; // unwrap_outputped angle representing encoder reading
    int16_t angle_last = 0;
    int32_t angle_out = 0;
    int32_t angle_out_last = 0;
    int16_t wrap_count = 0;
    int16_t angle_delta = 0;
    int16_t angle_unclipped = 0;
    int16_t angle_discrete = 0; // angle output value syncronized via libmapper
    int16_t angle_discrete_last = 0;
    int16_t torque = 0;
    int trigger = -1;
    int num_modes = 0;

    float velocity = 0;
    float velocity_out = 0;
    float target_velocity = 0; // [-500;500]
    float acceleration = 0; // [-100;100]
    float scale = 75.0;
    float stretch = 1; // Corresponds to detents in click and magnet mode.

    Reed_Basic reed_basic;
    std::vector<Mode * > mode_list = {&reed_basic};
    Mode * active_mode;

private:
    // Filter variables
    float a[3] = {1.0000,   -1.3329,    0.5053};
    float b[3] = { 0.0431,    0.0862,    0.0431};
};

int zero_crossing(int in);


inline int mod(int in, int hi) {
    const int lo = 0;
    if (in >= hi) {
        in -= hi;
        if (in < hi) {
            return in;
        }
    } else if (in < lo) {
        in += hi;
        if (in >= lo) {
            return in;
        }
    } else {
        return in;
    }
    if (hi == lo) {
        return lo;
    }
    int c;
    c = in % hi;
    if (c < 0) {
        c += hi;
    }
    return c;
}

inline int fold(int in, int lo, int hi) {
    int b = hi - lo;
    int b2 = b + b;
    int c = mod(in - lo, b2);
    if (c > b)
        c = b2 - c;
    return c + lo;
}

inline int clip(int in, int lo, int hi) {
    if (in > hi) {
        return hi;
    } else if (in < lo) {
        return lo;
    } else {
        return in;
    }
}

inline int sign(float x) {
    return (x < 0) ? -1 : (x > 0);
}


#endif
