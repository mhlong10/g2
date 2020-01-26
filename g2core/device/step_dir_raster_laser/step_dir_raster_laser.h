/*
 * step_dir_raster_laser.cpp - uses step to clock raster pixels from buffer to PWM
 * for laser.
 * 
 * This file is part of G2 project
 *
 * Copyright (c) 2016 Alden S. Hart, Jr.
 * Copyright (c) 2016 Robert Giseburt
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * As a special exception, you may use this file as part of a software library without
 * restriction. Specifically, if other files instantiate templates or use macros or
 * inline functions from this file, or you compile this file and link it with  other
 * files to produce an executable, this file does not by itself cause the resulting
 * executable to be covered by the GNU General Public License. This exception does not
 * however invalidate any other reasons why the executable file might be covered by the
 * GNU General Public License.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#ifndef STEP_DIR_RASTER_LASER_H_ONCE
#define STEP_DIR_RASTER_LASER_H_ONCE

#include "MotatePins.h"
#include "MotateTimers.h"

#include "stepper.h"
#include "spindle.h"

using Motate::Timeout;


// Motor structures
struct StepDirRasterLaser final : Stepper {
    /* stepper pin assignments */

    int16_t                _microsteps_per_step = 1;
    bool                   _step_is_forward = false;
    int32_t                _position = 0; // in steps from spindle.speed_min to spindle.speed_max
    float                  _speed = 0.0; // Speed scaled based on microsteps per step
    bool                   _enabled = false;

    // sets default pwm freq for all motor vrefs (commented line below also sets HiZ)
    StepDirRasterLaser(void) : Stepper{} {
        _speed = spindle.speed;
    };

    /* Functions that must be implemented in subclasses */

    bool canStep() override { return true; };

    void setMicrosteps(const uint8_t microsteps) override {
        _microsteps_per_step = microsteps;
    };

    void _enableImpl() override {
        if (!_enabled) {
            _enabled = true;
            if (cm_is_laser_tool()) {
                spindle_speed_immediate(_speed);
            }
        }
    };

    void _disableImpl() override {
        if (_enabled) {
            _enabled = false;
            if (cm_is_laser_tool()) {
                spindle_speed_immediate(0.0);
            }
        }
    };

    void stepStart() override {
        if (!_enabled) return;

        if (_step_is_forward) _position++;
        else _position--;

        // Only tick per full step.  We try to put the tick phase in the center
        // by rounding rather than truncating.
        if ((_position + _microsteps_per_step/2) % _microsteps_per_step == 0) {
            // 0% == spindle.speed_min -> 100% == spindle.speed_max
            if (cm_is_laser_tool()) {
                uint8_t v;
                if (pdb.read_next_byte(v))
                    _speed = v/2.55; // Fixed 0-255 -> 0-100%
                    if (_speed > spindle.speed_max) {
                        _speed = spindle.speed_max;
                    }
                    if (_speed < spindle.speed_min) {
                        _speed = spindle.speed_min;
                    }
                    spindle_speed_immediate(_speed);
            }
        }
    };

    void stepEnd() override {
    };

    void setDirection(uint8_t new_direction) override {
        _step_is_forward = (new_direction == DIRECTION_CW);
    };

    void setPowerLevel(float new_pl) override {
        ; // ignore this
    };
};

#endif  // STEP_DIR_RASTER_LASER_H_ONCE
