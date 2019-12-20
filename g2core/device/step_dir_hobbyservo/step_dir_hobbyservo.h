/*
 * step_dir_hobbyservo.cpp - control over a hobby servo (PWM-driven) using steper Step/Direction/Enable from software
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
#ifndef STEPP_DIR_HOBBYSERVO_H_ONCE
#define STEPP_DIR_HOBBYSERVO_H_ONCE

#include "MotatePins.h"
#include "MotateTimers.h"

#include "stepper.h"
#include "spindle.h"
#include "config_app.h" // For debug access to use_data

using Motate::Timeout;


// Motor structures
struct StepDirHobbyServo final : Stepper {
    /* stepper pin assignments */

    int16_t                _microsteps_per_step = 1;
    bool                   _step_is_forward = false;
    int32_t                _position = 0; // in steps from spindle.speed_min to spindle.speed_max
    float                  _speed = 0.0; // Speed scaled based on microsteps per step
    bool                   _enabled = false;
//    Motate::Timeout check_timer;

    // sets default pwm freq for all motor vrefs (commented line below also sets HiZ)
    StepDirHobbyServo(void) : Stepper{} {
        _speed = spindle.speed;
//        check_timer.set(1);
    };

    /* Optional override of init */

    /* Functions that must be implemented in subclasses */

    bool canStep() override { return true; };

    void setMicrosteps(const uint8_t microsteps) override {
        switch (microsteps) {
            case (1): {
                _microsteps_per_step = 32;
                break;
            }
            case (2): {
                _microsteps_per_step = 16;
                break;
            }
            case (4): {
                _microsteps_per_step = 8;
                break;
            }
            case (8): {
                _microsteps_per_step = 4;
                break;
            }
            case (16): {
                _microsteps_per_step = 2;
                break;
            }
            case (32): {
                _microsteps_per_step = 1;
                break;
            }
        }
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

        if (_step_is_forward) {
            _position += _microsteps_per_step;
        } else {
            _position -= _microsteps_per_step;
        }

//        if (!check_timer.isPast()) { return; }
//        check_timer.set(10);

        _speed = (float)_position / 32.0;
        if (_speed > spindle.speed_max) {
            _speed = spindle.speed_max;
        }
        if (_speed < spindle.speed_min) {
            _speed = spindle.speed_min;
        }

        // 0% == spindle.speed_min -> 100% == spindle.speed_max
        if (cm_is_laser_tool()) {
            spindle_speed_immediate(_speed);
        } else {
            // For now the way to zero this is to go move off laser tool.  The process should be
            // T1 (or any tool not T32) do G0W0 to set canonical machine to 0 as well.
            _position = 0;
        }
        cfg.user_data_a[0] = _position;
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

#endif  // STEPP_DIR_HOBBYSERVO_H_ONCE
