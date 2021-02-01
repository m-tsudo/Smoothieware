/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Kernel.h"
#include "MPG.h"
#include "MPGPublicAccess.h"
#include "checksumm.h"
#include "Config.h"
#include "ConfigValue.h"
#include "utils.h"
#include "Robot.h"
#include "Conveyor.h"
#include "StepperMotor.h"
#include "SlowTicker.h"
#include "PublicDataRequest.h"
#include "mbed.h"

// global config settings
#define enable_checksum               CHECKSUM("enable")
#define frequency_checksum            CHECKSUM("frequency")
#define delay_checksum                CHECKSUM("delay")
#define pilot_pin_checksum            CHECKSUM("pilot_pin")
#define axis_x_pin_checksum           CHECKSUM("axis_x_pin")
#define axis_y_pin_checksum           CHECKSUM("axis_y_pin")
#define axis_z_pin_checksum           CHECKSUM("axis_z_pin")
#define axis_4_pin_checksum           CHECKSUM("axis_4_pin")
#define multiplier_1_pin_checksum     CHECKSUM("multiplier_1_pin")
#define multiplier_10_pin_checksum    CHECKSUM("multiplier_10_pin")
#define multiplier_100_pin_checksum   CHECKSUM("multiplier_100_pin")
#define multiplier_1_value_checksum   CHECKSUM("multiplier_1_value")
#define multiplier_10_value_checksum  CHECKSUM("multiplier_10_value")
#define multiplier_100_value_checksum CHECKSUM("multiplier_100_value")
#define encoder_a_pin_checksum        CHECKSUM("encoder_a_pin")
#define encoder_b_pin_checksum        CHECKSUM("encoder_b_pin")

#define OFF_AXIS 255

typedef enum {
    IDLE,
    STEPPING,
} STATE;

MPG::MPG()
{
    delta = 0;
    axis = OFF_AXIS;
    multiplier = 1;
}

void MPG::on_module_loaded()
{
    // if the module is disabled -> do nothing
    if(! THEKERNEL->config->value(mpg_checksum, enable_checksum)->by_default(false)->as_bool()) {
        // as this module is not needed free up the resource
        delete this;
        return;
    }

    on_config_reload(this);

    register_for_event(ON_GET_PUBLIC_DATA);
    register_for_event(ON_IDLE);
    THEKERNEL->slow_ticker->attach(frequency, this, &MPG::read_pulse);
}

void MPG::on_config_reload(void *argument)
{
    // pilot
    this->pilot_pin.from_string(THEKERNEL->config->value( mpg_checksum, pilot_pin_checksum)->by_default("nc")->as_string())->as_output();
    this->frequency = THEKERNEL->config->value( mpg_checksum, frequency_checksum)->by_default("1000")->as_int();
    this->delay = THEKERNEL->config->value( mpg_checksum, delay_checksum)->by_default("100")->as_int();

    // axis
    this->axis_x_pin.from_string(THEKERNEL->config->value( mpg_checksum, axis_x_pin_checksum)->by_default("nc")->as_string())->as_input();
    this->axis_y_pin.from_string(THEKERNEL->config->value( mpg_checksum, axis_y_pin_checksum)->by_default("nc")->as_string())->as_input();
    this->axis_z_pin.from_string(THEKERNEL->config->value( mpg_checksum, axis_z_pin_checksum)->by_default("nc")->as_string())->as_input();
    this->axis_4_pin.from_string(THEKERNEL->config->value( mpg_checksum, axis_4_pin_checksum)->by_default("nc")->as_string())->as_input();

    // multiplier
    this->multiplier_1_pin.from_string(THEKERNEL->config->value( mpg_checksum, multiplier_1_pin_checksum)->by_default("nc")->as_string())->as_input();
    this->multiplier_10_pin.from_string(THEKERNEL->config->value( mpg_checksum, multiplier_10_pin_checksum)->by_default("nc")->as_string())->as_input();
    this->multiplier_100_pin.from_string(THEKERNEL->config->value( mpg_checksum, multiplier_100_pin_checksum)->by_default("nc")->as_string())->as_input();
    this->multiplier_1_value   = THEKERNEL->config->value( mpg_checksum, multiplier_1_value_checksum)->by_default(1)->as_int();
    this->multiplier_10_value   = THEKERNEL->config->value( mpg_checksum, multiplier_10_value_checksum)->by_default(10)->as_int();
    this->multiplier_100_value   = THEKERNEL->config->value( mpg_checksum, multiplier_100_value_checksum)->by_default(100)->as_int();

    // encoder
    this->encoder_a_pin.from_string(THEKERNEL->config->value( mpg_checksum, encoder_a_pin_checksum)->by_default("nc")->as_string())->as_input();
    this->encoder_b_pin.from_string(THEKERNEL->config->value( mpg_checksum, encoder_b_pin_checksum)->by_default("nc")->as_string())->as_input();
}

void MPG::on_get_public_data(void *argument)
{
    PublicDataRequest *pdr = static_cast<PublicDataRequest *>(argument);

    if (!pdr->starts_with(mpg_checksum)) return;

    struct mpg_state *state= static_cast<struct mpg_state *>(pdr->get_data_ptr());
    state->frequency = this->frequency;
    state->delay = this->delay;
    state->axis = this->axis;
    state->multiplier = this->multiplier;
    pdr->set_taken();
}

int MPG::readEncoderDelta()
{
    static int8_t enc_states[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
    static uint8_t old_AB = 0;
    if(this->encoder_a_pin.connected()) {
        old_AB <<= 2;                   //remember previous state
        old_AB |= ( this->encoder_a_pin.get() + ( this->encoder_b_pin.get() * 2 ) );  //add current state
        return  enc_states[(old_AB & 0x0f)];

    } else {
        return 0;
    }
}

// Called every millisecond in an ISR
uint32_t MPG::read_pulse(uint32_t dummy)
{
    delta +=  this->readEncoderDelta();

    return 0;
}


uint8_t MPG::read_axis()
{
    uint8_t axis = OFF_AXIS;

    if (this->axis_x_pin.get())
        axis = X_AXIS;
    else if (this->axis_y_pin.get())
        axis = Y_AXIS;
    else if (this->axis_z_pin.get())
        axis = Z_AXIS;
    else if (this->axis_4_pin.get())
        axis = A_AXIS;

    return axis;
}

uint8_t MPG::read_multiplier()
{
    uint8_t multiplier = 0;

    if (this->multiplier_1_pin.get())
        multiplier = this->multiplier_1_value;
    else if (this->multiplier_10_pin.get())
        multiplier = this->multiplier_10_value;
    else if (this->multiplier_100_pin.get())
        multiplier = this->multiplier_100_value;

    return multiplier;
}

void MPG::on_idle(void *)
{
    static STATE state = IDLE;
    if (state == STEPPING)
        return;

    uint8_t next_axis = this->read_axis();
    if (axis != next_axis) {
        axis = next_axis;
        delta = 0;
    }
    multiplier = this->read_multiplier();
    pilot_pin.set(axis != OFF_AXIS);

    int tmp = delta;
    delta = 0;
    bool dir = tmp > 0;
    int steps = std::abs(tmp);
    int n_motors= THEROBOT->get_number_registered_motors();
    if (THECONVEYOR->is_idle() && (axis < n_motors) && (0 < steps)) {
        state = STEPPING;
        int steps_m = steps * multiplier;
        for (int i = 0; i < steps_m; ++i) {
            if (THEKERNEL->is_halted()) break;
            if (i != 0) safe_delay_us(std::max(delay, 1000 / steps_m));
            THEROBOT->actuators[axis]->manual_step(dir);
        }
        // reset the position based on current actuator position
        THEROBOT->reset_position_from_current_actuator_position();
        state = IDLE;
    }
}

