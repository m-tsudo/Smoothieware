/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Kernel.h"
#include "PulseHandle.h"
#include "PulseHandlePublicAccess.h"
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
#define enable_checksum             CHECKSUM("enable")
#define axis_x_pin_checksum         CHECKSUM("axis_x_pin")
#define axis_y_pin_checksum         CHECKSUM("axis_y_pin")
#define axis_z_pin_checksum         CHECKSUM("axis_z_pin")
#define axis_4_pin_checksum         CHECKSUM("axis_4_pin")
#define multiplier_1_pin_checksum   CHECKSUM("multiplier_1_pin")
#define multiplier_10_pin_checksum  CHECKSUM("multiplier_10_pin")
#define multiplier_100_pin_checksum CHECKSUM("multiplier_100_pin")
#define encoder_a_pin_checksum      CHECKSUM("encoder_a_pin")
#define encoder_b_pin_checksum      CHECKSUM("encoder_b_pin")

#define OFF_AXIS 255

PulseHandle::PulseHandle()
{
    axis = OFF_AXIS;
    multiplier = 1;
}

void PulseHandle::on_module_loaded()
{
    // if the module is disabled -> do nothing
    if(! THEKERNEL->config->value(pulsehandle_checksum, enable_checksum)->by_default(false)->as_bool()) {
        // as this module is not needed free up the resource
        delete this;
        return;
    }

    on_config_reload(this);

    register_for_event(ON_GET_PUBLIC_DATA);
    register_for_event(ON_IDLE);
    THEKERNEL->slow_ticker->attach(1000, this, &PulseHandle::read_pulse);
}

void PulseHandle::on_config_reload(void *argument)
{
    // axis
    this->axis_x_pin.from_string(THEKERNEL->config->value( pulsehandle_checksum, axis_x_pin_checksum)->by_default("nc")->as_string())->as_input();
    this->axis_y_pin.from_string(THEKERNEL->config->value( pulsehandle_checksum, axis_y_pin_checksum)->by_default("nc")->as_string())->as_input();
    this->axis_z_pin.from_string(THEKERNEL->config->value( pulsehandle_checksum, axis_z_pin_checksum)->by_default("nc")->as_string())->as_input();
    this->axis_4_pin.from_string(THEKERNEL->config->value( pulsehandle_checksum, axis_4_pin_checksum)->by_default("nc")->as_string())->as_input();

    // multiplier
    this->multiplier_1_pin.from_string(THEKERNEL->config->value( pulsehandle_checksum, multiplier_1_pin_checksum)->by_default("nc")->as_string())->as_input();
    this->multiplier_10_pin.from_string(THEKERNEL->config->value( pulsehandle_checksum, multiplier_10_pin_checksum)->by_default("nc")->as_string())->as_input();
    this->multiplier_100_pin.from_string(THEKERNEL->config->value( pulsehandle_checksum, multiplier_100_pin_checksum)->by_default("nc")->as_string())->as_input();

    // encoder
    this->encoder_a_pin.from_string(THEKERNEL->config->value( pulsehandle_checksum, encoder_a_pin_checksum)->by_default("nc")->as_string())->as_input();
    this->encoder_b_pin.from_string(THEKERNEL->config->value( pulsehandle_checksum, encoder_b_pin_checksum)->by_default("nc")->as_string())->as_input();
}

void PulseHandle::on_get_public_data(void *argument)
{
    PublicDataRequest *pdr = static_cast<PublicDataRequest *>(argument);

    if (!pdr->starts_with(pulsehandle_checksum)) return;

    struct pulsehandle_state *state= static_cast<struct pulsehandle_state *>(pdr->get_data_ptr());
    state->axis = this->axis;
    state->multiplier = this->multiplier;
    pdr->set_taken();
}

int PulseHandle::readEncoderDelta()
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
uint32_t PulseHandle::read_pulse(uint32_t dummy)
{
    int change = this->readEncoderDelta();
    int n_motors= THEROBOT->get_number_registered_motors();

    if (change == 0) return 0;
    if (n_motors <= axis) return 0;
    if (!THECONVEYOR->is_idle()) return 0;
    if (THEKERNEL->is_halted()) return 0;

    for (int i = 0; i < multiplier; ++i) {
        if (i != 0) wait_us(100);
        THEROBOT->actuators[axis]->manual_step(change < 0);
    }

    // reset the position based on current actuator position
    THEROBOT->reset_position_from_current_actuator_position();

    return 0;
}


uint8_t PulseHandle::read_axis()
{
    uint8_t axis = OFF_AXIS;

    if (this->axis_x_pin.connected() && this->axis_x_pin.get()) 
        axis = X_AXIS;
    else if (this->axis_y_pin.connected() && this->axis_y_pin.get())
        axis = Y_AXIS;
    else if (this->axis_z_pin.connected() && this->axis_z_pin.get())
        axis = Z_AXIS;
    else if (this->axis_4_pin.connected() && this->axis_4_pin.get())
        axis = A_AXIS;

    return axis;
}

uint8_t PulseHandle::read_multiplier()
{
    uint8_t multiplier = 0;

    if (this->multiplier_1_pin.connected() && this->multiplier_1_pin.get()) 
        multiplier = 1;
    else if (this->multiplier_10_pin.connected() && this->multiplier_10_pin.get())
        multiplier = 10;
    else if (this->multiplier_100_pin.connected() && this->multiplier_100_pin.get())
        multiplier = 100;

    return multiplier;
}

void PulseHandle::on_idle(void *)
{
    axis = this->read_axis();
    multiplier = this->read_multiplier();
}

