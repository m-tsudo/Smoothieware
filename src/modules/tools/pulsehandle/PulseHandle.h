/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "libs/Module.h"
#include "libs/Pin.h"

class PulseHandle : public Module{
    public:
        PulseHandle();
        void on_module_loaded();
        void on_config_reload(void *argument);
        void on_idle(void *);

    private:
        Pin encoder_a_pin;
        Pin encoder_b_pin;
        Pin axis_x_pin;
        Pin axis_y_pin;
        Pin axis_z_pin;
        Pin axis_4_pin;
        Pin multiplier_1_pin;
        Pin multiplier_10_pin;
        Pin multiplier_100_pin;
		uint8_t axis;
		uint8_t multiplier;
        int readEncoderDelta();
        uint8_t read_axis();
        uint8_t read_multiplier();
		uint32_t read_pulse(uint32_t dummy);
};
