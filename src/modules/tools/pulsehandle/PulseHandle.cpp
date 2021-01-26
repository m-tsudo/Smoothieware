/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "PulseHandle.h"
#include "SlowTicker.h"

#define PULSEHANDLE_CHECKSUMS(X) {            \
    CHECKSUM(X "_TODO"),           \
}

// global config settings


PulseHandle::PulseHandle()
{
}

void PulseHandle::on_module_loaded()
{
    // check for new config syntax
    if(!load_config()) {
        delete this;
        return;
    }

    register_for_event(ON_IDLE);

    THEKERNEL->slow_ticker->attach(1000, this, &PulseHandle::read_pulse);
}

// Get config using new syntax supports ABC
bool PulseHandle::load_config()
{
    return true;
}

void PulseHandle::on_idle(void*)
{
}

// Called every millisecond in an ISR
uint32_t PulseHandle::read_pulse(uint32_t dummy)
{
    return 0;
}
