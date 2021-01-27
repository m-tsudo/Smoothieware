#ifndef __PULSEHANDLEPUBLICACCESS_H
#define __PULSEHANDLEPUBLICACCESS_H

// addresses used for public data access
#define pulsehandle_checksum              CHECKSUM("pulsehandle")
#define pulsehandle_state_checksum        CHECKSUM("pulsehandle_state")

struct pulsehandle_state {
	uint8_t axis;
	uint8_t multiplier;
};

#endif // __PULSEHANDLEPUBLICACCESS_H

