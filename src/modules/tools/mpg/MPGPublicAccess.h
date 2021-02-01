#ifndef __MANUALPULSEGENERATORPUBLICACCESS_H
#define __MANUALPULSEGENERATORPUBLICACCESS_H

// addresses used for public data access
#define mpg_checksum              CHECKSUM("mpg")
#define mpg_state_checksum        CHECKSUM("mpg_state")

struct mpg_state {
    int frequency;
    uint8_t axis;
    uint8_t multiplier;
};

#endif // __MANUALPULSEGENERATORPUBLICACCESS_H

