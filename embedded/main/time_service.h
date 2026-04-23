#pragma once

#include <stdint.h>

class TimeService {
public:
    static int64_t current_epoch_seconds();
};
