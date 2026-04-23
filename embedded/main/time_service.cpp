#include "time_service.h"

#include <time.h>

int64_t TimeService::current_epoch_seconds() {
    time_t now = 0;
    time(&now);
    return static_cast<int64_t>(now);
}
