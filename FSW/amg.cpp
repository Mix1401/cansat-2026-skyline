// amg.cpp
#include "amg.h"

static Adafruit_AMG88xx amg;
AMGData amg_data;
bool    is_AMG = false;

bool initAMG() {
    if (!amg.begin(0x69)) return false;
    delay(100);   // datasheet: 100 ms boot
    return true;
}

void readAMG() {
    amg.readPixels(amg_data.pixels);

    float mn =  1e6f, mx = -1e6f, sum = 0.0f;
    uint8_t hot = 0;
    for (uint8_t i = 0; i < 64; i++) {
        float v = amg_data.pixels[i];
        if (v < mn) mn = v;
        if (v > mx) { mx = v; hot = i; }
        sum += v;
    }
    amg_data.min_t   = mn;
    amg_data.max_t   = mx;
    amg_data.avg_t   = sum / 64.0f;
    amg_data.hot_idx = hot;
}
