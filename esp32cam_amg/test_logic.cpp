// test_logic.cpp — host-side tests for esp32cam_amg.ino pure logic
// Compile: g++ -std=c++17 -o test_logic test_logic.cpp && ./test_logic

#include <cassert>
#include <cstring>
#include <cstdio>
#include <cmath>
#include <cstdint>

// ── logic extracted from esp32cam_amg.ino ────────────────────────────────────
// Mirror exactly — do not change these to match tests.

static void computeIR(float* pixels, float& irMin, float& irMax, float& irAvg) {
    float s = 0, lo = pixels[0], hi = pixels[0];
    for (int i = 0; i < 64; i++) {
        if (pixels[i] < lo) lo = pixels[i];
        if (pixels[i] > hi) hi = pixels[i];
        s += pixels[i];
    }
    irMin = lo; irMax = hi; irAvg = s / 64.0f;
}

static int buildCamPath(char* buf, size_t bufsize, uint32_t imgCount) {
    return snprintf(buf, bufsize, "/cam_%04lu.jpg", (unsigned long)imgCount);
}

// Use uint32_t (not unsigned long) to match ESP32 32-bit arithmetic on 64-bit host
static bool shouldTick(uint32_t now, uint32_t prev, uint32_t interval) {
    return (now - prev) >= interval;
}

static bool shouldRetryAMG(bool amgOk, uint32_t now, uint32_t prevRetry, uint32_t retryMs) {
    return !amgOk && (now - prevRetry) >= retryMs;
}

// ── test harness ─────────────────────────────────────────────────────────────

static int pass_count = 0, fail_count = 0;

#define CHECK(cond) do { \
    if (!(cond)) { \
        printf("  FAIL  [line %d] %s\n", __LINE__, #cond); \
        fail_count++; \
    } else { \
        pass_count++; \
    } \
} while(0)

#define CHECK_NEAR(a, b, eps) CHECK(fabsf((float)(a) - (float)(b)) < (float)(eps))
#define SECTION(name) printf("\n  [%s]\n", name)
#define SUITE(name)   printf("\n== %s ==\n", name)

// ── computeIR tests ──────────────────────────────────────────────────────────

void test_computeIR() {
    SUITE("computeIR");

    SECTION("all same value (25°C)");
    {
        float px[64]; for (int i = 0; i < 64; i++) px[i] = 25.0f;
        float lo, hi, avg;
        computeIR(px, lo, hi, avg);
        CHECK_NEAR(lo,  25.0f, 0.001f);
        CHECK_NEAR(hi,  25.0f, 0.001f);
        CHECK_NEAR(avg, 25.0f, 0.001f);
    }

    SECTION("all zeros");
    {
        float px[64] = {};
        float lo, hi, avg;
        computeIR(px, lo, hi, avg);
        CHECK_NEAR(lo,  0.0f, 0.001f);
        CHECK_NEAR(hi,  0.0f, 0.001f);
        CHECK_NEAR(avg, 0.0f, 0.001f);
    }

    SECTION("one hot pixel at index 63 (100°C), rest 20°C");
    {
        float px[64]; for (int i = 0; i < 64; i++) px[i] = 20.0f;
        px[63] = 100.0f;
        float lo, hi, avg;
        computeIR(px, lo, hi, avg);
        CHECK_NEAR(lo,  20.0f,  0.001f);
        CHECK_NEAR(hi,  100.0f, 0.001f);
        float expected_avg = (63 * 20.0f + 100.0f) / 64.0f; // 21.25
        CHECK_NEAR(avg, expected_avg, 0.01f);
    }

    SECTION("one hot pixel at index 0 (first element = max)");
    {
        float px[64]; for (int i = 0; i < 64; i++) px[i] = 20.0f;
        px[0] = 100.0f;
        float lo, hi, avg;
        computeIR(px, lo, hi, avg);
        CHECK_NEAR(lo, 20.0f,  0.001f);
        CHECK_NEAR(hi, 100.0f, 0.001f);
    }

    SECTION("one cold pixel at index 0 (-10°C), rest 30°C");
    {
        float px[64]; for (int i = 0; i < 64; i++) px[i] = 30.0f;
        px[0] = -10.0f;
        float lo, hi, avg;
        computeIR(px, lo, hi, avg);
        CHECK_NEAR(lo,  -10.0f, 0.001f);
        CHECK_NEAR(hi,   30.0f, 0.001f);
        float expected_avg = (63 * 30.0f + (-10.0f)) / 64.0f; // 29.375
        CHECK_NEAR(avg, expected_avg, 0.01f);
    }

    SECTION("ascending 0..63 — min=0, max=63, avg=31.5");
    {
        float px[64]; for (int i = 0; i < 64; i++) px[i] = (float)i;
        float lo, hi, avg;
        computeIR(px, lo, hi, avg);
        CHECK_NEAR(lo,  0.0f,  0.001f);
        CHECK_NEAR(hi,  63.0f, 0.001f);
        CHECK_NEAR(avg, 31.5f, 0.01f);  // sum=2016, 2016/64=31.5
    }

    SECTION("descending 63..0 — same result as ascending");
    {
        float px[64]; for (int i = 0; i < 64; i++) px[i] = (float)(63 - i);
        float lo, hi, avg;
        computeIR(px, lo, hi, avg);
        CHECK_NEAR(lo,  0.0f,  0.001f);
        CHECK_NEAR(hi,  63.0f, 0.001f);
        CHECK_NEAR(avg, 31.5f, 0.01f);
    }

    SECTION("min at last index (index 63 = 0°C, rest = 50°C)");
    {
        float px[64]; for (int i = 0; i < 64; i++) px[i] = 50.0f;
        px[63] = 0.0f;
        float lo, hi, avg;
        computeIR(px, lo, hi, avg);
        CHECK_NEAR(lo, 0.0f,  0.001f);
        CHECK_NEAR(hi, 50.0f, 0.001f);
    }
}

// ── camPath tests ─────────────────────────────────────────────────────────────

void test_camPath() {
    SUITE("buildCamPath");
    char buf[32];

    SECTION("count=0 → /cam_0000.jpg");
    buildCamPath(buf, sizeof(buf), 0);
    CHECK(strcmp(buf, "/cam_0000.jpg") == 0);

    SECTION("count=1 → /cam_0001.jpg");
    buildCamPath(buf, sizeof(buf), 1);
    CHECK(strcmp(buf, "/cam_0001.jpg") == 0);

    SECTION("count=9999 → /cam_9999.jpg (max 4-digit padding)");
    buildCamPath(buf, sizeof(buf), 9999);
    CHECK(strcmp(buf, "/cam_9999.jpg") == 0);

    SECTION("count=10000 → /cam_10000.jpg (exceeds padding, still valid)");
    buildCamPath(buf, sizeof(buf), 10000);
    CHECK(strcmp(buf, "/cam_10000.jpg") == 0);

    SECTION("count=UINT32_MAX → fits in buf[32], no truncation");
    int len = buildCamPath(buf, sizeof(buf), 0xFFFFFFFFUL);
    CHECK(strcmp(buf, "/cam_4294967295.jpg") == 0);
    CHECK(len < 32);           // snprintf returns bytes that would be written
    CHECK(len == (int)strlen(buf));  // no truncation

    SECTION("result always null-terminated and within bounds");
    buildCamPath(buf, sizeof(buf), 123456);
    CHECK(strlen(buf) < sizeof(buf));
}

// ── timing logic tests ────────────────────────────────────────────────────────

void test_timing() {
    SUITE("shouldTick (interval)");

    SECTION("elapsed < interval → no tick");
    CHECK(!shouldTick(999, 0, 1000));

    SECTION("elapsed == interval → tick");
    CHECK(shouldTick(1000, 0, 1000));

    SECTION("elapsed > interval → tick");
    CHECK(shouldTick(1500, 0, 1000));

    SECTION("no tick when now == prev");
    CHECK(!shouldTick(1000, 1000, 1000));

    SECTION("uint32_t wraparound: now just crossed 0, prev near max");
    // prev=0xFFFFFFFC, now=5 → elapsed = 5 - 0xFFFFFFFC = 9 (mod 2^32)
    CHECK(shouldTick(5U, 0xFFFFFFFCU, 8U));   // 9 >= 8 → tick

    SECTION("uint32_t wraparound: not yet ready");
    // prev=0xFFFFFFFC, now=3 → elapsed = 7 < 8 → no tick
    CHECK(!shouldTick(3U, 0xFFFFFFFCU, 8U));

    SUITE("shouldRetryAMG");

    SECTION("amgOk=true → never retry");
    CHECK(!shouldRetryAMG(true, 10000, 0, 5000));

    SECTION("amgOk=false, elapsed < retryMs → no retry");
    CHECK(!shouldRetryAMG(false, 4999, 0, 5000));

    SECTION("amgOk=false, elapsed == retryMs → retry");
    CHECK(shouldRetryAMG(false, 5000, 0, 5000));

    SECTION("amgOk=false, first loop tick after boot (~6s, prevRetry=0) → retry fires");
    CHECK(shouldRetryAMG(false, 6000, 0, 5000));

    SECTION("amgOk=false, second retry window not yet open");
    CHECK(!shouldRetryAMG(false, 9999, 5000, 5000));

    SECTION("amgOk=false, second retry window open");
    CHECK(shouldRetryAMG(false, 10000, 5000, 5000));
}

// ── CSV format tests ──────────────────────────────────────────────────────────

void test_csvFormat() {
    SUITE("CSV format");
    char line[256];

    SECTION("cam ok: correct field order and precision");
    {
        const char* camPath = "/cam_0001.jpg";
        snprintf(line, sizeof(line), "%lu,%s,%.2f,%.2f,%.2f",
                 1000UL, camPath, 21.5f, 35.2f, 28.3f);
        CHECK(strcmp(line, "1000,/cam_0001.jpg,21.50,35.20,28.30") == 0);
    }

    SECTION("cam fail: writes FAIL not stale path");
    {
        const char* camPath = "/cam_0001.jpg";
        bool camOk = false;
        snprintf(line, sizeof(line), "%lu,%s,%.2f,%.2f,%.2f",
                 2000UL, camOk ? camPath : "FAIL", 21.5f, 35.2f, 28.3f);
        CHECK(strcmp(line, "2000,FAIL,21.50,35.20,28.30") == 0);
    }

    SECTION("pixel format %.1f: integer value");
    {
        char px[16];
        snprintf(px, sizeof(px), "%.1f", 0.0f);
        CHECK(strcmp(px, "0.0") == 0);
    }

    SECTION("pixel format %.1f: decimal truncated");
    {
        char px[16];
        snprintf(px, sizeof(px), "%.1f", 25.0f);
        CHECK(strcmp(px, "25.0") == 0);
    }

    SECTION("pixel format %.1f: negative temperature");
    {
        char px[16];
        snprintf(px, sizeof(px), "%.1f", -5.0f);
        CHECK(strcmp(px, "-5.0") == 0);
    }

    SECTION("CSV header has 5 fixed + 64 pixel columns = 69 total");
    {
        // count commas in a full row with 64 pixels
        char row[2048];
        int pos = snprintf(row, sizeof(row), "%lu,%s,%.2f,%.2f,%.2f",
                          0UL, "/cam_0000.jpg", 0.0f, 0.0f, 0.0f);
        for (int i = 0; i < 64; i++)
            pos += snprintf(row + pos, sizeof(row) - pos, ",%.1f", (float)i);

        int commas = 0;
        for (char* p = row; *p; p++) if (*p == ',') commas++;
        CHECK(commas == 68);  // 69 columns → 68 commas
    }

    SECTION("timestamp is unsigned long, no negative values");
    {
        uint32_t ts = 0xFFFFFFFFUL;
        snprintf(line, sizeof(line), "%lu", (unsigned long)ts);
        CHECK(strcmp(line, "4294967295") == 0);
    }
}

// ── main ─────────────────────────────────────────────────────────────────────

int main() {
    test_computeIR();
    test_camPath();
    test_timing();
    test_csvFormat();

    printf("\n─────────────────────────────────────\n");
    printf("PASSED: %d  FAILED: %d\n", pass_count, fail_count);
    return fail_count > 0 ? 1 : 0;
}
