
static int failures = 0;
#define CHECK(cond) \
    do { if (!(cond)) { \
    std::cerr << "FAIL: " #cond "  (line " << __LINE__ << ")\n"; ++failures; \
    } } while (0)
    