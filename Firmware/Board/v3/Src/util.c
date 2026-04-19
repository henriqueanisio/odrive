#include "util.h"
#include "macro.h"

int constrain(int x, int a, int b) { return _CONSTRAIN(x, a, b); }

char sign(int x) {
    if (x > 0) return (+1);
    if (x < 0) return (-1);
    return 0;
}
