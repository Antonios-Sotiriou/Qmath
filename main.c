//#define VECTORIZED_CODE
#define QMATH_IMPLEMENTATION
#include "qmath.h"

#include <stdio.h>
#include <time.h>

#define samples            (100)
#define iterations         (1000000)

int main() {

    /* Init Values. */
    mat4x4 tm = { 0 };
    quat q = quatFromMat(tm);

    float average = 0.f;
    for (int j = 0; j < samples; j++) {
        clock_t start = clock(), diff;

        for (int i = 0; i < iterations; i++) {
            /* functions to be tested for performance. */
            q = quatFromMat(tm);
        }

        diff = clock() - start;
        float secs = (float)diff / CLOCKS_PER_SEC;
        printf("%f secs\n", secs);
        average += secs;
    }

    printf("Average execution time: %f\n", average / samples);

	return 0;
}