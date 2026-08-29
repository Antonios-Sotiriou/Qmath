# Qmath

Qmath is a single‑header quaternion math library written in C, inspired by the extraordinary stb libraries (https://github.com/nothings/stb ).  
It provides quaternion and vector operations with optional SSE‑accelerated code paths.

## Features

- Header‑only design
- Pure C API (usable in C and C++)
- Quaternion operations: multiply, normalize, invert, rotate vectors
- Vector and matrix helpers
- Optional SSE intrinsics (`VECTORIZED_CODE`)
- No external dependencies

## Integration

Qmath follows the stb‑style single‑header pattern. Just download and include the Qmath.h header file.

### Implementation (in exactly one `.c` file)

```c
#define QMATH_IMPLEMENTATION
#include "qmath.h"
```

### Optional: enable SSE intrinsics
```c
#define VECTORIZED_CODE
#define QMATH_IMPLEMENTATION
#include "qmath.h"
```
### Include normally everywhere else
```c
#include "qmath.h"
```

### Contributions, improvements, and suggestions are always appreciated.
### Feel free to open issues or submit pull requests.








