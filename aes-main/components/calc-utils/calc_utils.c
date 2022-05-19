#include "calc_utils.h"

// constants

// global variables

// local variables
static const char *TAG = "calc_utils";
// SixAxis filter;

// constants

// function declarations

// function definitions
/**
 * @brief Get interpolation of parameter value on map Y = f(X)
 */
float calc_utils_interpolate(const float *x, const float *y, const uint8_t num_values, const float value)
{
    float out;

    if (value >= x[num_values - 1])
    {
        return y[num_values - 1];
    }
    if (value <= x[0])
    {
        return y[0];
    }

    uint8_t i = num_values - 2;
    while (value < x[i])
    {
        --i;
    }

    return y[i] + (((value - x[i]) * (y[i + 1] - y[i])) / (x[i + 1] - x[i]));
}