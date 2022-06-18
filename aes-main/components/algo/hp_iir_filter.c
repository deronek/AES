#include "hp_iir_filter.h"

// structs
typedef struct hp_iir_filter_type_tag
{
    float alpha;
    float y_n_1;
    float x_n_1;
} hp_iir_filter_type;

// constants

// global variables

// local variables
static const char *TAG = "algo-iir-filter";

// inline function declarations

// function definitions
hp_iir_filter_type *hp_iir_filter_init(float alpha)
{
    hp_iir_filter_type *filter = malloc(sizeof(hp_iir_filter_type));
    if (filter == NULL)
    {
        abort();
    }

    filter->alpha = alpha;
    filter->y_n_1 = 0;
    filter->x_n_1 = 0;

    return filter;
}

float hp_iir_filter_step(hp_iir_filter_type *filter, float xn)
{
    /**
     * @brief Calculate filter output.
     */
    float yn = (filter->alpha * filter->y_n_1) + (filter->alpha * (filter->x_n_1 - xn));

    /**
     * @brief Save values for use in the next iteration.
     */
    filter->y_n_1 = yn;
    filter->x_n_1 = xn;

    return yn;
}