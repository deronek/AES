#include "lp_iir_filter.h"

// structs
typedef struct lp_iir_filter_type_tag
{
    float alpha;
    float y_n_1;
} lp_iir_filter_type;

// constants

// global variables

// local variables
static const char *TAG = "lp-iir-filter";

// inline function declarations

// function definitions
lp_iir_filter_type *lp_iir_filter_init(float alpha)
{
    lp_iir_filter_type *filter = malloc(sizeof(lp_iir_filter_type));
    if (filter == NULL)
    {
        abort();
    }

    filter->alpha = alpha;
    filter->y_n_1 = 0;

    return filter;
}

float lp_iir_filter_step(lp_iir_filter_type *filter, float xn)
{
    /**
     * @brief Calculate filter output.
     */
    float yn = ((1 - filter->alpha) * filter->y_n_1) + (filter->alpha * xn);

    /**
     * @brief Save values for use in the next iteration.
     */
    filter->y_n_1 = yn;

    return yn;
}