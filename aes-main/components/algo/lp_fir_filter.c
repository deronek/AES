#include "lp_fir_filter.h"

// structs
typedef struct lp_fir_filter_type_tag
{
    float coeffs[7];
    float x_old[6];
} lp_fir_filter_type;

// constants

// global variables

// local variables
static const char *TAG = "lp-fir-filter";

// inline function declarations

// function definitions
lp_fir_filter_type *lp_fir_filter_init(float alpha)
{
    lp_fir_filter_type *filter = malloc(sizeof(lp_fir_filter_type));
    if (filter == NULL)
    {
        abort();
    }

    filter->coeffs[0] = -0.03294492055238773;
    filter->coeffs[1] = 0.14593537253343006;
    filter->coeffs[2] = -0.2589258245144724;
    filter->coeffs[3] = 0.31780252022070765;
    filter->coeffs[4] = -0.2589258245144724;
    filter->coeffs[5] = 0.14593537253343006;
    filter->coeffs[6] = -0.03294492055238773;

    lp_fir_filter_reset(filter);

    return filter;
}

void lp_fir_filter_reset(lp_fir_filter_type *filter)
{
    filter->x_old[0] = 0.0;
    filter->x_old[1] = 0.0;
    filter->x_old[2] = 0.0;
    filter->x_old[3] = 0.0;
    filter->x_old[4] = 0.0;
    filter->x_old[5] = 0.0;
}

float lp_fir_filter_step(lp_fir_filter_type *filter, float xn)
{
    float yn = (filter->coeffs[0] * xn) + (filter->coeffs[1] * filter->x_old[0]) + (filter->coeffs[2] * filter->x_old[1]) + (filter->coeffs[3] * filter->x_old[2]) + (filter->coeffs[4] * filter->x_old[3]) + (filter->coeffs[5] * filter->x_old[4]) + (filter->coeffs[6] * filter->x_old[5]);

    filter->x_old[5] = filter->x_old[4];
    filter->x_old[4] = filter->x_old[3];
    filter->x_old[3] = filter->x_old[2];
    filter->x_old[2] = filter->x_old[1];
    filter->x_old[1] = filter->x_old[0];
    filter->x_old[0] = xn;

    return yn;
}