#include "comp_iir_filter.h"

// structs
typedef struct comp_iir_filter_type_tag
{
    float alpha;
    float y_n_1;
    float lp_y_n_1;
    float hp_y_n_1;
    float hp_x_n_1;
} comp_iir_filter_type;

// constants

// global variables

// local variables
static const char *TAG = "algo-iir-filter";

// inline function declarations

// function definitions
comp_iir_filter_type *comp_iir_filter_init(float alpha)
{
    comp_iir_filter_type *filter = malloc(sizeof(comp_iir_filter_type));
    if (filter == NULL)
    {
        abort();
    }

    filter->alpha = alpha;
    filter->y_n_1 = 0;
    filter->lp_y_n_1 = 0;
    filter->hp_y_n_1 = 0;
    filter->hp_x_n_1 = 0;

    return filter;
}

float comp_iir_filter_step(comp_iir_filter_type *filter, float lp_xn, float hp_xn)
{
    /**
     * @brief Low pass step
     */
    float lp_yn = ((1 - filter->alpha) * lp_xn) + (filter->alpha * filter->lp_y_n_1);
    filter->lp_y_n_1 = lp_yn;

    /**
     * @brief High pass step
     */
    float hp_yn = (filter->alpha * filter->hp_y_n_1) + (filter->alpha * (filter->hp_x_n_1 - hp_xn));
    filter->hp_y_n_1 = hp_yn;
    filter->hp_x_n_1 = hp_xn;

    /**
     * @brief Summation step
     * Return calculated value from step.
     */
    filter->y_n_1 = lp_yn + hp_yn;
    return filter->y_n_1;
}