#pragma once

#ifndef _LP_FIR_FILTER_H
#define _LP_FIR_FILTER_H

#include "task_utils.h"

// constants

// structs
typedef struct lp_fir_filter_type_tag lp_fir_filter_type;

// global variables

// function declarations
lp_fir_filter_type *lp_fir_filter_init(float alpha);
float lp_fir_filter_step(lp_fir_filter_type *filter, float xn);

#endif