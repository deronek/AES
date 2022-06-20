#pragma once

#ifndef _LP_IIR_FILTER_H
#define _LP_IIR_FILTER_H

#include "task_utils.h"

// constants

// structs
typedef struct lp_iir_filter_type_tag lp_iir_filter_type;

// global variables

// function declarations
lp_iir_filter_type *lp_iir_filter_init(float alpha);
float lp_iir_filter_step(lp_iir_filter_type *filter, float xn);

#endif