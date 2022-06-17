#pragma once

#ifndef _COMP_IIR_FILTER_H
#define _COMP_IIR_FILTER_H

#include "task_utils.h"

// constants

// structs
typedef struct comp_iir_filter_type_tag comp_iir_filter_type;

// global variables

// function declarations
comp_iir_filter_type *comp_iir_filter_init(float alpha);
float comp_iir_filter_step(comp_iir_filter_type *filter, float lp_xn, float hp_xn);

#endif