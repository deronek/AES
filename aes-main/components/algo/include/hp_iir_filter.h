#pragma once

#ifndef _HP_IIR_FILTER_H
#define _HP_IIR_FILTER_H

#include "task_utils.h"

// constants

// structs
typedef struct hp_iir_filter_type_tag hp_iir_filter_type;

// global variables

// function declarations
hp_iir_filter_type *hp_iir_filter_init(float alpha);
float hp_iir_filter_step(hp_iir_filter_type *filter, float xn);

#endif