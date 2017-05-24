/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
 * @file umn_algorithm.cpp
 *
 * @author HMokhtarzadeh <hamid@organicnavigation.com>
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <cmath>
#include <drivers/drv_hrt.h>

// #include <mathlib/mathlib.h> // Math and matrix library.
#include <matrix/math.hpp>

// Include headers for uORB topics
#include <uORB/uORB.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/wind_estimate.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/umn_output.h>

#include "umn_algorithm.h"

// using math::Matrix;
// using math::Vector;
using namespace matrix;

/* 3.) Skew Operator - Evan */

	Matrix<float, 3, 3> skew(Matrix<float, 3, 1> a);

	Matrix<float, 3, 3> skew(Matrix<float, 3, 1> a) {

		float S_data[9] = {0, -a(2, 0), a(1, 0), a(2, 0), 0, -a(0, 0), -a(1, 0), a(0, 0), 0};
		Matrix3f S(S_data);
		
		return S;
	}
	

bool update(uint64_t time_usec, struct sensor_combined_s *sensorCombined_ptr, struct airspeed_s *airSpeed_ptr, struct vehicle_gps_position_s *gps_ptr, struct vehicle_land_detected_s *landDetected_ptr, struct umn_output_s *uout_ptr){
	
	
	return true;
}


