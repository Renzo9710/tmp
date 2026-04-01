/**
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown
 *
 * @author D. Duberg, KTH Royal Institute of Technology, Copyright (c) 2020.
 * @see https://github.com/UnknownFreeOccupied/ufomap
 * License: BSD 3
 *
 * 
 * MODIFIED by: Heiko Renz, 2026, Institute of Control Theory and Systems Engineering, TU Dortmund University, Germany
 */

/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2020, D. Duberg, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef UFO_MAP_PREDICTION_H
#define UFO_MAP_PREDICTION_H

// CUDA
#include <cuda_runtime.h>
#include <cuda/std/array>  // NOLINT(build/include_alpha)
#include <cuda/std/cmath>
#ifdef __CUDACC__
#ifndef CUDA_CALL
#define CUDA_CALL __host__ __device__
#endif
#else
#ifndef CUDA_CALL
#define CUDA_CALL
#endif
#endif

// STD
#include <cstdint>
#include <tuple>
#include <vector>

namespace ufo::map
{
const size_t array_size_prediction = 32;
struct Prediction
{
  // TODO(renz): Check options to switch to dynamic size option which is also suitable fpr __device__ calls (maybe
  // thrust vector). Currently only supporting fixed size of max 30 extrapolation steps with 0.1 increments.
  cuda::std::array<bool, array_size_prediction> _is_occupied_at_time;
  cuda::std::array<double, array_size_prediction> _time_from_now;

  CUDA_CALL Prediction()
  {
	_is_occupied_at_time.fill(false);
	// _is_set.fill(false);
	// time from now from 0 to 2.9 in 0.1 increments
	for (int i = 0; i < array_size_prediction; i++)
	{
	  _time_from_now[i] = i * 0.1;
	}
  }

  CUDA_CALL Prediction(Prediction const& other)
  {
	_is_occupied_at_time = other._is_occupied_at_time;
	_time_from_now = other._time_from_now;
  }

  bool operator==(Prediction const& other) const
  {
	return other._time_from_now == _time_from_now && other._is_occupied_at_time == _is_occupied_at_time;
  }

  bool operator!=(Prediction const& other) const
  {
	return other._time_from_now != _time_from_now || other._is_occupied_at_time != _is_occupied_at_time;
  }

  CUDA_CALL Prediction& operator=(Prediction const& rhs)
  {
	_is_occupied_at_time = rhs._is_occupied_at_time;
	_time_from_now = rhs._time_from_now;
	return *this;
  }

  CUDA_CALL bool setPredictionAtTime(double time, bool is_occupied)
  {
	// set the occupied value at the time
	for (int i = 0; i < array_size_prediction; i++)
	{
	  // check if time is the same with tolerance of 1e-6
	  if (cuda::std::fabs(time - _time_from_now[i]) < 1e-3)
	  {
		_is_occupied_at_time[i] = is_occupied;
		return true;
	  }
	}
	printf("Time %f not found in prediction array \n", time);
	return false;
  }

  // Only possible with fixed size array and known time index
  CUDA_CALL bool setPredictionAtElem(int elem, bool is_occupied)
  {
	// set the occupied value at the time
	if (elem < array_size_prediction)
	{
	  _is_occupied_at_time[elem] = is_occupied;
	  return true;
	}
	else
	{
	  printf("Elem %d not found in prediction array \n", elem);
	  return false;
	}
  }

  CUDA_CALL bool getPredictionAtTime(double time)
  {
	// get the occupied value at the time
	// if the time is greater than 2.9, return the last value
	if (time > 2.9)
	{
	  return _is_occupied_at_time[array_size_prediction - 1];
	}
	// check from 0 to 2.9 in 0.1 increments
	for (int i = 0; i < array_size_prediction; i++)
	{
	  if (cuda::std::fabs(time - _time_from_now[i]) < 1e-3)
	  {
		if (_is_occupied_at_time[i])
		  printf("Is occupied %d at time %f\n", _is_occupied_at_time[i], time);
		return _is_occupied_at_time[i];
	  }
	}

	return false;
  }

  CUDA_CALL bool getPredictionAtElement(int elem)
  {
	// get the occupied value at the time
	if (elem < array_size_prediction)
	{
	  return _is_occupied_at_time[elem];
	}
	else
	{
	  printf("Elem %d\n not found in prediction array", elem);
	  return false;	 // return the as free
	}
  }

  // CUDA_CALL bool getIsPredictionSetAtElement(int elem)
  // {
  // 	// get the occupied value at the time
  // 	if (elem < array_size_prediction)
  // 	{
  // 		return _is_set[elem];
  // 	}
  // 	else
  // 	{
  // 		printf("Elem %d\n not found in prediction array", elem);
  // 		return false; // return the as free
  // 	}
  // }

  CUDA_CALL cuda::std::array<bool, array_size_prediction>& getAllPrediction()
  {
	return _is_occupied_at_time;
  }

  CUDA_CALL cuda::std::array<double, array_size_prediction>& getAllTime()
  {
	return _time_from_now;
  }

  CUDA_CALL bool integrate(Prediction const& other)
  {
	bool changed = false;
	for (int i = 0; i < array_size_prediction; i++)
	{
	  if (cuda::std::fabs(_time_from_now[i] - other._time_from_now[i]) < 1e-3)
	  {
		if (_is_occupied_at_time[i] != other._is_occupied_at_time[i])
		{
		  changed = true;
		}
		_is_occupied_at_time[i] = other._is_occupied_at_time[i];
	  }
	}
	return changed;
  }

  CUDA_CALL bool integrate(Prediction const& other, int elem)
  {
	bool changed = false;
	if (elem < array_size_prediction)
	{
	  if (_is_occupied_at_time[elem] != other._is_occupied_at_time[elem])
	  {
		changed = true;
	  }
	  _is_occupied_at_time[elem] = other._is_occupied_at_time[elem];
	}
	return changed;
  }

  CUDA_CALL void reset()
  {
	_is_occupied_at_time.fill(false);
  }
};
}  // namespace ufo::map

#endif  // UFO_MAP_PREDICTION_H
