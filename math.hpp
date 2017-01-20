/***********************************************************************************
 * Copyright (c) 2016-2017, UT-Battelle
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the <organization> nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **********************************************************************************/
#ifndef COMMON_MATH_HPP_
#define COMMON_MATH_HPP_

#include "compiler.hpp"
#include "exception/QCIError.hpp"

namespace math {
	/**
	 * \brief Round a number up to a given multiple.
	 * 
	 * \warning If T is an integral type, only use with a signed if necessary.
	 *          Using an unsigned type results in much more efficient operation.
	 * 
	 * \note Pay special attention to the constness of the parameters. Supplying
	 *       a const results in much more efficient operation.
	 */
	 template <typename T>
	 		constexpr inline T round_up(const T number, const T multiple) {
		static_assert(std::is_arithmetic<T>::value, "T not arithmetic type.");
		
		return (((number + multiple - 1) / multiple) * multiple);
	}
	
	/**
	 * \brief Change a block of unsigned ASCII digits to arithmetic type T and
	 *        do not check each character for validity.
	 * 
	 * \warning Only use if you can guarantee that the size is not 0 and all
	 *          chars within the buffer are valid ASCII digits.
	 */
	template <typename T, bool check = true,
			typename std::enable_if<!std::is_floating_point<T>::value, bool>::type = true,
			typename std::enable_if<!check, bool>::type = true>
	        inline T sta(void *const _buf,
	        			const std::size_t size) {
		static_assert(std::is_arithmetic<T>::value, "T not arithmetic type.");
		
		/** \TODO: Allow negatives */
		const char *const buf = static_cast<const char *const>(_buf);
		T result = buf[0] - '0';
		
		for(size_t i = 1; i < size; i++) {
	        result *= 10;
	        result += buf[i] - '0';
	    }
		
		return result;
	}
	
	/**
	 * \brief Change a block of unsigned ASCII digits to arithmetic type T and
	 * 		  check each character for validity.
	 * 
	 * \throws Throws std::invalid_argument if the size of the buffer is given
	 * to be zero or if any character other than ASCII 0 through 9 is found.
	 */
	template <typename T, bool check = true,
			typename std::enable_if<!std::is_floating_point<T>::value, bool>::type = true,
			typename std::enable_if<check, bool>::type = true>
	        inline T sta(void *const _buf,
	        			const std::size_t size) {
		static_assert(std::is_arithmetic<T>::value, "T not arithmetic type.");
		
		/** \TODO: Allow negatives */
		const char *const buf = static_cast<const char *const>(_buf);
		T result;
		
		if(UNLIKELY(size == 0)) QCIError("Zero length buffer.");
		if(UNLIKELY(!('0' <= buf[0] && buf[0] <= '9'))) goto invalid_digit;
		
		result = buf[0] - '0';
		
		for(size_t i = 1; i < size; i++) {
        	if(UNLIKELY(!('0' <= buf[i] && buf[i] <= '9'))) goto invalid_digit;
	        result *= 10;
	        result += buf[i] - '0';
	    }
		
		return result;
	
	 invalid_digit:
		QCIError("Char not ASCII digit.");
	}
	
	/** \TODO strta for floating points */
}

#endif
