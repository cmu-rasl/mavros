/**
 * @brief some useful utils
 * @file thread_utils.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup mavutils
 * @{
 *  @brief Some useful utils
 */
/*
 * libmavconn
 * Copyright 2014,2015 Vladimir Ermakov, All rights reserved.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#pragma once

#include <thread>
#include <cstdio>
#include <sstream>
#include <cstdarg>
#include <pthread.h>

namespace mavutils {

/**
 * @brief Create std::thread and set name with string
 * @param[in] thread name
 * @param[in] threading function
 * @return resulting std::thread
*/

// Source: http://stackoverflow.com/a/31897686
template <class F>
std::thread launch_named_thread(const std::string& name, F&& f)
{
  return std::thread([name, f]() {
#if (defined(__APPLE__) && defined(__MACH__))
      pthread_setname_np(name.c_str());
#elif defined(__linux__)
      pthread_setname_np(std::thread::native_handle(), name.c_str());
#else
      #error Unhandled pthread_setname_np OS
#endif
      f();
    });
}

/**
 * @brief Convert to string objects with operator <<
 */
template <typename T>
inline const std::string to_string_ss(T &obj)
{
	std::ostringstream ss;
	ss << obj;
	return ss.str();
}

}; // namespace mavutils
