/****************************************************************************
 *
 *   Copyright (c) 2022 ModalAI, Inc. All rights reserved.
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

#include <string.h>
#include "uORBAppsProtobufChannel.hpp"
#include "uORB/uORBManager.hpp"
#include "fc_sensor.h"

extern "C" {
	__EXPORT int muorb_main(int argc, char *argv[]);
	__EXPORT int muorb_init();
}

static bool enable_debug = false;

static void receive_cb(const char *topic,
			      const uint8_t *data,
			      uint32_t length_in_bytes)
{

}
static void advertise_cb(const char *topic)
{

}
static void add_subscription_cb(const char *topic)
{

}
static void remove_subscription_cb(const char *topic)
{

}

int
muorb_main(int argc, char *argv[])
{
	return muorb_init();
}

int
muorb_init()
{
	fc_callbacks funcs = {&receive_cb,&advertise_cb,&add_subscription_cb,&remove_subscription_cb};
	fc_sensor_initialize(enable_debug,&funcs);
	// uORB::AppsProtobufChannel *channel = uORB::AppsProtobufChannel::GetInstance();

	// PX4_INFO("Got muorb init command");

	// if (channel && channel->Initialize(enable_debug)) {
	// 	uORB::Manager::get_instance()->set_uorb_communicator(channel);

	// 	return OK;
	// }

	return OK;
}
