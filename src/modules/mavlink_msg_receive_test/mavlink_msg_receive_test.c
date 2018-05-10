/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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

/**
 * @file mavlink_msg_receive_test.c
 * mavlink_msg_receive application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */
#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include <stdlib.h>
//#include <nuttx/sched.h>

#include <uORB/uORB.h>

#include <uORB/topics/task_status_change_p2m.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

static bool thread_should_exit = false;		/**< mavlink_msg_receive exit flag */
static bool thread_running = false;			/**< mavlink_msg_receive status flag */
static int mavlink_msg_receive_task;			/**< Handle of mavlink_msg_receive task / thread */

/**
 * mavlink_msg_receive management function.
 */
__EXPORT int mavlink_msg_receive_test_main(int argc, char *argv[]);

/**
 * Mainloop of mavlink_msg_receive.
 */
int mavlink_msg_receive_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	warnx("usage: mavlink_msg_receive {start|stop|status} [-p <additional params>]\n\n");
}

/**
 * The mavlink_msg_receive app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int mavlink_msg_receive_test_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("mavlink_msg_receive already running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		mavlink_msg_receive_task = px4_task_spawn_cmd("mavlink_msg_receive",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT,
						 2000,
						 mavlink_msg_receive_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");

		} else {
			warnx("\tnot started\n");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}

int mavlink_msg_receive_thread_main(int argc, char *argv[])
{

	warnx("[mavlink_msg_receive] starting\n");

	thread_running = true;

	/* subscribe to sensor_combined topic */
////	int fixed_target_position_sub_fd = orb_subscribe(ORB_ID(fixed_target_position));
//	int obstacle_position_sub_fd = orb_subscribe(ORB_ID(obstacle_position));
//	int task_status_change_sub_fd = orb_subscribe(ORB_ID(task_status_change));
//	int task_status_monitor_sub_fd = orb_subscribe(ORB_ID(task_status_monitor));
//	int vision_one_num_get_sub_fd = orb_subscribe(ORB_ID(vision_one_num_get));
//	int vision_num_scan_sub_fd = orb_subscribe(ORB_ID(vision_num_scan));
//	int yaw_sp_calculated_sub_fd = orb_subscribe(ORB_ID(yaw_sp_calculated));
//	//orb_set_interval(motor_sub_fd, 1000);
	int task_status_change_p2m_sub_fd = orb_subscribe(ORB_ID(task_status_change_p2m));

	px4_pollfd_struct_t fds[1];
	fds[0].fd     = task_status_change_p2m_sub_fd;
	fds[0].events = POLLIN;

//	px4_pollfd_struct_t fds[] = {
//				{.fd = fixed_target_return_m2p_sub_fd, .events = POLLIN},
//				{.fd = obstacle_position_m2p_sub_fd, .events = POLLIN},
//				{.fd = task_status_monitor_m2p_sub_fd, .events = POLLIN},
//				{.fd = vision_num_scan_m2p_sub_fd, .events = POLLIN},
//				{.fd = vision_one_num_get_m2p_sub_fd, .events = POLLIN},
//				{.fd = yaw_sp_calculated_m2p_sub_fd, .events = POLLIN},
//
//		};

	struct task_status_change_p2m_s task_status_change_p2m_data;

	while (!thread_should_exit) {
		int poll_ret = px4_poll(fds, 6, 500);
		if(poll_ret < 0)
		{
			continue;
		}
		if(poll_ret == 0)
		{
			continue;
		}
		if (fds[0].revents & POLLIN) {
			orb_copy(ORB_ID(task_status_change_p2m), task_status_change_p2m_sub_fd, &task_status_change_p2m_data);
			PX4_WARN("task_status_change_p2m: %5.3f %5.3f %5.3f",
				(double)task_status_change_p2m_data.spray_duration,
				(double)task_status_change_p2m_data.task_status,
				(double)task_status_change_p2m_data.loop_value);
		}

	}

	warnx("[mavlink_msg_receive] exiting.\n");

	thread_running = false;

	return 0;
}
