@###############################################
@#
@# EmPy template for generating <msg>_uRTPS_UART.cpp file
@#
@###############################################
@# Start of Template
@#
@# Context:
@#  - fastrtps_version (str) FastRTPS version installed on the system
@#  - ros2_distro (str) ROS2 distro name
@#  - spec (msggen.MsgSpec) Parsed specification of the .msg file
@###############################################
@{
import genmsg.msgs
from packaging import version
import re

topic = alias if alias else spec.short_name
try:
    ros2_distro = ros2_distro.decode("utf-8")
except AttributeError:
    pass

topic_name = topic

# For ROS, use the topic pattern convention defined in
# http://wiki.ros.org/ROS/Patterns/Conventions
if ros2_distro:
    topic_name_split = re.sub( r"([A-Z])", r" \1", topic).split()
    topic_name = topic_name_split[0]
    for w in topic_name_split[1:]:
        topic_name += "_" + w
    topic_name = topic_name.lower()
}@
/****************************************************************************
 *
 * Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
 * Copyright (c) 2018-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*!
 * @@file @(topic)_Subscriber.cpp
 * This file contains the implementation of the subscriber functions.
 *
 * This file was adapted from the fastrtpsgen tool.
 */

#include "@(topic)_Subscriber.h"

#include <fastrtps/Domain.h>
#include <fastrtps/participant/Participant.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/subscriber/Subscriber.h>
#include <fastrtps/attributes/SubscriberAttributes.h>
#include <fastrtps/transport/UDPv4TransportDescriptor.h>
@[if version.parse(fastrtps_version) >= version.parse('2.0')]@
#include <fastdds/rtps/transport/shared_mem/SharedMemTransportDescriptor.h>

using SharedMemTransportDescriptor = eprosima::fastdds::rtps::SharedMemTransportDescriptor;
@[end if]@


@(topic)_Subscriber::@(topic)_Subscriber()
	: mp_participant(nullptr),
	  mp_subscriber(nullptr)
{ }

@(topic)_Subscriber::~@(topic)_Subscriber()
{
	Domain::removeParticipant(mp_participant);
}

bool @(topic)_Subscriber::init(uint8_t topic_ID, std::condition_variable *t_send_queue_cv,
			       std::mutex *t_send_queue_mutex, std::queue<uint8_t> *t_send_queue, const std::string &ns,
			       std::string topic_name)
{
	m_listener.topic_ID = topic_ID;
	m_listener.t_send_queue_cv = t_send_queue_cv;
	m_listener.t_send_queue_mutex = t_send_queue_mutex;
	m_listener.t_send_queue = t_send_queue;

	// Create RTPSParticipant
	ParticipantAttributes PParam;
@[if version.parse(fastrtps_version) < version.parse('2.0')]@
	PParam.rtps.builtin.domainId = 0;
@[else]@
	PParam.domainId = 0;
@[end if]@
@[if version.parse(fastrtps_version) <= version.parse('1.8.4')]@
	PParam.rtps.builtin.leaseDuration = c_TimeInfinite;
@[else]@
	PParam.rtps.builtin.discovery_config.leaseDuration = c_TimeInfinite;
@[end if]@
@[if ros2_distro]@
	// ROS2 default memory management policy
	PParam.rtps.builtin.writerHistoryMemoryPolicy = PREALLOCATED_WITH_REALLOC_MEMORY_MODE;
@[end if]@
	std::string nodeName = ns;
	nodeName.append("@(topic)_subscriber");
	PParam.rtps.setName(nodeName.c_str());

@[if ros2_distro]@
	// Check if ROS_LOCALHOST_ONLY is set. This means that one wants to use only
	// the localhost network for data sharing. If FastRTPS/DDS >= 2.0 and
	// RMW_IMPLEMENTATION is FastDDS then the Shared Memory transport is used
	const char* localhost_only = std::getenv("ROS_LOCALHOST_ONLY");
	const char* rmw_implementation = std::getenv("RMW_IMPLEMENTATION");
	const char* ros_distro = std::getenv("ROS_DISTRO");
	if (localhost_only && strcmp(localhost_only, "1") == 0
	    && ((rmw_implementation && ((strcmp(rmw_implementation, "rmw_fastrtps_cpp") == 0)
	    || (strcmp(rmw_implementation, "rmw_fastrtps_dynamic_cpp") == 0)))
	    || (!rmw_implementation && ros_distro && strcmp(ros_distro, "foxy") == 0))) {
		// Create a custom network UDPv4 transport descriptor
		// to whitelist the localhost
		auto localhostUdpTransport = std::make_shared<UDPv4TransportDescriptor>();
		localhostUdpTransport->interfaceWhiteList.emplace_back("127.0.0.1");

		// Disable the built-in Transport Layer
		PParam.rtps.useBuiltinTransports = false;

		// Add the descriptor as a custom user transport
		PParam.rtps.userTransports.push_back(localhostUdpTransport);

@[    if version.parse(fastrtps_version) >= version.parse('2.0')]@
		// Add shared memory transport when available
		auto shmTransport = std::make_shared<SharedMemTransportDescriptor>();
		PParam.rtps.userTransports.push_back(shmTransport);
@[    end if]@
	}
@[end if]@

	mp_participant = Domain::createParticipant(PParam);

	if (mp_participant == nullptr) {
		return false;
	}

	// Register the type
	Domain::registerType(mp_participant, static_cast<TopicDataType *>(&@(topic)DataType));

	// Create Subscriber
	SubscriberAttributes Rparam;
	Rparam.topic.topicKind = NO_KEY;
	Rparam.topic.topicDataType = @(topic)DataType.getName();
@[if ros2_distro]@
@[    if ros2_distro == "ardent"]@
	Rparam.qos.m_partition.push_back("rt");
	std::string topicName = ns;
@[    else]@
	std::string topicName = "rt/";
	topicName.append(ns);
@[    end if]@
@[else]@
	std::string topicName = ns;
@[end if]@
	topic_name.empty() ? topicName.append("fmu/@(topic_name)/in") : topicName.append(topic_name);
	Rparam.topic.topicName = topicName;
	mp_subscriber = Domain::createSubscriber(mp_participant, Rparam, static_cast<SubscriberListener *>(&m_listener));

	if (mp_subscriber == nullptr) {
		return false;
	}

	return true;
}

void @(topic)_Subscriber::SubListener::onSubscriptionMatched(Subscriber *sub, MatchingInfo &info)
{
@# Since the time sync runs on the bridge itself, it is required that there is a
@# match between two topics of the same entity
@[if topic != 'Timesync' and topic != 'timesync' and topic != 'TimesyncStatus' and topic != 'timesync_status']@
	// The first 6 values of the ID guidPrefix of an entity in a DDS-RTPS Domain
	// are the same for all its subcomponents (publishers, subscribers)
	bool is_different_endpoint = false;

	for (size_t i = 0; i < 6; i++) {
		if (sub->getGuid().guidPrefix.value[i] != info.remoteEndpointGuid.guidPrefix.value[i]) {
			is_different_endpoint = true;
			break;
		}
	}

	// If the matching happens for the same entity, do not make a match
	if (is_different_endpoint) {
		if (info.status == MATCHED_MATCHING) {
			n_matched++;
			std::cout << "\033[0;37m[   micrortps_agent   ]\t@(topic) subscriber matched\033[0m" << std::endl;

		} else {
			n_matched--;
			std::cout << "\033[0;37m[   micrortps_agent   ]\t@(topic) subscriber unmatched\033[0m" << std::endl;
		}
	}

@[else]@
	(void)sub;

	if (info.status == MATCHED_MATCHING) {
		n_matched++;

	} else {
		n_matched--;
	}
@[end if]@
}

void @(topic)_Subscriber::SubListener::onNewDataMessage(Subscriber *sub)
{
	if (n_matched > 0) {
		std::unique_lock<std::mutex> has_msg_lock(has_msg_mutex);

		if (has_msg.load() == true) { // Check if msg has been fetched
			has_msg_cv.wait(has_msg_lock); // Wait till msg has been fetched
		}

		has_msg_lock.unlock();

		// Take data
		if (sub->takeNextData(&msg, &m_info)) {
			if (m_info.sampleKind == ALIVE) {
				std::unique_lock<std::mutex> lk(*t_send_queue_mutex);

				++n_msg;
				has_msg = true;

				t_send_queue->push(topic_ID);
				lk.unlock();
				t_send_queue_cv->notify_one();

			}
		}
	}
}

bool @(topic)_Subscriber::hasMsg()
{
	if (m_listener.n_matched > 0) {
		return m_listener.has_msg.load();
	}

	return false;
}

@(topic)_msg_t @(topic)_Subscriber::getMsg()
{
	return m_listener.msg;
}

void @(topic)_Subscriber::unlockMsg()
{
	if (m_listener.n_matched > 0) {
		std::unique_lock<std::mutex> has_msg_lock(m_listener.has_msg_mutex);
		m_listener.has_msg = false;
		has_msg_lock.unlock();
		m_listener.has_msg_cv.notify_one();
	}
}
