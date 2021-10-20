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
 * @@file @(topic)_Publisher.cpp
 * This file contains the implementation of the publisher functions.
 *
 * This file was adapted from the fastrtpsgen tool.
 */

#include "@(topic)_Publisher.h"

#include <fastrtps/Domain.h>
#include <fastrtps/participant/Participant.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/publisher/Publisher.h>
#include <fastrtps/attributes/PublisherAttributes.h>
#include <fastrtps/transport/UDPv4TransportDescriptor.h>
@[if version.parse(fastrtps_version) >= version.parse('2.0')]@
#include <fastdds/rtps/transport/shared_mem/SharedMemTransportDescriptor.h>

using SharedMemTransportDescriptor = eprosima::fastdds::rtps::SharedMemTransportDescriptor;
@[end if]@


@(topic)_Publisher::@(topic)_Publisher()
	: mp_participant(nullptr),
	  mp_publisher(nullptr)
{ }

@(topic)_Publisher::~@(topic)_Publisher()
{
	Domain::removeParticipant(mp_participant);
}

bool @(topic)_Publisher::init(const std::string &ns, std::string topic_name)
{
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
	nodeName.append("@(topic)_publisher");
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

	// Create Publisher
	PublisherAttributes Wparam;
	Wparam.topic.topicKind = NO_KEY;
	Wparam.topic.topicDataType = @(topic)DataType.getName();
@[if ros2_distro]@
@[    if ros2_distro == "ardent"]@
	Wparam.qos.m_partition.push_back("rt");
	std::string topicName = ns;
@[    else]@
	std::string topicName = "rt/";
	topicName.append(ns);
@[    end if]@
	// ROS2 default publish mode QoS policy
	Wparam.qos.m_publishMode.kind = ASYNCHRONOUS_PUBLISH_MODE;
@[else]@
	std::string topicName = ns;
@[end if]@
	topic_name.empty() ? topicName.append("fmu/@(topic_name)/out") : topicName.append(topic_name);
	Wparam.topic.topicName = topicName;
	mp_publisher = Domain::createPublisher(mp_participant, Wparam, static_cast<PublisherListener *>(&m_listener));

	if (mp_publisher == nullptr) {
		return false;
	}

	return true;
}

void @(topic)_Publisher::PubListener::onPublicationMatched(Publisher *pub, MatchingInfo &info)
{
	// The first 6 values of the ID guidPrefix of an entity in a DDS-RTPS Domain
	// are the same for all its subcomponents (publishers, subscribers)
	bool is_different_endpoint = false;

	for (size_t i = 0; i < 6; i++) {
		if (pub->getGuid().guidPrefix.value[i] != info.remoteEndpointGuid.guidPrefix.value[i]) {
			is_different_endpoint = true;
			break;
		}
	}

	// If the matching happens for the same entity, do not make a match
	if (is_different_endpoint) {
		if (info.status == MATCHED_MATCHING) {
			n_matched++;
			std::cout << "\033[0;37m[   micrortps_agent   ]\t@(topic) publisher matched\033[0m" << std::endl;

		} else {
			n_matched--;
			std::cout << "\033[0;37m[   micrortps_agent   ]\t@(topic) publisher unmatched\033[0m" << std::endl;
		}
	}
}

void @(topic)_Publisher::publish(@(topic)_msg_t *st)
{
	mp_publisher->write(st);
}
