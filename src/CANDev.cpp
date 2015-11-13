/*
 Copyright (c) 2014, Robot Control and Pattern Recognition Group, Warsaw University of Technology
 All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
     * Neither the name of the Warsaw University of Technology nor the
       names of its contributors may be used to endorse or promote products
       derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>

#include <iostream>
#include <cstring>
#include <string>

#include "can_driver/CANDev.h"

#if !defined(HAVE_RTNET)
#define rt_dev_socket socket
#define rt_dev_ioctl ioctl
#define rt_dev_bind bind
#define rt_dev_close close
#define rt_dev_setsockopt setsockopt
#define rt_dev_send(a, b, c, d) write((a), (b), (c))
#define rt_dev_recv(a, b, c, d) read((a), (b), (c))
#endif

CANDev::FilterElement::FilterElement(uint32_t can_id, uint32_t can_mask) {
    can_id_ = can_id;
    can_mask_ = can_mask;
}

CANDev::CANDev(std::string dev_name, const std::vector<CANDev::FilterElement > &filter_vec) :
    frame_buf(1000),
    buf_size(0)
{
    struct sockaddr_can addr;
    struct ifreq ifr;
  
    if((dev = rt_dev_socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        std::cout<< "Error while opening socket" << std::endl;
        dev = -1;
        return;
    }

    strcpy(ifr.ifr_name, dev_name.c_str());
    rt_dev_ioctl(dev, SIOCGIFINDEX, &ifr);

    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex; 

    int nfilters = filter_vec.size();
    struct can_filter *rfilter = new can_filter[nfilters];
    for (int i = 0; i < nfilters; i++) {
        rfilter[i].can_id   = filter_vec[i].can_id_;
        rfilter[i].can_mask = filter_vec[i].can_mask_;
    }

    if (rt_dev_setsockopt(dev, SOL_CAN_RAW, CAN_RAW_FILTER, rfilter,
                                        sizeof(struct can_filter) * nfilters) < 0)
    {
        std::cout << "Error: filter" << std::endl;
        rt_dev_close(dev);
        delete[] rfilter;
        dev = -1;
        return;
    }

    delete[] rfilter;

    if(rt_dev_bind(dev, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        std::cout << "Error in socket bind" << std::endl;
        rt_dev_close(dev);
        dev = -1;
    }
}

CANDev::~CANDev() {
  if(dev > -1) {
    rt_dev_close(dev);
  }
}

void CANDev::send(const uint32_t can_id, const uint8_t len, const uint8_t *data) {
  struct can_frame frame;
  
  frame.can_id = can_id;
  frame.can_dlc = len;
  
  memcpy(frame.data, data, len);
  rt_dev_send(dev, reinterpret_cast<void*>(&frame), sizeof(frame), 0);
}

uint32_t CANDev::waitForReply(uint32_t can_id, uint8_t *data) {
    struct can_frame frame;

    // search frame buffer
    for(size_t i = 0; i < buf_size; i++) {
        if(frame_buf[i].can_id == can_id) {
            memcpy(data, frame_buf[i].data, frame_buf[i].can_dlc);
            memcpy(data, frame_buf[i].data, frame_buf[i].can_dlc);
            uint8_t can_dlc = frame_buf[i].can_dlc;

            // erase
            for (size_t j = i+1; j < buf_size; j++) {
                frame_buf[j-1] = frame_buf[j];
            }
            buf_size--;
            return can_dlc;
        }
    }

    // wait for new data
    while(1) {
        size_t ret = rt_dev_recv(dev, reinterpret_cast<void*>(&frame), sizeof(frame), MSG_DONTWAIT);
        if(ret != sizeof(frame)) {
            return 0;
        }

        if(frame.can_id == can_id) {
            memcpy(data, frame.data, frame.can_dlc);
            return frame.can_dlc;
        }
        if (buf_size >= frame_buf.size()) {
            return 0;
        }
        frame_buf[buf_size] = frame;
        buf_size++;
    }
}

bool CANDev::isOpened() {
	return dev != -1;
}

void CANDev::setMultiFrameReceiver(uint32_t nbytes) {
    multi_buf_size = 0;
    multi_frame_buf.resize( (nbytes+7)/8 );
    multi_nbytes = nbytes;
}

uint32_t CANDev::readMultiFrameData(uint32_t can_id, uint8_t *data, CANDev::Header &header) {
    struct can_frame frame;

    if (multi_buf_size <= 0) {
        while(1) {
            size_t ret = rt_dev_recv(dev, reinterpret_cast<void*>(&frame), sizeof(frame), 0);
            if(ret != sizeof(frame)) {
                return 0;
            }
//            std::cout << int(frame.data[0]) << " " << int(frame.data[1]) << " " << int(frame.data[2]) << " " << int(frame.data[3]) << std::endl;
            if(frame.can_id == can_id && header[0] == frame.data[0] && header[1] == frame.data[1] && header[2] == frame.data[2] && header[3] == frame.data[3]) {
                multi_frame_buf[0] = frame;
                multi_buf_size = 1;
                break;
            }
        }
    }

    while(1) {
        size_t ret = rt_dev_recv(dev, reinterpret_cast<void*>(&frame), sizeof(frame), 0);
        if(ret != sizeof(frame)) {
            return 0;
        }
        if(frame.can_id == can_id) {
            multi_frame_buf[multi_buf_size] = frame;
            multi_buf_size++;
            if (multi_buf_size == multi_frame_buf.size()) {
                for (int i = 0, frame_idx=0; i < multi_nbytes; i += 8, frame_idx++) {
                    memcpy(&data[i], multi_frame_buf[frame_idx].data, std::min(8u, multi_nbytes - i));
                }
                multi_buf_size = 0;
                return multi_nbytes;
            }
        }
    }
}

