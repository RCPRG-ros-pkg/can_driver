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

#ifndef CANDEV_H_
#define CANDEV_H_

#if !defined(HAVE_RTNET)
#include <linux/can.h>
#include <linux/can/raw.h>
#else
#include <rtdm/rtcan.h>  // Defines for the RT CAN socket
#endif

#include <inttypes.h>

#include <string>
#include <vector>



class CANDev {
 public:
  typedef uint8_t Header[4];

  class FilterElement {
   public:
    FilterElement(uint32_t can_id, uint32_t can_mask);
    uint32_t can_id_;
    uint32_t can_mask_;
  };

  CANDev(const std::string &dev_name, const std::string &name,
         const std::vector<FilterElement> &filter_vec);
  ~CANDev();

  void setMultiFrameReceiver(uint32_t nbytes);
  uint32_t readMultiFrameData(uint32_t can_id, uint8_t *data,
                              const CANDev::Header &header);

  bool send(uint32_t can_id, uint8_t len, const uint8_t *data);
  uint32_t waitForReply(uint32_t can_id, uint8_t *data);
  bool isOpened();

 protected:
  int dev;

  std::vector<can_frame> frame_buf;
  int buf_size;

  std::vector<can_frame> multi_frame_buf;
  int multi_buf_size;
  uint32_t multi_nbytes;
  std::string dev_name_;
  std::string name_;
};

#endif  // CANDEV_H_

