/*
 * Copyright (c) 2008 The Regents of the University  of California.
 * All rights reserved."
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the copyright holders nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */


/*
 * Header file for the 6lowpan/IPv6 stack.
 *
 * @author Stephen Dawson-Haggerty
 * 
 */

#ifndef __6LOWPAN_H__
#define __6LOWPAN_H__

#include <stdint.h>

#include "blip-platform.h"
/*
 * Typedefs and static library data.
 */
typedef uint8_t ip6_addr_t [16];
typedef uint16_t cmpr_ip6_addr_t;
#ifdef PC
#include "../config.h"
typedef uint16_t ieee154_saddr_t;
typedef uint16_t hw_pan_t;
enum {
  HW_BROADCAST_ADDR = 0xffff,
};
#else
#include <Ieee154.h>
#endif

/*
 * shared variables which contain addressing information for 6lowpan
 * devices
 */
extern uint8_t globalPrefix;
extern uint8_t multicast_prefix[8];
extern uint8_t linklocal_prefix[8];

uint8_t cmpPfx(ip6_addr_t a, uint8_t *pfx);

void ip_memclr(uint8_t *buf, uint16_t len);
void *ip_memcpy(void *dst0, const void *src0, uint16_t len);  

/*
 * A packed 6lowpan packet. 
 *
 * The data buffer points at the start of 6lowpan packed data.  We
 * included a few other fields from L2 with this information so that
 * we are able to infer things like source and destination IP from it.
 *
 */
typedef struct packed_lowmsg {
  uint8_t headers;
  uint8_t len;
  // we preprocess the headers bitmap for easy processing.
  ieee154_saddr_t src;
  ieee154_saddr_t dst;
  uint8_t *data;
} packed_lowmsg_t;

/*
 * bit fields we use to keep track of which optional header fields are
 * present in a message
 */
enum {
  LOWMSG_MESH_HDR  = (1 << 0),
  LOWMSG_BCAST_HDR = (1 << 1),
  LOWMSG_FRAG1_HDR = (1 << 2),
  LOWMSG_FRAGN_HDR = (1 << 3),
  LOWMSG_NALP      = (1 << 4),
  LOWMSG_IPNH_HDR  = (1 << 5),
};

/*
 * lengths of different lowpan headers
 */
enum {
  LOWMSG_MESH_LEN = 5,
  LOWMSG_BCAST_LEN = 2,
  LOWMSG_FRAG1_LEN = 4,
  LOWMSG_FRAGN_LEN = 5,
};

enum {
  LOWPAN_LINK_MTU = 110,
  INET_MTU = 1280,
  LIB6LOWPAN_MAX_LEN = LOWPAN_LINK_MTU,
};

/*
 * magic numbers from rfc4944; some of them shifted: mostly dispatch values.
 */
enum {
  LOWPAN_NALP_PATTERN = 0x0,
  LOWPAN_MESH_PATTERN = 0x2,
  LOWPAN_FRAG1_PATTERN = 0x18,
  LOWPAN_FRAGN_PATTERN = 0x1c,
  LOWPAN_BCAST_PATTERN = 0x50,
  LOWPAN_HC1_PATTERN = 0x42,
  LOWPAN_HC_LOCAL_PATTERN = 0x3,
  LOWPAN_HC_CRP_PATTERN = 0x4,
};

enum {
  LOWPAN_MESH_V_MASK = 0x20,
  LOWPAN_MESH_F_MASK = 0x10,
  LOWPAN_MESH_HOPS_MASK = 0x0f,
};

/*
 * constants to unpack HC-packed headers
 */
enum {
  LOWPAN_IPHC_VTF_MASK      = 0x80,
  LOWPAN_IPHC_VTF_INLINE    = 0,
  LOWPAN_IPHC_NH_MASK       = 0x40,
  LOWPAN_IPHC_NH_INLINE     = 0,
  LOWPAN_IPHC_HLIM_MASK     = 0x20,
  LOWPAN_IPHC_HLIM_INLINE   = 0,

  LOWPAN_IPHC_SC_OFFSET      = 3,
  LOWPAN_IPHC_DST_OFFSET     = 1,
  LOWPAN_IPHC_ADDRFLAGS_MASK = 0x3,

  LOWPAN_IPHC_ADDR_128       = 0x0,
  LOWPAN_IPHC_ADDR_64        = 0x1,
  LOWPAN_IPHC_ADDR_16        = 0x2,
  LOWPAN_IPHC_ADDR_0         = 0x3,

  LOWPAN_IPHC_SHORT_MASK     = 0x80,
  LOWPAN_IPHC_SHORT_LONG_MASK= 0xe0,

  LOWPAN_IPHC_HC1_MCAST      = 0x80,
  LOWPAN_IPHC_HC_MCAST       = 0xa0,

  LOWPAN_HC_MCAST_SCOPE_MASK = 0x1e,
  LOWPAN_HC_MCAST_SCOPE_OFFSET = 1,

  LOWPAN_UDP_PORT_BASE_MASK  = 0xfff0,
  LOWPAN_UDP_PORT_BASE       = 0xf0b0,
  LOWPAN_UDP_DISPATCH        = 0x80,

  LOWPAN_UDP_S_MASK          = 0x40,
  LOWPAN_UDP_D_MASK          = 0x20,
  LOWPAN_UDP_C_MASK          = 0x10,
};



// AT: These are really 16 bit values, but after a certain point the numbers
//  beyond 255 aren't important to us, or rather no different than 255
struct topology_entry {
  uint8_t etx;
  uint8_t conf;
  ieee154_saddr_t hwaddr;
};
struct topology_header {
  uint16_t seqno;
  struct topology_entry topo[0];
};
struct topology_header_package {
  uint16_t reporter;
  uint16_t len;
  uint16_t seqno;
  struct topology_entry topo[0];
};

#endif
