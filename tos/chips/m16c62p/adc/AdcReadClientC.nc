/* $Id$
 * Copyright (c) 2005 Intel Corporation
 * All rights reserved.
 *
 * This file is distributed under the terms in the attached INTEL-LICENSE     
 * file. If you do not find these files, copies can be found by writing to
 * Intel Research Berkeley, 2150 Shattuck Avenue, Suite 1300, Berkeley, CA, 
 * 94704.  Attention:  Intel License Inquiry.
 */
/**
 * Provide, as per TEP101, arbitrated access via a Read interface to the
 * M16c62p ADC.  Users of this component must link it to an
 * implementation of M16c62pAdcConfig which provides the ADC parameters
 * (channel, etc).
 * 
 * @author Fan Zhang <fanzha@ltu.se>
 */

#include "Adc.h"

generic configuration AdcReadClientC() {
  provides interface Read<uint16_t>;
  uses {
    interface M16c62pAdcConfig;
    interface ResourceConfigure;
  }
}
implementation {
  components WireAdcP, M16c62pAdcC;

  enum {
    ID = unique(UQ_ADC_READ),
    HAL_ID = unique(UQ_M16c62pADC_RESOURCE)
  };

  Read = WireAdcP.Read[ID];
  M16c62pAdcConfig = WireAdcP.M16c62pAdcConfig[ID];
  WireAdcP.Resource[ID] -> M16c62pAdcC.Resource[HAL_ID];
  ResourceConfigure = M16c62pAdcC.ResourceConfigure[HAL_ID];
}
