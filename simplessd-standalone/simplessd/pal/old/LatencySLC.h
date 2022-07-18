/**
 * This file is part of SimpleSSD.
 *
 * SimpleSSD is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * SimpleSSD is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with SimpleSSD.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __LatencySLC_h__
#define __LatencySLC_h__

#include "Latency.h"

class LatencySLC : public Latency {
 public:
  LatencySLC(SimpleSSD::PAL::Config::NANDTiming,
             SimpleSSD::PAL::Config::NANDPower);
  ~LatencySLC();

  uint64_t GetLatency(uint32_t, uint8_t, uint8_t) override;
  inline uint8_t GetPageType(uint32_t) override;
};

#endif  //__LatencyTLC_h__
