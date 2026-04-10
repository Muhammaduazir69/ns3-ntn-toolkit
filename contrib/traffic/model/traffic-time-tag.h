/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2014 Magister Solutions Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Jani Puttonen <jani.puttonen@magister.fi>
 */

#ifndef TRAFFIC_TIME_TAG_H
#define TRAFFIC_TIME_TAG_H

#include "ns3/nstime.h"
#include "ns3/tag.h"

#include <ostream>

namespace ns3
{

/**
 * @ingroup traffic
 * @brief Time tag used at the traffic model to time stamp a generated
 * packet. Time tag may be used to calculated delay and jitter statistics
 * at the receiver side.
 */
class TrafficTimeTag : public Tag
{
  public:
    static TypeId GetTypeId(void);
    virtual TypeId GetInstanceTypeId(void) const;

    TrafficTimeTag();
    TrafficTimeTag(Time senderTimestamp);

    virtual void Serialize(TagBuffer i) const;
    virtual void Deserialize(TagBuffer i);
    virtual uint32_t GetSerializedSize() const;
    virtual void Print(std::ostream& os) const;

    Time GetSenderTimestamp(void) const;
    void SetSenderTimestamp(Time senderTimestamp);

  private:
    Time m_senderTimestamp;
};

} // namespace ns3

#endif /* TRAFFIC_TIME_TAG_H */
