/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2013 Magister Solutions
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
 * Author: Budiarto Herman <budiarto.herman@magister.fi>
 *
 */

#ifndef NRTV_HEADER_H
#define NRTV_HEADER_H

#include "ns3/header.h"
#include "ns3/nstime.h"

#include <ostream>

namespace ns3
{

/**
 * @ingroup nrtv
 * @brief Simple packet header for use in NRTV traffic models.
 *
 * The header is 24 bytes in length. There are 6 fields in the header:
 * - frame number (4 bytes, an index starting from 0);
 * - number of frames in the current video session (4 bytes);
 * - slice number (2 bytes, an index starting from 0);
 * - number of slices in the current frame (2 bytes);
 * - slice size in bytes, not including the header (4 bytes); and
 * - arrival time (8 bytes) in nanoseconds.
 *
 * The following is the usage example in the case of sending a packet. First,
 * create a plain header:
 *
 *     NrtvHeader nrtvHeader;
 *
 * Then, specify the underlying values, for example:
 *
 *     nrtvHeader.SetFrameNumber (1);
 *     nrtvHeader.SetNumOfFrames (3000);
 *     nrtvHeader.SetSliceNumber (1);
 *     nrtvHeader.SetNumOfSlices (8);
 *     nrtvHeader.SetSliceSize (250);
 *
 * The arrival time field is automatically filled with the current time.
 * Finally, we can append the header to a packet, e.g.:
 *
 *     Ptr<Packet> packet = Create<Packet> (250);
 *     packet->AddHeader (nrtvHeader);
 *
 * The header is 24 bytes long, so the resulting packet in the above example
 * will become 274 bytes long.
 *
 * Another use case is upon receiving a packet and reading the header content.
 * First of all, make sure the received packet is at least 24 bytes long (we
 * may use GetStaticSerializedSize() to avoid hard-coding a bare figure).
 * Then strip the header from the packet to read its content, for example:
 *
 *     if (packet->GetSize () < NrtvHeader::GetStaticSerializedSize ())
 *       {
 *         // there is definitely no NRTV header in this packet
 *       }
 *     else
 *       {
 *         NrtvHeader nrtvHeader;
 *         packet->RemoveHeader (nrtvHeader);
 *         uint32_t frameNumber = nrtvHeader.GetFrameNumber ();
 *         uint32_t numOfFrames = nrtvHeader.GetNumOfFrames ();
 *         uint32_t sliceNumber = nrtvHeader.GetSliceNumber ();
 *         uint32_t numOfSlices = nrtvHeader.GetNumOfSlices ();
 *         uint32_t sliceSize = nrtvHeader.GetSliceSize ();
 *         Time arrivalTime = nrtvHeader.GetArrivalTime ();
 *       }
 *
 * Instead of Packet::RemoveHeader(), we may use Packet::PeekHeader() if we
 * want to keep the header in the packet.
 *
 * @warning You will get an error if you invoke Packet::RemoveHeader() or
 *          Packet::PeekHeader() on a packet smaller than 24 bytes,
 *
 */
class NrtvHeader : public Header
{
  public:
    /// Create a plain new instance of NRTV header.
    NrtvHeader();

    // Inherited from ObjectBase base class
    static TypeId GetTypeId(void);

    /**
     * @param frameNumber the value for the "frame number" field of this header
     *                    instance
     */
    void SetFrameNumber(uint32_t frameNumber);

    /**
     * @return the current value of the "frame number" field of this header
     *         instance
     */
    uint32_t GetFrameNumber() const;

    /**
     * @param numOfFrames the value for the "number of frames" field of this
     *                    header instance
     */
    void SetNumOfFrames(uint32_t numOfFrames);

    /**
     * @return the current value of the "number of frames" field of this header
     *         instance
     */
    uint32_t GetNumOfFrames() const;

    /**
     * @param sliceNumber the value for the "slice number" field of this header
     *                    instance
     */
    void SetSliceNumber(uint16_t sliceNumber);

    /**
     * @return the current value of the "slice number" field of this header
     *         instance
     */
    uint16_t GetSliceNumber() const;

    /**
     * @param numOfSlices the value for the "number of slices" field of this
     *                    header instance
     */
    void SetNumOfSlices(uint16_t numOfSlices);

    /**
     * @return the current value of the "number of slices" field of this header
     *         instance
     */
    uint16_t GetNumOfSlices() const;

    /**
     * @param sliceSize the value for the "slice size" field of this header
     *                    instance
     */
    void SetSliceSize(uint32_t sliceSize);

    /**
     * @return the current value of the "slice size" field of this header
     *         instance
     */
    uint32_t GetSliceSize() const;

    /**
     * @return the time the packet arrives on the sender side (can be used to
     *         calculate packet delay)
     */
    Time GetArrivalTime() const;

    // Inherited from Header base class
    virtual uint32_t GetSerializedSize() const;
    virtual void Serialize(Buffer::Iterator start) const;
    virtual uint32_t Deserialize(Buffer::Iterator start);
    virtual void Print(std::ostream& os) const;

    // Inherited from ObjectBase base class
    virtual TypeId GetInstanceTypeId() const;

  private:
    uint32_t m_frameNumber; ///< Frame number field (an index starting from 0).
    uint32_t m_numOfFrames; ///< Number of frames field.
    uint16_t m_sliceNumber; ///< Slice number field (an index starting from 0).
    uint16_t m_numOfSlices; ///< Number of slices field.
    uint16_t m_sliceSize;   ///< Slice size field.
    Time m_arrivalTime;     ///< Arrival time field in Time format.

}; // end of `class NrtvHeader`

} // namespace ns3

#endif /* NRTV_HEADER_H */
