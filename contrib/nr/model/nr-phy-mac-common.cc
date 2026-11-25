// Copyright (c) 2015, NYU WIRELESS, Tandon School of Engineering, New York University
// Copyright (c) 2018 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
// Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "nr-phy-mac-common.h"

#include "ns3/attribute-accessor-helper.h"
#include "ns3/log.h"

#include <algorithm>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrPhyMacCommon");

uint8_t
SlotAllocInfo::GetAllocInfoNumSymbols(uint8_t type) const
{
    NS_LOG_FUNCTION(this << +type);
    using DciType = DciInfoElementTdma::VarTtiType;
    NS_ASSERT_MSG(DciType::SRS <= type &&
                      type <= (DciType::SRS | DciType::CTRL | DciType::DATA | DciType::MSG3),
                  "Unknown combination of DciInfoElementTdma::VarTtiTypes");
    std::array<bool, 14> symUsed{false};
    for (const auto& allocation : m_varTtiAllocInfo)
    {
        // If DCI does not match one of the specified types, ignore it
        if (allocation.m_dci->m_type & type)
        {
            auto startSym = allocation.m_dci->m_symStart;
            auto lastSym = startSym + allocation.m_dci->m_numSym;
            std::fill(symUsed.begin() + startSym, symUsed.begin() + lastSym, true);
        }
    }
    return std::count(symUsed.begin(), symUsed.end(), true);
}

void
SlotAllocInfo::Merge(const SlotAllocInfo& other)
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT(other.m_type != NONE && m_type != NONE);
    NS_ASSERT(other.m_sfnSf == m_sfnSf);

    if (other.m_type * m_type == 6)
    {
        m_type = BOTH;
    }

    for (const auto& extAlloc : other.m_varTtiAllocInfo)
    {
        m_varTtiAllocInfo.push_front(extAlloc);
    }

    m_numSymAlloc = GetAllocInfoNumSymbols();
    NS_ASSERT_MSG(m_numSymAlloc <= 14,
                  "Invalid number of symbols: " << m_numSymAlloc << " symbols. ");

    // Sort over the symStart of the DCI (VarTtiAllocInfo::operator <)
    std::stable_sort(m_varTtiAllocInfo.begin(), m_varTtiAllocInfo.end());
}

bool
SlotAllocInfo::ContainsDataAllocation() const
{
    NS_LOG_FUNCTION(this);
    for (const auto& allocation : m_varTtiAllocInfo)
    {
        if (allocation.m_dci->m_type == DciInfoElementTdma::DATA)
        {
            return true;
        }
    }
    return false;
}

bool
SlotAllocInfo::ContainsDlCtrlAllocation() const
{
    NS_LOG_FUNCTION(this);

    for (const auto& allocation : m_varTtiAllocInfo)
    {
        if (allocation.m_dci->m_type == DciInfoElementTdma::CTRL &&
            allocation.m_dci->m_format == DciInfoElementTdma::DL)
        {
            return true;
        }
    }
    return false;
}

bool
SlotAllocInfo::ContainsUlMsg3Allocation() const
{
    NS_LOG_FUNCTION(this);
    for (const auto& allocation : m_varTtiAllocInfo)
    {
        if (allocation.m_dci->m_type == DciInfoElementTdma::MSG3)
        {
            return true;
        }
    }
    return false;
}

bool
SlotAllocInfo::ContainsUlCtrlAllocation() const
{
    NS_LOG_FUNCTION(this);

    for (const auto& allocation : m_varTtiAllocInfo)
    {
        if (allocation.m_dci->m_type == DciInfoElementTdma::SRS)
        {
            return true;
        }
    }
    return false;
}

bool
SlotAllocInfo::operator<(const SlotAllocInfo& rhs) const
{
    return m_sfnSf < rhs.m_sfnSf;
}

std::ostream&
operator<<(std::ostream& os, const DciInfoElementTdma::DciFormat& item)
{
    if (item == DciInfoElementTdma::DL)
    {
        os << "DL";
    }
    else if (item == DciInfoElementTdma::UL)
    {
        os << "UL";
    }
    else
    {
        os << "NA";
    }
    return os;
}

std::ostream&
operator<<(std::ostream& os, const DlHarqInfo& item)
{
    if (item.IsReceivedOk())
    {
        os << "ACK feedback ";
    }
    else
    {
        os << "NACK feedback ";
    }
    os << "for ProcessID: " << static_cast<uint32_t>(item.m_harqProcessId) << " of UE "
       << static_cast<uint32_t>(item.m_rnti)
       << " Num Retx: " << static_cast<uint32_t>(item.m_numRetx)
       << " BWP index: " << static_cast<uint32_t>(item.m_bwpIndex);
    return os;
}

std::ostream&
operator<<(std::ostream& os, const UlHarqInfo& item)
{
    if (item.IsReceivedOk())
    {
        os << "ACK feedback ";
    }
    else
    {
        os << "NACK feedback ";
    }
    os << "for ProcessID: " << static_cast<uint32_t>(item.m_harqProcessId) << " of UE "
       << static_cast<uint32_t>(item.m_rnti)
       << " Num Retx: " << static_cast<uint32_t>(item.m_numRetx);

    return os;
}

std::ostream&
operator<<(std::ostream& os, const SfnSf& item)
{
    os << "FrameNum: " << static_cast<uint32_t>(item.GetFrame())
       << " SubFrameNum: " << static_cast<uint32_t>(item.GetSubframe())
       << " SlotNum: " << static_cast<uint32_t>(item.GetSlot());
    return os;
}

std::ostream&
operator<<(std::ostream& os, const SlotAllocInfo& item)
{
    os << "Allocation for slot " << item.m_sfnSf
       << " total symbols allocated: " << item.m_numSymAlloc << " of type " << item.m_type
       << ", tti: " << item.m_varTtiAllocInfo.size()
       << " composed by the following allocations: " << std::endl;
    for (const auto& alloc : item.m_varTtiAllocInfo)
    {
        std::stringstream direction;
        std::stringstream type;
        if (alloc.m_dci->m_type == DciInfoElementTdma::CTRL)
        {
            type << "CTRL";
        }
        else if (alloc.m_dci->m_type == DciInfoElementTdma::SRS)
        {
            type << "SRS";
        }
        else if (alloc.m_dci->m_type == DciInfoElementTdma::MSG3)
        {
            type << "MSG3";
        }
        else
        {
            type << "DATA";
        }

        if (alloc.m_dci->m_format == DciInfoElementTdma::UL)
        {
            direction << "UL";
        }
        else
        {
            direction << "DL";
        }
        os << "[Allocation from sym " << static_cast<uint32_t>(alloc.m_dci->m_symStart)
           << " to sym " << static_cast<uint32_t>(alloc.m_dci->m_numSym + alloc.m_dci->m_symStart)
           << " direction " << direction.str() << " type " << type.str() << "]" << std::endl;
    }
    return os;
}

std::ostream&
operator<<(std::ostream& os, const SlotAllocInfo::AllocationType& item)
{
    switch (item)
    {
    case SlotAllocInfo::NONE:
        os << "NONE";
        break;
    case SlotAllocInfo::DL:
        os << "DL";
        break;
    case SlotAllocInfo::UL:
        os << "UL";
        break;
    case SlotAllocInfo::BOTH:
        os << "BOTH";
        break;
    }

    return os;
}

std::ostream&
operator<<(std::ostream& os, const DciInfoElementTdma& item)
{
    os << "RNTI=" << item.m_rnti << "|" << item.m_format << "|SYM=" << +item.m_symStart
       << "|NSYM=" << +item.m_numSym << "|MCS=" << +item.m_mcs << "|TBS=" << item.m_tbSize
       << "|NDI=" << +item.m_ndi << "|RV=" << +item.m_rv << "|TYPE=" << item.m_type
       << "|BWP=" << +item.m_bwpIndex << "|HARQP=" << +item.m_harqProcess << "|RBG=";

    uint16_t start = 65000;
    uint16_t end = 0;
    bool canPrint = false;
    for (uint32_t i = 0; i < item.m_rbgBitmask.size(); ++i)
    {
        if (item.m_rbgBitmask[i] == 1)
        {
            canPrint = true;
        }

        if (item.m_rbgBitmask[i] == 1 && end < i)
        {
            end = i;
        }
        if (item.m_rbgBitmask[i] == 1 && start > i)
        {
            start = i;
        }

        if (item.m_rbgBitmask[i] == 0 && canPrint)
        {
            os << "[" << +start << ";" << +end << "]";
            start = 65000;
            end = 0;
            canPrint = false;
        }
    }

    if (canPrint)
    {
        os << "[" << +start << ";" << +end << "]";
    }

    return os;
}

namespace nr
{
uint8_t
CountUsedSymbolsFromVarAllocTtiRange(uint8_t startSym,
                                     std::deque<VarTtiAllocInfo>::iterator begin,
                                     std::deque<VarTtiAllocInfo>::iterator end)
{
    auto usedSymbols = 0;
    if (std::distance(begin, end) == 0)
    {
        return usedSymbols;
    }

    // Count number of allocated symbols from newly scheduled DCIs
    auto smallestStartSymbol = std::numeric_limits<uint8_t>::max();
    auto largestFinalSymbol = startSym;
    for (auto it = begin; it != end; it++)
    {
        if (smallestStartSymbol > it->m_dci->m_symStart)
        {
            smallestStartSymbol = it->m_dci->m_symStart;
        }
        if (largestFinalSymbol < (it->m_dci->m_symStart + it->m_dci->m_numSym))
        {
            largestFinalSymbol = it->m_dci->m_symStart + it->m_dci->m_numSym;
        }
    }
    NS_ASSERT_MSG(smallestStartSymbol != std::numeric_limits<uint8_t>::max(),
                  "There must have been a valid starting symbol");
    usedSymbols = largestFinalSymbol - smallestStartSymbol;
    return usedSymbols;
}

int
NumRbsPerRbg(int numRbs)
{
    NS_ABORT_MSG_IF(numRbs <= 0, "Number of RBs should be bigger than 0");
    if (numRbs <= 36)
    {
        return 2;
    }
    if (numRbs <= 72)
    {
        return 4;
    }
    if (numRbs <= 144)
    {
        return 8;
    }
    return 16;
}

} // namespace nr
} // namespace ns3
