/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ntn-spectrum-seam-model.h"

#include "ns3/log.h"
#include "ns3/matrix-array.h"
#include "ns3/spectrum-signal-parameters.h"
#include "ns3/spectrum-value.h"

#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnSpectrumSeamModel");
NS_OBJECT_ENSURE_REGISTERED(NtnSpectrumSeamModel);

TypeId
NtnSpectrumSeamModel::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NtnSpectrumSeamModel")
                            .SetParent<PhasedArraySpectrumPropagationLossModel>()
                            .SetGroupName("NtnTraffic")
                            .AddConstructor<NtnSpectrumSeamModel>();
    return tid;
}

void
NtnSpectrumSeamModel::SetInnerModel(Ptr<PhasedArraySpectrumPropagationLossModel> inner)
{
    m_inner = inner;
}

void
NtnSpectrumSeamModel::AddPlugin(Ptr<SpectrumPropagationLossModel> plugin)
{
    if (plugin)
    {
        m_plugins.push_back(plugin);
    }
}

void
NtnSpectrumSeamModel::DoDispose()
{
    m_inner = nullptr;
    m_plugins.clear();
    PhasedArraySpectrumPropagationLossModel::DoDispose();
}

Ptr<SpectrumSignalParameters>
NtnSpectrumSeamModel::DoCalcRxPowerSpectralDensity(
    Ptr<const SpectrumSignalParameters> params,
    Ptr<const MobilityModel> a,
    Ptr<const MobilityModel> b,
    Ptr<const PhasedArrayModel> aPhasedArrayModel,
    Ptr<const PhasedArrayModel> bPhasedArrayModel) const
{
    // ---- 1. the real 3GPP spatial channel first ---------------------------
    // Array gain, small-scale fading and the MIMO spectrumChannelMatrix all
    // come from here. Bypassing this is precisely the bug S1 fixes.
    Ptr<SpectrumSignalParameters> rxParams;
    if (m_inner)
    {
        rxParams = m_inner->CalcRxPowerSpectralDensity(params, a, b, aPhasedArrayModel,
                                                       bPhasedArrayModel);
    }
    else
    {
        rxParams = params->Copy();
    }
    if (m_plugins.empty() || !rxParams || !rxParams->psd)
    {
        return rxParams;
    }

    // ---- 2. module transfer functions on top ------------------------------
    for (const auto& plugin : m_plugins)
    {
        // Snapshot the pre-plugin PSD so the plugin's per-RB gain can be
        // recovered and mirrored onto the MIMO matrix.
        Ptr<SpectrumValue> before = rxParams->psd->Copy();
        Ptr<SpectrumValue> after = plugin->CalcRxPowerSpectralDensity(rxParams, a, b);
        if (!after)
        {
            continue;
        }
        rxParams->psd = after;

        // ---- 3. keep |H|^2 consistent with the PSD the plugin produced ----
        // NR rank/PMI adaptation (NrPmSearchFull) reads spectrumChannelMatrix,
        // not the PSD. If the plugin attenuates the PSD but the matrix keeps
        // the unattenuated magnitude, the rank search optimises a channel that
        // does not exist. psd ~ |H|^2, so scale H by sqrt(gain) per RB.
        if (rxParams->spectrumChannelMatrix)
        {
            const size_t nRb = rxParams->spectrumChannelMatrix->GetNumPages();
            auto scaled = Create<ComplexMatrixArray>(
                rxParams->spectrumChannelMatrix->GetNumRows(),
                rxParams->spectrumChannelMatrix->GetNumCols(),
                nRb);
            for (size_t rb = 0; rb < nRb; ++rb)
            {
                double gain = 1.0;
                if (rb < before->GetValuesN())
                {
                    const double pre = (*before)[rb];
                    const double post = (*rxParams->psd)[rb];
                    gain = (pre > 0.0) ? (post / pre) : 1.0;
                }
                const double amp = std::sqrt(std::max(gain, 0.0));
                for (size_t r = 0; r < scaled->GetNumRows(); ++r)
                {
                    for (size_t c = 0; c < scaled->GetNumCols(); ++c)
                    {
                        scaled->Elem(r, c, rb) =
                            rxParams->spectrumChannelMatrix->Elem(r, c, rb) * amp;
                    }
                }
            }
            rxParams->spectrumChannelMatrix = scaled;
        }
    }
    return rxParams;
}

int64_t
NtnSpectrumSeamModel::DoAssignStreams(int64_t stream)
{
    int64_t used = 0;
    if (m_inner)
    {
        used += m_inner->AssignStreams(stream + used);
    }
    for (auto& p : m_plugins)
    {
        used += p->AssignStreams(stream + used);
    }
    return used;
}

} // namespace ns3
