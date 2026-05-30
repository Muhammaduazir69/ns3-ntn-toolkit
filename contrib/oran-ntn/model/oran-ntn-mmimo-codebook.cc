/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "oran-ntn-mmimo-codebook.h"

#include "ns3/log.h"

#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnMmimoCodebook");

TypeId
OranNtnMmimoCodebook::GetTypeId()
{
    static TypeId tid = TypeId("ns3::OranNtnMmimoCodebook")
                            .SetParent<Object>()
                            .SetGroupName("OranNtn")
                            .AddConstructor<OranNtnMmimoCodebook>();
    return tid;
}

OranNtnMmimoCodebook::OranNtnMmimoCodebook() = default;
OranNtnMmimoCodebook::~OranNtnMmimoCodebook() = default;

void
OranNtnMmimoCodebook::DoDispose()
{
    m_entries.clear();
    Object::DoDispose();
}

void
OranNtnMmimoCodebook::Configure(uint32_t num_tx,
                                  uint32_t num_entries_hint)
{
    m_numTx = num_tx;
    m_entries.clear();
    m_entries.reserve(num_entries_hint);
}

void
OranNtnMmimoCodebook::AddEntry(const std::vector<float>& re_im)
{
    if (m_numTx == 0)
    {
        NS_LOG_WARN("AddEntry before Configure()");
        return;
    }
    if (re_im.size() != static_cast<size_t>(2 * m_numTx))
    {
        NS_LOG_WARN("Codebook entry size mismatch: got "
                     << re_im.size() << " expected " << (2 * m_numTx));
        return;
    }
    m_entries.push_back(re_im);
}

void
OranNtnMmimoCodebook::PopulateDftAzimuthSweep(uint32_t num_tx,
                                                 uint32_t num_entries,
                                                 double az_min_rad,
                                                 double az_max_rad)
{
    Configure(num_tx, num_entries);
    if (num_entries == 0 || num_tx == 0)
    {
        return;
    }
    // Uniform-linear-array steering vector with half-wavelength
    // spacing: a_n(θ) = (1/√N) · exp(j · π · n · sin(θ)).
    const double step = (num_entries == 1)
                           ? 0.0
                           : (az_max_rad - az_min_rad) /
                                 (num_entries - 1);
    const double norm = 1.0 / std::sqrt(static_cast<double>(num_tx));
    for (uint32_t k = 0; k < num_entries; ++k)
    {
        const double az = az_min_rad + step * k;
        const double s = std::sin(az);
        std::vector<float> entry(2 * num_tx, 0.0f);
        for (uint32_t n = 0; n < num_tx; ++n)
        {
            const double phase = M_PI * static_cast<double>(n) * s;
            entry[2 * n] = static_cast<float>(norm * std::cos(phase));
            entry[2 * n + 1] = static_cast<float>(norm * std::sin(phase));
        }
        m_entries.push_back(std::move(entry));
    }
}

uint32_t
OranNtnMmimoCodebook::BestMatch(
    const std::vector<float>& target_re_im) const
{
    if (m_entries.empty())
    {
        return 0;
    }
    const auto scores = ScoreAll(target_re_im);
    uint32_t best = 0;
    float best_score = scores.front();
    for (uint32_t k = 1; k < scores.size(); ++k)
    {
        if (scores[k] > best_score)
        {
            best_score = scores[k];
            best = k;
        }
    }
    return best;
}

std::vector<float>
OranNtnMmimoCodebook::ScoreAll(
    const std::vector<float>& target_re_im) const
{
    std::vector<float> scores(m_entries.size(), 0.0f);
    if (m_numTx == 0 ||
        target_re_im.size() != static_cast<size_t>(2 * m_numTx))
    {
        return scores;
    }
    for (size_t k = 0; k < m_entries.size(); ++k)
    {
        const auto& entry = m_entries[k];
        // Conjugate inner product:
        //   <c, t> = Σ_n (c_n* · t_n)
        //   |·|² = (Σ_n re_c·re_t + im_c·im_t)² + (Σ_n re_c·im_t - im_c·re_t)²
        double re_sum = 0.0;
        double im_sum = 0.0;
        for (uint32_t n = 0; n < m_numTx; ++n)
        {
            const double re_c = entry[2 * n];
            const double im_c = entry[2 * n + 1];
            const double re_t = target_re_im[2 * n];
            const double im_t = target_re_im[2 * n + 1];
            re_sum += re_c * re_t + im_c * im_t;
            im_sum += re_c * im_t - im_c * re_t;
        }
        scores[k] = static_cast<float>(re_sum * re_sum +
                                          im_sum * im_sum);
    }
    return scores;
}

} // namespace ns3
