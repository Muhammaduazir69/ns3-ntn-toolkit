/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-only
 * Author: Muhammad Uzair
 *
 * THz-NTN RIS Phase Optimization Controller
 *
 * Controller for optimizing RIS phase configurations using codebook-based,
 * search-based, alternating optimization, or DRL-based methods.
 * Integrates with MobilityModel for position-based beam selection.
 *
 * Features:
 *   - DFT-based codebook with beam directions from actual wavelength
 *   - Codebook entry selection using MobilityModel positions
 *   - Alternating optimization for optimal phase profiles
 *   - DRL callback interface for ML-based phase prediction
 *   - Multi-panel coordination across distributed RIS
 *
 * References:
 *   [1] Q. Wu and R. Zhang, "Beamforming optimization for wireless powered
 *       communication networks with intelligent reflecting surface,"
 *       IEEE TWC, 2020.
 *   [2] C. Huang et al., "Indoor signal focusing with deep learning designed
 *       reconfigurable intelligent surfaces," IEEE SPAWC, 2019.
 */

#ifndef THZ_NTN_RIS_CONTROLLER_H
#define THZ_NTN_RIS_CONTROLLER_H

#include "thz-ntn-ris.h"

#include <ns3/callback.h>
#include <ns3/mobility-model.h>
#include <ns3/object.h>
#include <ns3/ptr.h>

#include <cstdint>
#include <vector>

namespace ns3
{

/**
 * \brief RIS phase optimization method.
 */
enum class OptimizationMethod : uint8_t
{
    CODEBOOK = 0,        ///< Pre-computed DFT codebook selection
    RANDOM_SEARCH,       ///< Random search over phase space
    ALTERNATING_OPT,     ///< Alternating optimization (element-by-element)
    DRL_BASED            ///< Deep reinforcement learning callback
};

/**
 * \brief Callback signature for DRL-based phase prediction.
 *
 * Input:  vector of pilot measurements (channel observations)
 * Output: vector of phase shifts in radians
 */
typedef Callback<std::vector<double>, const std::vector<double>&> RisPhasePredictionCallback;

/**
 * \ingroup thz-ntn
 * \brief RIS phase optimization controller.
 *
 * Manages the phase configuration of one or more ThzNtnRis panels.
 * Supports codebook-based beam selection using MobilityModel positions,
 * iterative optimization, and an external DRL callback.
 */
class ThzNtnRisController : public Object
{
  public:
    /**
     * \brief Get the type ID.
     * \return the object TypeId
     */
    static TypeId GetTypeId();

    ThzNtnRisController();
    ~ThzNtnRisController() override;

    /**
     * \brief Attach a RIS panel to this controller.
     * \param ris pointer to the RIS panel
     */
    void SetRis(Ptr<ThzNtnRis> ris);

    /**
     * \brief Generate a DFT-based codebook with given number of entries.
     *
     * Beam directions computed from actual wavelength and element spacing
     * derived from the RIS panel's configured frequency.
     *
     * \param numEntries number of codebook entries (beams)
     */
    void GenerateCodebook(uint32_t numEntries);

    /**
     * \brief Select the best codebook entry using actual MobilityModel positions.
     *
     * Computes TX->RIS->RX angles from actual 3D positions and selects
     * the codebook entry with highest gain for the geometry.
     *
     * \param tx transmitter MobilityModel
     * \param ris RIS panel MobilityModel
     * \param rx receiver MobilityModel
     */
    void SelectCodebookEntry(Ptr<MobilityModel> tx,
                             Ptr<MobilityModel> ris,
                             Ptr<MobilityModel> rx);

    /**
     * \brief Select the best codebook entry for given angle pair (fallback).
     * \param thetaIn incidence elevation angle (deg)
     * \param phiIn incidence azimuth angle (deg)
     * \param thetaOut desired reflection elevation angle (deg)
     * \param phiOut desired reflection azimuth angle (deg)
     */
    void SelectCodebookEntry(double thetaIn, double phiIn, double thetaOut, double phiOut);

    /**
     * \brief Optimize phases using the specified method with MobilityModel positions.
     * \param tx transmitter MobilityModel
     * \param ris RIS panel MobilityModel
     * \param rx receiver MobilityModel
     * \param method optimization method to use
     */
    void OptimizePhases(Ptr<MobilityModel> tx,
                        Ptr<MobilityModel> ris,
                        Ptr<MobilityModel> rx,
                        OptimizationMethod method);

    /**
     * \brief Optimize phases using the specified method with angles (fallback).
     * \param thetaIn incidence elevation angle (deg)
     * \param phiIn incidence azimuth angle (deg)
     * \param thetaOut desired reflection elevation angle (deg)
     * \param phiOut desired reflection azimuth angle (deg)
     * \param method optimization method to use
     */
    void OptimizePhases(double thetaIn,
                        double phiIn,
                        double thetaOut,
                        double phiOut,
                        OptimizationMethod method);

    /**
     * \brief Estimate effective channel quality from pilot measurements.
     * \param pilotMeasurements vector of pilot signal measurements
     * \return estimated effective channel gain (linear)
     */
    double EstimateChannel(const std::vector<double>& pilotMeasurements);

    /**
     * \brief Coordinate phase configurations across multiple RIS panels.
     * \param panels vector of RIS panel pointers
     */
    void CoordinateMultiPanel(const std::vector<Ptr<ThzNtnRis>>& panels);

    /**
     * \brief Set the DRL-based phase prediction callback.
     * \param cb callback that maps pilot measurements to phase profiles
     */
    void SetDrlCallback(RisPhasePredictionCallback cb);

  protected:
    void DoDispose() override;

  private:
    Ptr<ThzNtnRis> m_ris;                    ///< Managed RIS panel
    std::string m_optimizationMethodStr;     ///< Optimization method string for TypeId
    OptimizationMethod m_optimizationMethod; ///< Active optimization method
    uint32_t m_codebookSize;                 ///< Number of codebook entries
    double m_updateInterval_ms;              ///< Phase update interval (ms)
    uint32_t m_maxIterations;                ///< Maximum iterations for optimization

    /// DFT codebook: each entry is a complete phase profile
    std::vector<std::vector<double>> m_codebook;

    /// DRL callback for external phase prediction
    RisPhasePredictionCallback m_drlCallback;
    bool m_hasDrlCallback;

    static constexpr double PI_VAL = 3.14159265358979323846;
    static constexpr double DEG2RAD = PI_VAL / 180.0;

    /**
     * \brief Compute (theta, phi) angles from RIS position to a target.
     */
    void ComputeAngles(const Vector& risPos, const Vector& targetPos,
                       double& theta_deg, double& phi_deg) const;

    void RandomSearchOptimize(double thetaIn, double phiIn,
                              double thetaOut, double phiOut);

    void AlternatingOptimize(double thetaIn, double phiIn,
                             double thetaOut, double phiOut);

    void DrlOptimize(double thetaIn, double phiIn,
                     double thetaOut, double phiOut);

    static OptimizationMethod ParseOptMethod(const std::string& s);
};

} // namespace ns3

#endif /* THZ_NTN_RIS_CONTROLLER_H */
