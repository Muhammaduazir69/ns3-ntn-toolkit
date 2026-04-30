/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Muhammad Uzair
 *
 * NTN Measurement Model - RSRP/SINR computation from satellite beams
 * using 3GPP TR 38.811 NTN propagation models.
 */

#ifndef NTN_MEASUREMENT_MODEL_H
#define NTN_MEASUREMENT_MODEL_H

#include "ntn-orbit-predictor.h"

#include <ns3/channel-condition-model.h>
#include <ns3/node-container.h>
#include <ns3/object.h>
#include <ns3/random-variable-stream.h>
#include <ns3/three-gpp-propagation-loss-model.h>
#include <ns3/traced-callback.h>

#include <vector>

namespace ns3
{

/**
 * \ingroup ntn-cho
 * \brief Computes RSRP/SINR measurements from satellite beams using 3GPP NTN models
 *
 * Integrates ThreeGppNTN*PropagationLossModel with satellite antenna gain patterns
 * to produce realistic signal measurements from LEO satellite beams.
 *
 * Reference: 3GPP TR 38.811 (Study on NTN)
 */
class NtnMeasurementModel : public Object
{
  public:
    /**
     * \brief A single measurement from a satellite beam
     */
    struct Measurement
    {
        uint16_t cellId = 0;
        uint32_t satId = 0;
        uint32_t beamId = 0;
        double rsrp_dBm = -140.0;         //!< Reference Signal Received Power
        double sinr_dB = -20.0;            //!< Signal-to-Interference+Noise Ratio
        double pathLoss_dB = 200.0;        //!< Total path loss
        double antennaGain_dB = 0.0;       //!< Satellite antenna gain toward UE
        double elevationAngle_deg = 0.0;   //!< Satellite elevation angle
        double slantRange_km = 0.0;        //!< Distance UE to satellite
        Time propagationDelay;             //!< One-way propagation delay
        double dopplerShift_Hz = 0.0;      //!< Doppler frequency shift
    };

    /**
     * \brief NTN channel scenario types per 3GPP TR 38.811
     */
    enum NtnScenario
    {
        NTN_DENSE_URBAN,
        NTN_URBAN,
        NTN_SUBURBAN,
        NTN_RURAL
    };

    static TypeId GetTypeId();
    NtnMeasurementModel();
    ~NtnMeasurementModel() override;

    /**
     * \brief Set the orbit predictor for satellite position/beam information
     */
    void SetOrbitPredictor(Ptr<NtnOrbitPredictor> predictor);

    /**
     * \brief Configure the NTN channel scenario
     * \param scenario One of the 3GPP NTN scenarios
     */
    void SetNtnScenario(NtnScenario scenario);

    /**
     * \brief Set carrier frequency
     * \param freqHz Carrier frequency in Hz (e.g., 2e9 for S-band, 20e9 for Ka-band)
     */
    void SetCarrierFrequency(double freqHz);

    /**
     * \brief Set bandwidth
     * \param bwHz Bandwidth in Hz
     */
    void SetBandwidth(double bwHz);

    /**
     * \brief Set satellite transmit power
     * \param txPower_dBm Per-beam EIRP in dBm
     */
    void SetSatelliteTxPower(double txPower_dBm);

    /**
     * \brief Set UE noise figure
     * \param nf_dB Noise figure in dB
     */
    void SetUeNoiseFigure(double nf_dB);

    /**
     * \brief Compute measurement for a specific satellite beam
     * \param uePosition UE ground position
     * \param satId Satellite identifier
     * \param beamId Beam identifier
     * \return Measurement struct with all computed metrics
     */
    Measurement ComputeMeasurement(GeoCoordinate uePosition,
                                   uint32_t satId,
                                   uint32_t beamId) const;

    /**
     * \brief Scan all visible satellites and return measurements sorted by SINR
     * \param uePosition UE ground position
     * \param minElevation_deg Minimum elevation angle to consider
     * \return Vector of measurements sorted by SINR descending
     */
    std::vector<Measurement> ScanVisibleBeams(GeoCoordinate uePosition,
                                              double minElevation_deg = 10.0) const;

    /**
     * \brief Compute Doppler shift for a satellite link
     * \param uePosition UE ground position
     * \param satId Satellite identifier
     * \return Doppler shift in Hz
     */
    double ComputeDopplerShift(GeoCoordinate uePosition, uint32_t satId) const;

    // Trace source for measurement events
    TracedCallback<uint32_t, uint32_t, double, double> m_measurementTrace;
    //              satId,  beamId,  rsrp,   sinr

  protected:
    void DoDispose() override;

  private:
    /**
     * \brief Compute free-space path loss for NTN link
     */
    double ComputeFreeSpacePathLoss(double distance_m, double freqHz) const;

    /**
     * \brief Compute thermal noise power
     */
    double ComputeNoisePower() const;

    Ptr<NtnOrbitPredictor> m_orbitPredictor;
    NtnScenario m_scenario;
    double m_carrierFreqHz;
    double m_bandwidthHz;
    Ptr<NormalRandomVariable> m_shadowFadingRng;       //!< per-call N(0,1) sampler for shadow fading
    Ptr<NormalRandomVariable> m_scintillationRng;      //!< per-call N(0,1) sampler for scintillation
    double m_satTxPower_dBm;
    double m_ueNoiseFigure_dB;

    static constexpr double BOLTZMANN_CONSTANT = 1.380649e-23;   //!< k_B (J/K)
    static constexpr double TEMPERATURE = 290.0;                 //!< Reference temperature (K)
    static constexpr double SPEED_OF_LIGHT = 299792458.0;        //!< c (m/s)
};

} // namespace ns3

#endif // NTN_MEASUREMENT_MODEL_H
