// Copyright (c) 2020 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "hexagonal-grid-scenario-helper.h"

#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/double.h"
#include "ns3/fast-fading-constant-position-mobility-model.h"
#include "ns3/hexagonal-wraparound-model.h"
#include "ns3/mobility-helper.h"
#include "ns3/random-direction-2d-mobility-model.h"
#include "ns3/random-direction-disc-2d-mobility-model.h"
#include "ns3/rectangle.h"
#include "ns3/string.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace ns3
{

void
HexagonalGridScenarioHelper::SetResultsDir(std::string resultsDir)
{
    m_resultsDir = resultsDir;
}

void
HexagonalGridScenarioHelper::SetSimTag(std::string simTag)
{
    m_simTag = simTag;
}

HexagonalGridScenarioHelper::HexagonalGridScenarioHelper()
{
    m_r = CreateObject<UniformRandomVariable>();
    m_theta = CreateObject<UniformRandomVariable>();
    m_resultsDir = ".";
}

HexagonalGridScenarioHelper::~HexagonalGridScenarioHelper()
{
}

const double distTo2ndRing = std::sqrt(3);
const double distTo4thRing = std::sqrt(7);
// Site positions in terms of distance and angle w.r.t. the central site
std::vector<double> HexagonalGridScenarioHelper::siteDistances{0,
                                                               1,
                                                               1,
                                                               1,
                                                               1,
                                                               1,
                                                               1,
                                                               distTo2ndRing,
                                                               distTo2ndRing,
                                                               distTo2ndRing,
                                                               distTo2ndRing,
                                                               distTo2ndRing,
                                                               distTo2ndRing,
                                                               2,
                                                               2,
                                                               2,
                                                               2,
                                                               2,
                                                               2,
                                                               distTo4thRing,
                                                               distTo4thRing,
                                                               distTo4thRing,
                                                               distTo4thRing,
                                                               distTo4thRing,
                                                               distTo4thRing,
                                                               distTo4thRing,
                                                               distTo4thRing,
                                                               distTo4thRing,
                                                               distTo4thRing,
                                                               distTo4thRing,
                                                               distTo4thRing,
                                                               3,
                                                               3,
                                                               3,
                                                               3,
                                                               3,
                                                               3};

/*
 * Site angles w.r.t. the central site center.
 *
 * Note that the angles in the following vector are when looking the deployment in which hexagons
 * are oriented in the following way:
 *
 *    ^               ______
 *    |              /      \
 *    |       ______/        \
 *    |      /      \        /
 *  y |     /        \______/
 *    |     \        /      \
 *    |      \______/        \
 *    |             \        /
 *    |              \______/
 *    ------------------------>
 *          x
 *
 *  This is important to note because the gnuplot function of this the HexagonalGridScenarioHelper
 * plots hexagon in different orientation pointing towards top-bottom, e.g.:
 *
 *     /\
 *   /    \
 *  |      |
 *  |      |
 *   \    /
 *     \/
 *
 */

// the angle of the first hexagon of the fourth ring in the first quadrant
const double ang4thRingAlpha1 = atan2(1, (3 * sqrt(3))) * (180 / M_PI);
// the angle of the second hexagon of the fourth ring in the first quadrant
const double ang4thRingAlpha2 = 90 - atan2(sqrt(3), 2) * (180 / M_PI);
// the angle of the third hexagon of the fourth ring in the first quadrant
const double ang4thRingAlpha3 = 90 - atan2(3, (5 * sqrt(3))) * (180 / M_PI);

std::vector<double> HexagonalGridScenarioHelper::siteAngles{
    0, // 0 ring
    30,
    90,
    150,
    210,
    270,
    330, // 1. ring
    0,
    60,
    120,
    180,
    240,
    300, // 2. ring
    30,
    90,
    150,
    210,
    270,
    330, // 3. ring
    ang4thRingAlpha1,
    ang4thRingAlpha2,
    ang4thRingAlpha3, // 4. ring 1. quadrant
    180 - ang4thRingAlpha3,
    180 - ang4thRingAlpha2,
    180 - ang4thRingAlpha1, // 4. ring 2. quadrant
    180 + ang4thRingAlpha1,
    180 + ang4thRingAlpha2,
    180 + ang4thRingAlpha3, // 4. ring 3. quadrant
    -ang4thRingAlpha3,
    -ang4thRingAlpha2,
    -ang4thRingAlpha1, // 4. ring 4. quadrant
    30,
    90,
    150,
    210,
    270,
    330 // 5. ring
};

/**
 * @brief Creates a GNUPLOT with the hexagonal deployment including base stations
 * (BS), their hexagonal cell areas and user terminals (UT). Positions and cell
 * radius must be given in meters
 *
 * @param sitePosVector Vector of site positions
 * @param cellCenterVector Vector of cell center positions
 * @param utPosVector Vector of user terminals positions
 * @param cellRadius Hexagonal cell radius in meters
 * @param picoCellPosVector Vector of pico cell positions
 */
static void
PlotHexagonalDeployment(const Ptr<const ListPositionAllocator>& sitePosVector,
                        const Ptr<const ListPositionAllocator>& cellCenterVector,
                        const Ptr<const ListPositionAllocator>& picoCellPosVector,
                        const Ptr<const ListPositionAllocator>& utPosVector,
                        double cellRadius,
                        std::string resultsDir,
                        std::string simTag)
{
    uint16_t numCells = cellCenterVector->GetSize();
    uint16_t numSites = sitePosVector->GetSize();
    uint16_t numSectors = numCells / numSites;
    uint16_t numPicos = picoCellPosVector->GetSize();
    uint16_t numUts = utPosVector->GetSize();
    NS_ASSERT_MSG(numCells > 0, "no cells");
    NS_ASSERT_MSG(numSites > 0, "no sites");
    NS_ASSERT_MSG(numUts > 0, "no uts");

    // Try to open a new GNUPLOT file
    std::ofstream topologyOutfile;
    std::string topologyFileRoot = resultsDir + "/hexagonal-topology";
    std::string topologyFileName = topologyFileRoot + simTag + ".gnuplot";
    topologyOutfile.open(topologyFileName.c_str(), std::ios_base::out | std::ios_base::trunc);
    if (!topologyOutfile.is_open())
    {
        NS_ABORT_MSG("Can't open " << topologyFileName);
    }

    topologyOutfile << "set term pdf" << std::endl;
    topologyOutfile << "set output \"" << topologyFileName << ".pdf\"" << std::endl;
    topologyOutfile << "set style arrow 1 lc \"black\" lt 1 head filled" << std::endl;
    //  topologyOutfile << "set autoscale" << std::endl;

    uint16_t margin =
        (12 * cellRadius) + 1; //!< This is the farthest hexagonal vertex from the cell center
    topologyOutfile << "set xrange [-" << margin << ":" << margin << "]" << std::endl;
    topologyOutfile << "set yrange [-" << margin << ":" << margin << "]" << std::endl;
    // FIXME: Need to recalculate ranges if the scenario origin is different to (0,0)

    double arrowLength =
        cellRadius /
        4.0; //<! Control the arrow length that indicates the orientation of the sectorized antenna
    std::vector<double> hx{0.0, -0.5, -0.5, 0.0, 0.5, 0.5, 0.0};   //<! Hexagon vertices in x-axis
    std::vector<double> hy{-1.0, -0.5, 0.5, 1.0, 0.5, -0.5, -1.0}; //<! Hexagon vertices in y-axis
    Vector sitePos;

    for (uint16_t cellId = 0; cellId < numCells; ++cellId)
    {
        Vector cellPos = cellCenterVector->GetNext();
        double angleDeg = 30 + 120 * (cellId % 3);
        double angleRad = angleDeg * M_PI / 180;
        double x;
        double y;

        if (cellId % numSectors == 0)
        {
            sitePos = sitePosVector->GetNext();
        }
        topologyOutfile << "set arrow " << cellId + 1 << " from " << sitePos.x << "," << sitePos.y
                        << " rto " << arrowLength * std::cos(angleRad) << ","
                        << arrowLength * std::sin(angleRad) << " arrowstyle 1 \n";

        // Draw the hexagon around the cell center
        topologyOutfile << "set object " << cellId + 1 << " polygon from \\\n";

        for (uint16_t vertexId = 0; vertexId <= 6; ++vertexId)
        {
            // angle of the vertex w.r.t. y-axis
            x = cellRadius * std::sqrt(3.0) * hx.at(vertexId) + cellPos.x;
            y = cellRadius * hy.at(vertexId) + cellPos.y;
            topologyOutfile << x << ", " << y;
            if (vertexId == 6)
            {
                topologyOutfile << " front fs empty \n";
            }
            else
            {
                topologyOutfile << " to \\\n";
            }
        }

        topologyOutfile << "set label " << cellId + 1 << " \"" << (cellId + 1) << "\" at "
                        << cellPos.x << " , " << cellPos.y << " center" << std::endl;
    }

    for (uint16_t picoId = 0; picoId < numPicos; ++picoId)
    {
        Vector picoPos = picoCellPosVector->GetNext();
        auto cellId = numCells + picoId + 1;
        //      set label at xPos, yPos, zPos "" point pointtype 7 pointsize 2
        topologyOutfile << "set label " << cellId << " \"" << cellId << "\" at " << picoPos.x
                        << " , " << picoPos.y << " point pointtype 1 pointsize 0.5 center"
                        << std::endl;
    }

    for (uint16_t utId = 0; utId < numUts; ++utId)
    {
        Vector utPos = utPosVector->GetNext();
        //      set label at xPos, yPos, zPos "" point pointtype 7 pointsize 2
        topologyOutfile << "set label at " << utPos.x << " , " << utPos.y
                        << " point pointtype 7 pointsize 0.2 center" << std::endl;
    }

    topologyOutfile << "unset key" << std::endl; //!< Disable plot legends
    topologyOutfile << "plot 1/0" << std::endl;  //!< Need to plot a function
}

static Vector
GetClosestSitePosition(Vector cellCenterPos, const Ptr<ListPositionAllocator>& sitePosVector)
{
    double minDistance = UINT32_MAX;
    Vector closestSitePosition;

    for (uint32_t i = 0; i < sitePosVector->GetSize(); i++)
    {
        Vector sitePos = sitePosVector->GetNext();
        double d_x = sitePos.x - cellCenterPos.x;
        double d_y = sitePos.y - cellCenterPos.y;
        double distance2D = sqrt(d_x * d_x + d_y * d_y);
        if (distance2D < minDistance)
        {
            minDistance = distance2D;
            closestSitePosition = sitePos;
        }
    }

    NS_ABORT_MSG_IF(minDistance == UINT32_MAX,
                    "Get closest site position function not executed properly.");
    return closestSitePosition;
}

void
HexagonalGridScenarioHelper::SetNumRings(uint8_t numRings)
{
    NS_ABORT_MSG_IF(numRings > 5, "Unsupported number of outer rings (Maximum is 5");

    m_numRings = numRings;

    /*
     * 0 rings = 1 + 6 * 0 = 1 site
     * 1 rings = 1 + 6 * 1 = 7 sites
     * 2 rings = 1 + 6 * 2 = 13 sites
     * 3 rings = 1 + 6 * 3 = 19 site5
     * 4 rings = 1 + 6 * 5 = 31 sites
     * 5 rings = 1 + 6 * 6 = 37 sites
     */
    switch (numRings)
    {
    case 0:
        m_numSites = 1;
        break;
    case 1:
        m_numSites = 7;
        break;
    case 2:
        m_numSites = 13;
        break;
    case 3:
        m_numSites = 19;
        break;
    case 4:
        m_numSites = 31;
        break;
    case 5:
        m_numSites = 37;
        break;
    }
    SetSitesNumber(m_numSites);
}

double
HexagonalGridScenarioHelper::GetHexagonalCellRadius() const
{
    return m_hexagonalRadius;
}

Vector
HexagonalGridScenarioHelper::GetHexagonalCellCenter(const Vector& sitePos, uint16_t cellId) const
{
    Vector center(sitePos);

    auto sectors = GetNumSectorsPerSite();
    switch (sectors)
    {
    case 0:
        NS_ABORT_MSG("Number of sectors has not been defined");
        break;

    case 1:
        break;

    case 3:
        switch (GetSectorIndex(cellId))
        {
        case 0:
            center.x += m_hexagonalRadius * std::sqrt(0.75);
            center.y += m_hexagonalRadius / 2;
            break;

        case 1:
            center.x -= m_hexagonalRadius * std::sqrt(0.75);
            center.y += m_hexagonalRadius / 2;
            break;

        case 2:
            center.y -= m_hexagonalRadius;
            break;

        default:
            NS_ABORT_MSG("Unknown sector number: " << GetSectorIndex(cellId));
        }
        break;

    default:
        NS_ABORT_MSG("Unsupported number of sectors");
        break;
    }

    return center;
}

void
HexagonalGridScenarioHelper::InstallWraparound(bool installWraparoundModel)
{
    m_installWraparound = installWraparoundModel;
}

void
HexagonalGridScenarioHelper::CreateScenario()
{
    CreateScenarioWithMobility(Vector(0, 0, 0), Vector(0, 0, 0), 0);
}

void
HexagonalGridScenarioHelper::CreateScenarioWithMobility(const Vector& indoorUeSpeed,
                                                        const Vector& outdoorUeSpeed,
                                                        double percentage,
                                                        const std::string& mobilityModel)
{
    m_hexagonalRadius = m_isd / 3;

    m_bs.Create(m_numBs);
    m_ut.Create(m_numUt);

    NS_ASSERT(m_isd > 0);
    NS_ASSERT(m_numRings < 6);
    NS_ASSERT(m_hexagonalRadius > 0);
    NS_ASSERT(m_bsHeight >= 0.0);
    NS_ASSERT(m_utHeight >= 0.0);
    NS_ASSERT(m_bs.GetN() > 0);
    NS_ASSERT(m_ut.GetN() > 0);
    NS_ASSERT_MSG(percentage >= 0 || percentage <= 1,
                  "Percentage must between 0"
                  " and 1");

    MobilityHelper mobility;
    MobilityHelper ueMobility;
    Ptr<ListPositionAllocator> bsPosVector = CreateObject<ListPositionAllocator>();
    Ptr<ListPositionAllocator> picoBsPosVector = CreateObject<ListPositionAllocator>();
    Ptr<ListPositionAllocator> bsCenterVector = CreateObject<ListPositionAllocator>();
    Ptr<ListPositionAllocator> sitePosVector = CreateObject<ListPositionAllocator>();
    Ptr<ListPositionAllocator> utPosVector = CreateObject<ListPositionAllocator>();

    Ptr<HexagonalWraparoundModel> wraparound = nullptr;
    if (m_installWraparound)
    {
        wraparound = CreateObject<HexagonalWraparoundModel>(m_isd, GetNumSites());
    }

    Rectangle outdoorBoundingBox{0, 0, 0, 0};
    // BS position
    for (std::size_t cellId = 0; cellId < m_numBs; cellId++)
    {
        uint16_t siteIndex = GetSiteIndex(cellId);
        Vector sitePos(m_centralPos);
        const double dist = siteDistances.at(siteIndex);
        const double angleRad = siteAngles.at(siteIndex) * M_PI / 180;
        sitePos.x += m_isd * dist * cos(angleRad);
        sitePos.y += m_isd * dist * sin(angleRad);
        sitePos.z = m_bsHeight;

        if (sitePos.x - m_isd < outdoorBoundingBox.xMin)
        {
            outdoorBoundingBox.xMin = sitePos.x - m_isd;
        }
        if (sitePos.x + m_isd > outdoorBoundingBox.xMax)
        {
            outdoorBoundingBox.xMax = sitePos.x + m_isd;
        }
        if (sitePos.y - m_isd < outdoorBoundingBox.yMin)
        {
            outdoorBoundingBox.yMin = sitePos.y - m_isd;
        }
        if (sitePos.y + m_isd > outdoorBoundingBox.yMax)
        {
            outdoorBoundingBox.yMax = sitePos.y + m_isd;
        }

        if (GetSectorIndex(cellId) == 0)
        {
            // std::cout << GetSectorIndex(cellId) << std::endl;
            sitePosVector->Add(sitePos);
            if (wraparound)
            {
                wraparound->AddSitePosition(sitePos);
            }
        }

        // FIXME: Until sites can have more than one antenna array, it is necessary to apply some
        // distance offset from the site center (gNBs cannot have the same location)
        Vector bsPos = GetAntennaPosition(sitePos, cellId);

        bsPosVector->Add(bsPos);

        // Store cell center position for plotting the deployment
        Vector cellCenterPos = GetHexagonalCellCenter(bsPos, cellId);
        bsCenterVector->Add(cellCenterPos);

        // What about the antenna orientation? It should be dealt with when installing the gNB
    }

    // Install mobility model of macro cells
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(bsPosVector);
    mobility.Install(m_bs);

    // Install mobility model of pico cells
    if (m_installPicoCells)
    {
        std::vector<Vector3D> picoCellCoordinate;
        constexpr double ONE_180TH = 1 / 180.0;
        for (std::size_t cellId = 0; cellId < m_numBs; cellId++)
        {
            uint16_t siteIndex = GetSiteIndex(cellId);
            if (GetSectorIndex(cellId) != 0)
            {
                continue;
            }
            // 3GPP TR 36.839 no-wraparound layout: picos sit only on the 6 ring-1 sites
            // (sites at distance ISD from the central site, siteDistances == 1).
            if (m_ringOnePicosOnly && siteDistances.at(siteIndex) != 1.0)
            {
                continue;
            }
            // Compute site position from hex grid definition
            Vector sitePos(m_centralPos);
            const double dist = siteDistances.at(siteIndex);
            const double angleRad = siteAngles.at(siteIndex) * M_PI / 180.0;
            sitePos.x += m_isd * dist * std::cos(angleRad);
            sitePos.y += m_isd * dist * std::sin(angleRad);
            sitePos.z = m_bsHeight;
            // Six pico cells around the site, every 60 degrees
            const double picoRadius = m_isd * 0.6;
            // Merge radius in meters: pico positions generated from different
            // sites (or wraparound images) that land within this distance are
            // treated as the same pico. Distinct picos on the 0.6 * ISD ring are
            // spaced much farther apart at calibration ISDs (>= 100 m at
            // ISD 500 m), so only coincident positions collapse.
            const double eps = 30;

            NS_ASSERT_MSG(m_picoBsHeight != -1.0, "Scenario does not support picocells");
            for (uint8_t picoCellOffset = 0; picoCellOffset < 6; ++picoCellOffset)
            {
                const double ang = (60.0 + picoCellOffset * 60.0) * M_PI * ONE_180TH;

                Vector picoCellPos(sitePos);
                picoCellPos.x += picoRadius * std::cos(ang);
                picoCellPos.y += picoRadius * std::sin(ang);
                picoCellPos.z = m_picoBsHeight;

                // Skip this pico if an equivalent position was already added
                auto it = std::find_if(picoCellCoordinate.begin(),
                                       picoCellCoordinate.end(),
                                       [eps, &picoCellPos](const Vector3D& a) {
                                           return CalculateDistance(a, picoCellPos) < eps;
                                       });

                if (it == picoCellCoordinate.end())
                {
                    picoCellCoordinate.push_back(picoCellPos);
                    picoBsPosVector->Add(picoCellPos);
                }
            }
        }
        m_picoBs.Create(picoCellCoordinate.size());

        mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        mobility.SetPositionAllocator(picoBsPosVector);
        mobility.Install(m_picoBs);
    }

    // UE placement.
    //
    // Two modes are supported:
    //   * Disc mode (m_ueDiscRadius > 0, used by the TR 36.839 no-wraparound layout):
    //     UEs are sampled uniformly in a disc of radius m_ueDiscRadius around the
    //     central site. sqrt(U(0, R^2)) gives a uniform radial sample inside the disc;
    //     the angle is uniform in [-pi, pi]. Mobility is bounded by the same circle via
    //     RandomDirectionDisc2dMobilityModel, so placement and movement region agree.
    //   * Sector mode (default): UEs are sampled uniformly in the hexagon of their cell
    //     (TR 38.901 Table 7.2-1), by rejection from its bounding box, and kept if their 2D
    //     distance to the site is between m_minBsUtDistance and m_maxUeDistanceToClosestSite.
    //     With 3 sectors, the cell is the sector hexagon of circumradius ISD/3, with the site at
    //     a vertex; with 1 sector, it is the site hexagon of circumradius ISD/sqrt(3).
    const bool threeSectors = GetNumSectorsPerSite() == 3;
    const double cellRadius = threeSectors ? m_hexagonalRadius : m_isd / std::sqrt(3);
    if (m_ueDiscRadius > 0.0)
    {
        m_r->SetAttribute("Min", DoubleValue(0));
        m_r->SetAttribute("Max", DoubleValue(m_ueDiscRadius * m_ueDiscRadius));
        m_theta->SetAttribute("Min", DoubleValue(-1.0 * M_PI));
        m_theta->SetAttribute("Max", DoubleValue(M_PI));
    }
    else
    {
        // Bounding box of a hexagon of unit circumradius, with a vertex on the y axis (3 sectors)
        // or on the x axis (1 sector)
        const double halfX = threeSectors ? std::sqrt(3) / 2 : 1;
        const double halfY = threeSectors ? 1 : std::sqrt(3) / 2;
        m_r->SetAttribute("Min", DoubleValue(-halfX));
        m_r->SetAttribute("Max", DoubleValue(halfX));
        m_theta->SetAttribute("Min", DoubleValue(-halfY));
        m_theta->SetAttribute("Max", DoubleValue(halfY));
    }

    // UT position
    NodeContainer indoorUes;
    NodeContainer outdoorUes;
    uint32_t numUesWithRandomUtHeight = 0;
    if (percentage != 0)
    {
        numUesWithRandomUtHeight = percentage * m_ut.GetN();
    }

    for (uint32_t utId = 0; utId < m_ut.GetN(); ++utId)
    {
        Vector cellCenterPos = bsCenterVector->GetNext();
        Vector utPos;

        if (m_ueDiscRadius > 0.0)
        {
            // Disc mode: place uniformly in the disc around the central site.
            double d = std::sqrt(m_r->GetValue());
            double t = m_theta->GetValue();
            utPos = m_centralPos;
            utPos.x += d * std::cos(t);
            utPos.y += d * std::sin(t);
        }
        else
        {
            const Vector sitePosition = GetClosestSitePosition(cellCenterPos, sitePosVector);
            uint16_t sanityCounter = 0;
            while (true)
            {
                NS_ABORT_MSG_IF(sanityCounter++ > 1000,
                                "Algorithm needs too many trials to find correct UE position. "
                                "Please check parameters.");
                const double u = m_r->GetValue();
                const double v = m_theta->GetValue();
                // inside the hexagon: |a| <= sqrt(3)/2 and |b| <= 1 - |a|/sqrt(3), with b along
                // the axis of the vertex
                const double a = std::abs(threeSectors ? u : v);
                const double b = std::abs(threeSectors ? v : u);
                if (b > 1 - a / std::sqrt(3))
                {
                    continue;
                }
                utPos = cellCenterPos;
                utPos.x += u * cellRadius;
                utPos.y += v * cellRadius;
                const double distance2DToSite =
                    std::hypot(utPos.x - sitePosition.x, utPos.y - sitePosition.y);
                if (distance2DToSite >= m_minBsUtDistance &&
                    distance2DToSite <= m_maxUeDistanceToClosestSite)
                {
                    break;
                }
            }
        }

        if (numUesWithRandomUtHeight > 0)
        {
            indoorUes.Add(m_ut.Get(utId));
            Ptr<UniformRandomVariable> uniformRandomVariable =
                CreateObject<UniformRandomVariable>();
            // TR 38.901 Table 7.4.1-1: floor n_fl uniform in 1..N_fl, N_fl uniform in 4..8
            const uint32_t Nfl = uniformRandomVariable->GetInteger(4, 8);
            const uint32_t nfl = uniformRandomVariable->GetInteger(1, Nfl);

            if (m_bsHeight == 10)
            {
                utPos.z = std::min(3 * (nfl - 1) + 1.5, 9.99);
            }
            else
            {
                utPos.z = 3 * (nfl - 1) + 1.5;
            }

            numUesWithRandomUtHeight--;
        }
        else
        {
            utPos.z = m_utHeight;
            outdoorUes.Add(m_ut.Get(utId));
        }

        utPosVector->Add(utPos);
    }

    if (indoorUeSpeed.GetLength() || outdoorUeSpeed.GetLength())
    {
        if (mobilityModel == "ns3::ConstantVelocityMobilityModel")
        {
            ueMobility.SetMobilityModel(mobilityModel);
            ueMobility.SetPositionAllocator(utPosVector);
            ueMobility.Install(m_ut);

            for (uint32_t i = 0; i < indoorUes.GetN(); i++)
            {
                indoorUes.Get(i)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(
                    indoorUeSpeed);
            }
            for (uint32_t i = 0; i < outdoorUes.GetN(); i++)
            {
                outdoorUes.Get(i)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(
                    outdoorUeSpeed);
            }
        }
        else if (mobilityModel == "ns3::RandomDirection2dMobilityModel")
        {
            // In disc mode use the TR 36.839 style circular boundary
            // (RandomDirectionDisc2dMobilityModel), otherwise fall back to the legacy
            // rectangle-bounded model and the gNB-derived bounding box.
            if (m_ueDiscRadius > 0.0)
            {
                ueMobility.SetMobilityModel("ns3::RandomDirectionDisc2dMobilityModel",
                                            "CenterX",
                                            DoubleValue(m_centralPos.x),
                                            "CenterY",
                                            DoubleValue(m_centralPos.y),
                                            "Radius",
                                            DoubleValue(m_ueDiscRadius));
            }
            else
            {
                ueMobility.SetMobilityModel(mobilityModel,
                                            "Bounds",
                                            RectangleValue(outdoorBoundingBox));
            }
            ueMobility.SetPositionAllocator(utPosVector);
            ueMobility.Install(m_ut);
            std::stringstream ss;
            ss << "ns3::ConstantRandomVariable[Constant=" << indoorUeSpeed.GetLength() << "]";
            // Set bounding boxes and velocity for indoor and outdoor UEs. The cast must
            // match whichever model was actually installed above (disc vs rectangle).
            for (uint32_t i = 0; i < indoorUes.GetN(); i++)
            {
                auto uePos = indoorUes.Get(i)->GetObject<MobilityModel>()->GetPosition();
                if (m_ueDiscRadius > 0.0)
                {
                    auto m = indoorUes.Get(i)->GetObject<RandomDirectionDisc2dMobilityModel>();
                    // Indoor disc is a 100-m circle around the UE's initial position.
                    m->SetAttribute("CenterX", DoubleValue(uePos.x));
                    m->SetAttribute("CenterY", DoubleValue(uePos.y));
                    m->SetAttribute("Radius", DoubleValue(100.0));
                    m->SetAttribute("Speed", StringValue(ss.str()));
                }
                else
                {
                    auto m = indoorUes.Get(i)->GetObject<RandomDirection2dMobilityModel>();
                    auto indoorBox =
                        Rectangle(uePos.x - 100, uePos.x + 100, uePos.y - 100, uePos.y + 100);
                    m->SetAttribute("Bounds", RectangleValue(indoorBox));
                    m->SetAttribute("Speed", StringValue(ss.str()));
                }
            }
            // stringstream::clear() only resets the error flags; str("") empties the
            // buffer. With clear() alone the outdoor spec was appended after the
            // indoor one and the attribute parser kept the first (indoor) value, so
            // every outdoor UE silently moved at the indoor speed.
            ss.str("");
            ss << "ns3::ConstantRandomVariable[Constant=" << outdoorUeSpeed.GetLength() << "]";
            for (uint32_t i = 0; i < outdoorUes.GetN(); i++)
            {
                if (m_ueDiscRadius > 0.0)
                {
                    outdoorUes.Get(i)
                        ->GetObject<RandomDirectionDisc2dMobilityModel>()
                        ->SetAttribute("Speed", StringValue(ss.str()));
                }
                else
                {
                    outdoorUes.Get(i)->GetObject<RandomDirection2dMobilityModel>()->SetAttribute(
                        "Speed",
                        StringValue(ss.str()));
                }
            }
        }
        else if (mobilityModel == "ns3::FastFadingConstantPositionMobilityModel")
        {
            ueMobility.SetMobilityModel(mobilityModel, "FakeVelocity", VectorValue(indoorUeSpeed));
            ueMobility.SetPositionAllocator(utPosVector);
            ueMobility.Install(m_ut);
            for (uint32_t i = 0; i < outdoorUes.GetN(); i++)
            {
                outdoorUes.Get(i)
                    ->GetObject<FastFadingConstantPositionMobilityModel>()
                    ->SetAttribute("FakeVelocity", VectorValue(outdoorUeSpeed));
            }
        }
        else
        {
            NS_ABORT_MSG("not supported mobility model");
        }
    }
    else
    {
        mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        mobility.SetPositionAllocator(utPosVector);
        mobility.Install(m_ut);
    }
    if (m_installWraparound)
    {
        m_wraparound = wraparound;
    }
    PlotHexagonalDeployment(sitePosVector,
                            bsCenterVector,
                            picoBsPosVector,
                            utPosVector,
                            m_hexagonalRadius,
                            m_resultsDir,
                            m_simTag);
}

int64_t
HexagonalGridScenarioHelper::AssignStreams(int64_t stream)
{
    m_r->SetStream(stream);
    m_theta->SetStream(stream + 1);
    return 2;
}

void
HexagonalGridScenarioHelper::SetMaxUeDistanceToClosestSite(double maxUeDistanceToClosestSite)
{
    NS_ASSERT(maxUeDistanceToClosestSite > 0 + (m_minBsUtDistance > 0) ? m_minBsUtDistance : 0);
    m_maxUeDistanceToClosestSite = maxUeDistanceToClosestSite;
}

Ptr<WraparoundModel>
HexagonalGridScenarioHelper::GetWraparoundModel() const
{
    return m_wraparound;
}

void
HexagonalGridScenarioHelper::InstallPicoCells(bool installPicoCells)
{
    m_installPicoCells = installPicoCells;
}

void
HexagonalGridScenarioHelper::SetRingOneOnlyPicos(bool ringOnePicosOnly)
{
    m_ringOnePicosOnly = ringOnePicosOnly;
}

void
HexagonalGridScenarioHelper::SetUeDiscRadius(double radius)
{
    NS_ASSERT_MSG(radius >= 0.0, "UE disc radius must be non-negative");
    m_ueDiscRadius = radius;
}

uint16_t
HexagonalGridScenarioHelper::GetNumInnerSites() const
{
    const uint16_t numSites = GetNumSites();
    uint16_t inner = 0;
    for (uint16_t i = 0; i < numSites; ++i)
    {
        if (siteDistances.at(i) <= 1.0)
        {
            ++inner;
        }
    }
    return inner;
}
} // namespace ns3
