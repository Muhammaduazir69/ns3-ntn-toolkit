/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 Magister Solutions
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
 * Modified: Lauri Sormunen <lauri.sormunen@magister.fi>
 *
 */

#ifndef STATS_HELPER_H
#define STATS_HELPER_H

#include "ns3/attribute.h"
#include "ns3/net-device-container.h"
#include "ns3/node-container.h"
#include "ns3/object.h"
#include "ns3/ptr.h"

#include <map>
#include <string>

namespace ns3
{

class Node;
class CollectorMap;
class DataCollectionObject;

/**
 * @ingroup stats
 * @defgroup stats Statistics
 *
 * Data Collection Framework (DCF) implementation on Stats module. For
 * usage in simulation script, see comments of inherited classes.
 */

/**
 * @ingroup stats
 * @brief Parent abstract class of other statistics helpers.
 *
 * A helper is responsible to locate source objects, create probes, collectors,
 * and aggregators, and connect them together in a proper way to produce the
 * required statistics.
 *
 * The main inputs for the helper are a
 * name, an identifier type, an output type and nodes which are monitored.
 * After all the necessary inputs have been set, the statistics
 * can be started into action by invoking Install(). For example:
 * @code
 *     NodeContainer nodes;
 *     nodes.Create (2);
 *     // ... (snip) ...
 *     Ptr<StatsAppThroughputHelper> stat
 *         = CreateObject<StatsAppThroughputHelper> ();
 *     stat->SetName ("name");
 *     stat->SetIdentifierType (StatsHelper::IDENTIFIER_GLOBAL);
 *     stat->SetOutputType (StatsHelper::OUTPUT_SCALAR_FILE);
 *     stat->InstallNodes (nodes);
 *     stat->Install ();
 *     // ... (run simulation) ...
 *     stat->Dispose (); // Disposing creates output files
 * @endcode
 *
 * This parent abstract class hosts several protected methods which are intended
 * to simplify the development of child classes. These methods handle
 * tasks related to DCF components.
 */
class StatsHelper : public Object
{
  public:
    // COMMON ENUM DATA TYPES ///////////////////////////////////////////////////

    /**
     * @enum IdentifierType_t
     * @brief Possible categorization of statistics output.
     */
    typedef enum
    {
        IDENTIFIER_GLOBAL = 0,
        IDENTIFIER_NODE
    } IdentifierType_t;

    /**
     * @param identifierType an arbitrary identifier type.
     * @return representation of the identifier type in string.
     */
    static std::string GetIdentifierTypeName(IdentifierType_t identifierType);

    /**
     * @enum OutputType_t
     * @brief Possible types and formats of statistics output.
     */
    typedef enum
    {
        OUTPUT_NONE = 0,
        OUTPUT_SCALAR_FILE,
        OUTPUT_SCATTER_FILE,
        OUTPUT_HISTOGRAM_FILE,
        OUTPUT_PDF_FILE, // probability distribution function
        OUTPUT_CDF_FILE, // cumulative distribution function
        OUTPUT_SCALAR_PLOT,
        OUTPUT_SCATTER_PLOT,
        OUTPUT_HISTOGRAM_PLOT,
        OUTPUT_PDF_PLOT, // probability distribution function
        OUTPUT_CDF_PLOT, // cumulative distribution function
    } OutputType_t;

    /**
     * @param outputType an arbitrary output type.
     * @return representation of the output type in string.
     */
    static std::string GetOutputTypeName(OutputType_t outputType);

    // CONSTRUCTOR AND DESTRUCTOR ///////////////////////////////////////////////

    /**
     * @brief Creates a new helper instance.
     */
    StatsHelper();

    /**
     * / Destructor.
     */
    virtual ~StatsHelper();

    /**
     * inherited from ObjectBase base class
     */
    static TypeId GetTypeId();

    /**
     * Check if a directory is valid.
     * @param path Path that is checked
     */
    static bool IsValidDirectory(std::string path);

    // PUBLIC METHODS ///////////////////////////////////////////////////////////

    /**
     * @brief Install the probes, collectors, and aggregators necessary to
     *        produce the statistics output.
     *
     * Behaviour should be implemented by child class in DoInstall().
     */
    void Install();

    // SETTER AND GETTER METHODS ////////////////////////////////////////////////

    /**
     * @param name string to be prepended on every output file name.
     */
    void SetName(std::string name);

    /**
     * @return the name of this helper instance.
     */
    std::string GetName() const;

    /**
     * Install nodes to this helper instance.
     * @param nodes Nodes to which statistics collectors are installed
     */
    void InstallNodes(NodeContainer nodes);

    /**
     * Install node to this helper instance.
     * @param node Node to which statistics collector is installed
     */
    void InstallNodes(Ptr<Node> node);

    /**
     * @param identifierType categorization of statistics output.
     * @warning Does not have any effect if invoked after Install().
     */
    void SetIdentifierType(IdentifierType_t identifierType);

    /**
     * @return the currently active categorization of statistics output.
     */
    IdentifierType_t GetIdentifierType() const;

    /**
     * @param outputType types and formats of statistics output.
     * @warning Does not have any effect if invoked after Install().
     */
    void SetOutputType(OutputType_t outputType);

    /**
     * @return the currently active types and formats of statistics output.
     */
    OutputType_t GetOutputType() const;

    /**
     * @return true if Install() has been invoked, otherwise false.
     */
    bool IsInstalled() const;

  protected:
    /**
     * @brief Install the probes, collectors, and aggregators necessary to
     *        produce the statistics output.
     *
     * An abstract method of StatsHelper which must be implemented by child
     * classes. It will be invoked by Install().
     */
    virtual void DoInstall() = 0;

    /**
     * @return the path where statistics output file should be created.
     *
     * Path is determined by OutputPath attribute.
     */
    virtual std::string GetOutputPath() const;

    /**
     * @param the path where statistics output should be written to.
     *
     * Set the default output path.
     */
    virtual void SetOutputPath(std::string outputPath);

    /**
     * @brief Compute the path and file name where statistics output should be
     *        written to.
     * @return path and file name (without extension)
     *
     * Path is determined by by OutputPath attribute.
     */
    virtual std::string GetOutputFileName() const;

    /**
     * @param dataLabel the short name of the main data of this statistics
     * @return a string to be printed as the first line of output, consisting of
     *         the identifier title and the given data label
     */
    virtual std::string GetIdentifierHeading(std::string dataLabel) const;

    /**
     * @param dataLabel the short name of the main data of this statistics
     * @return a string to be printed as the first line of output, consisting of
     *         the title of the time column and the given data label
     */
    virtual std::string GetTimeHeading(std::string dataLabel) const;

    /**
     * @param dataLabel the short name of the main data of this statistics
     * @return a string to be printed as the first line of output, consisting of
     *         the given data label and the title of the distribution column
     */
    virtual std::string GetDistributionHeading(std::string dataLabel) const;

    /**
     * @brief Create the aggregator according to the output type.
     * @param aggregatorTypeId the type of aggregator to be created.
     * @param n1 the name of the attribute to be set on the aggregator created.
     * @param v1 the value of the attribute to be set on the aggregator created.
     * @param n2 the name of the attribute to be set on the aggregator created.
     * @param v2 the value of the attribute to be set on the aggregator created.
     * @param n3 the name of the attribute to be set on the aggregator created.
     * @param v3 the value of the attribute to be set on the aggregator created.
     * @param n4 the name of the attribute to be set on the aggregator created.
     * @param v4 the value of the attribute to be set on the aggregator created.
     * @param n5 the name of the attribute to be set on the aggregator created.
     * @param v5 the value of the attribute to be set on the aggregator created.
     * @return a pointer to the created aggregator.
     */
    Ptr<DataCollectionObject> CreateAggregator(std::string aggregatorTypeId,
                                               std::string n1 = "",
                                               const AttributeValue& v1 = EmptyAttributeValue(),
                                               std::string n2 = "",
                                               const AttributeValue& v2 = EmptyAttributeValue(),
                                               std::string n3 = "",
                                               const AttributeValue& v3 = EmptyAttributeValue(),
                                               std::string n4 = "",
                                               const AttributeValue& v4 = EmptyAttributeValue(),
                                               std::string n5 = "",
                                               const AttributeValue& v5 = EmptyAttributeValue());

    /**
     * @brief Create one collector instance for each identifier in the simulation.
     * @param collectorMap the CollectorMap where the collectors will be created.
     * @return number of collector instances created.
     *
     * The identifier is determined by the currently active identifier type, as
     * previously selected by SetIdentifierType() method or `IdentifierType`
     * attribute. Then the method creates identifiers for installed nodes based on
     * current identifier type, creates a collector instance for each identifier,
     * assigns the collector instance a name, and put the collector instance into
     * the CollectorMap.
     */
    uint32_t CreateCollectorPerIdentifier(CollectorMap& collectorMap) const;

    /**
     * @brief Get nodes installed to this helper instance.
     * @return nodes installed
     */
    inline NodeContainer GetNodes()
    {
        return m_nodes;
    };

  private:
    std::string m_name;       ///< Name of the helper and file, should not contain file extension
    std::string m_outputPath; ///< Output path for statistics files produced by helper instance
    IdentifierType_t m_identifierType; ///< Identifier type
    OutputType_t m_outputType;         ///< Output type
    bool m_isInstalled;                ///< Installation status
    NodeContainer m_nodes;             ///< Nodes to which statistics collectors are installed

}; // end of class StatsHelper

} // end of namespace ns3

#endif /* STATS_HELPER_H */
