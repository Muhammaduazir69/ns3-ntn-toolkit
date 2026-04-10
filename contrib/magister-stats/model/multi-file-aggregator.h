/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 Bucknell University
 * Copyright (c) 2014 Magister Solutions
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
 * Authors of original work (file-aggregator.h):
 * - L. Felipe Perrone (perrone@bucknell.edu)
 * - Mitch Watrous (watrous@u.washington.edu)
 *
 * Modified to support writing to multiple files according to context and with
 * special heading information by:
 * - Budiarto Herman (budiarto.herman@magister.fi)
 *
 * Modified to improve performance by:
 * - Bastien Tauran <bastien.tauran@viveris.fr>
 */

#ifndef MULTI_FILE_AGGREGATOR_H
#define MULTI_FILE_AGGREGATOR_H

#include "ns3/data-collection-object.h"

#include <algorithm>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <string>

namespace ns3
{

/**
 * @ingroup aggregator
 * @brief This aggregator sends values it receives to one or more files.
 *
 * ### Input ###
 * This class provides 10 methods for receiving input values in `double`. Each
 * of these methods is a function with a signature similar to the following:
 * @code
 *   void WritePd (std::string context, double v1, double v2, ... double vP);
 * @endcode
 * where `P` is a number between 1 and 10. In addition, the class provides the
 * method WriteString() which accepts a string input. These input methods
 * usually act as trace sinks of output from collectors' trace sources.
 *
 * ### Output ###
 * Each invocation to the input methods described above will produce a single
 * line of output. The `double` input arguments will be printed using the
 * formatting type selected using SetFileType() method or `FileType` attribute.
 * The `string` argument, on the other hand, will be printed as it is.
 *
 * The first argument of each of the above mentioned input methods is a short
 * string indicating the context of the input sample. When the `MultiFileMode`
 * attribute is enabled (the default), this aggregator will create an
 * individual file for each unique context value, and then send each input
 * sample to the corresponding file.
 *
 * When the `EnableContextPrinting` attribute is enabled (disabled by default),
 * each output line will begin with the context string and then a space. This
 * style is useful to determine the context of different data when all the
 * contexts are mixed together in one file, i.e., when `MultiFileMode` is
 * disabled.
 *
 * The name of every file created begins with the value of the `OutputFileName`
 * attribute, and then followed by the context string. Finally, a ".txt"
 * extension is added at the end.
 *
 * @note All outputs are stored internally in string buffers. Upon destruction,
 *       e.g., at the end of simulation, the whole buffer content is written to
 *       the destination files.
 */
class MultiFileAggregator : public DataCollectionObject
{
  public:
    /// The type of file written by the aggregator.
    enum FileType
    {
        FORMATTED,
        SPACE_SEPARATED,
        COMMA_SEPARATED,
        TAB_SEPARATED
    };

    /**
     * @brief Get the type ID.
     * @return the object TypeId
     */
    static TypeId GetTypeId();

    /**
     * Constructs a multi-file aggregator instance. When the writing methods
     * (e.g., Write1d(), Write2d(), etc.) are invoked, the instance will write
     * the given values into several files.
     *
     * The `OutputFileName` attribute should be set immediately after this,
     * otherwise the instance will write to a default file named "untitled".
     */
    MultiFileAggregator();

    virtual ~MultiFileAggregator();

    /**
     * @param fileType file type specifies the separator to use in
     * printing the file.
     *
     * @brief Set the file type to create, which determines the
     * separator to use when printing values to the file.
     */
    void SetFileType(enum FileType fileType);

    /**
     * @param heading the heading string.
     *
     * @brief Adds a heading string that will be printed on the first
     *        line of each output file, regardless of context.
     *
     * Subsequent calls will append the previous heading string.
     *
     * General heading will be printed after the context heading, if exists.
     */
    void AddGeneralHeading(std::string heading);

    /**
     * @param context the specific context where the heading should apply.
     * @param heading the heading string.
     *
     * @brief Adds a context-specific heading string that will be printed on the
     *        first line of the context's output file.
     *
     * Subsequent calls will append the previous heading string.
     *
     * Context heading will be printed before the general heading.
     */
    void AddContextHeading(std::string context, std::string heading);

    /**
     * @brief Adds a visible warning to the output file name.
     * @param context the specific context where the warning should apply.
     *
     * When enabled, the output file name of the given context will be modified
     * to attract the attention of the user. It may be used to clearly indicate
     * that the output file contains, for example, unreliable content.
     */
    void EnableContextWarning(std::string context);

    /**
     * @param format the 1D format string.
     *
     * @brief Sets the 1D format string for the C-style sprintf()
     * function.
     */
    void Set1dFormat(const std::string& format);

    /**
     * @param format the 2D format string.
     *
     * @brief Sets the 2D format string for the C-style sprintf()
     * function.
     */
    void Set2dFormat(const std::string& format);

    /**
     * @param format the 3D format string.
     *
     * @brief Sets the 3D format string for the C-style sprintf()
     * function.
     */
    void Set3dFormat(const std::string& format);

    /**
     * @param format the 4D format string.
     *
     * @brief Sets the 4D format string for the C-style sprintf()
     * function.
     */
    void Set4dFormat(const std::string& format);

    /**
     * @param format the 5D format string.
     *
     * @brief Sets the 5D format string for the C-style sprintf()
     * function.
     */
    void Set5dFormat(const std::string& format);

    /**
     * @param format the 6D format string.
     *
     * @brief Sets the 6D format string for the C-style sprintf()
     * function.
     */
    void Set6dFormat(const std::string& format);

    /**
     * @param format the 7D format string.
     *
     * @brief Sets the 7D format string for the C-style sprintf()
     * function.
     */
    void Set7dFormat(const std::string& format);

    /**
     * @param format the 8D format string.
     *
     * @brief Sets the 8D format string for the C-style sprintf()
     * function.
     */
    void Set8dFormat(const std::string& format);

    /**
     * @param format the 9D format string.
     *
     * @brief Sets the 9D format string for the C-style sprintf()
     * function.
     */
    void Set9dFormat(const std::string& format);

    /**
     * @param format the 10D format string.
     *
     * @brief Sets the 10D format string for the C-style sprintf()
     * function.
     */
    void Set10dFormat(const std::string& format);

    // Below are hooked to connectors exporting data
    // They are not overloaded since it confuses the compiler when made
    // into callbacks

    /**
     * @param context specifies the 1D dataset these values came from.
     * @param v1 string value to be printed.
     *
     * @brief Writes 1 arbitrary string value to the file.
     */
    void WriteString(std::string context, std::string v1);

    /**
     * @param context specifies the 1D dataset these values came from.
     * @param v1 value for the new data point.
     *
     * @brief Writes 1 value to the file.
     */
    void Write1d(std::string context, double v1);

    /**
     * @param context specifies the 2D dataset these values came from.
     * @param v1 first value for the new data point.
     * @param v2 second value for the new data point.
     *
     * @brief Writes 2 values to the file.
     */
    void Write2d(std::string context, double v1, double v2);

    /**
     * @param context specifies the 3D dataset these values came from.
     * @param v1 first value for the new data point.
     * @param v2 second value for the new data point.
     * @param v3 third value for the new data point.
     *
     * @brief Writes 3 values to the file.
     */
    void Write3d(std::string context, double v1, double v2, double v3);

    /**
     * @param context specifies the 4D dataset these values came from.
     * @param v1 first value for the new data point.
     * @param v2 second value for the new data point.
     * @param v3 third value for the new data point.
     * @param v4 fourth value for the new data point.
     *
     * @brief Writes 4 values to the file.
     */
    void Write4d(std::string context, double v1, double v2, double v3, double v4);

    /**
     * @param context specifies the 5D dataset these values came from.
     * @param v1 first value for the new data point.
     * @param v2 second value for the new data point.
     * @param v3 third value for the new data point.
     * @param v4 fourth value for the new data point.
     * @param v5 fifth value for the new data point.
     *
     * @brief Writes 5 values to the file.
     */
    void Write5d(std::string context, double v1, double v2, double v3, double v4, double v5);

    /**
     * @param context specifies the 6D dataset these values came from.
     * @param v1 first value for the new data point.
     * @param v2 second value for the new data point.
     * @param v3 third value for the new data point.
     * @param v4 fourth value for the new data point.
     * @param v5 fifth value for the new data point.
     * @param v6 sixth value for the new data point.
     *
     * @brief Writes 6 values to the file.
     */
    void Write6d(std::string context,
                 double v1,
                 double v2,
                 double v3,
                 double v4,
                 double v5,
                 double v6);

    /**
     * @param context specifies the 7D dataset these values came from.
     * @param v1 first value for the new data point.
     * @param v2 second value for the new data point.
     * @param v3 third value for the new data point.
     * @param v4 fourth value for the new data point.
     * @param v5 fifth value for the new data point.
     * @param v6 sixth value for the new data point.
     * @param v7 seventh value for the new data point.
     *
     * @brief Writes 7 values to the file.
     */
    void Write7d(std::string context,
                 double v1,
                 double v2,
                 double v3,
                 double v4,
                 double v5,
                 double v6,
                 double v7);

    /**
     * @param context specifies the 8D dataset these values came from.
     * @param v1 first value for the new data point.
     * @param v2 second value for the new data point.
     * @param v3 third value for the new data point.
     * @param v4 fourth value for the new data point.
     * @param v5 fifth value for the new data point.
     * @param v6 sixth value for the new data point.
     * @param v7 seventh value for the new data point.
     * @param v8 eighth value for the new data point.
     *
     * @brief Writes 8 values to the file.
     */
    void Write8d(std::string context,
                 double v1,
                 double v2,
                 double v3,
                 double v4,
                 double v5,
                 double v6,
                 double v7,
                 double v8);

    /**
     * @param context specifies the 9D dataset these values came from.
     * @param v1 first value for the new data point.
     * @param v2 second value for the new data point.
     * @param v3 third value for the new data point.
     * @param v4 fourth value for the new data point.
     * @param v5 fifth value for the new data point.
     * @param v6 sixth value for the new data point.
     * @param v7 seventh value for the new data point.
     * @param v8 eighth value for the new data point.
     * @param v9 nineth value for the new data point.
     *
     * @brief Writes 9 values to the file.
     */
    void Write9d(std::string context,
                 double v1,
                 double v2,
                 double v3,
                 double v4,
                 double v5,
                 double v6,
                 double v7,
                 double v8,
                 double v9);

    /**
     * @param context specifies the 10D dataset these values came from.
     * @param v1 first value for the new data point.
     * @param v2 second value for the new data point.
     * @param v3 third value for the new data point.
     * @param v4 fourth value for the new data point.
     * @param v5 fifth value for the new data point.
     * @param v6 sixth value for the new data point.
     * @param v7 seventh value for the new data point.
     * @param v8 eighth value for the new data point.
     * @param v9 nineth value for the new data point.
     * @param v10 tenth value for the new data point.
     *
     * @brief Writes 10 values to the file.
     */
    void Write10d(std::string context,
                  double v1,
                  double v2,
                  double v3,
                  double v4,
                  double v5,
                  double v6,
                  double v7,
                  double v8,
                  double v9,
                  double v10);

  private:
    /**
     * @param context determines to which file to write.
     *
     * @brief Add this context to the list of known stream and remove old trace file
     * if it is the first occurence of this context. If the current active mode is
     * single-file, then "0" is used as the key.
     */
    void SetContext(std::string context);

    /**
     * Get the full name of a file.
     * @param context determines which file name stream to get.
     * @param additionalData add additional characters at the end of file name.
     */
    std::string GetFullName(std::string context, std::string additionalData = "");

    /**
     * open a stream to a file
     * @param context determines which context to write.
     * @param ofs the stream to write to the file.
     */
    void OpenStream(std::string context, std::ofstream* ofs);

    /// The file name.
    std::string m_outputFileName;

    /// List of known contexts.
    std::set<std::string> m_contexts;

    /// Determines the kind of file written by the aggregator.
    enum FileType m_fileType;

    /// If true, write each context to a separate output file.
    bool m_isMultiFileMode;

    /// If true, write the context string in front of every output line.
    bool m_isContextPrinted;

    /// List of contexts which have warning flag enabled.
    std::set<std::string> m_contextWarningEnabled;

    /// Printed between values in the file.
    std::string m_separator;

    /// Context-specific heading string, indexed by context.
    std::map<std::string, std::string> m_contextHeading;

    /// Cross-context heading string.
    std::string m_generalHeading;

    std::string m_1dFormat;  //!< Format string for 1D C-style sprintf() function.
    std::string m_2dFormat;  //!< Format string for 2D C-style sprintf() function.
    std::string m_3dFormat;  //!< Format string for 3D C-style sprintf() function.
    std::string m_4dFormat;  //!< Format string for 4D C-style sprintf() function.
    std::string m_5dFormat;  //!< Format string for 5D C-style sprintf() function.
    std::string m_6dFormat;  //!< Format string for 6D C-style sprintf() function.
    std::string m_7dFormat;  //!< Format string for 7D C-style sprintf() function.
    std::string m_8dFormat;  //!< Format string for 8D C-style sprintf() function.
    std::string m_9dFormat;  //!< Format string for 9D C-style sprintf() function.
    std::string m_10dFormat; //!< Format string for 10D C-style sprintf() function.

}; // class MultiFileAggregator

} // namespace ns3

#endif // MULTI_FILE_AGGREGATOR_H
