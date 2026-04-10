/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2013 University of Washington
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
 * Author of original work (file-aggregator.h):
 * - Mitch Watrous (watrous@u.washington.edu)
 *
 * Modified to support writing to multiple files according to context and with
 * special heading information by:
 * - Budiarto Herman (budiarto.herman@magister.fi)
 *
 * Modified to improve performance by:
 * - Bastien Tauran <bastien.tauran@viveris.fr>
 */

#include "multi-file-aggregator.h"

#include "ns3/boolean.h"
#include "ns3/enum.h"
#include "ns3/log.h"
#include "ns3/string.h"

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <ios>
#include <iostream>
#include <map>
#include <sstream>
#include <string>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("MultiFileAggregator");

NS_OBJECT_ENSURE_REGISTERED(MultiFileAggregator);

TypeId
MultiFileAggregator::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::MultiFileAggregator")
            .SetParent<DataCollectionObject>()
            .AddConstructor<MultiFileAggregator>()
            .AddAttribute("OutputFileName",
                          "The file name. In multi-file mode, this would be used as "
                          "the first part or the actual file name produced, so the "
                          "value typically does not contain any extension.",
                          StringValue("untitled"),
                          MakeStringAccessor(&MultiFileAggregator::m_outputFileName),
                          MakeStringChecker())
            .AddAttribute(
                "FileType",
                "Determines the kind of file written by the aggregator.",
                EnumValue(MultiFileAggregator::SPACE_SEPARATED),
                MakeEnumAccessor<MultiFileAggregator::FileType>(&MultiFileAggregator::SetFileType),
                MakeEnumChecker(MultiFileAggregator::FORMATTED,
                                "FORMATTED",
                                MultiFileAggregator::SPACE_SEPARATED,
                                "SPACE_SEPARATED",
                                MultiFileAggregator::COMMA_SEPARATED,
                                "COMMA_SEPARATED",
                                MultiFileAggregator::TAB_SEPARATED,
                                "TAB_SEPARATED"))
            .AddAttribute("MultiFileMode",
                          "If true, write each context to a separate output file. "
                          "Otherwise, write all contexts to a single file.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&MultiFileAggregator::m_isMultiFileMode),
                          MakeBooleanChecker())
            .AddAttribute("EnableContextPrinting",
                          "If true, include the context string in front of every "
                          "output line. Useful when MultiFileMode is disabled.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&MultiFileAggregator::m_isContextPrinted),
                          MakeBooleanChecker())
            .AddAttribute("GeneralHeading",
                          "Sets the heading string that will be printed on the first "
                          "line of each file.",
                          StringValue(""),
                          MakeStringAccessor(&MultiFileAggregator::AddGeneralHeading),
                          MakeStringChecker());
    return tid;
}

MultiFileAggregator::MultiFileAggregator()
    : m_outputFileName("untitled.txt"),
      m_fileType(MultiFileAggregator::SPACE_SEPARATED),
      m_isMultiFileMode(true),
      m_isContextPrinted(false),
      m_1dFormat("%e"),
      m_2dFormat("%e %e"),
      m_3dFormat("%e %e %e"),
      m_4dFormat("%e %e %e %e"),
      m_5dFormat("%e %e %e %e %e"),
      m_6dFormat("%e %e %e %e %e %e"),
      m_7dFormat("%e %e %e %e %e %e %e"),
      m_8dFormat("%e %e %e %e %e %e %e %e"),
      m_9dFormat("%e %e %e %e %e %e %e %e %e"),
      m_10dFormat("%e %e %e %e %e %e %e %e %e %e")
{
    NS_LOG_FUNCTION(this);
}

MultiFileAggregator::~MultiFileAggregator()
{
    NS_LOG_FUNCTION(this);

    for (std::string context : m_contexts)
    {
        // Remove any space and slash characters from the context.
        for (size_t pos = context.find(" /"); pos != std::string::npos;
             pos = context.find(" /", pos + 1, 1))
        {
            context[pos] = '_';
        }

        // Creating a file for output.
        std::string fileNameOut = GetFullName(context);
        std::string fileNameTemp = GetFullName(context, ".temp");

        // We do it only if we have a header to write
        if (m_contextHeading.find(context) != m_contextHeading.end())
        {
            if (std::rename(fileNameOut.c_str(), fileNameTemp.c_str()) == 0)
            {
                // Rename success, write header on a new file then append old file.
                NS_LOG_INFO("Creating a new file " << fileNameOut);

                std::ifstream ifs(fileNameTemp);
                std::ofstream ofs(fileNameOut);

                if (!ifs || !(ifs.is_open()))
                {
                    NS_FATAL_ERROR("Error reading file " << fileNameTemp);
                }
                if (!ofs || !(ofs.is_open()))
                {
                    NS_FATAL_ERROR("Error creating file " << fileNameOut << " for output");
                }

                // Find the context-specific heading for this context.
                std::map<std::string, std::string>::iterator it2 = m_contextHeading.find(context);

                if ((it2 != m_contextHeading.end()) && !it2->second.empty())
                {
                    ofs << it2->second << std::endl;
                }

                char ch;
                while (ifs.get(ch))
                {
                    ofs << ch;
                }
                ofs << std::endl;

                ifs.close();
                ofs.close();
            }
            else
            {
                NS_FATAL_ERROR("Cannot rename " << fileNameOut << " to " << fileNameTemp);
            }
            std::remove(fileNameTemp.c_str());
        }
        else
        {
            // We append a blank line at the end.
            std::ofstream ofs(fileNameOut, std::ios::out | std::ios::app);
            if (!ofs || !(ofs.is_open()))
            {
                NS_FATAL_ERROR("Error creating file " << fileNameOut << " for output");
            }
            ofs << std::endl;
            ofs.close();
        }
    }
}

void
MultiFileAggregator::OpenStream(std::string context, std::ofstream* ofs)
{
    NS_LOG_FUNCTION(this << context << ofs);

    SetContext(context);

    // Remove any space and slash characters from the context.
    for (size_t pos = context.find(" /"); pos != std::string::npos;
         pos = context.find(" /", pos + 1, 1))
    {
        context[pos] = '_';
    }

    // Creating a file for output.
    std::string fileName = GetFullName(context);
    NS_LOG_INFO("Creating or appending a file " << fileName);
    ofs->open(fileName, std::ios::out | std::ios::app);
    if (!(*ofs) || !(ofs->is_open()))
    {
        NS_FATAL_ERROR("Error creating file " << fileName << " for output");
    }
}

void
MultiFileAggregator::SetFileType(enum FileType fileType)
{
    NS_LOG_FUNCTION(this << fileType);
    m_fileType = fileType;

    // Set the values separator.
    switch (m_fileType)
    {
    case COMMA_SEPARATED:
        m_separator = ",";
        break;
    case TAB_SEPARATED:
        m_separator = "\t";
        break;
    default:
        // Space separated.
        m_separator = " ";
        break;
    }
}

void
MultiFileAggregator::AddGeneralHeading(std::string heading)
{
    NS_LOG_FUNCTION(this << heading);
    m_generalHeading += heading;
}

void
MultiFileAggregator::AddContextHeading(std::string context, std::string heading)
{
    NS_LOG_FUNCTION(this << context << heading);

    if (!m_isMultiFileMode)
    {
        context = "0";
    }

    std::map<std::string, std::string>::iterator it = m_contextHeading.find(context);

    if (it == m_contextHeading.end())
    {
        m_contextHeading[context] = heading;
    }
    else
    {
        it->second += heading;
    }
}

void
MultiFileAggregator::EnableContextWarning(std::string context)
{
    NS_LOG_FUNCTION(this);
    m_contextWarningEnabled.insert(context);
}

void
MultiFileAggregator::Set1dFormat(const std::string& format)
{
    NS_LOG_FUNCTION(this << format);
    m_1dFormat = format;
}

void
MultiFileAggregator::Set2dFormat(const std::string& format)
{
    NS_LOG_FUNCTION(this << format);
    m_2dFormat = format;
}

void
MultiFileAggregator::Set3dFormat(const std::string& format)
{
    NS_LOG_FUNCTION(this << format);
    m_3dFormat = format;
}

void
MultiFileAggregator::Set4dFormat(const std::string& format)
{
    NS_LOG_FUNCTION(this << format);
    m_4dFormat = format;
}

void
MultiFileAggregator::Set5dFormat(const std::string& format)
{
    NS_LOG_FUNCTION(this << format);
    m_5dFormat = format;
}

void
MultiFileAggregator::Set6dFormat(const std::string& format)
{
    NS_LOG_FUNCTION(this << format);
    m_6dFormat = format;
}

void
MultiFileAggregator::Set7dFormat(const std::string& format)
{
    NS_LOG_FUNCTION(this << format);
    m_7dFormat = format;
}

void
MultiFileAggregator::Set8dFormat(const std::string& format)
{
    NS_LOG_FUNCTION(this << format);
    m_8dFormat = format;
}

void
MultiFileAggregator::Set9dFormat(const std::string& format)
{
    NS_LOG_FUNCTION(this << format);
    m_9dFormat = format;
}

void
MultiFileAggregator::Set10dFormat(const std::string& format)
{
    NS_LOG_FUNCTION(this << format);
    m_10dFormat = format;
}

void
MultiFileAggregator::WriteString(std::string context, std::string v1)
{
    NS_LOG_FUNCTION(this << context << v1);

    if (m_enabled)
    {
        std::ofstream ofs;
        OpenStream(context, &ofs);

        if (m_isContextPrinted)
        {
            // Write the context and the value with the proper separator.
            ofs << context << m_separator << v1 << std::endl;
        }
        else
        {
            // Write the value.
            ofs << v1 << std::endl;
        }
        ofs.close();
    }
}

void
MultiFileAggregator::Write1d(std::string context, double v1)
{
    NS_LOG_FUNCTION(this << context << v1);

    if (m_enabled)
    {
        std::ofstream ofs;
        OpenStream(context, &ofs);

        // Write the 1D data point to the file.
        if (m_fileType == FORMATTED)
        {
            // Initially, have the C-style string in the buffer, which
            // is terminated by a null character, be of length zero.
            char buffer[500];
            int maxBufferSize = 500;
            buffer[0] = 0;

            // Format the value.
            int charWritten = snprintf(buffer, maxBufferSize, m_1dFormat.c_str(), v1);
            if (charWritten < 0)
            {
                NS_LOG_DEBUG("Error writing value to output file");
            }

            // Write the formatted value.
            ofs << buffer << std::endl;
        }
        else if (m_isContextPrinted)
        {
            // Write the context and the value with the proper separator.
            ofs << context << m_separator << v1 << std::endl;
        }
        else
        {
            // Write the value.
            ofs << v1 << std::endl;
        }
        ofs.close();
    }
}

void
MultiFileAggregator::Write2d(std::string context, double v1, double v2)
{
    NS_LOG_FUNCTION(this << context << v1 << v2);

    if (m_enabled)
    {
        std::ofstream ofs;
        OpenStream(context, &ofs);

        // Write the 2D data point to the file.
        if (m_fileType == FORMATTED)
        {
            // Initially, have the C-style string in the buffer, which
            // is terminated by a null character, be of length zero.
            char buffer[500];
            int maxBufferSize = 500;
            buffer[0] = 0;

            // Format the values.
            int charWritten = snprintf(buffer, maxBufferSize, m_2dFormat.c_str(), v1, v2);
            if (charWritten < 0)
            {
                NS_LOG_DEBUG("Error writing values to output file");
            }

            // Write the formatted values.
            ofs << buffer << std::endl;
        }
        else if (m_isContextPrinted)
        {
            // Write the context and the values with the proper separator.
            ofs << context << m_separator << v1 << m_separator << v2 << std::endl;
        }
        else
        {
            // Write the values with the proper separator.
            ofs << v1 << m_separator << v2 << std::endl;
        }
        ofs.close();
    }
}

void
MultiFileAggregator::Write3d(std::string context, double v1, double v2, double v3)
{
    NS_LOG_FUNCTION(this << context << v1 << v2 << v3);

    if (m_enabled)
    {
        std::ofstream ofs;
        OpenStream(context, &ofs);

        // Write the 3D data point to the file.
        if (m_fileType == FORMATTED)
        {
            // Initially, have the C-style string in the buffer, which
            // is terminated by a null character, be of length zero.
            char buffer[500];
            int maxBufferSize = 500;
            buffer[0] = 0;

            // Format the values.
            int charWritten = snprintf(buffer, maxBufferSize, m_3dFormat.c_str(), v1, v2, v3);
            if (charWritten < 0)
            {
                NS_LOG_DEBUG("Error writing values to output file");
            }

            // Write the formatted values.
            ofs << buffer << std::endl;
        }
        else if (m_isContextPrinted)
        {
            // Write the context and the values with the proper separator.
            ofs << context << m_separator << v1 << m_separator << v2 << m_separator << v3
                << std::endl;
        }
        else
        {
            // Write the values with the proper separator.
            ofs << v1 << m_separator << v2 << m_separator << v3 << std::endl;
        }
        ofs.close();
    }
}

void
MultiFileAggregator::Write4d(std::string context, double v1, double v2, double v3, double v4)
{
    NS_LOG_FUNCTION(this << context << v1 << v2 << v3 << v4);

    if (m_enabled)
    {
        std::ofstream ofs;
        OpenStream(context, &ofs);

        // Write the 4D data point to the file.
        if (m_fileType == FORMATTED)
        {
            // Initially, have the C-style string in the buffer, which
            // is terminated by a null character, be of length zero.
            char buffer[500];
            int maxBufferSize = 500;
            buffer[0] = 0;

            // Format the values.
            int charWritten = snprintf(buffer, maxBufferSize, m_4dFormat.c_str(), v1, v2, v3, v4);
            if (charWritten < 0)
            {
                NS_LOG_DEBUG("Error writing values to output file");
            }

            // Write the formatted values.
            ofs << buffer << std::endl;
        }
        else if (m_isContextPrinted)
        {
            // Write the context and the values with the proper separator.
            ofs << context << m_separator << v1 << m_separator << v2 << m_separator << v3
                << m_separator << v4 << std::endl;
        }
        else
        {
            // Write the values with the proper separator.
            ofs << v1 << m_separator << v2 << m_separator << v3 << m_separator << v4 << std::endl;
        }
        ofs.close();
    }
}

void
MultiFileAggregator::Write5d(std::string context,
                             double v1,
                             double v2,
                             double v3,
                             double v4,
                             double v5)
{
    NS_LOG_FUNCTION(this << context << v1 << v2 << v3 << v4 << v5);

    if (m_enabled)
    {
        std::ofstream ofs;
        OpenStream(context, &ofs);

        // Write the 5D data point to the file.
        if (m_fileType == FORMATTED)
        {
            // Initially, have the C-style string in the buffer, which
            // is terminated by a null character, be of length zero.
            char buffer[500];
            int maxBufferSize = 500;
            buffer[0] = 0;

            // Format the values.
            int charWritten =
                snprintf(buffer, maxBufferSize, m_5dFormat.c_str(), v1, v2, v3, v4, v5);
            if (charWritten < 0)
            {
                NS_LOG_DEBUG("Error writing values to output file");
            }

            // Write the formatted values.
            ofs << buffer << std::endl;
        }
        else if (m_isContextPrinted)
        {
            // Write the context and the values with the proper separator.
            ofs << context << m_separator << v1 << m_separator << v2 << m_separator << v3
                << m_separator << v4 << m_separator << v5 << std::endl;
        }
        else
        {
            // Write the values with the proper separator.
            ofs << v1 << m_separator << v2 << m_separator << v3 << m_separator << v4 << m_separator
                << v5 << std::endl;
        }
        ofs.close();
    }
}

void
MultiFileAggregator::Write6d(std::string context,
                             double v1,
                             double v2,
                             double v3,
                             double v4,
                             double v5,
                             double v6)
{
    NS_LOG_FUNCTION(this << context << v1 << v2 << v3 << v4 << v5 << v6);

    if (m_enabled)
    {
        std::ofstream ofs;
        OpenStream(context, &ofs);

        // Write the 6D data point to the file.
        if (m_fileType == FORMATTED)
        {
            // Initially, have the C-style string in the buffer, which
            // is terminated by a null character, be of length zero.
            char buffer[500];
            int maxBufferSize = 500;
            buffer[0] = 0;

            // Format the values.
            int charWritten =
                snprintf(buffer, maxBufferSize, m_6dFormat.c_str(), v1, v2, v3, v4, v5, v6);
            if (charWritten < 0)
            {
                NS_LOG_DEBUG("Error writing values to output file");
            }

            // Write the formatted values.
            ofs << buffer << std::endl;
        }
        else if (m_isContextPrinted)
        {
            // Write the context and the values with the proper separator.
            ofs << context << m_separator << v1 << m_separator << v2 << m_separator << v3
                << m_separator << v4 << m_separator << v5 << m_separator << v6 << std::endl;
        }
        else
        {
            // Write the values with the proper separator.
            ofs << v1 << m_separator << v2 << m_separator << v3 << m_separator << v4 << m_separator
                << v5 << m_separator << v6 << std::endl;
        }
        ofs.close();
    }
}

void
MultiFileAggregator::Write7d(std::string context,
                             double v1,
                             double v2,
                             double v3,
                             double v4,
                             double v5,
                             double v6,
                             double v7)
{
    NS_LOG_FUNCTION(this << context << v1 << v2 << v3 << v4 << v5 << v6 << v7);

    if (m_enabled)
    {
        std::ofstream ofs;
        OpenStream(context, &ofs);

        // Write the 7D data point to the file.
        if (m_fileType == FORMATTED)
        {
            // Initially, have the C-style string in the buffer, which
            // is terminated by a null character, be of length zero.
            char buffer[500];
            int maxBufferSize = 500;
            buffer[0] = 0;

            // Format the values.
            int charWritten =
                snprintf(buffer, maxBufferSize, m_7dFormat.c_str(), v1, v2, v3, v4, v5, v6, v7);
            if (charWritten < 0)
            {
                NS_LOG_DEBUG("Error writing values to output file");
            }

            // Write the formatted values.
            ofs << buffer << std::endl;
        }
        else if (m_isContextPrinted)
        {
            // Write the context and the values with the proper separator.
            ofs << context << m_separator << v1 << m_separator << v2 << m_separator << v3
                << m_separator << v4 << m_separator << v5 << m_separator << v6 << m_separator << v7
                << std::endl;
        }
        else
        {
            // Write the values with the proper separator.
            ofs << v1 << m_separator << v2 << m_separator << v3 << m_separator << v4 << m_separator
                << v5 << m_separator << v6 << m_separator << v7 << std::endl;
        }
        ofs.close();
    }
}

void
MultiFileAggregator::Write8d(std::string context,
                             double v1,
                             double v2,
                             double v3,
                             double v4,
                             double v5,
                             double v6,
                             double v7,
                             double v8)
{
    NS_LOG_FUNCTION(this << context << v1 << v2 << v3 << v4 << v5 << v6 << v7 << v8);

    if (m_enabled)
    {
        std::ofstream ofs;
        OpenStream(context, &ofs);

        // Write the 8D data point to the file.
        if (m_fileType == FORMATTED)
        {
            // Initially, have the C-style string in the buffer, which
            // is terminated by a null character, be of length zero.
            char buffer[500];
            int maxBufferSize = 500;
            buffer[0] = 0;

            // Format the values.
            int charWritten =
                snprintf(buffer, maxBufferSize, m_8dFormat.c_str(), v1, v2, v3, v4, v5, v6, v7, v8);
            if (charWritten < 0)
            {
                NS_LOG_DEBUG("Error writing values to output file");
            }

            // Write the formatted values.
            ofs << buffer << std::endl;
        }
        else if (m_isContextPrinted)
        {
            // Write the context and the values with the proper separator.
            ofs << context << m_separator << v1 << m_separator << v2 << m_separator << v3
                << m_separator << v4 << m_separator << v5 << m_separator << v6 << m_separator << v7
                << m_separator << v8 << std::endl;
        }
        else
        {
            // Write the values with the proper separator.
            ofs << v1 << m_separator << v2 << m_separator << v3 << m_separator << v4 << m_separator
                << v5 << m_separator << v6 << m_separator << v7 << m_separator << v8 << std::endl;
        }
        ofs.close();
    }
}

void
MultiFileAggregator::Write9d(std::string context,
                             double v1,
                             double v2,
                             double v3,
                             double v4,
                             double v5,
                             double v6,
                             double v7,
                             double v8,
                             double v9)
{
    NS_LOG_FUNCTION(this << context << v1 << v2 << v3 << v4 << v5 << v6 << v7 << v8 << v9);

    if (m_enabled)
    {
        std::ofstream ofs;
        OpenStream(context, &ofs);

        // Write the 9D data point to the file.
        if (m_fileType == FORMATTED)
        {
            // Initially, have the C-style string in the buffer, which
            // is terminated by a null character, be of length zero.
            char buffer[500];
            int maxBufferSize = 500;
            buffer[0] = 0;

            // Format the values.
            int charWritten = snprintf(buffer,
                                       maxBufferSize,
                                       m_9dFormat.c_str(),
                                       v1,
                                       v2,
                                       v3,
                                       v4,
                                       v5,
                                       v6,
                                       v7,
                                       v8,
                                       v9);
            if (charWritten < 0)
            {
                NS_LOG_DEBUG("Error writing values to output file");
            }

            // Write the formatted values.
            ofs << buffer << std::endl;
        }
        else if (m_isContextPrinted)
        {
            // Write the context and the values with the proper separator.
            ofs << context << m_separator << v1 << m_separator << v2 << m_separator << v3
                << m_separator << v4 << m_separator << v5 << m_separator << v6 << m_separator << v7
                << m_separator << v8 << m_separator << v9 << std::endl;
        }
        else
        {
            // Write the values with the proper separator.
            ofs << v1 << m_separator << v2 << m_separator << v3 << m_separator << v4 << m_separator
                << v5 << m_separator << v6 << m_separator << v7 << m_separator << v8 << m_separator
                << v9 << std::endl;
        }
        ofs.close();
    }
}

void
MultiFileAggregator::Write10d(std::string context,
                              double v1,
                              double v2,
                              double v3,
                              double v4,
                              double v5,
                              double v6,
                              double v7,
                              double v8,
                              double v9,
                              double v10)
{
    NS_LOG_FUNCTION(this << context << v1 << v2 << v3 << v4 << v5 << v6 << v7 << v8 << v9 << v10);

    if (m_enabled)
    {
        std::ofstream ofs;
        OpenStream(context, &ofs);

        // Write the 10D data point to the file.
        if (m_fileType == FORMATTED)
        {
            // Initially, have the C-style string in the buffer, which
            // is terminated by a null character, be of length zero.
            char buffer[500];
            int maxBufferSize = 500;
            buffer[0] = 0;

            // Format the values.
            int charWritten = snprintf(buffer,
                                       maxBufferSize,
                                       m_10dFormat.c_str(),
                                       v1,
                                       v2,
                                       v3,
                                       v4,
                                       v5,
                                       v6,
                                       v7,
                                       v8,
                                       v9,
                                       v10);
            if (charWritten < 0)
            {
                NS_LOG_DEBUG("Error writing values to output file");
            }

            // Write the formatted values.
            ofs << buffer << std::endl;
        }
        else if (m_isContextPrinted)
        {
            // Write the context and the values with the proper separator.
            ofs << context << m_separator << v1 << m_separator << v2 << m_separator << v3
                << m_separator << v4 << m_separator << v5 << m_separator << v6 << m_separator << v7
                << m_separator << v8 << m_separator << v9 << m_separator << v10 << std::endl;
        }
        else
        {
            // Write the values with the proper separator.
            ofs << v1 << m_separator << v2 << m_separator << v3 << m_separator << v4 << m_separator
                << v5 << m_separator << v6 << m_separator << v7 << m_separator << v8 << m_separator
                << v9 << m_separator << v10 << std::endl;
        }
        ofs.close();
    }
}

void
MultiFileAggregator::SetContext(std::string context)
{
    NS_LOG_FUNCTION(this << context);

    if (!m_isMultiFileMode)
    {
        context = "0";
    }

    if (!std::binary_search(m_contexts.begin(), m_contexts.end(), context))
    {
        // This is a new context.
        m_contexts.insert(context);
        std::remove(GetFullName(context).c_str());

        std::ofstream ofs(GetFullName(context));
        ofs << m_generalHeading << std::endl;
        ofs.close();
    }
}

std::string
MultiFileAggregator::GetFullName(std::string context, std::string additionalData)
{
    NS_LOG_FUNCTION(this << context << additionalData);

    std::ostringstream fileName;
    fileName << m_outputFileName;
    if (m_isMultiFileMode)
    {
        fileName << '-' + context;
    }
    if (m_contextWarningEnabled.count(context) > 0)
    {
        fileName << "-ATTN";
    }
    fileName << ".txt";
    fileName << additionalData;
    return fileName.str();
}

} // namespace ns3
