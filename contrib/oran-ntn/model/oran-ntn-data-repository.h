/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef ORAN_NTN_DATA_REPOSITORY_H
#define ORAN_NTN_DATA_REPOSITORY_H

// Persistent data store for KPM reports, RC actions and xApp decisions
// (2026 realism roadmap §4.1.4 — "OranNtnDataRepository SQLite
// logger (NIST pattern)").
//
// Two backends:
//   - InMemory   — always available, std::vector-backed, fast, volatile
//   - Sqlite     — compiled only when HAVE_SQLITE3 is set at ns-3 configure
//                  time; writes a queryable .db file
//
// Mirrors the NIST ns3-oran abstraction
// <https://github.com/usnistgov/ns3-oran/blob/main/model/oran-data-repository.h>
// so toolkit xApps can be ported across without code changes; field
// names follow the NIST conventions (`gnb_id`, `xapp_name`,
// `target_ue_id`) so a downstream SQL query written against either
// repository is interchangeable.

#include "oran-ntn-types.h"

#include <ns3/object.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace ns3
{

/// One xApp decision record persisted alongside KPM/RC traces.
struct XappRecord
{
    double timestamp;
    uint32_t xappId;
    std::string xappName;
    uint8_t decision;          //!< E2RcActionType enum value
    double confidence;
    double latencyMs;
};

/**
 * \ingroup oran-ntn
 * \brief Abstract base for the toolkit's structured data repository.
 *
 * Roadmap §4.1.4. All accesses are by-value to keep the interface
 * portable across the in-memory and SQLite backends.
 *
 * \note Currently exercised by unit tests only (both backends, via
 *       Create("memory") / Create("sqlite")); no example wires a repository
 *       yet.
 */
class OranNtnDataRepository : public Object
{
  public:
    static TypeId GetTypeId();
    OranNtnDataRepository() = default;
    ~OranNtnDataRepository() override = default;

    /// Factory. `backend` is one of: "memory" (default), "sqlite".
    /// `path` is used only by the sqlite backend; ignored for memory.
    /// Returns nullptr if `backend == "sqlite"` but the toolkit was
    /// built without `HAVE_SQLITE3`.
    static Ptr<OranNtnDataRepository> Create(const std::string& backend,
                                              const std::string& path = "");

    /// Returns the schema version this repository writes / expects.
    static constexpr uint32_t kSchemaVersion = 1;

    /// Backend name; for diagnostics + reproducibility manifest.
    virtual std::string GetBackendName() const = 0;

    // Lifecycle.
    virtual void Open(const std::string& path) = 0;
    virtual void Close() = 0;
    virtual bool IsOpen() const = 0;

    // Inserts. All four must be safe to call before Open() returns
    // (records are buffered until Open is called).
    virtual void LogKpmReport(const E2KpmReport& report) = 0;
    virtual void LogRcAction(const E2RcAction& action) = 0;
    virtual void LogXappRecord(const XappRecord& record) = 0;

    // Read queries.
    virtual std::vector<E2KpmReport>
        GetKpmReportsForUe(uint32_t ueId,
                           double tStart,
                           double tEnd) const = 0;
    virtual std::vector<E2RcAction>
        GetRcActionsByXapp(const std::string& xappName,
                           double tStart,
                           double tEnd) const = 0;
    virtual std::vector<XappRecord>
        GetXappRecords(double tStart, double tEnd) const = 0;

    virtual size_t CountKpmReports() const = 0;
    virtual size_t CountRcActions() const = 0;
    virtual size_t CountXappRecords() const = 0;
};

/// std::vector-backed, in-memory repository. No persistence. Always
/// available. Used as the default backend and by unit tests.
class OranNtnInMemoryDataRepository : public OranNtnDataRepository
{
  public:
    static TypeId GetTypeId();
    OranNtnInMemoryDataRepository() = default;
    ~OranNtnInMemoryDataRepository() override = default;

    std::string GetBackendName() const override { return "memory"; }

    void Open(const std::string& path) override;
    void Close() override;
    bool IsOpen() const override { return m_open; }

    void LogKpmReport(const E2KpmReport& report) override;
    void LogRcAction(const E2RcAction& action) override;
    void LogXappRecord(const XappRecord& record) override;

    std::vector<E2KpmReport> GetKpmReportsForUe(uint32_t ueId,
                                                 double tStart,
                                                 double tEnd) const override;
    std::vector<E2RcAction> GetRcActionsByXapp(const std::string& xappName,
                                                 double tStart,
                                                 double tEnd) const override;
    std::vector<XappRecord> GetXappRecords(double tStart,
                                            double tEnd) const override;
    size_t CountKpmReports() const override { return m_kpmReports.size(); }
    size_t CountRcActions() const override { return m_rcActions.size(); }
    size_t CountXappRecords() const override { return m_xappRecords.size(); }

  private:
    bool m_open{false};
    std::vector<E2KpmReport> m_kpmReports;
    std::vector<E2RcAction> m_rcActions;
    std::vector<XappRecord> m_xappRecords;
};

#ifdef HAVE_SQLITE3
class OranNtnSqliteDataRepositoryImpl;

/// SQLite-backed repository. WAL mode, prepared statements, per-table
/// schema with a `meta` table carrying the schema version + git SHA.
/// Available only when ns-3 was configured with HAVE_SQLITE3.
class OranNtnSqliteDataRepository : public OranNtnDataRepository
{
  public:
    static TypeId GetTypeId();
    OranNtnSqliteDataRepository();
    ~OranNtnSqliteDataRepository() override;

    std::string GetBackendName() const override { return "sqlite"; }

    void Open(const std::string& path) override;
    void Close() override;
    bool IsOpen() const override;

    void LogKpmReport(const E2KpmReport& report) override;
    void LogRcAction(const E2RcAction& action) override;
    void LogXappRecord(const XappRecord& record) override;

    std::vector<E2KpmReport> GetKpmReportsForUe(uint32_t ueId,
                                                 double tStart,
                                                 double tEnd) const override;
    std::vector<E2RcAction> GetRcActionsByXapp(const std::string& xappName,
                                                 double tStart,
                                                 double tEnd) const override;
    std::vector<XappRecord> GetXappRecords(double tStart,
                                            double tEnd) const override;
    size_t CountKpmReports() const override;
    size_t CountRcActions() const override;
    size_t CountXappRecords() const override;

  private:
    std::unique_ptr<OranNtnSqliteDataRepositoryImpl> m_impl;
};
#endif // HAVE_SQLITE3

} // namespace ns3

#endif // ORAN_NTN_DATA_REPOSITORY_H
