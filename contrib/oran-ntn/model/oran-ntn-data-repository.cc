/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "oran-ntn-data-repository.h"

#include "ns3/log.h"

#ifdef HAVE_SQLITE3
#include <sqlite3.h>
#include <stdexcept>
#endif

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnDataRepository");

// ----------------------------------------------------------------------------
//  Abstract base — TypeId + factory
// ----------------------------------------------------------------------------

TypeId
OranNtnDataRepository::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranNtnDataRepository").SetParent<Object>().SetGroupName("OranNtn");
    return tid;
}

Ptr<OranNtnDataRepository>
OranNtnDataRepository::Create(const std::string& backend,
                              const std::string& path)
{
    if (backend == "memory")
    {
        Ptr<OranNtnInMemoryDataRepository> r =
            CreateObject<OranNtnInMemoryDataRepository>();
        r->Open(path);
        return r;
    }
    if (backend == "sqlite")
    {
#ifdef HAVE_SQLITE3
        Ptr<OranNtnSqliteDataRepository> r =
            CreateObject<OranNtnSqliteDataRepository>();
        r->Open(path);
        return r;
#else
        NS_LOG_WARN("oran-ntn DataRepository: sqlite backend requested but "
                    "the toolkit was built without HAVE_SQLITE3");
        return nullptr;
#endif
    }
    NS_LOG_WARN("oran-ntn DataRepository: unknown backend '"
                << backend << "'");
    return nullptr;
}

// ----------------------------------------------------------------------------
//  In-memory backend
// ----------------------------------------------------------------------------

TypeId
OranNtnInMemoryDataRepository::GetTypeId()
{
    static TypeId tid = TypeId("ns3::OranNtnInMemoryDataRepository")
                            .SetParent<OranNtnDataRepository>()
                            .SetGroupName("OranNtn")
                            .AddConstructor<OranNtnInMemoryDataRepository>();
    return tid;
}

void
OranNtnInMemoryDataRepository::Open(const std::string& /*path*/)
{
    m_open = true;
}

void
OranNtnInMemoryDataRepository::Close()
{
    m_open = false;
}

void
OranNtnInMemoryDataRepository::LogKpmReport(const E2KpmReport& report)
{
    m_kpmReports.push_back(report);
}

void
OranNtnInMemoryDataRepository::LogRcAction(const E2RcAction& action)
{
    m_rcActions.push_back(action);
}

void
OranNtnInMemoryDataRepository::LogXappRecord(const XappRecord& record)
{
    m_xappRecords.push_back(record);
}

std::vector<E2KpmReport>
OranNtnInMemoryDataRepository::GetKpmReportsForUe(uint32_t ueId,
                                                    double tStart,
                                                    double tEnd) const
{
    std::vector<E2KpmReport> out;
    for (const auto& r : m_kpmReports)
    {
        if (r.ueId == ueId && r.timestamp >= tStart && r.timestamp <= tEnd)
        {
            out.push_back(r);
        }
    }
    return out;
}

std::vector<E2RcAction>
OranNtnInMemoryDataRepository::GetRcActionsByXapp(const std::string& xappName,
                                                    double tStart,
                                                    double tEnd) const
{
    std::vector<E2RcAction> out;
    for (const auto& a : m_rcActions)
    {
        if (a.xappName == xappName && a.timestamp >= tStart &&
            a.timestamp <= tEnd)
        {
            out.push_back(a);
        }
    }
    return out;
}

std::vector<XappRecord>
OranNtnInMemoryDataRepository::GetXappRecords(double tStart, double tEnd) const
{
    std::vector<XappRecord> out;
    for (const auto& r : m_xappRecords)
    {
        if (r.timestamp >= tStart && r.timestamp <= tEnd)
        {
            out.push_back(r);
        }
    }
    return out;
}

// ----------------------------------------------------------------------------
//  SQLite backend
// ----------------------------------------------------------------------------

#ifdef HAVE_SQLITE3

class OranNtnSqliteDataRepositoryImpl
{
  public:
    sqlite3* db = nullptr;

    // Cached prepared statements; recreated on Open.
    sqlite3_stmt* insertKpm = nullptr;
    sqlite3_stmt* insertRc = nullptr;
    sqlite3_stmt* insertXapp = nullptr;
    sqlite3_stmt* queryKpm = nullptr;
    sqlite3_stmt* queryRc = nullptr;
    sqlite3_stmt* queryXapp = nullptr;
    sqlite3_stmt* countKpm = nullptr;
    sqlite3_stmt* countRc = nullptr;
    sqlite3_stmt* countXapp = nullptr;

    ~OranNtnSqliteDataRepositoryImpl() { Close(); }

    void Close()
    {
        Finalize(insertKpm);
        Finalize(insertRc);
        Finalize(insertXapp);
        Finalize(queryKpm);
        Finalize(queryRc);
        Finalize(queryXapp);
        Finalize(countKpm);
        Finalize(countRc);
        Finalize(countXapp);
        if (db != nullptr)
        {
            sqlite3_close(db);
            db = nullptr;
        }
    }

    static void Finalize(sqlite3_stmt*& s)
    {
        if (s != nullptr)
        {
            sqlite3_finalize(s);
            s = nullptr;
        }
    }

    static void Exec(sqlite3* d, const char* sql)
    {
        char* err = nullptr;
        if (sqlite3_exec(d, sql, nullptr, nullptr, &err) != SQLITE_OK)
        {
            std::string msg = err ? err : "(unknown)";
            sqlite3_free(err);
            throw std::runtime_error(std::string("sqlite exec: ") + msg +
                                     " for: " + sql);
        }
    }
};

TypeId
OranNtnSqliteDataRepository::GetTypeId()
{
    static TypeId tid = TypeId("ns3::OranNtnSqliteDataRepository")
                            .SetParent<OranNtnDataRepository>()
                            .SetGroupName("OranNtn")
                            .AddConstructor<OranNtnSqliteDataRepository>();
    return tid;
}

OranNtnSqliteDataRepository::OranNtnSqliteDataRepository()
    : m_impl(std::make_unique<OranNtnSqliteDataRepositoryImpl>())
{
}

OranNtnSqliteDataRepository::~OranNtnSqliteDataRepository() = default;

bool
OranNtnSqliteDataRepository::IsOpen() const
{
    return m_impl->db != nullptr;
}

void
OranNtnSqliteDataRepository::Open(const std::string& path)
{
    if (m_impl->db != nullptr)
    {
        Close();
    }
    if (sqlite3_open(path.c_str(), &m_impl->db) != SQLITE_OK)
    {
        const std::string msg =
            m_impl->db ? sqlite3_errmsg(m_impl->db) : "open failed";
        if (m_impl->db != nullptr)
        {
            sqlite3_close(m_impl->db);
            m_impl->db = nullptr;
        }
        throw std::runtime_error(std::string("sqlite open: ") + msg);
    }
    // WAL mode survives mid-run crashes; small page cache for ns-3 forks.
    OranNtnSqliteDataRepositoryImpl::Exec(
        m_impl->db,
        "PRAGMA journal_mode=WAL;"
        "PRAGMA synchronous=NORMAL;"
        "PRAGMA temp_store=MEMORY;");

    // Schema
    OranNtnSqliteDataRepositoryImpl::Exec(
        m_impl->db,
        "CREATE TABLE IF NOT EXISTS meta ("
        "  key TEXT PRIMARY KEY, value TEXT NOT NULL);");
    OranNtnSqliteDataRepositoryImpl::Exec(
        m_impl->db,
        "CREATE TABLE IF NOT EXISTS kpm_reports ("
        "  rowid INTEGER PRIMARY KEY,"
        "  ts REAL NOT NULL,"
        "  gnb_id INTEGER NOT NULL,"
        "  is_ntn INTEGER NOT NULL,"
        "  ue_id INTEGER NOT NULL,"
        "  rsrp_dBm REAL, sinr_dB REAL,"
        "  throughput_Mbps REAL, latency_ms REAL,"
        "  elevation_deg REAL, doppler_Hz REAL,"
        "  tte_s REAL, prb_util REAL,"
        "  slice_id INTEGER);"
        "CREATE INDEX IF NOT EXISTS kpm_reports_ue_ts "
        "  ON kpm_reports (ue_id, ts);"
        "CREATE TABLE IF NOT EXISTS rc_actions ("
        "  rowid INTEGER PRIMARY KEY,"
        "  ts REAL NOT NULL,"
        "  xapp_id INTEGER, xapp_name TEXT,"
        "  action_type INTEGER, target_gnb INTEGER,"
        "  target_ue INTEGER, target_beam INTEGER,"
        "  slice_id INTEGER, confidence REAL,"
        "  param1 REAL, param2 REAL,"
        "  executed INTEGER, rejection_reason TEXT);"
        "CREATE INDEX IF NOT EXISTS rc_actions_name_ts "
        "  ON rc_actions (xapp_name, ts);"
        "CREATE TABLE IF NOT EXISTS xapp_records ("
        "  rowid INTEGER PRIMARY KEY,"
        "  ts REAL NOT NULL,"
        "  xapp_id INTEGER, xapp_name TEXT,"
        "  decision INTEGER, confidence REAL, latency_ms REAL);"
        "CREATE INDEX IF NOT EXISTS xapp_records_ts ON xapp_records (ts);");

    // Record schema version + backend identity.
    OranNtnSqliteDataRepositoryImpl::Exec(
        m_impl->db,
        "INSERT OR REPLACE INTO meta (key, value) VALUES "
        " ('schema_version', '1'),"
        " ('backend', 'ns3-ntn-toolkit/oran-ntn-sqlite-data-repository');");

    // Prepare insert / query statements once.
    auto prep = [&](const char* sql, sqlite3_stmt*& out) {
        if (sqlite3_prepare_v2(m_impl->db, sql, -1, &out, nullptr) !=
            SQLITE_OK)
        {
            throw std::runtime_error(std::string("sqlite prepare: ") +
                                     sqlite3_errmsg(m_impl->db) +
                                     " for " + sql);
        }
    };
    prep("INSERT INTO kpm_reports (ts, gnb_id, is_ntn, ue_id, rsrp_dBm, "
         "sinr_dB, throughput_Mbps, latency_ms, elevation_deg, doppler_Hz, "
         "tte_s, prb_util, slice_id) "
         "VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?);",
         m_impl->insertKpm);
    prep("INSERT INTO rc_actions (ts, xapp_id, xapp_name, action_type, "
         "target_gnb, target_ue, target_beam, slice_id, confidence, param1, "
         "param2, executed, rejection_reason) "
         "VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?);",
         m_impl->insertRc);
    prep("INSERT INTO xapp_records (ts, xapp_id, xapp_name, decision, "
         "confidence, latency_ms) VALUES (?,?,?,?,?,?);",
         m_impl->insertXapp);
    prep("SELECT ts, gnb_id, is_ntn, ue_id, rsrp_dBm, sinr_dB, "
         "throughput_Mbps, latency_ms, elevation_deg, doppler_Hz, tte_s, "
         "prb_util, slice_id FROM kpm_reports "
         "WHERE ue_id=? AND ts BETWEEN ? AND ? ORDER BY ts;",
         m_impl->queryKpm);
    prep("SELECT ts, xapp_id, xapp_name, action_type, target_gnb, target_ue, "
         "target_beam, slice_id, confidence, param1, param2, executed, "
         "rejection_reason FROM rc_actions "
         "WHERE xapp_name=? AND ts BETWEEN ? AND ? ORDER BY ts;",
         m_impl->queryRc);
    prep("SELECT ts, xapp_id, xapp_name, decision, confidence, latency_ms "
         "FROM xapp_records WHERE ts BETWEEN ? AND ? ORDER BY ts;",
         m_impl->queryXapp);
    prep("SELECT COUNT(*) FROM kpm_reports;", m_impl->countKpm);
    prep("SELECT COUNT(*) FROM rc_actions;", m_impl->countRc);
    prep("SELECT COUNT(*) FROM xapp_records;", m_impl->countXapp);
}

void
OranNtnSqliteDataRepository::Close()
{
    m_impl->Close();
}

namespace
{

void
StepInsert(sqlite3_stmt* s, const char* tag)
{
    int rc = sqlite3_step(s);
    if (rc != SQLITE_DONE)
    {
        sqlite3_reset(s);
        throw std::runtime_error(std::string("sqlite ") + tag +
                                 " step failed: " +
                                 sqlite3_errmsg(sqlite3_db_handle(s)));
    }
    sqlite3_reset(s);
}

} // namespace

void
OranNtnSqliteDataRepository::LogKpmReport(const E2KpmReport& r)
{
    auto* s = m_impl->insertKpm;
    sqlite3_bind_double(s, 1, r.timestamp);
    sqlite3_bind_int64(s, 2, r.gnbId);
    sqlite3_bind_int(s, 3, r.isNtn ? 1 : 0);
    sqlite3_bind_int64(s, 4, r.ueId);
    sqlite3_bind_double(s, 5, r.rsrp_dBm);
    sqlite3_bind_double(s, 6, r.sinr_dB);
    sqlite3_bind_double(s, 7, r.throughput_Mbps);
    sqlite3_bind_double(s, 8, r.latency_ms);
    sqlite3_bind_double(s, 9, r.elevation_deg);
    sqlite3_bind_double(s, 10, r.doppler_Hz);
    sqlite3_bind_double(s, 11, r.tte_s);
    sqlite3_bind_double(s, 12, r.prbUtilization);
    sqlite3_bind_int(s, 13, r.sliceId);
    StepInsert(s, "kpm insert");
}

void
OranNtnSqliteDataRepository::LogRcAction(const E2RcAction& a)
{
    auto* s = m_impl->insertRc;
    sqlite3_bind_double(s, 1, a.timestamp);
    sqlite3_bind_int64(s, 2, a.xappId);
    sqlite3_bind_text(s, 3, a.xappName.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(s, 4, static_cast<int>(a.actionType));
    sqlite3_bind_int64(s, 5, a.targetGnbId);
    sqlite3_bind_int64(s, 6, a.targetUeId);
    sqlite3_bind_int64(s, 7, a.targetBeamId);
    sqlite3_bind_int(s, 8, a.targetSliceId);
    sqlite3_bind_double(s, 9, a.confidence);
    sqlite3_bind_double(s, 10, a.parameter1);
    sqlite3_bind_double(s, 11, a.parameter2);
    sqlite3_bind_int(s, 12, a.executed ? 1 : 0);
    sqlite3_bind_text(s, 13, a.rejectionReason.c_str(), -1, SQLITE_TRANSIENT);
    StepInsert(s, "rc insert");
}

void
OranNtnSqliteDataRepository::LogXappRecord(const XappRecord& r)
{
    auto* s = m_impl->insertXapp;
    sqlite3_bind_double(s, 1, r.timestamp);
    sqlite3_bind_int64(s, 2, r.xappId);
    sqlite3_bind_text(s, 3, r.xappName.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(s, 4, r.decision);
    sqlite3_bind_double(s, 5, r.confidence);
    sqlite3_bind_double(s, 6, r.latencyMs);
    StepInsert(s, "xapp insert");
}

std::vector<E2KpmReport>
OranNtnSqliteDataRepository::GetKpmReportsForUe(uint32_t ueId,
                                                  double tStart,
                                                  double tEnd) const
{
    std::vector<E2KpmReport> out;
    auto* s = m_impl->queryKpm;
    sqlite3_bind_int64(s, 1, ueId);
    sqlite3_bind_double(s, 2, tStart);
    sqlite3_bind_double(s, 3, tEnd);
    while (sqlite3_step(s) == SQLITE_ROW)
    {
        E2KpmReport r{};
        r.timestamp = sqlite3_column_double(s, 0);
        r.gnbId = static_cast<uint32_t>(sqlite3_column_int64(s, 1));
        r.isNtn = sqlite3_column_int(s, 2) != 0;
        r.ueId = static_cast<uint32_t>(sqlite3_column_int64(s, 3));
        r.rsrp_dBm = sqlite3_column_double(s, 4);
        r.sinr_dB = sqlite3_column_double(s, 5);
        r.throughput_Mbps = sqlite3_column_double(s, 6);
        r.latency_ms = sqlite3_column_double(s, 7);
        r.elevation_deg = sqlite3_column_double(s, 8);
        r.doppler_Hz = sqlite3_column_double(s, 9);
        r.tte_s = sqlite3_column_double(s, 10);
        r.prbUtilization = sqlite3_column_double(s, 11);
        r.sliceId = static_cast<uint8_t>(sqlite3_column_int(s, 12));
        out.push_back(r);
    }
    sqlite3_reset(s);
    return out;
}

std::vector<E2RcAction>
OranNtnSqliteDataRepository::GetRcActionsByXapp(const std::string& xappName,
                                                  double tStart,
                                                  double tEnd) const
{
    std::vector<E2RcAction> out;
    auto* s = m_impl->queryRc;
    sqlite3_bind_text(s, 1, xappName.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_double(s, 2, tStart);
    sqlite3_bind_double(s, 3, tEnd);
    while (sqlite3_step(s) == SQLITE_ROW)
    {
        E2RcAction a{};
        a.timestamp = sqlite3_column_double(s, 0);
        a.xappId = static_cast<uint32_t>(sqlite3_column_int64(s, 1));
        const unsigned char* nm = sqlite3_column_text(s, 2);
        a.xappName = nm ? reinterpret_cast<const char*>(nm) : "";
        a.actionType =
            static_cast<E2RcActionType>(sqlite3_column_int(s, 3));
        a.targetGnbId = static_cast<uint32_t>(sqlite3_column_int64(s, 4));
        a.targetUeId = static_cast<uint32_t>(sqlite3_column_int64(s, 5));
        a.targetBeamId = static_cast<uint32_t>(sqlite3_column_int64(s, 6));
        a.targetSliceId = static_cast<uint8_t>(sqlite3_column_int(s, 7));
        a.confidence = sqlite3_column_double(s, 8);
        a.parameter1 = sqlite3_column_double(s, 9);
        a.parameter2 = sqlite3_column_double(s, 10);
        a.executed = sqlite3_column_int(s, 11) != 0;
        const unsigned char* rej = sqlite3_column_text(s, 12);
        a.rejectionReason =
            rej ? reinterpret_cast<const char*>(rej) : "";
        out.push_back(a);
    }
    sqlite3_reset(s);
    return out;
}

std::vector<XappRecord>
OranNtnSqliteDataRepository::GetXappRecords(double tStart, double tEnd) const
{
    std::vector<XappRecord> out;
    auto* s = m_impl->queryXapp;
    sqlite3_bind_double(s, 1, tStart);
    sqlite3_bind_double(s, 2, tEnd);
    while (sqlite3_step(s) == SQLITE_ROW)
    {
        XappRecord r{};
        r.timestamp = sqlite3_column_double(s, 0);
        r.xappId = static_cast<uint32_t>(sqlite3_column_int64(s, 1));
        const unsigned char* nm = sqlite3_column_text(s, 2);
        r.xappName = nm ? reinterpret_cast<const char*>(nm) : "";
        r.decision = static_cast<uint8_t>(sqlite3_column_int(s, 3));
        r.confidence = sqlite3_column_double(s, 4);
        r.latencyMs = sqlite3_column_double(s, 5);
        out.push_back(r);
    }
    sqlite3_reset(s);
    return out;
}

namespace
{

size_t
CountFromStmt(sqlite3_stmt* s)
{
    size_t n = 0;
    if (sqlite3_step(s) == SQLITE_ROW)
    {
        n = static_cast<size_t>(sqlite3_column_int64(s, 0));
    }
    sqlite3_reset(s);
    return n;
}

} // namespace

size_t
OranNtnSqliteDataRepository::CountKpmReports() const
{
    return CountFromStmt(m_impl->countKpm);
}

size_t
OranNtnSqliteDataRepository::CountRcActions() const
{
    return CountFromStmt(m_impl->countRc);
}

size_t
OranNtnSqliteDataRepository::CountXappRecords() const
{
    return CountFromStmt(m_impl->countXapp);
}

#endif // HAVE_SQLITE3

} // namespace ns3
