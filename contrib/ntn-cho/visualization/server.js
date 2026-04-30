/**
 * NTN-CHO 3D Visualization Server
 *
 * Serves the CesiumJS-based 3D globe visualization for LEO satellite
 * handover analysis. Reads simulation output data (CSV, GeoJSON) and
 * provides it to the browser for real-time 3D rendering.
 *
 * Author: Muhammad Uzair
 * License: GPL-2.0
 *
 * Usage: node server.js [--data <path-to-output-dir>] [--port <port>]
 */

const express = require('express');
const fs = require('fs');
const path = require('path');

const app = express();

// Parse command line args
const args = process.argv.slice(2);
let dataDir = path.resolve(__dirname, '../../../ntn-cho-output/tte-aware');
let port = 8080;

for (let i = 0; i < args.length; i++) {
    if (args[i] === '--data' && args[i+1]) dataDir = path.resolve(args[++i]);
    if (args[i] === '--port' && args[i+1]) port = parseInt(args[++i]);
}

console.log(`NTN-CHO 3D Visualizer`);
console.log(`Data directory: ${dataDir}`);
console.log(`Port: ${port}`);

// Serve static files
app.use(express.static(path.join(__dirname, 'public')));

// API: list available datasets
app.get('/api/datasets', (req, res) => {
    const parentDir = path.dirname(dataDir);
    try {
        const dirs = fs.readdirSync(parentDir).filter(d => {
            return fs.existsSync(path.join(parentDir, d, 'satellite_tracks.csv'));
        });
        res.json({ datasets: dirs, current: path.basename(dataDir) });
    } catch (e) {
        res.json({ datasets: [path.basename(dataDir)], current: path.basename(dataDir) });
    }
});

// API: serve CSV data as JSON
app.get('/api/data/:filename', (req, res) => {
    const filepath = path.join(dataDir, req.params.filename);
    if (!fs.existsSync(filepath)) {
        return res.status(404).json({ error: 'File not found: ' + req.params.filename });
    }

    const ext = path.extname(filepath);
    if (ext === '.geojson' || ext === '.json') {
        return res.sendFile(filepath);
    }

    // Parse CSV to JSON
    const content = fs.readFileSync(filepath, 'utf8');
    const lines = content.trim().split('\n');
    if (lines.length < 2) return res.json([]);

    const headers = lines[0].split(',');
    const data = [];
    for (let i = 1; i < lines.length; i++) {
        const values = lines[i].split(',');
        const row = {};
        headers.forEach((h, idx) => {
            const v = values[idx];
            row[h.trim()] = isNaN(v) ? v : parseFloat(v);
        });
        data.push(row);
    }
    res.json(data);
});

// API: serve GeoJSON directly
app.get('/api/geojson/:filename', (req, res) => {
    const filepath = path.join(dataDir, req.params.filename);
    if (!fs.existsSync(filepath)) {
        return res.status(404).json({ error: 'File not found' });
    }
    res.sendFile(filepath);
});

// API: KPI summary
app.get('/api/kpi', (req, res) => {
    const kpiFile = path.join(dataDir, 'kpi_summary.txt');
    if (!fs.existsSync(kpiFile)) {
        return res.status(404).json({ error: 'KPI file not found' });
    }
    const content = fs.readFileSync(kpiFile, 'utf8');
    const kpi = {};
    content.split('\n').forEach(line => {
        const match = line.match(/^\s*(.+?):\s+(.+)$/);
        if (match) {
            kpi[match[1].trim()] = match[2].trim();
        }
    });
    res.json(kpi);
});

// Switch dataset
app.get('/api/switch/:dataset', (req, res) => {
    const newDir = path.join(path.dirname(dataDir), req.params.dataset);
    if (fs.existsSync(newDir)) {
        dataDir = newDir;
        res.json({ success: true, dataset: req.params.dataset });
    } else {
        res.status(404).json({ error: 'Dataset not found' });
    }
});

app.listen(port, () => {
    console.log(`\n  Open in browser: http://localhost:${port}\n`);
});
