// CanSat Ground Station Application
// Author: Sky's Rift Team
// Description: Real-time telemetry system with Serial connection support

// ========================================
// GLOBAL STATE
// ========================================
let port = null;
let reader = null;
let readingSerial = false;
let mapInitialized = false;
let map = null;
let markerCount = 0;
let runtime = 0;
let chart = null;

// Data storage
const logData = ["timestamp,cmd,module,data1,data2,data3,data4,data5,data6"];
const chartData = {
    labels: [],
    temp: [],
    pressure: [],
    altitude: [],
    ax: [],
    ay: [],
    az: []
};

// ========================================
// INITIALIZATION
// ========================================
document.addEventListener('DOMContentLoaded', () => {
    appendLog('System initialized');
    startClock();
    startRuntimeCounter();
    initializeChart();
    
    // Initialize map with default location
    const defaultLat = 14.543041;
    const defaultLon = 100.910398;
    initMap(defaultLat, defaultLon);
});

// ========================================
// MAP FUNCTIONS
// ========================================
function initMap(lat, lon) {
    if (mapInitialized) return;
    
    mapInitialized = true;
    map = L.map('map', { 
        center: [lat, lon], 
        zoom: 16,
        zoomControl: true 
    });
    
    // Satellite layer
    L.tileLayer('http://{s}.google.com/vt/lyrs=s&x={x}&y={y}&z={z}', {
        maxZoom: 20,
        attribution: "Sky's Rift CanSat",
        subdomains: ['mt0', 'mt1', 'mt2', 'mt3']
    }).addTo(map);
    
    // Add initial marker
    addMarker(lat, lon, 0);
    updateCoordinates(lat, lon);
    appendLog(`Map initialized at ${lat.toFixed(6)}, ${lon.toFixed(6)}`);
}

function addMarker(lat, lon, time) {
    if (!mapInitialized) return;
    
    markerCount++;
    const marker = L.marker([lat, lon]).addTo(map);
    marker.bindPopup(`
        <strong>Position ${markerCount}</strong><br>
        Time: ${time.toFixed(1)}s<br>
        Lat: ${lat.toFixed(6)}<br>
        Lon: ${lon.toFixed(6)}
    `).openPopup();
    
    map.panTo([lat, lon]);
    document.getElementById('positionCount').textContent = markerCount;
}

function updateCoordinates(lat, lon) {
    document.getElementById('currentLat').textContent = lat.toFixed(6);
    document.getElementById('currentLon').textContent = lon.toFixed(6);
}

// ========================================
// CHART FUNCTIONS
// ========================================
function initializeChart() {
    const ctx = document.getElementById('sensorChart').getContext('2d');
    
    chart = new Chart(ctx, {
        type: 'line',
        data: {
            labels: chartData.labels,
            datasets: [
                {
                    label: 'Temperature (°C)',
                    data: chartData.temp,
                    borderColor: '#ff6384',
                    backgroundColor: 'rgba(255, 99, 132, 0.1)',
                    tension: 0.4,
                    yAxisID: 'y'
                },
                {
                    label: 'Pressure (hPa)',
                    data: chartData.pressure,
                    borderColor: '#36a2eb',
                    backgroundColor: 'rgba(54, 162, 235, 0.1)',
                    tension: 0.4,
                    yAxisID: 'y1'
                },
                {
                    label: 'Altitude (m)',
                    data: chartData.altitude,
                    borderColor: '#4bc0c0',
                    backgroundColor: 'rgba(75, 192, 192, 0.1)',
                    tension: 0.4,
                    yAxisID: 'y2'
                }
            ]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            interaction: {
                mode: 'index',
                intersect: false
            },
            plugins: {
                legend: {
                    labels: {
                        color: '#e8eefc',
                        font: { family: 'Prompt' }
                    }
                }
            },
            scales: {
                x: {
                    ticks: { color: '#94a3c7' },
                    grid: { color: 'rgba(255,255,255,0.05)' }
                },
                y: {
                    type: 'linear',
                    position: 'left',
                    ticks: { color: '#ff6384' },
                    grid: { color: 'rgba(255,255,255,0.05)' }
                },
                y1: {
                    type: 'linear',
                    position: 'right',
                    ticks: { color: '#36a2eb' },
                    grid: { display: false }
                },
                y2: {
                    type: 'linear',
                    position: 'right',
                    ticks: { color: '#4bc0c0' },
                    grid: { display: false }
                }
            }
        }
    });
}

function updateChart(time, temp, pressure, altitude) {
    const timeStr = time.toFixed(1) + 's';
    
    // Keep only last 50 points
    if (chartData.labels.length > 50) {
        chartData.labels.shift();
        chartData.temp.shift();
        chartData.pressure.shift();
        chartData.altitude.shift();
    }
    
    chartData.labels.push(timeStr);
    chartData.temp.push(temp);
    chartData.pressure.push(pressure);
    chartData.altitude.push(altitude);
    
    chart.update('none');
}

// ========================================
// SERIAL CONNECTION
// ========================================
async function connectSerial() {
    if (!('serial' in navigator)) {
        appendLog('ERROR: Web Serial API not supported. Use Chrome/Edge browser.');
        alert('Web Serial API is not supported in this browser. Please use Chrome or Edge.');
        return;
    }

    try {
        appendLog('Requesting serial port...');
        port = await navigator.serial.requestPort();
        await port.open({ baudRate: 115200 });

        // Update UI
        setConnectionStatus(true);
        appendLog('Serial connected successfully');

        // Start reading
        const decoder = new TextDecoderStream();
        port.readable.pipeTo(decoder.writable);
        reader = decoder.readable.getReader();
        readingSerial = true;

        readSerialLoop();

    } catch (err) {
        appendLog('Serial connection failed: ' + err.message);
        setConnectionStatus(false);
    }
}

async function readSerialLoop() {
    let buffer = '';
    
    try {
        while (readingSerial && reader) {
            const { value, done } = await reader.read();
            
            if (done) {
                appendLog('Serial reader closed');
                break;
            }

            buffer += value;
            const lines = buffer.split(/\r?\n/);
            buffer = lines.pop() || '';

            for (const line of lines) {
                if (line.trim()) {
                    handleSerialData(line.trim());
                }
            }
        }
    } catch (err) {
        appendLog('Serial read error: ' + err.message);
        setConnectionStatus(false);
    }
}

function handleSerialData(line) {
    try {
        // Try to parse as JSON
        const data = JSON.parse(line);
        
        // Log raw data
        appendLog('RX: ' + line);
        
        // Expected format: [cmd, time, moduleData]
        // cmd 0 = Module data
        // cmd 1 = Status data
        
        if (Array.isArray(data) && data.length >= 2) {
            const cmd = data[0];
            const time = data[1] || runtime;
            
            if (cmd === 0 && data[2]) {
                // Module data
                processModuleData(data[2], time);
            } else if (cmd === 1 && data[2]) {
                // Status data
                processStatusData(data[2]);
            }
            
            // Log to CSV
            logToCSV(data);
        } else if (data.cmd !== undefined) {
            // Alternative format from RainScout: {cmd, time, type, data}
            const time = data.time || runtime;
            if (data.data && Array.isArray(data.data)) {
                // Assume it's BMP280-like data
                processModuleData([0, ...data.data], time);
            }
        }
        
        updateLastUpdate();
        
    } catch (err) {
        // Not valid JSON, log as text
        appendLog('Non-JSON: ' + line);
    }
}

// ========================================
// DATA PROCESSING
// ========================================
function processModuleData(moduleData, time) {
    if (!Array.isArray(moduleData) || moduleData.length < 2) return;
    
    const moduleType = moduleData[0];
    
    switch (moduleType) {
        case 0: // BMP280
            if (moduleData.length >= 4) {
                const temp = parseFloat(moduleData[1]);
                const pressure = parseFloat(moduleData[2]);
                const altitude = parseFloat(moduleData[3]);
                
                document.getElementById('temp').textContent = temp.toFixed(2);
                document.getElementById('pressure').textContent = pressure.toFixed(2);
                document.getElementById('bmpAlt').textContent = altitude.toFixed(2);
                
                updateChart(time, temp, pressure, altitude);
                appendLog(`BMP280: ${temp.toFixed(1)}°C, ${pressure.toFixed(1)}hPa, ${altitude.toFixed(1)}m`);
            }
            break;
            
        case 1: // GY-521 (Accelerometer + Gyroscope)
            if (moduleData.length >= 7) {
                document.getElementById('ax').textContent = parseFloat(moduleData[1]).toFixed(2);
                document.getElementById('ay').textContent = parseFloat(moduleData[2]).toFixed(2);
                document.getElementById('az').textContent = parseFloat(moduleData[3]).toFixed(2);
                document.getElementById('gx').textContent = parseFloat(moduleData[4]).toFixed(2);
                document.getElementById('gy').textContent = parseFloat(moduleData[5]).toFixed(2);
                document.getElementById('gz').textContent = parseFloat(moduleData[6]).toFixed(2);
                
                appendLog(`GY-521: ax=${moduleData[1]}, ay=${moduleData[2]}, az=${moduleData[3]}`);
            }
            break;
            
        case 2: // GPS
            if (moduleData.length >= 4) {
                const lat = parseFloat(moduleData[1]);
                const lon = parseFloat(moduleData[2]);
                const alt = parseFloat(moduleData[3]);
                
                document.getElementById('gpsLat').textContent = lat.toFixed(6);
                document.getElementById('gpsLon').textContent = lon.toFixed(6);
                document.getElementById('gpsAlt').textContent = alt.toFixed(2);
                
                addMarker(lat, lon, time);
                updateCoordinates(lat, lon);
                
                appendLog(`GPS: ${lat.toFixed(6)}, ${lon.toFixed(6)}, ${alt.toFixed(1)}m`);
            }
            break;
    }
}

function processStatusData(statusData) {
    if (!Array.isArray(statusData) || statusData.length < 1) return;
    
    const status = statusData[0];
    
    switch (status) {
        case 1: // Launch
            document.getElementById('statusLaunch').textContent = '🟢';
            document.getElementById('statusFlight').textContent = '🟡';
            appendLog('Status: LAUNCH');
            break;
        case 2: // In Flight
            document.getElementById('statusFlight').textContent = '🟢';
            appendLog('Status: IN FLIGHT');
            break;
        case 3: // Landed
            document.getElementById('statusFlight').textContent = '🟢';
            document.getElementById('statusLanded').textContent = '🟢';
            appendLog('Status: LANDED');
            break;
        case 5: // Deployed
            document.getElementById('statusDeploy').textContent = '🟢';
            appendLog('Status: DEPLOYED');
            break;
    }
}

// ========================================
// LOGGING FUNCTIONS
// ========================================
function logToCSV(data) {
    const timestamp = new Date().toISOString();
    
    if (!Array.isArray(data) || data.length < 2) return;
    
    const cmd = data[0];
    const moduleData = data[2] || [];
    
    if (cmd === 0 && Array.isArray(moduleData)) {
        const moduleType = moduleData[0];
        let moduleName = 'UNKNOWN';
        
        switch (moduleType) {
            case 0: moduleName = 'BMP280'; break;
            case 1: moduleName = 'GY-521'; break;
            case 2: moduleName = 'GPS'; break;
        }
        
        const dataStr = moduleData.slice(1).join(',');
        logData.push(`${timestamp},${cmd},${moduleName},${dataStr}`);
    } else if (cmd === 1) {
        logData.push(`${timestamp},${cmd},STATUS,${moduleData[0] || 0}`);
    }
}

function appendLog(message) {
    const logBox = document.getElementById('logBox');
    const timestamp = new Date().toLocaleTimeString();
    const entry = `[${timestamp}] ${message}\n`;
    
    logBox.textContent = entry + logBox.textContent;
    
    // Keep only last 100 lines
    const lines = logBox.textContent.split('\n');
    if (lines.length > 100) {
        logBox.textContent = lines.slice(0, 100).join('\n');
    }
}

// ========================================
// DOWNLOAD FUNCTIONS
// ========================================
function downloadLog() {
    const csvContent = logData.join('\n');
    const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' });
    const url = URL.createObjectURL(blob);
    
    const a = document.createElement('a');
    a.href = url;
    a.download = `cansat_log_${Date.now()}.csv`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
    
    appendLog('Downloaded full log');
}

function downloadModuleLog(moduleName) {
    const filteredData = logData.filter(line => line.includes(moduleName));
    
    if (filteredData.length <= 1) {
        appendLog(`No data for ${moduleName}`);
        return;
    }
    
    const csvContent = filteredData.join('\n');
    const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' });
    const url = URL.createObjectURL(blob);
    
    const a = document.createElement('a');
    a.href = url;
    a.download = `${moduleName}_log_${Date.now()}.csv`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
    
    appendLog(`Downloaded ${moduleName} log`);
}

function clearLogs() {
    if (confirm('Are you sure you want to clear all logs?')) {
        logData.length = 1; // Keep header
        document.getElementById('logBox').textContent = '';
        appendLog('All logs cleared');
    }
}

// ========================================
// UI UPDATES
// ========================================
function setConnectionStatus(connected) {
    const statusEl = document.getElementById('connectionStatus');
    const dotEl = document.getElementById('connectionDot');
    
    if (connected) {
        statusEl.textContent = 'CONNECTED';
        statusEl.classList.remove('disconnected');
        dotEl.style.background = 'var(--good)';
        dotEl.style.boxShadow = '0 0 10px var(--good), 0 0 30px var(--good)';
    } else {
        statusEl.textContent = 'DISCONNECTED';
        statusEl.classList.add('disconnected');
        dotEl.style.background = 'var(--bad)';
        dotEl.style.boxShadow = '0 0 10px var(--bad), 0 0 30px var(--bad)';
    }
}

function updateLastUpdate() {
    const now = new Date().toLocaleTimeString();
    document.getElementById('lastUpdate').textContent = now;
}

function startClock() {
    function updateClock() {
        // Clock is now in lastUpdate field
    }
    setInterval(updateClock, 1000);
}

function startRuntimeCounter() {
    setInterval(() => {
        runtime++;
        document.getElementById('runtime').textContent = runtime + ' s';
    }, 1000);
}

// ========================================
// TEST DATA GENERATOR (for development)
// ========================================
function generateTestData() {
    const testInterval = setInterval(() => {
        // Simulate BMP280 data
        const bmpData = [
            0, // cmd
            runtime, // time
            [0, 25 + Math.random() * 5, 1010 + Math.random() * 20, 100 + Math.random() * 50]
        ];
        handleSerialData(JSON.stringify(bmpData));
        
        // Simulate GY-521 data
        const gyData = [
            0,
            runtime,
            [1, Math.random() * 2 - 1, Math.random() * 2 - 1, Math.random() * 2 - 1,
                Math.random() * 10 - 5, Math.random() * 10 - 5, Math.random() * 10 - 5]
        ];
        handleSerialData(JSON.stringify(gyData));
        
        // Simulate GPS data (moving)
        const gpsData = [
            0,
            runtime,
            [2, 14.543 + Math.random() * 0.001, 100.910 + Math.random() * 0.001, 150 + Math.random() * 10]
        ];
        handleSerialData(JSON.stringify(gpsData));
        
    }, 2000);
    
    appendLog('Test data generator started');
}

// Uncomment to enable test data:
// setTimeout(generateTestData, 2000);