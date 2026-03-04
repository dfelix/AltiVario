// AltiVario WiFi Portal Configuration

document.addEventListener('DOMContentLoaded', () => {
    loadConfig();
    loadFiles();
});

function loadConfig() {
    fetch('/api/config')
        .then(r => r.json())
        .then(cfg => {
            document.getElementById('climbThreshold').value = cfg.climbThreshold;
            document.getElementById('sinkThreshold').value = cfg.sinkThreshold;
            document.getElementById('audioMode').value = cfg.audioMode;
            document.getElementById('volume').value = cfg.volume;
            document.getElementById('qnh').value = cfg.qnh;
            document.getElementById('bleEnabled').checked = cfg.bleEnabled === 1;
            document.getElementById('gpsEnabled').checked = cfg.gpsEnabled === 1;
            document.getElementById('sdEnabled').checked = cfg.sdEnabled === 1;
        })
        .catch(() => {
            showStatus('Failed to load config', 'error');
        });
}

function saveConfig() {
    const data = new URLSearchParams();
    data.append('climbThreshold', document.getElementById('climbThreshold').value);
    data.append('sinkThreshold', document.getElementById('sinkThreshold').value);
    data.append('audioMode', document.getElementById('audioMode').value);
    data.append('volume', document.getElementById('volume').value);
    data.append('qnh', document.getElementById('qnh').value);
    data.append('bleEnabled', document.getElementById('bleEnabled').checked ? '1' : '0');
    data.append('gpsEnabled', document.getElementById('gpsEnabled').checked ? '1' : '0');
    data.append('sdEnabled', document.getElementById('sdEnabled').checked ? '1' : '0');

    fetch('/api/config', {
        method: 'POST',
        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
        body: data.toString()
    })
    .then(r => r.json())
    .then(res => {
        if (res.ok) {
            showStatus('Configuration saved!', 'ok');
        }
    })
    .catch(() => {
        showStatus('Save failed', 'error');
    });
}

function loadFiles() {
    const container = document.getElementById('files');

    fetch('/api/files')
        .then(r => r.json())
        .then(files => {
            if (files.length === 0) {
                container.innerHTML = '<p style="color:#666">No flight logs found</p>';
                return;
            }

            container.innerHTML = files.map(f =>
                `<div class="file-item">
                    <span>${f}</span>
                    <a href="/api/download?f=${encodeURIComponent(f)}" download>Download</a>
                </div>`
            ).join('');
        })
        .catch(() => {
            container.innerHTML = '<p style="color:#ff6b6b">SD card not available</p>';
        });
}

function showStatus(msg, type) {
    // Remove existing status
    const existing = document.querySelector('.status');
    if (existing) existing.remove();

    const div = document.createElement('div');
    div.className = `status ${type}`;
    div.textContent = msg;
    document.querySelector('#config-form').appendChild(div);

    setTimeout(() => div.remove(), 3000);
}
