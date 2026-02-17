// static/app.js
const DateTime = luxon.DateTime;

const el = (id) => document.getElementById(id);

// Charts
let gasChart, envChart;
function initCharts() {
  const common = {
    responsive: true,
    maintainAspectRatio: false,
    scales: {
      x: { type: 'time', time: { unit: 'minute' }, grid: { color: 'rgba(255,255,255,.06)' }, ticks: { color: '#9fb0c9' } },
      y: { grid: { color: 'rgba(255,255,255,.06)' }, ticks: { color: '#9fb0c9' } }
    },
    plugins: {
      legend: { labels: { color: '#cfe3ff' } },
      tooltip: { mode: 'index', intersect: false }
    }
  };

  gasChart = new Chart(document.getElementById('gasChart').getContext('2d'), {
    type: 'line',
    data: { datasets: [
      { label: 'MQ2', data: [], borderColor: '#60a5fa', tension: .2 },
      { label: 'MQ4', data: [], borderColor: '#34d399', tension: .2 }
    ]},
    options: common
  });

  envChart = new Chart(document.getElementById('envChart').getContext('2d'), {
    type: 'line',
    data: { datasets: [
      { label: 'Temp °C', data: [], borderColor: '#fbbf24', tension: .2, yAxisID: 'y' },
      { label: 'Humidity %', data: [], borderColor: '#a78bfa', tension: .2, yAxisID: 'y1' },
      { label: 'Pressure hPa', data: [], borderColor: '#f472b6', tension: .2, yAxisID: 'y2' },
    ]},
    options: {
      ...common,
      scales: {
        x: common.scales.x,
        y: { ...common.scales.y, position: 'left' },
        y1: { ...common.scales.y, position: 'right', grid: { display: false } },
        y2: { ...common.scales.y, position: 'right', grid: { display: false }, ticks: { display: false } },
      }
    }
  });
}

function msFromTs(ts) { return (Number(ts) || 0) * 1000; }

function updateCards(d) {
  el('kpiTemp').textContent = (Number(d.temperature)).toFixed(1);
  el('kpiPres').textContent = (Number(d.pressure)).toFixed(1);
  el('kpiHum').textContent  = Number(d.humidity);
  el('kpiIr').textContent   = Number(d.status);

  el('gpsText').textContent = `${Number(d.latitude).toFixed(6)}, ${Number(d.longitude).toFixed(6)}`;
  el('gpsLink').href = `https://maps.google.com/?q=${d.latitude},${d.longitude}`;
  el('gpsLink').textContent = 'Open in Google Maps';

  el('lastUpdate').textContent = DateTime.fromMillis(msFromTs(d.timestamp), {zone:'utc'}).toFormat('yyyy-LL-dd HH:mm:ss');
  el('statusBadge').className = 'badge bg-success';
  el('statusBadge').textContent = 'online';
}

function fillCharts(history) {
  const gasMQ2 = history.map(r => ({ x: msFromTs(r.timestamp), y: r.mq2 }));
  const gasMQ4 = history.map(r => ({ x: msFromTs(r.timestamp), y: r.mq4 }));
  const t = history.map(r => ({ x: msFromTs(r.timestamp), y: r.temperature }));
  const h = history.map(r => ({ x: msFromTs(r.timestamp), y: r.humidity }));
  const p = history.map(r => ({ x: msFromTs(r.timestamp), y: r.pressure }));

  gasChart.data.datasets[0].data = gasMQ2;
  gasChart.data.datasets[1].data = gasMQ4;
  gasChart.update('none');

  envChart.data.datasets[0].data = t;
  envChart.data.datasets[1].data = h;
  envChart.data.datasets[2].data = p;
  envChart.update('none');
}

let dataTable;
function fillTable(history) {
  const tbody = document.querySelector("#historyTable tbody");
  tbody.innerHTML = "";
  history.slice().reverse().forEach(r => {
    const dt = DateTime.fromMillis(msFromTs(r.timestamp), {zone:'utc'}).toFormat('yyyy-LL-dd HH:mm:ss');
    const tr = document.createElement('tr');
    tr.innerHTML = `
      <td>${dt}</td>
      <td>${r.mq2}</td>
      <td>${r.mq4}</td>
      <td>${Number(r.temperature).toFixed(1)}</td>
      <td>${r.humidity}</td>
      <td>${Number(r.pressure).toFixed(1)}</td>
      <td>${Number(r.latitude).toFixed(6)}</td>
      <td>${Number(r.longitude).toFixed(6)}</td>
      <td>${r.status}</td>`;
    tbody.appendChild(tr);
  });

  // upgrade to sortable table (once)
  if (!dataTable) {
    dataTable = new simpleDatatables.DataTable("#historyTable", {
      searchable: true,
      fixedHeight: true,
      perPage: 10
    });
  } else {
    dataTable.refresh();
  }
}

async function loadFirst() {
  try {
    const res = await fetch("/api/history?limit=1000");
    const j = await res.json();
    if (!j.ok) throw new Error("history not ok");
    fillCharts(j.data);
    fillTable(j.data);
    if (j.data.length) updateCards(j.data[j.data.length-1]);
  } catch(e) {
    console.error(e);
  }
}

async function pollLatest() {
  try {
    const res = await fetch("/api/latest", { cache: "no-store" });
    const j = await res.json();
    if (!j.ok || !j.data) {
      el('statusBadge').className = 'badge bg-secondary';
      el('statusBadge').textContent = 'offline';
      return;
    }

    updateCards(j.data);

    // Append to charts/table in-place
    const pointX = msFromTs(j.data.timestamp);
    gasChart.data.datasets[0].data.push({ x: pointX, y: j.data.mq2 });
    gasChart.data.datasets[1].data.push({ x: pointX, y: j.data.mq4 });
    gasChart.update('none');

    envChart.data.datasets[0].data.push({ x: pointX, y: j.data.temperature });
    envChart.data.datasets[1].data.push({ x: pointX, y: j.data.humidity });
    envChart.data.datasets[2].data.push({ x: pointX, y: j.data.pressure });
    envChart.update('none');

    // Table: prepend a row (cap 1000 rows)
    const tbody = document.querySelector("#historyTable tbody");
    const dt = DateTime.fromMillis(pointX, {zone:'utc'}).toFormat('yyyy-LL-dd HH:mm:ss');
    const tr = document.createElement('tr');
    tr.innerHTML = `
      <td>${dt}</td>
      <td>${j.data.mq2}</td>
      <td>${j.data.mq4}</td>
      <td>${Number(j.data.temperature).toFixed(1)}</td>
      <td>${j.data.humidity}</td>
      <td>${Number(j.data.pressure).toFixed(1)}</td>
      <td>${Number(j.data.latitude).toFixed(6)}</td>
      <td>${Number(j.data.longitude).toFixed(6)}</td>
      <td>${j.data.status}</td>`;
    tbody.prepend(tr);
    if (tbody.children.length > 1000) tbody.removeChild(tbody.lastChild);
    if (window.dataTable) dataTable.refresh();
  } catch (e) {
    console.warn("poll failed", e);
    el('statusBadge').className = 'badge bg-warning';
    el('statusBadge').textContent = 'degraded';
  }
}

// boot
document.addEventListener("DOMContentLoaded", async () => {
  initCharts();
  await loadFirst();
  setInterval(pollLatest, 2500);
});
