(() => {
  'use strict';
  const byId = id => document.getElementById(id);
  const setText = (id, value) => { byId(id).textContent = String(value); };
  const finite = value => typeof value === 'number' && Number.isFinite(value);
  const format = (value, digits=0) => finite(value) ? value.toFixed(digits) : '—';
  const names = {idle:'Ready', starting:'Waiting for GPS', running:'Advice current', stale:'Road data delayed',
                 stopped:'Ride stopped', finished:'Ride complete', error:'SUMO error', offline:'Server offline'};
  let gpsWatch = null, latest = null, busy = false, lastAdvice = '';

  function roadFeature(id, feature) {
    const status = feature?.status || 'unknown';
    const ahead = status === 'ahead' && finite(feature.distance_m);
    setText(id, ahead ? format(feature.distance_m) : status === 'none' ? 'None' : status === 'inside' ? 'Crossing' : '—');
    setText(id+'Unit', ahead ? 'm ahead' : '');
    setText(id+'Detail', ahead ? 'On the known road ahead' : status === 'none' ? 'On the known road' :
            status === 'inside' ? 'Inside the junction' : 'Route ahead uncertain');
  }

  function render(data) {
    latest = data;
    const sample = data.telemetry, context = data.road_context || {}, advice = data.advice;
    setText('status', names[data.status] || 'Waiting');
    byId('connection').dataset.state = data.status;
    byId('advicePanel').dataset.level = advice.level || 'neutral';
    const adviceKey = [advice.code, advice.action, advice.reason].join('|');
    if (adviceKey !== lastAdvice) {
      setText('adviceAction', advice.action); setText('adviceReason', advice.reason);
      lastAdvice = adviceKey;
    }
    setText('adviceIcon', advice.level === 'good' ? '✓' : ['warning','caution'].includes(advice.level) ? '!' : '—');
    setText('adviceSource', sample ? data.source === 'demo' ? 'SUMO demo road conditions' : 'Current matched road conditions' : 'No current road data');
    setText('speed', format(finite(sample?.speed_mps) ? sample.speed_mps * 3.6 : null));
    setText('acceleration', finite(sample?.accel_mps2) ? (sample.accel_mps2 > 0 ? '+' : '') + sample.accel_mps2.toFixed(1) : '—');
    const angle = finite(sample?.angle_deg) && sample.angle_deg >= 0 ? sample.angle_deg % 360 : null;
    setText('heading', angle === null ? '—' : ['N','NE','E','SE','S','SW','W','NW'][Math.round(angle/45)%8]);
    setText('angle', angle === null ? '' : Math.round(angle)+'°');
    setText('limit', format(context.speed_limit_kmh));
    setText('limitDetail', context.limit_on_junction ? 'Current junction lane' : 'Current matched road');
    roadFeature('intersection', context.intersection);
    roadFeature('trafficLight', context.traffic_light);
    const signal = context.traffic_light?.signal;
    byId('signalState').hidden = !signal;
    setText('signalText', signal || '');
    byId('signalDot').dataset.color = signal?.startsWith('Red') ? 'red' : signal?.startsWith('Green') ? 'green' : signal === 'Yellow' ? 'yellow' : 'unknown';
    setText('sampleStatus', sample ? 'Telemetry current · Updated after the latest SUMO placement' : 'No current telemetry');
    setText('simulationTime', sample ? `SUMO time ${format(sample.time_s)} s · ${data.source === 'demo' ? 'Demo' : 'Live GPS'}` : '');
    setText('route', sample ? `Edge: ${sample.edge_id} · Lane: ${sample.lane_id || '—'} · Context: ${sample.route_source || 'unknown'}` : 'No current lane');
    const demoActive = data.source === 'demo' && ['starting','running','stale'].includes(data.status);
    byId('startGps').disabled = gpsWatch !== null || demoActive;
    byId('stopGps').disabled = gpsWatch === null;
    byId('startDemo').disabled = busy || demoActive || gpsWatch !== null;
    byId('stopDemo').disabled = busy || !demoActive;
  }

  function unavailable() {
    render({status:'offline', source:'live', telemetry:null, road_context:{},
            advice:{code:'offline',level:'neutral',action:'Advice unavailable',reason:'Connection to the feedback server was lost.'}});
  }

  async function refresh() {
    try {
      const response = await fetch('/api/feedback/status', {cache:'no-store', signal:AbortSignal.timeout(3000)});
      if (!response.ok) throw new Error('Status request failed');
      const data = await response.json();
      if (!data.advice || !data.road_context) throw new Error('Invalid response');
      render(data);
    } catch (_) { unavailable(); }
  }

  function startGps() {
    if (gpsWatch !== null) return;
    if (!navigator.geolocation) { setText('gpsStatus', 'This browser does not support GPS.'); return; }
    setText('gpsStatus', 'Waiting for a location fix…');
    gpsWatch = navigator.geolocation.watchPosition(async position => {
      const c = position.coords;
      const speed = finite(c.speed) && c.speed >= 0 ? c.speed : null;
      const heading = finite(c.heading) && c.heading >= 0 ? c.heading : null;
      const payload = {lat:c.latitude, lon:c.longitude, speed_mps:speed,
                       speed_kmh:speed === null ? null : speed * 3.6,
                       course_deg:heading, accuracy_m:c.accuracy,
                       timestamp:new Date(position.timestamp).toISOString()};
      try {
        const response = await fetch('/update', {method:'POST', headers:{'Content-Type':'application/json'},
                          body:JSON.stringify(payload), signal:AbortSignal.timeout(3000)});
        if (!response.ok) throw new Error(`GPS upload failed (${response.status})`);
        setText('gpsStatus', `GPS sent · accuracy ${format(c.accuracy)} m`);
      } catch (error) { setText('gpsStatus', error.message || 'GPS upload failed'); }
    }, error => { setText('gpsStatus', error.message || 'GPS unavailable'); },
    {enableHighAccuracy:true, maximumAge:0, timeout:10000});
    byId('startGps').disabled = true; byId('stopGps').disabled = false;
  }

  function stopGps() {
    if (gpsWatch !== null) navigator.geolocation.clearWatch(gpsWatch);
    gpsWatch = null;
    setText('gpsStatus', 'GPS stopped. Advice will clear when the last sample expires.');
    byId('startGps').disabled = false; byId('stopGps').disabled = true;
  }

  async function demo(action) {
    if (busy) return;
    if (action === 'start' && !byId('duration').checkValidity()) {
      byId('setup').open = true; byId('duration').reportValidity(); return;
    }
    busy = true; byId('error').hidden = true;
    try {
      const response = await fetch(`/api/feedback/demo/${action}`, {method:'POST',
        headers:{'Content-Type':'application/json'},
        body:JSON.stringify(action === 'start' ? {max_time:Number(byId('duration').value)} : {}),
        signal:AbortSignal.timeout(10000)});
      const body = await response.json();
      if (!response.ok) throw new Error(body.error || 'Could not control demo');
    } catch (error) { setText('error', error.message || 'Demo request failed'); byId('error').hidden = false; }
    finally { busy = false; await refresh(); }
  }

  byId('startGps').addEventListener('click', startGps);
  byId('stopGps').addEventListener('click', stopGps);
  byId('startDemo').addEventListener('click', () => demo('start'));
  byId('stopDemo').addEventListener('click', () => demo('stop'));
  document.addEventListener('visibilitychange', () => { if (!document.hidden) refresh(); });
  async function poll() { await refresh(); setTimeout(poll, 1000); }
  poll();
})();
