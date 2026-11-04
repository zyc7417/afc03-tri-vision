(() => {
  'use strict';
  const byId = id => document.getElementById(id);
  const nodes = {
    badge: byId('state-badge'), state: byId('state-label'), connection: byId('connection-label'),
    message: byId('status-message'), error: byId('error-message'), fps: byId('fps-value'),
    frames: byId('frame-count'), targets: byId('detection-count'), uptime: byId('uptime-value'),
    run: byId('run-id'), image: byId('preview-image'), placeholder: byId('preview-placeholder'),
    placeholderTitle: byId('placeholder-title'), placeholderMessage: byId('placeholder-message'),
    previewLabel: byId('preview-label'), warning: byId('preview-warning'), live: byId('live-indicator'),
    title: byId('view-title'), start: byId('start-button'), stop: byId('stop-button'),
    previewFps: byId('preview-fps-value'), previewRateNote: byId('preview-rate-note'),
    shutdown: byId('shutdown-button'), modal: byId('shutdown-modal'),
    cancel: byId('cancel-shutdown'), confirm: byId('confirm-shutdown'), fullscreen: byId('fullscreen-button'),
  };
  const states = {idle:'待机', starting:'正在启动', switching:'正在接续', running:'运行中', stopping:'正在停止', fault:'运行异常'};
  const views = new Set(['mosaic', 'A0', 'A1', 'B0']);
  const viewButtons = [...document.querySelectorAll('[data-view]')];
  let status = null, connected = false, actionPending = false, shuttingDown = false;
  let selectedView = 'mosaic', previewBusy = false, previewTimer = null, previewGeneration = 0;
  let previewUrl = null, requestController = null, hasPreview = false, actionError = '';
  let statusObservedAt = 0, previewCapturedAt = null, previewRunId = null, pauseTimer = null, previewSuppressed = false;
  let lastSequence = null, lastOrdinal = null, lastSourceFrame = null, lastEpoch = null;
  let decodedTimes = [], decodedTotal = 0, metricRenderedAt = 0;
  let rawRenderer = null, latestPending = null, drawRaf = null;
  let fetchedSequence=null,fetchedOrdinal=null,fetchedFrame=null,fetchedEpoch=null;
  const setText = (node, value) => { const text = String(value); if (node.textContent !== text) node.textContent = text; };
  const finiteNumber = value => typeof value === 'number' && Number.isFinite(value) && value >= 0;
  const frameNumber = value => finiteNumber(value) ? Math.floor(value).toLocaleString('zh-CN') : '—';
  const elapsed = value => {
    if (!finiteNumber(value)) return '—';
    const seconds = Math.floor(value), hours = Math.floor(seconds / 3600);
    const minutes = Math.floor(seconds / 60) % 60, remaining = seconds % 60;
    return hours ? `${hours}:${String(minutes).padStart(2,'0')}:${String(remaining).padStart(2,'0')}` : `${minutes}:${String(remaining).padStart(2,'0')}`;
  };
  const showError = value => {
    const text = typeof value === 'string' ? value : '';
    nodes.error.hidden = !text; setText(nodes.error, text);
  };
  const warning = value => { nodes.warning.hidden = !value; setText(nodes.warning, value || ''); };
  const previewHz = () => status?.preview_mode === 'raw' && finiteNumber(status.preview_max_hz) && status.preview_max_hz > 0 ? Math.min(60,status.preview_max_hz) : 8;
  const uintHeader = (headers,name,limit) => {
    const value = headers.get(name);
    if (!value || !/^[0-9]{1,20}$/.test(value)) throw new Error('预览帧标识缺失或无效');
    const parsed = BigInt(value);
    if (parsed > limit) throw new Error('预览帧标识超出范围');
    return parsed;
  };

const rawMetrics={state:()=>{},clear:()=>{}};
const nativePage=crypto.randomUUID();let nativeSerial=0;
function nativeRect(){
 const b=document.getElementById('preview-stage').getBoundingClientRect(),d=devicePixelRatio;
 const x=Math.max(0,Math.round((screenX+b.left)*d)),y=Math.max(0,Math.round((screenY+outerHeight-innerHeight+b.top)*d));
 const w=Math.max(1,Math.min(960,1024-x,Math.floor(b.width*d))),h=Math.max(1,Math.min(540,600-y,Math.floor(b.height*d)));
 return [x,y,w,h];
}
async function nativeSend(visible){
 if(!status?.csrf_token)return null;
 const controller=new AbortController(),timeout=setTimeout(()=>controller.abort(),1500);
 try{
  const r=await fetch('/api/native-panel',{method:'POST',headers:{'Content-Type':'application/json','X-CSRF-Token':status.csrf_token},
   body:JSON.stringify({page:nativePage,serial:++nativeSerial,run:status.run_id,view:selectedView,visible,rect:nativeRect()}),signal:controller.signal});
  const body=await r.json();if(!r.ok)throw Error(body.error||'显示服务未就绪');return body;
 }finally{clearTimeout(timeout);}
}
function nativeHide(){void nativeSend(false).catch(()=>{});}

  function resetPreviewIdentity() {
    lastSequence = lastOrdinal = lastSourceFrame = lastEpoch = null;
    fetchedSequence=fetchedOrdinal=fetchedFrame=fetchedEpoch=null; latestPending=null;
    decodedTimes = []; decodedTotal = 0; renderDecodedRate(true);
  }
  function renderDecodedRate() {
 const n=status?.native_display,ready=connected&&!previewSuppressed&&status?.state==='running'&&n?.visible&&n?.run_id===status.run_id&&!n.error;
 setText(nodes.previewFps,ready&&finiteNumber(n.window_fps)?n.window_fps.toFixed(1):'—');
 nodes.previewFps.dataset.uniqueFrames=String(n?.completed||0);
 setText(nodes.previewRateNote,'近2秒不同源帧显示完成 · 非面板物理刷新');
 }

  function updateControls() {
    const ready = connected && !actionPending && !shuttingDown && typeof status?.csrf_token === 'string' && status.csrf_token.length > 0;
    nodes.start.disabled = !ready || status?.can_start !== true;
    nodes.stop.disabled = !ready || status?.can_stop !== true;
    nodes.shutdown.disabled = !ready || status?.can_shutdown !== true;
    viewButtons.forEach(button => { button.disabled = !ready; });
    nodes.confirm.disabled = !ready || status?.can_shutdown !== true;
  }

  function displayView(view) {
    selectedView = view;
    setText(nodes.title, view === 'mosaic' ? '三路总览' : `${view} · 单路放大`);
    viewButtons.forEach(button => {
      const selected = button.dataset.view === view;
      button.classList.toggle('active', selected);
      button.setAttribute('aria-pressed', String(selected));
    });
  }

  function renderStatus(next) {
    if (status && status.run_id !== next.run_id) { cancelPreview(true); warning(''); previewSuppressed = false; resetPreviewIdentity(); }
    status = next; connected = true; statusObservedAt = performance.now();
    rawMetrics.state(next.state,next.run_id,next.csrf_token);
    const state = Object.hasOwn(states, next.state) ? next.state : 'fault';
    nodes.badge.dataset.state = state; setText(nodes.state, states[state]);
    setText(nodes.connection, '板端已连接');
    setText(nodes.message, typeof next.message === 'string' && next.message ? next.message : states[state]);
    setText(nodes.fps, state === 'running' && finiteNumber(next.fps) ? next.fps.toFixed(1) : '—');
    setText(nodes.frames, frameNumber(next.frame_count));
    const detections = Array.isArray(next.detections) ? next.detections.length : next.detections;
    setText(nodes.targets, frameNumber(detections)); setText(nodes.uptime, elapsed(next.uptime_s));
    setText(nodes.run, next.run_id ? `运行：${String(next.run_id)}` : '尚未启动');
    const active = Array.isArray(next.active_cameras) ? next.active_cameras : [];
    document.querySelectorAll('[data-camera]').forEach(node => {
      node.classList.toggle('active-camera', active.includes(node.dataset.camera));
      node.title = active.includes(node.dataset.camera) ? '已配置为参与采集，不代表单独测得接收状态' : '未参与采集';
    });
    if (views.has(next.view) && !actionPending) displayView(next.view);
    showError(actionError || (typeof next.last_error === 'string' ? next.last_error : ''));
    presentPreview();
    renderDecodedRate(true);
    updateControls();
    if (state === 'running' && !shuttingDown) schedulePreview(0);
  }

  function presentPreview() {
    clearTimeout(pauseTimer); pauseTimer = null;
    const state = status?.state, transition = state === 'starting' || state === 'switching';
    const sameRun = hasPreview && typeof status?.run_id === 'string' && status.run_id.length > 0 && previewRunId === status.run_id;
    const live = connected && !shuttingDown && !previewSuppressed && state === 'running' && (status?.preview_available === true || sameRun);
    if (sameRun && previewCapturedAt !== null && performance.now()-previewCapturedAt >= 3000) cancelPreview(true);
    nodes.live.classList.toggle('active', live && sameRun && hasPreview);
    if (live) {
      setText(nodes.previewLabel, sameRun && hasPreview ? '实时图像 · NPU 结果' : '等待新图像');
      warning('');
      return;
    }
    // A loaded image may be older than the newest server JPEG. Both measured
    // ages must be fresh; repeated status polls must never extend this deadline.
    const deadline = sameRun && previewCapturedAt !== null && finiteNumber(status?.preview_age_s)
      ? Math.min(previewCapturedAt + 3000, statusObservedAt + 3000 - status.preview_age_s * 1000) : 0;
    if (connected && !shuttingDown && !previewSuppressed && transition && deadline > performance.now()) {
      cancelPreview(false);
      setText(nodes.previewLabel, '暂停画面 · 非实时');
      warning('画面暂停，正在接续');
      pauseTimer = setTimeout(presentPreview, Math.max(1, deadline - performance.now()));
      return;
    }
    cancelPreview(true); warning('');
    setText(nodes.previewLabel, transition ? '等待接续图像' : '等待图像');
    setText(nodes.placeholderTitle, transition ? '正在准备采集链路' : state === 'fault' ? '设备运行异常' : '准备开始采集');
    setText(nodes.placeholderMessage, transition ? '等待新的实时画面，请勿重复点击' : state === 'fault' ? '请查看下方错误说明' : '点击右侧「开始识别」，等待实时画面');
  }

  function offline(message) {
    connected = false; nodes.badge.dataset.state = 'offline';
    rawMetrics.state('offline',status?.run_id,status?.csrf_token);
    setText(nodes.state, '连接中断'); setText(nodes.connection, '等待恢复');
    setText(nodes.message, '暂时无法读取板端状态，正在自动重连…');
    showError(message); nodes.live.classList.remove('active');
    cancelPreview(true); warning('');
    setText(nodes.previewLabel, '连接中断 · 非实时'); renderDecodedRate(true); updateControls();
  }

  async function pollStatus() {
    if (shuttingDown) return;
    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), 2500);
    try {
      const response = await fetch('/api/status', {cache:'no-store', credentials:'same-origin', signal:controller.signal});
      if (!response.ok) throw new Error(`设备状态读取失败（HTTP ${response.status}）`);
      const next = await response.json();
      if (!next || typeof next !== 'object' || !Object.hasOwn(states, next.state)) throw new Error('设备状态格式不正确');
      if (!shuttingDown) renderStatus(next);
    } catch (error) {
      if (!shuttingDown) offline(error.name === 'AbortError' ? '设备状态请求超时' : error.message);
    } finally {
      clearTimeout(timeout);
      if (!shuttingDown) setTimeout(pollStatus, 500);
    }
  }

  function cancelPreview(clearImage) {
    clearTimeout(previewTimer); previewTimer = null;
    previewGeneration += 1; latestPending=null;
    if(drawRaf!==null)cancelAnimationFrame(drawRaf);drawRaf=null;
    if (requestController) requestController.abort();
    requestController = null;
    if (clearImage) {
      nativeHide();
      clearTimeout(pauseTimer); pauseTimer = null;
      nodes.image.hidden = true; nodes.placeholder.hidden = false; hasPreview = false; rawMetrics.clear();
      previewCapturedAt = null; previewRunId = null;
      if (rawRenderer) rawRenderer.clear();
      if (previewUrl) URL.revokeObjectURL(previewUrl);
      previewUrl = null;
    }
  }

  function schedulePreview(delay = 1000 / previewHz()) {
    if (previewTimer !== null || previewBusy || previewSuppressed || !connected || status?.state !== 'running' || shuttingDown) return;
    previewTimer = setTimeout(() => { previewTimer = null; refreshPreview(); }, Math.ceil(delay));
  }

  function queueDraw() {
    if(drawRaf!==null||!latestPending)return;
    drawRaf=requestAnimationFrame(()=>{
      drawRaf=null;const job=latestPending;latestPending=null;
      if(!job||job.generation!==previewGeneration||shuttingDown||previewSuppressed||!connected||status?.state!=='running'||status.run_id!==job.run||selectedView!==job.view||performance.now()-job.capturedAt>=3000)return;
      try{
        const i=job.identity;
        if(lastSequence!==null&&(i.sequence<=lastSequence||i.ordinal<=lastOrdinal||i.frame<=lastSourceFrame))return;
        if(lastEpoch!==null&&i.epoch!==lastEpoch)throw new Error('预览来源时段发生变化');
        if(!rawRenderer)rawRenderer=new RawRenderer(nodes.image);
        rawMetrics.stage('queue',performance.now()-job.readyAt);
        const uploadBefore=rawRenderer.uploadMs,drawBefore=rawRenderer.submitMs,errorBefore=rawRenderer.errorCheckMs,totalBefore=rawRenderer.totalMs;
        rawRenderer.draw(job.packet);
        rawMetrics.stage('upload',rawRenderer.uploadMs-uploadBefore);rawMetrics.stage('draw',rawRenderer.submitMs-drawBefore);rawMetrics.stage('error_check',rawRenderer.errorCheckMs-errorBefore);rawMetrics.stage('renderer_total',rawRenderer.totalMs-totalBefore);
        rawMetrics.draw(job.packet,rawRenderer);
        nodes.image.hidden=false;nodes.placeholder.hidden=true;hasPreview=true;
        previewCapturedAt=job.capturedAt;previewRunId=job.run;
        lastSequence=i.sequence;lastOrdinal=i.ordinal;lastSourceFrame=i.frame;lastEpoch=i.epoch;
        decodedTimes.push(performance.now());if(decodedTimes.length>120)decodedTimes.shift();decodedTotal++;
        presentPreview();renderDecodedRate();
      }catch(error){cancelPreview(true);nodes.live.classList.remove('active');warning(error.message);}
      if(latestPending)queueDraw();
    });
  }

  async function refreshPreview() {
 if(previewBusy||previewSuppressed||!connected||status?.state!=='running'||shuttingDown)return;
 previewBusy=true;const generation=previewGeneration;
 try{
  const visible=document.visibilityState==='visible'&&nodes.modal.hidden;
  const n=await nativeSend(visible);
  if(generation!==previewGeneration||!n)return;
  status.native_display=n;
  if(n.error)throw Error(n.error);
  if(visible&&n.visible&&n.run_id===status.run_id&&n.last?.view===selectedView&&n.last_completion_age_s<1){
   nodes.image.hidden=false;nodes.placeholder.hidden=true;hasPreview=true;
   previewRunId=status.run_id;previewCapturedAt=performance.now()-n.last_completion_age_s*1000;
   nodes.live.classList.add('active');setText(nodes.previewLabel,'实时图像 · NPU 结果');warning('');
  }
  renderDecodedRate();
 }catch(e){if(generation===previewGeneration)warning(e.message);}
 finally{previewBusy=false;schedulePreview(250);}
 }

  async function action(name, view = selectedView) {
    if (actionPending || shuttingDown || !connected || !status?.csrf_token) return false;
    actionPending = true; actionError = ''; showError(''); updateControls();
    const controller = new AbortController(); const timeout = setTimeout(() => controller.abort(), 15000);
    try {
      const response = await fetch('/api/action', {
        method:'POST', credentials:'same-origin', cache:'no-store', signal:controller.signal,
        headers:{'Content-Type':'application/json','X-CSRF-Token':status.csrf_token},
        body:JSON.stringify({action:name, view}),
      });
      const payload = await response.json().catch(() => ({}));
      if (!response.ok || payload.ok === false) throw new Error(typeof payload.error === 'string' ? payload.error : `操作未成功（HTTP ${response.status}）`);
      if (name === 'view') { cancelPreview(true); displayView(view); schedulePreview(0); }
      if (name === 'start') { cancelPreview(true); warning(''); previewSuppressed = false; resetPreviewIdentity(); setText(nodes.message, '开始请求已提交，等待设备进入运行状态…'); }
      if (name === 'stop') {
        rawMetrics.state('stopping',status.run_id,status.csrf_token);
        previewSuppressed = true; cancelPreview(true); warning(''); nodes.live.classList.remove('active');
        setText(nodes.message, '停止请求已提交，正在等待设备释放采集资源…');
      }
      if (name === 'shutdown') {
        rawMetrics.state('stopping',status.run_id,status.csrf_token);
        shuttingDown = true; cancelPreview(true); warning(''); nodes.modal.hidden = true;
        nodes.badge.dataset.state = 'stopping'; setText(nodes.state, '正在关机');
        setText(nodes.message, '关机请求已提交。请等待系统关机完成后再拔电。');
        setText(nodes.connection, '关机处理中'); nodes.live.classList.remove('active');
        setText(nodes.previewLabel, hasPreview ? '最后一帧 · 关机中' : '关机中');
      }
      return true;
    } catch (error) {
      actionError = error.name === 'AbortError' ? '操作响应超时，结果未知；请等待状态更新，不要重复点击或拔电。' : error.message;
      showError(actionError); return false;
    } finally { clearTimeout(timeout); actionPending = false; updateControls(); }
  }

  nodes.start.addEventListener('click', () => action('start'));
  nodes.stop.addEventListener('click', () => action('stop'));
  viewButtons.forEach(button => button.addEventListener('click', () => {
    if (button.dataset.view !== selectedView) action('view', button.dataset.view);
  }));
  nodes.shutdown.addEventListener('click', () => { nativeHide(); nodes.modal.hidden = false; nodes.cancel.focus(); });
  const dismissModal = () => { if (!actionPending) { nodes.modal.hidden = true; nodes.shutdown.focus(); } };
  nodes.cancel.addEventListener('click', dismissModal);
  nodes.confirm.addEventListener('click', () => action('shutdown'));
  nodes.modal.addEventListener('click', event => { if (event.target === nodes.modal) dismissModal(); });
  document.addEventListener('keydown', event => {
    if (nodes.modal.hidden) return;
    if (event.key === 'Escape') dismissModal();
    if (event.key === 'Tab') {
      event.preventDefault();
      if (document.activeElement === nodes.cancel && !nodes.confirm.disabled) nodes.confirm.focus(); else nodes.cancel.focus();
    }
  });
  nodes.fullscreen.addEventListener('click', () => {
    const expanded = document.querySelector('.console').classList.toggle('expanded-preview');
    setText(nodes.fullscreen, expanded ? '返回操作' : '放大画面');
    nodes.fullscreen.setAttribute('aria-pressed', String(expanded));
  });
  window.addEventListener('pagehide', () => { shuttingDown = true; cancelPreview(true); });
  document.addEventListener('visibilitychange',()=>{if(document.visibilityState!=='visible')nativeHide();});
  updateControls(); pollStatus();
})();
