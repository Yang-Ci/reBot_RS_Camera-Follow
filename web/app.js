(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  $('joints').innerHTML = Array.from({length: 6}, (_, i) => `<div class="joint"><span>J0${i+1}</span><div class="bar"><i id="bar${i}"></i></div><b id="joint${i}">—</b></div>`).join('');
  const host = $('scene'), scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(38, 1, .01, 100);
  const renderer = new THREE.WebGLRenderer({antialias:true, alpha:true});
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2)); renderer.outputEncoding = THREE.sRGBEncoding; host.appendChild(renderer.domElement);
  scene.add(new THREE.HemisphereLight(0xe4f0de,0x40483e,1.25));
  const light = new THREE.DirectionalLight(0xffffff,1.0); light.position.set(3,5,4); scene.add(light);
  const grid = new THREE.GridHelper(3, 36, 0x505b4d, 0x303a30); grid.position.y=-.002; scene.add(grid);
  const robotFrame = new THREE.Group(); robotFrame.rotation.x = -Math.PI / 2; scene.add(robotFrame);
  let mode = 'monitor';
  let controllerState = {running:false, ready:false, stopping:false}, robotActionPending = false, currentFollowing = false, telemetryOnline = false;
  let controllerConnected = false, speedPresets = null, servoFault = false;
  let robot = null, angle = .72, elevation = .43, distance = 1.8, pointer = null;
  const target = new THREE.Vector3(0,.27,0);
  function view(){camera.position.set(target.x+distance*Math.cos(elevation)*Math.sin(angle),target.y+distance*Math.sin(elevation),target.z+distance*Math.cos(elevation)*Math.cos(angle));camera.lookAt(target);}
  function resize(){const w=host.clientWidth,h=host.clientHeight; if (!w||!h)return;camera.aspect=w/h;camera.fov=2*Math.atan(Math.tan(THREE.MathUtils.degToRad(19))*Math.max(1,1.1/camera.aspect))*180/Math.PI;camera.updateProjectionMatrix();renderer.setSize(w,h);}
  view(); resize(); new ResizeObserver(resize).observe(host);
  host.addEventListener('pointerdown', e=>{pointer={x:e.clientX,y:e.clientY};host.setPointerCapture(e.pointerId)});
  host.addEventListener('pointermove',e=>{if(!pointer)return;angle-=(e.clientX-pointer.x)*.008;elevation=Math.max(-.15,Math.min(1.35,elevation+(e.clientY-pointer.y)*.006));pointer={x:e.clientX,y:e.clientY};view()});
  host.addEventListener('pointerup',()=>pointer=null);host.addEventListener('pointercancel',()=>pointer=null);
  host.addEventListener('wheel',e=>{e.preventDefault();distance=Math.max(.7,Math.min(3.5,distance*Math.exp(e.deltaY*.001)));view()},{passive:false});
  $('resetView').addEventListener('click',()=>{angle=.72;elevation=.43;distance=1.8;view()});
  function loader(manager){const l=new URDFLoader(manager);l.loadMeshCb=(url,m,done)=>new THREE.STLLoader(m).load(url,g=>done(new THREE.Mesh(g,new THREE.MeshPhongMaterial())),undefined,e=>done(null,e));return l;}
  const manager=new THREE.LoadingManager();let failure=false;
  manager.onError=()=>{failure=true;$('modelStatus').textContent='模型资源加载失败，请检查数字孪生路径';};
  manager.onLoad=()=>{if(robot&&!failure)$('modelStatus').textContent='URDF 模型已加载';};
  const armLoader=loader(manager);armLoader.packages={rebotarm_bringup:'/assets'};
  armLoader.load('/assets/description/urdf/ReBot_Arm_RS.urdf',r=>{robot=r;robotFrame.add(r);const wrist=r.getObjectByName('gripper_end');if(wrist){const camLoader=loader(manager);camLoader.load('/model/gemini336.urdf',c=>wrist.add(c),undefined,()=>{$('modelStatus').textContent='相机模型加载失败';});}},undefined,()=>{$('modelStatus').textContent='模型加载失败，请检查数字孪生路径';});
  let simulationJoints = WebcamControl.middle.slice(), simulationTarget = simulationJoints.slice();
  const motionSmoother = new WebcamControl.MotionSmoother(simulationJoints);
  let grip = .035, gripTarget = .035, renderAt = performance.now();
  let monitorTarget = null, monitorJoints = null;
  function render(now){
    requestAnimationFrame(render);
    const dt = Math.min(.1, Math.max(0, (now - renderAt) / 1000)); renderAt = now;
    if (mode === 'camera' && robot) {
      simulationJoints = motionSmoother.update(simulationTarget, dt);
      simulationJoints.forEach((v, i) => {
        robot.setJointValue('joint' + (i + 1), v);
        $('joint' + i).textContent = (v * 180 / Math.PI).toFixed(1) + '°';
        $('bar' + i).style.width = Math.min(100, Math.abs(v) / Math.PI * 100) + '%';
      });
      grip += Math.max(-.08 * dt, Math.min(.08 * dt, gripTarget - grip));
      const [leftTravel, rightTravel] = WebcamControl.gripperJointValues(grip);
      robot.setJointValue('gripper_joint1', leftTravel);
      robot.setJointValue('gripper_joint2', rightTravel);
    }
    if (mode === 'monitor' && robot && monitorTarget && monitorJoints) {
      const alpha = 1 - Math.exp(-dt / .035);
      monitorJoints = monitorJoints.map((value, i) => value + (monitorTarget[i] - value) * alpha);
      monitorJoints.forEach((value, i) => robot.setJointValue('joint' + (i + 1), value));
    }
    renderer.render(scene,camera);
  }
  requestAnimationFrame(render);

  const video = $('cameraVideo'), controls = $('cameraControls');
  let stream = null, worker = null, epoch = 0, enabled = false, busy = false, starting = false;
  let frameId = 0, lastSent = 0, lastSeen = 0, anchor = null, anchorType = null;
  let fpsWindow = performance.now(), fpsFrames = 0, lastPreviewFrame = null, captureRate = null, videoFrameCallback = null;
  function resetFps() { fpsWindow = performance.now(); fpsFrames = 0; lastPreviewFrame = null; captureRate = null; $('previewFps').textContent = '采集 — · 预览 — FPS'; }
  function countPreviewFrame(id, rate) {
    if (Number.isFinite(rate)) captureRate = rate;
    if (id != null && id === lastPreviewFrame) return;
    lastPreviewFrame = id;
    fpsFrames++;
    const now = performance.now(), seconds = (now - fpsWindow) / 1000;
    if (seconds >= 1) {
      const preview = (fpsFrames / seconds).toFixed(1);
      $('previewFps').textContent = mode === 'camera' ? `预览 ${preview} FPS` : `采集 ${captureRate === null ? '—' : captureRate.toFixed(1)} · 预览 ${preview} FPS`;
      fpsWindow = now; fpsFrames = 0;
    }
  }
  function countVideoFrame(_now, metadata) {
    if (mode !== 'camera' || !stream) return;
    countPreviewFrame(metadata.presentedFrames, null);
    videoFrameCallback = video.requestVideoFrameCallback(countVideoFrame);
  }

  let pinch = new RebotHandPolicy.PinchGripper();
  const inputFilter = new WebcamControl.InputFilter();
  function message(value, error = false) { $('cameraMessage').textContent = value; $('cameraMessage').classList.toggle('error', error); }
  function stopCamera(reason = '摄像头已关闭。') {
    if (videoFrameCallback !== null && video.cancelVideoFrameCallback) video.cancelVideoFrameCallback(videoFrameCallback); videoFrameCallback = null; resetFps();
    epoch++; enabled = false; busy = false; starting = false; anchor = null; anchorType = null; inputFilter.reset();
    cancelAnimationFrame(frameId); frameId = 0;
    if (worker) { worker.terminate(); worker = null; }
    if (stream) { stream.getTracks().forEach(track => track.stop()); stream = null; }
    video.srcObject = null; video.parentElement.classList.remove('active'); if (mode === 'camera') { $('cameraPlaceholder').textContent = '摄像头未开启'; $('previewStatus').textContent = '未开启'; }
    $('cameraToggle').disabled = false; $('cameraToggle').textContent = '开启摄像头';
    $('cameraDevice').disabled = false; $('recenter').disabled = true; pinch.reset();
    if (mode === 'camera') { $('connection').textContent = '模拟已暂停'; $('dot').classList.remove('online'); $('palm').textContent = '—'; $('hold').textContent = '已保持'; }
    message(reason);
  }
  function chooseMode(next) {
    if (next === mode) return;
    if (next === 'camera' && controllerState.preview_running) { cameraPreviewAction('stop').then(ok => { if (ok) chooseMode(next); }); return; }
    if (next === 'camera' && controllerState.running) { robotMessage('请先退出真机程序，再切换网页模拟。', true); return; }
    if (next === 'monitor') stopCamera();
    mode = next;
    monitorTarget = null; monitorJoints = null;
    resetRealPreview();
    video.hidden = mode !== 'camera';
    $('realCameraImage').hidden = true;
    $('previewTitle').textContent = mode === 'camera' ? '电脑摄像头' : 'GEMINI 336';
    $('cameraPlaceholder').textContent = mode === 'camera' ? '摄像头未开启' : '等待真机启动';
    $('previewStatus').textContent = mode === 'camera' ? '未开启' : '等待真机启动';
    $('modeMonitor').setAttribute('aria-pressed', String(mode === 'monitor'));
    $('modeCamera').setAttribute('aria-pressed', String(mode === 'camera'));
    controls.hidden = mode !== 'camera';
    $('robotControls').hidden = mode !== 'monitor';
    $('modeLabel').textContent = mode === 'camera' ? '网页模拟' : '真机控制';
    $('modeNote').textContent = mode === 'camera' ? '模拟动作仅作用于网页模型，不发送运动命令。普通摄像头仅支持平面跟随。' : '真机按钮只控制本网页启动的程序。关节反馈可用时显示实际位置，否则显示指令位置。';
    if (mode === 'camera') {
      simulationTarget = WebcamControl.middle.slice(); simulationJoints = simulationTarget.slice(); motionSmoother.reset(simulationJoints);
      $('source').textContent = '电脑摄像头 · 网页模拟'; $('connection').textContent = '模拟已暂停'; $('dot').classList.remove('online');
      $('follow').textContent = '等待摄像头'; $('palm').textContent = '—'; $('hold').textContent = '已保持';
    } else update();
  }
  $('modeMonitor').addEventListener('click', () => chooseMode('monitor'));
  $('modeCamera').addEventListener('click', () => chooseMode('camera'));
  $('recenter').addEventListener('click', () => { anchor = null; anchorType = null; inputFilter.reset(); message('请保持手掌在画面中，正在重新定位。'); });
  async function updateDevices() {
    if (!navigator.mediaDevices?.enumerateDevices) return;
    const devices = await navigator.mediaDevices.enumerateDevices();
    const selected = $('cameraDevice').value;
    $('cameraDevice').replaceChildren(new Option('默认摄像头', ''));
    devices.filter(d => d.kind === 'videoinput').forEach((d, i) => $('cameraDevice').add(new Option(d.label || `摄像头 ${i + 1}`, d.deviceId)));
    if ([...$('cameraDevice').options].some(option => option.value === selected)) $('cameraDevice').value = selected;
  }
  async function startCamera() {
    if (!navigator.mediaDevices?.getUserMedia) { message('浏览器无法访问摄像头，请使用支持摄像头的浏览器并通过 localhost 打开。', true); return; }
    const token = ++epoch;
    starting = true; $('cameraToggle').disabled = false; $('cameraToggle').textContent = '取消启动'; $('cameraDevice').disabled = true; message('正在请求摄像头权限…');
    try {
      const deviceId = $('cameraDevice').value;
      const captured = await navigator.mediaDevices.getUserMedia({video: deviceId ? {deviceId: {exact: deviceId}} : {facingMode:'user'}, audio:false});
      if (token !== epoch || mode !== 'camera') { captured.getTracks().forEach(t => t.stop()); return; }
      stream = captured; video.srcObject = stream; await video.play();
      if (token !== epoch || mode !== 'camera') return;
      video.parentElement.classList.add('active'); $('previewStatus').textContent = '实时画面';
      if (video.requestVideoFrameCallback) videoFrameCallback = video.requestVideoFrameCallback(countVideoFrame);
      stream.getVideoTracks()[0]?.addEventListener('ended', () => { if (token === epoch) stopCamera('摄像头连接已结束。'); });
      await updateDevices().catch(() => {});
      if (token !== epoch || mode !== 'camera') return;
      worker = new Worker('hand-follow-worker.js', {type:'module'});
      worker.onmessage = event => {
        if (token !== epoch || mode !== 'camera') return;
        const data = event.data;
        if (data.type === 'ready') {
          enabled = true; starting = false; busy = false; $('cameraToggle').disabled = false; $('cameraToggle').textContent = '关闭摄像头';
          $('recenter').disabled = false; $('cameraDevice').disabled = false; $('connection').textContent = '摄像头模拟'; $('dot').classList.add('online');
          $('follow').textContent = '等待手掌'; message('摄像头已开启，展示手掌开始模拟。'); frameId = requestAnimationFrame(captureFrame);
        } else if (data.type === 'error') stopCamera('手势识别失败：' + (data.message || '请检查模型资源。'));
        else if (data.type === 'result') {
          busy = false;
          const now = performance.now();
          if (now - data.timestamp > 400) return;
          const sample = RebotHandPolicy.measureHand(data.landmarks, video.videoWidth / video.videoHeight || 4 / 3);
          if (!sample) { anchor = null; anchorType = null; inputFilter.reset(); $('palm').textContent = '未检测'; $('follow').textContent = '手掌丢失，保持'; $('hold').textContent = '已保持'; return; }
          lastSeen = now;
          if (anchor && anchorType !== sample.anchor) { anchor = null; inputFilter.reset(); }
          const point = inputFilter.hand(sample, data.timestamp);
          if (!anchor) {
            anchor = point; anchorType = sample.anchor;
            // Reanchor around current pose to avoid a jump after hand loss.
            controlBase = simulationTarget.slice();
          }
          const goal = WebcamControl.target(point, anchor);
          if (goal) simulationTarget = WebcamControl.boundedFromAnchor(goal, controlBase);
          const nextGrip = pinch.observe(sample.pinchWidthRatio);
          if (nextGrip !== null) gripTarget = inputFilter.gripper(nextGrip, data.timestamp);
          $('palm').textContent = '已检测'; $('follow').textContent = '网页模型跟随中'; $('hold').textContent = '跟随中';
          message('移动手掌控制模型，捏合控制夹爪。');
        }
      };
      worker.onerror = () => { if (token === epoch) stopCamera('手势识别加载失败，请检查数字孪生资源路径。'); };
      worker.postMessage({type:'init', wasmUrl:new URL('/assets/mediapipe/wasm', location.href).href, modelUrl:new URL('/assets/hand_landmarker.task', location.href).href});
      message('正在加载手势识别模型…');
    } catch (error) {
      if (token === epoch) { stopCamera(); message(error.name === 'NotAllowedError' ? '摄像头权限被拒绝，请在浏览器中允许访问。' : '无法启动摄像头：' + error.message, true); }
    }
  }
  let controlBase = simulationTarget.slice();
  async function captureFrame(now) {
    if (!enabled || mode !== 'camera') return;
    frameId = requestAnimationFrame(captureFrame);
    if (lastSeen && now - lastSeen > 400) { anchor = null; anchorType = null; inputFilter.reset(); $('follow').textContent = '等待画面，保持'; $('hold').textContent = '已保持'; }
    if (busy || now - lastSent < 45 || video.readyState < 2) return;
    busy = true; lastSent = now; const token = epoch;
    try {
      const bitmap = await createImageBitmap(video);
      if (token !== epoch || !worker || !enabled) { bitmap.close(); return; }
      worker.postMessage({type:'frame', bitmap, timestamp:now}, [bitmap]);
    } catch (error) { busy = false; if (token === epoch) stopCamera('无法读取摄像头画面：' + error.message); }
  }
  $('cameraDevice').addEventListener('change', () => { if (stream) { stopCamera('正在切换摄像头…'); startCamera(); } });
  $('cameraToggle').addEventListener('click', () => { if (stream || starting) stopCamera(); else startCamera(); });
  document.addEventListener('visibilitychange', () => { if (document.hidden && mode === 'camera' && (stream || starting)) stopCamera('页面已切到后台，摄像头已关闭。'); });
  window.addEventListener('pagehide', () => stopCamera());

  function robotMessage(value, error = false) { $('robotMessage').textContent = value; $('robotMessage').classList.toggle('error', error); }
  const motionFields = {gain:'robotGain', gain_x:'robotGainX', max_vel:'robotMaxVel'};
  function showMotion(settings) {
    for (const [key, id] of Object.entries(motionFields)) $(id).value = settings?.[key] ?? '';
  }
  function selectSpeedPreset() {
    if (speedPresets) showMotion(speedPresets[$('robotSpeed').value]);
    $('robotSafetyConfirm').checked = false;
    $('robotParameterHint').textContent = '切换档位加载预设；修改在下次初始化时生效。已调整速度档的 HOME 速度可能更快。';
    updateRobotButtons();
  }
  $('robotSpeed').addEventListener('change', selectSpeedPreset);
  for (const id of Object.values(motionFields)) $(id).addEventListener('input', () => {
    $('robotSafetyConfirm').checked = false;
    $('robotParameterHint').textContent = '参数已调整；仅在下次初始化时生效。';
    updateRobotButtons();
  });
  for (const id of ['robotGripper', 'robotCameraDevice']) $(id).addEventListener('change', () => {
    $('robotSafetyConfirm').checked = false;
    updateRobotButtons();
  });
  async function loadSpeedPresets() {
    try {
      const response = await fetch('/api/robot/presets', {cache:'no-store'}), data = await response.json();
      if (!response.ok) throw Error(data.error || '读取速度配置失败');
      speedPresets = data.presets;
      if (!controllerState.running) selectSpeedPreset();
      updateRobotButtons();
    } catch (error) { speedPresets = null; robotMessage(error.message, true); updateRobotButtons(); }
  }
  function updateRobotButtons() {
    const active = controllerState.running, ready = active && controllerState.ready && !controllerState.stopping;
    const unavailable = !controllerConnected || robotActionPending;
    $('robotStart').disabled = unavailable || !speedPresets || active || !$('robotSafetyConfirm').checked;
    $('robotSpeed').disabled = unavailable || !speedPresets || active;
    $('robotGripper').disabled = unavailable || active;
    $('robotSafetyConfirm').disabled = unavailable || active;
    for (const id of Object.values(motionFields)) $(id).disabled = unavailable || !speedPresets || active;
    $('robotFollow').disabled = unavailable || !ready || !telemetryOnline || currentFollowing || servoFault;
    $('robotPause').disabled = unavailable || !ready || servoFault;
    $('robotStop').disabled = unavailable || !active || controllerState.stopping;
    $('modeCamera').disabled = active || robotActionPending;
    $('robotCameraDevice').disabled = unavailable || active || controllerState.preview_running;
    $('robotCameraRefresh').disabled = unavailable || active || controllerState.preview_running;
    $('robotCameraToggle').disabled = unavailable || active;
    $('robotCameraToggle').textContent = controllerState.preview_running ? '关闭摄像头' : '打开摄像头';
  }
  async function robotAction(action, body = {}) {
    if (mode !== 'monitor' || robotActionPending) return;
    robotActionPending = true; updateRobotButtons();
    try {
      const response = await fetch('/api/robot/' + action, {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body)});
      const result = await response.json();
      if (!response.ok) throw new Error(result.error || '请求失败');
      robotMessage(result.message);
      await updateController();
    } catch (error) { robotMessage(error.message, true); }
    finally { robotActionPending = false; updateRobotButtons(); }
  }
  $('robotStart').addEventListener('click', () => {
    for (const id of Object.values(motionFields)) if (!$(id).reportValidity()) return;
    const motion = Object.fromEntries(Object.entries(motionFields).map(([key, id]) => [key, Number($(id).value)]));
    robotAction('start', {speed:$('robotSpeed').value, gripper:$('robotGripper').value === 'gesture', serial:$('robotCameraDevice').value || null, motion, confirm_home:$('robotSafetyConfirm').checked});
  });
  $('robotSafetyConfirm').addEventListener('change', updateRobotButtons);
  $('robotFollow').addEventListener('click', () => robotAction('follow', {enabled:true}));
  $('robotPause').addEventListener('click', () => robotAction('follow', {enabled:false}));
  $('robotStop').addEventListener('click', () => robotAction('shutdown'));
  document.addEventListener('keydown', event => {
    if (mode !== 'monitor' || event.repeat || event.ctrlKey || event.metaKey || event.altKey) return;
    const tag = event.target?.tagName;
    if ((tag === 'INPUT' || tag === 'TEXTAREA' || event.target?.isContentEditable) && event.key !== 'Escape') return;
    if ((event.key === 'q' || event.key === 'Q' || event.key === 'Escape') && controllerState.running && !controllerState.stopping) {
      event.preventDefault(); if (!$('robotStop').disabled) $('robotStop').click();
    } else if ((event.key === 'r' || event.key === 'R') && tag !== 'SELECT') {
      event.preventDefault(); if (!$('robotFollow').disabled) $('robotFollow').click();
    } else if ((event.code === 'Space' || event.key === ' ') && tag !== 'SELECT') {
      event.preventDefault();
      if (!$('robotPause').disabled) $('robotPause').click();
    }
  });
  async function refreshRobotCameras() {
    try {
      const response = await fetch('/api/cameras', {cache:'no-store'}), data = await response.json();
      if (!response.ok) throw Error(data.error || '无法查找相机');
      const select = $('robotCameraDevice'), selected = select.value;
      select.replaceChildren();
      if (!data.cameras.length) select.add(new Option('未找到 Gemini 336', ''));
      data.cameras.forEach(camera => select.add(new Option(`${camera.name} · ${camera.serial}`, camera.serial)));
      if ([...select.options].some(option => option.value === selected)) select.value = selected;
      $('robotCameraMessage').textContent = data.cameras.length ? '预览不连接机械臂，可在初始化前检查画面。' : '请连接 Gemini 336 后刷新。';
    } catch (error) { $('robotCameraMessage').textContent = '相机列表不可用：' + error.message; }
  }
  async function cameraPreviewAction(action) {
    if (robotActionPending) return;
    robotActionPending = true; updateRobotButtons();
    try {
      const response = await fetch('/api/camera/' + action, {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({serial:$('robotCameraDevice').value})});
      const result = await response.json();
      if (!response.ok) throw Error(result.error || '相机操作失败');
      $('robotCameraMessage').textContent = result.message;
      await updateController();
      return true;
    } catch (error) { $('robotCameraMessage').textContent = error.message; return false; }
    finally { robotActionPending = false; updateRobotButtons(); }
  }
  $('robotCameraRefresh').addEventListener('click', refreshRobotCameras);
  $('robotCameraToggle').addEventListener('click', () => cameraPreviewAction(controllerState.preview_running ? 'stop' : 'start'));
  refreshRobotCameras();

  async function updateController() {
    try {
      const response = await fetch('/api/controller', {cache:'no-store'});
      if (!response.ok) throw Error('service');
      const state = await response.json(), old = controllerState;
      controllerConnected = true;
      controllerState = state;
      if (state.running && !old.running) {
        $('robotSafetyConfirm').checked = false;
        if (state.speed_mode) $('robotSpeed').value = state.speed_mode;
        showMotion(state.motion_settings);
        if (!state.motion_settings) $('robotParameterHint').textContent = '当前运行的参数不可用；请查看程序日志。';
      }
      if (state.preview_failed && !old.preview_failed) $('robotCameraMessage').textContent = '相机打开失败，请检查连接或其他程序占用后重试。';
      if (state.running && !old.running) robotMessage('程序启动中：机械臂将移动到 HOME。');
      if (state.ready && !old.ready) robotMessage('HOME 已就绪，按 R 或张开手掌 3 秒开始跟随。');
      if (state.stopping && !old.stopping) robotMessage('正在退出：机械臂将回零并失能。');
      if (!state.running && (old.running || state.error_summary)) robotMessage(state.exit_code === 0 ? '真机程序已退出。' : (state.error_summary || '真机程序已退出，请检查程序信息。'), state.exit_code !== 0);
      $('robotError').hidden = !state.error_log; $('robotErrorText').textContent = state.error_log || '';
      if (state.running && mode === 'camera') chooseMode('monitor');
      updateRobotButtons();
    } catch (_) {
      controllerConnected = false;
      controllerState = {running:false, ready:false, stopping:false};
      updateRobotButtons();
      robotMessage('网页服务未连接，无法操作真机。', true);
    }
  }
  updateController(); loadSpeedPresets(); setInterval(updateController, 500);

  let previewURL = null, previewPending = false, previewEpoch = 0;
  function resetRealPreview() {
    previewEpoch++;
    resetFps();
    if (previewURL) URL.revokeObjectURL(previewURL);
    previewURL = null;
    $('realCameraImage').removeAttribute('src');
    $('realCameraImage').hidden = true;
    $('liveCameraFrame').classList.remove('active');
  }
  async function updateRealPreview() {
    if (mode !== 'monitor' || document.hidden || !(controllerState.running || controllerState.preview_running)) {
      if (mode === 'monitor') {
        resetRealPreview();
        $('cameraPlaceholder').textContent = '请打开摄像头';
        $('previewStatus').textContent = '未开启';
      }
      return;
    }
    if (previewPending) return;
    previewPending = true;
    const requestEpoch = previewEpoch;
    try {
      const response = await fetch('/api/camera.jpg', {cache:'no-store'});
      if (!response.ok || response.status === 204) throw Error('no frame');
      const frameKey = response.headers.get('X-Frame-Id');
      const rawCaptureFps = response.headers.get('X-Capture-Fps');
      const captureFps = rawCaptureFps === null ? null : Number(rawCaptureFps);
      const blob = await response.blob();
      if (requestEpoch !== previewEpoch || mode !== 'monitor' || !(controllerState.running || controllerState.preview_running)) return;
      const nextURL = URL.createObjectURL(blob), oldURL = previewURL;
      previewURL = nextURL;
      $('realCameraImage').onload = () => { if (requestEpoch === previewEpoch && mode === 'monitor') countPreviewFrame(frameKey, captureFps); };
      $('realCameraImage').src = nextURL;
      $('realCameraImage').hidden = false;
      $('liveCameraFrame').classList.add('active');
      $('previewStatus').textContent = '实时画面';
      if (oldURL) URL.revokeObjectURL(oldURL);
    } catch (_) {
      if (requestEpoch !== previewEpoch || mode !== 'monitor') return;
      resetRealPreview();
      $('cameraPlaceholder').textContent = '等待相机画面';
      $('previewStatus').textContent = '等待画面';
    } finally { previewPending = false; }
  }
  setInterval(updateRealPreview, 100);

  let stateRequestPending = false;
  async function update(){if (mode !== 'monitor' || stateRequestPending) return; stateRequestPending = true; try{const res=await fetch('/api/state',{cache:'no-store'});if(!res.ok)throw Error('network');const s=await res.json();if(mode !== 'monitor')return;telemetryOnline=!!s.online;currentFollowing=!!s.following;servoFault=!!s.servo_failed;updateRobotButtons();$('dot').classList.toggle('online',s.online);$('connection').textContent=s.online?'数据更新中':'未连接';$('source').textContent=s.online?(s.source==='actual'?(s.estimated_model_joints ? `模型估算 · 数值为实际反馈（目标 ${s.feedback_rate_hz || '—'} Hz）` : '实际反馈'):s.source==='command'?'关节指令（非实测）':'等待有效关节数据'):'暂无实时数据';if(s.online && !s.model_joints && s.joints) $('source').textContent+=' · 缺少零位修正，模型未更新';$('follow').textContent=s.online?(s.phase==='home'?'正在前往 HOME':s.phase==='return'?'正在回零':s.following?'正在跟随':'等待跟随'):'等待数据';$('robotSafetyStatus').textContent=(s.online&&((!s.following&&/暂停|丢失|反馈|错误|无效|异常|超时/.test(s.error||''))||/深度估算|低速恢复/.test(s.error||'')))?s.error:'';$('palm').textContent=s.online?(s.visible?'已检测':'未检测'):'—';$('hold').textContent=s.online?(s.hold?'已保持':'未保持'):'—';if(s.online && Array.isArray(s.joints)){s.joints.forEach((v,i)=>{const deg=v*180/Math.PI;$('joint'+i).textContent=deg.toFixed(1)+'°';$('bar'+i).style.width=Math.min(100,Math.abs(deg)/180*100)+'%';});const nextModel = s.estimated_model_joints || s.model_joints;if(Array.isArray(nextModel) && nextModel.length === 6 && nextModel.every(Number.isFinite)){monitorTarget=nextModel.slice();if(!monitorJoints)monitorJoints=monitorTarget.slice();}}else{monitorTarget=null;}}catch(_){if(mode !== 'monitor')return;monitorTarget=null;telemetryOnline=false;updateRobotButtons();$('dot').classList.remove('online');$('connection').textContent='服务未连接';$('source').textContent='暂无实时数据';$('follow').textContent='等待数据';$('palm').textContent='—';$('hold').textContent='—';}finally{stateRequestPending=false;}}
  update();setInterval(update,33);
})();
