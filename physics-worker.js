// 물리 계산 전용 스레드 (index.html의 physicsCore와 같은 코드 — 페이지에서 자동 생성)
(async () => { try {
  const { default: RAPIER } = await import('https://cdn.jsdelivr.net/npm/@dimforge/rapier3d-compat@0.14.0/rapier.es.js');
  await RAPIER.init();
  const core = (function physicsCore(RAPIER, post) {
  const STEP = 1 / 60;
  let cfg = { gravity: 14, pushF: 50, R: 2.8 }, hulls = [];
  let world = null, walls = [], bodies = [], gen = 0;
  let input = { gx: 0, gz: 0, tiltAmt: 0, homeAmt: 1, pointerActive: false, px: 0, pz: 0, tapBoost: 1, on: true, at: -1e9 };
  let acc = 0, last = 0, steps = 0, ms = 0;
  const INTRO_STEPS = 180; let intro = 0; // 쏟아지는 인트로 남은 걸음
  const lerp = (a, b, t) => a + (b - a) * t;
  const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));

  function makeWorld() {
    if (world) world.free();
    world = new RAPIER.World({ x: 0, y: -cfg.gravity, z: 0 });
    world.timestep = STEP; // 고정 간격: 간격이 들쭉날쭉하면 쌓인 도형이 떨림
    world.integrationParameters.numSolverIterations = cfg.solverIters || 8; // 쌓인 무더기에서도 서로 파고들지 않게 정밀하게 (태블릿 4)
    if (cfg.frictionIters) world.integrationParameters.numAdditionalFrictionIterations = cfg.frictionIters; // 마찰 추가 계산 (기본 4, 태블릿 1)
    // 바닥: 매끄러워 기울이면 미끄러짐
    world.createCollider(RAPIER.ColliderDesc.cuboid(60, 0.5, 60).setTranslation(0, -0.5, 0)
      .setFriction(0.05).setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min));
    walls = [];
  }
  // 벽 (화면 가장자리): 벽에서는 튕김
  function buildWalls(hx, hz) {
    walls.forEach(c => world.removeCollider(c, false));
    const t = 1, h = 10;
    walls = [
      [hx + t, 0, t, h, hz + 2 * t], [-hx - t, 0, t, h, hz + 2 * t],
      [0, hz + t, hx + 2 * t, h, t], [0, -hz - t, hx + 2 * t, h, t],
    ].map(([x, z, ex, ey, ez]) => world.createCollider(
      RAPIER.ColliderDesc.cuboid(ex, ey, ez).setTranslation(x, ey, z).setFriction(0.15)
        .setRestitution(0.55).setRestitutionCombineRule(RAPIER.CoefficientCombineRule.Max)));
  }
  // 도형별 충돌 모양 (Three.js 지오메트리 치수 × 크기)
  function colliderDescs(d) {
    const s = d.s;
    switch (d.kind) {
      case 0: return [RAPIER.ColliderDesc.ball(0.5 * s)];                           // 구
      case 1: return [RAPIER.ColliderDesc.cuboid(0.425 * s, 0.425 * s, 0.425 * s)]; // 정육면체
      case 2: return [RAPIER.ColliderDesc.cone(0.475 * s, 0.48 * s)];               // 원뿔
      case 3: {                                                                     // 링: 고리를 따라 작은 구 12개
        const n = 12, R = 0.38 * s, r = 0.15 * s;
        return Array.from({ length: n }, (_, i) => {
          const a = (i / n) * Math.PI * 2;
          return RAPIER.ColliderDesc.ball(r).setTranslation(Math.cos(a) * R, Math.sin(a) * R, 0);
        });
      }
      case 4: return [RAPIER.ColliderDesc.cylinder(0.4 * s, 0.4 * s)];              // 원기둥
      default: {                                                                    // 딩벳: 볼록 껍질
        const src = hulls[d.hull], pts = new Float32Array(src.length);
        for (let i = 0; i < src.length; i++) pts[i] = src[i] * s;
        return [RAPIER.ColliderDesc.convexHull(pts)];
      }
    }
  }
  function populate(list, hx, hz) {
    makeWorld();
    buildWalls(hx, hz);
    bodies = list.map(d => {
      const b = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic()
        .setTranslation(d.x, d.y, d.z)
        .setRotation({ x: d.q[0], y: d.q[1], z: d.q[2], w: d.q[3] })
        .setLinearDamping(d.homeDrag)
        .setAngularDamping(0.35)   // 잘 굴러감
        .setCcdEnabled(d.ccd));    // 납작한 딩벳은 서로 파고들지 않게 연속 충돌 검사
      for (const c of colliderDescs(d)) {
        if (d.ring) {
          // 유리 링: 50% 가볍고 탱탱하게 튐 (부딪히는 상대와 관계없이 높은 반발)
          c.setDensity(0.5).setFriction(0.35).setRestitution(0.9).setRestitutionCombineRule(RAPIER.CoefficientCombineRule.Max);
        } else {
          c.setDensity(1).setFriction(0.35).setRestitution(d.bounce); // 도형끼리 적당히 붙잡음
        }
        world.createCollider(c, b);
      }
      return { b, d, mass: b.mass(), hx: 0, hz: 0, returning: false, stuck: 0 };
    });
    // 미리 안착시키지 않고 바로 보여 줌: 위에 놓인 도형이 실제로 떨어져 쌓이는 모습이 첫 화면 연출
    // 처음 3초(인트로)는 돌아올 자리가 도형을 따라다니고, 끝나는 순간 자리 잡은 곳이 '제자리'가 됨
    intro = INTRO_STEPS;
    for (const o of bodies) { const p = o.b.translation(); o.hx = p.x; o.hz = p.z; }
    send();
  }
  // 상자 크기가 조금 바뀜: 벽만 옮기고 밖에 나간 도형을 안으로
  function setBounds(hx, hz) {
    if (!world) return;
    buildWalls(hx, hz);
    for (const o of bodies) {
      const p = o.b.translation(), r = 0.5 * o.d.s;
      const cx = clamp(p.x, -hx + r, hx - r), cz = clamp(p.z, -hz + r, hz - r);
      if (cx !== p.x || cz !== p.z) o.b.setTranslation({ x: cx, y: p.y, z: cz }, true);
      o.hx = clamp(o.hx, -hx + r, hx - r); o.hz = clamp(o.hz, -hz + r, hz - r);
    }
    send();
  }
  // 화면이 돌아가거나 비율이 크게 바뀜: 도형을 새로 만들지 않고, 기기와 함께 돈 만큼 배치를 돌리고 새 크기에 맞춤
  //  rot: 화면이 돈 각도(라디안) → 도형은 실제 기기 위에서 그대로 있는 것처럼 보임
  function reflow(hx, hz, rot, sx, sz) {
    if (!world) return;
    buildWalls(hx, hz);
    const c = Math.cos(rot), s = Math.sin(rot), qa = Math.sin(rot / 2), qb = Math.cos(rot / 2);
    for (const o of bodies) {
      const b = o.b, p = b.translation(), q = b.rotation(), v = b.linvel(), r = 0.5 * o.d.s;
      const x = clamp((p.x * c + p.z * s) * sx, -hx + r, hx - r), z = clamp((-p.x * s + p.z * c) * sz, -hz + r, hz - r);
      b.setTranslation({ x, y: p.y, z }, true);
      b.setRotation({ x: qb * q.x + qa * q.z, y: qb * q.y + qa * q.w, z: qb * q.z - qa * q.x, w: qb * q.w - qa * q.y }, true); // y축 회전 × 기존 회전
      b.setLinvel({ x: v.x * c + v.z * s, y: v.y, z: -v.x * s + v.z * c }, true);
      const h = o.hx;
      o.hx = clamp((h * c + o.hz * s) * sx, -hx + r, hx - r);
      o.hz = clamp((-h * s + o.hz * c) * sz, -hz + r, hz - r);
      o.returning = false; o.stuck = 0;
    }
    send();
  }
  // 한 걸음 분량의 힘: 기울기 흐름 · 제자리 복귀 · 손가락/마우스 밀기
  function forces(dt) {
    const { gx, gz, tiltAmt, homeAmt, pointerActive, px, pz, tapBoost } = input;
    const level = homeAmt > 0.01;
    if (intro > 0) intro--;
    for (const o of bodies) {
      const b = o.b, d = o.d, p = b.translation(), m = o.mass;
      if (intro > 0) { o.hx = p.x; o.hz = p.z; o.returning = false; } // 인트로 중: 제자리 = 지금 자리
      const hx = o.hx - p.x, hz = o.hz - p.z, dist = Math.hypot(hx, hz);
      // 실제로 밀려난 도형만 당김: 0.25 넘게 벗어나면 시작해 0.08 안으로 들어올 때까지 (경계에서 들락날락하며 떨지 않게)
      if (dist > 0.25) o.returning = true; else if (dist < 0.08) o.returning = false;
      let ix = 0, iy = 0, iz = 0;
      if (tiltAmt > 0) { ix += gx * d.flow; iz += gz * d.flow; } // 기울기 흐름
      if (o.returning && level) {
        const v = b.linvel();
        // 길이 막혀 1.5초 넘게 못 움직이면 그 자리를 새 자리로 (서로 밀며 떨지 않게)
        o.stuck = Math.hypot(v.x, v.z) < 0.15 ? o.stuck + dt : 0;
        if (o.stuck > 1.5) { o.hx = p.x; o.hz = p.z; o.stuck = 0; }
        else {
          // 수평일 때 원래 자리로: 무게별 목표 속도 (바닥 마찰에 막히지 않게 속도를 맞춤)
          const speed = Math.min(dist * 2, d.homeSpeed), gain = Math.min(1, 6 * dt) * homeAmt / dt;
          ix += (hx / dist * speed - v.x) * gain;
          iz += (hz / dist * speed - v.z) * gain;
        }
      }
      if (pointerActive) { // 밀어내기: 가벼울수록 더 멀리
        const dx = p.x - px, dz = p.z - pz, dd = Math.hypot(dx, dz);
        if (dd < cfg.R && dd > 0.001) {
          const f = (1 - dd / cfg.R) * cfg.pushF * (0.7 / d.rel) * d.pushK * tapBoost;
          ix += dx / dd * f; iz += dz / dd * f; iy += f * d.hopK;
        }
      }
      if (ix || iy || iz) b.applyImpulse({ x: ix * m * dt, y: iy * m * dt, z: iz * m * dt }, true);
      // 저항: 수평일 땐 복귀용 감쇠, 기울이면 무게별 저항
      if (!b.isSleeping()) b.setLinearDamping(lerp(1.5, d.drag, tiltAmt));
    }
  }
  // 상태 보내기: 도형마다 위치3 · 회전4 · 속도3 · 회전속도² · 잠듦 = 12
  // stamp: 이 상태가 해당하는 실제 시각 (아직 계산 안 한 남은 시간만큼 앞) → 화면이 정확한 시점으로 보간
  function send(stamp = performance.timeOrigin + performance.now()) {
    const buf = new Float32Array(bodies.length * 12);
    bodies.forEach((o, i) => {
      const b = o.b, p = b.translation(), q = b.rotation(), v = b.linvel(), w = b.angvel(), k = i * 12;
      buf[k] = p.x; buf[k + 1] = p.y; buf[k + 2] = p.z;
      buf[k + 3] = q.x; buf[k + 4] = q.y; buf[k + 5] = q.z; buf[k + 6] = q.w;
      buf[k + 7] = v.x; buf[k + 8] = v.y; buf[k + 9] = v.z;
      buf[k + 10] = w.x * w.x + w.y * w.y + w.z * w.z;
      buf[k + 11] = b.isSleeping() ? 1 : 0;
    });
    post({ type: 'state', gen, buf, steps, ms, stamp }, [buf.buffer]);
    steps = 0; ms = 0;
  }
  // 고정 간격 반복: 화면이 그리는 동안(입력 신호가 온 지 0.3초 이내)만 돌고, 쉬는 중에는 멈춤
  function loop() {
    setTimeout(loop, 4);
    const now = performance.now();
    if (!world || !bodies.length || !input.on || now - input.at > 300) { last = now; acc = 0; return; }
    acc = Math.min(acc + (now - last) / 1000, 4 * STEP); last = now;
    if (acc < STEP) return;
    // 걸음마다 바로 상태를 보냄 (여러 걸음을 몰아 보내면 화면 보간 간격이 벌어져 멈칫거림)
    while (acc >= STEP) {
      const t0 = performance.now();
      forces(STEP); world.step(); acc -= STEP; steps++;
      ms += performance.now() - t0;
      send(performance.timeOrigin + now - acc * 1000);
    }
  }
  loop();
  return {
    handle(m) {
      if (m.type === 'config') { cfg = m.cfg; hulls = m.hulls; }
      else if (m.type === 'populate') { gen = m.gen; populate(m.list, m.hx, m.hz); }
      else if (m.type === 'bounds') setBounds(m.hx, m.hz);
      else if (m.type === 'reflow') { gen = m.gen; reflow(m.hx, m.hz, m.rot, m.sx, m.sz); }
      else if (m.type === 'input') { input = m.input; input.at = performance.now(); }
    },
  };
})(RAPIER, (m, t) => postMessage(m, t));
  onmessage = e => core.handle(e.data);
  postMessage({ type: 'ready' });
} catch (err) { postMessage({ type: 'fail', msg: String((err && err.message) || err) }); } })();
