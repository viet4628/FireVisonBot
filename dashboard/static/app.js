(() => {
  const boot = window.__BOOT__ || {};
  const wsDot = document.getElementById("ws-dot");
  const wsLabel = document.getElementById("ws-label");
  const logOut = document.getElementById("log-out");
  const dlg = document.getElementById("dlg");
  const btnSettings = document.getElementById("btn-settings");
  const btnClose = document.getElementById("btn-close");
  const cfgForm = document.getElementById("cfg-form");
  const modelBadge = document.getElementById("model-badge");
  const aiStream = document.getElementById("ai-stream");

  function appendLog(line) {
    logOut.textContent += line + "\n";
    logOut.scrollTop = logOut.scrollHeight;
  }

  function refreshAiFeed() {
    aiStream.src = "/ai_feed?" + Date.now();
  }

  function setSensors(data) {
    const ok = document.getElementById("s-ok");
    const fl = document.getElementById("s-fl");
    const fr = document.getElementById("s-fr");
    const dist = document.getElementById("s-dist");
    const relay = document.getElementById("s-relay");
    const state = document.getElementById("s-state");
    const up = document.getElementById("s-up");

    if (!data._ok) {
      ok.textContent = data.error ? String(data.error) : "Offline / lỗi";
      ok.style.whiteSpace = "pre-wrap";
      ok.style.textAlign = "left";
      ok.style.maxWidth = "100%";
      ok.style.color = "var(--bad)";
      fl.textContent = fr.textContent = dist.textContent = "—";
      relay.textContent = state.textContent = up.textContent = "—";
      const aic = document.getElementById("s-aic");
      const aio = document.getElementById("s-aio");
      if (aic) aic.textContent = "—";
      if (aio) aio.textContent = "—";
      return;
    }
    ok.textContent = "OK";
    ok.style.whiteSpace = "normal";
    ok.style.textAlign = "";
    ok.style.maxWidth = "";
    ok.style.color = "var(--ok)";
    fl.textContent = data.flame_left ? "LỬA" : "bình thường";
    fl.style.color = data.flame_left ? "var(--bad)" : "var(--muted)";
    fr.textContent = data.flame_right ? "LỬA" : "bình thường";
    fr.style.color = data.flame_right ? "var(--bad)" : "var(--muted)";
    dist.textContent = Number(data.distance_cm).toFixed(1);
    relay.textContent = data.relay_on ? "BẬT" : "tắt";
    relay.style.color = data.relay_on ? "var(--warn)" : "var(--muted)";
    state.textContent = data.state || "—";
    up.textContent = data.uptime_ms != null ? String(data.uptime_ms) : "—";
    const aic = document.getElementById("s-aic");
    const aio = document.getElementById("s-aio");
    if (aic) {
      aic.textContent =
        data.ai_confidence != null ? Number(data.ai_confidence).toFixed(3) : "—";
    }
    if (aio) {
      aio.textContent =
        data.ai_fresh_above_65 === true
          ? "có"
          : data.ai_fresh_above_65 === false
            ? "không"
            : "—";
      aio.style.color =
        data.ai_fresh_above_65 === true ? "var(--ok)" : "var(--muted)";
    }
  }

  function setDetection(meta) {
    document.getElementById("d-lat").textContent =
      meta.latency_ms != null ? meta.latency_ms + " ms" : "—";
    document.getElementById("d-n").textContent =
      meta.num_detections != null ? String(meta.num_detections) : "—";
    const el = document.getElementById("d-boxes");
    if (meta.boxes && meta.boxes.length) {
      el.textContent = meta.boxes
        .map(
          (b) =>
            `${b.name} ${(b.conf * 100).toFixed(0)}%  xyxy=[${b.xyxy.join(", ")}]`
        )
        .join("\n");
    } else {
      el.textContent = "(không có đối tượng vượt ngưỡng)";
    }
  }

  function connectWs() {
    const proto = location.protocol === "https:" ? "wss" : "ws";
    const ws = new WebSocket(`${proto}://${location.host}/ws`);
    ws.onopen = () => {
      wsDot.classList.add("on");
      wsLabel.textContent = "WebSocket · đã nối";
    };
    ws.onclose = () => {
      wsDot.classList.remove("on");
      wsLabel.textContent = "WebSocket · mất kết nối";
      setTimeout(connectWs, 2000);
    };
    ws.onmessage = (ev) => {
      try {
        const msg = JSON.parse(ev.data);
        if (msg.type === "sensors") setSensors(msg.data);
        if (msg.type === "detection") setDetection(msg.data);
        if (msg.type === "log") appendLog(msg.line);
        if (msg.type === "log_bulk" && Array.isArray(msg.lines)) {
          logOut.textContent = msg.lines.join("\n") + "\n";
          logOut.scrollTop = logOut.scrollHeight;
        }
      } catch (_) {}
    };
  }

  async function loadConfig() {
    const r = await fetch("/api/config");
    const c = await r.json();
    document.getElementById("f-stream").value = c.stream_url || "";
    document.getElementById("f-robot").value = c.robot_status_url || "";
    const aip = document.getElementById("f-aipush");
    if (aip) aip.value = c.robot_ai_push_url || "";
    document.getElementById("f-imgsz").value = c.yolo_imgsz;
    document.getElementById("f-conf").value = c.yolo_conf;
    document.getElementById("f-inf").value = c.infer_interval_s;
    document.getElementById("f-poll").value = c.robot_poll_s;
    const rt = document.getElementById("f-rtout");
    if (rt) rt.value = c.robot_http_timeout_s ?? 12;
    document.getElementById("f-aiq").value = c.ai_feed_jpeg_quality ?? 95;
    document.getElementById("f-model-path").textContent = c.model_path || "";
    modelBadge.textContent = c.model_exists ? "Model: sẵn sàng" : "Model: thiếu file";

    try {
      const inf = await fetch("/api/inference");
      const m = await inf.json();
      if (m && Object.keys(m).length) setDetection(m);
    } catch (_) {}
  }

  btnSettings.addEventListener("click", async () => {
    await loadConfig();
    dlg.showModal();
  });
  btnClose.addEventListener("click", () => dlg.close());

  cfgForm.addEventListener("submit", async (e) => {
    e.preventDefault();
    const body = {
      stream_url: document.getElementById("f-stream").value.trim(),
      robot_status_url: document.getElementById("f-robot").value.trim(),
      yolo_imgsz: Number(document.getElementById("f-imgsz").value),
      yolo_conf: Number(document.getElementById("f-conf").value),
      infer_interval_s: Number(document.getElementById("f-inf").value),
      robot_poll_s: Number(document.getElementById("f-poll").value),
      robot_http_timeout_s: Number(
        document.getElementById("f-rtout")
          ? document.getElementById("f-rtout").value
          : 12
      ),
      ai_feed_jpeg_quality: Number(document.getElementById("f-aiq").value),
      robot_ai_push_url: document.getElementById("f-aipush")
        ? document.getElementById("f-aipush").value.trim()
        : "",
    };
    await fetch("/api/config", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    });
    refreshAiFeed();
    dlg.close();
  });

  aiStream.onerror = () => {
    appendLog("[UI] Lỗi hiển thị /ai_feed — kiểm tra MJPEG URL trong cấu hình hoặc tải lại trang.");
  };

  connectWs();
  loadConfig();
  if (boot.modelPath) {
    document.getElementById("f-model-path").textContent = boot.modelPath;
  }
})();
