const BASE = "/api";

async function handleResponse(resp) {
  const body = await resp.json();
  if (!resp.ok) {
    // routes/plan.py 등은 {error_code, cause, action}(ApiError) 형식을 쓰고,
    // routes/robot.py(cart-scan/trunk-scan/pick-and-place/send)는 {status, message}
    // 형식을 쓴다 - 실제 로봇/Isaac Sim 에러 메시지(routes/robot.py의 message)가
    // 여기서 그냥 버려지고 있었다(cause만 보고 없으면 무조건 "요청이 실패했습니다").
    // err.cause에도 같은 폴백을 넣는다 - 컴포넌트들(ControlPanel.jsx 등)이 전부
    // catch(err)에서 err.cause를 화면에 그대로 찍는 관례라, cause가 비어있으면
    // {status,message} 형식 에러는 화면에 빈 문자열만 뜨는 문제가 있었다.
    const cause = body.cause || body.message;
    const err = new Error(cause || "요청이 실패했습니다");
    err.error_code = body.error_code;
    err.cause = cause;
    err.action = body.action;
    throw err;
  }
  return body;
}

export async function fetchTrunkMaps() {
  const resp = await fetch(`${BASE}/trunk-maps`);
  const body = await handleResponse(resp);
  return body.trunk_maps;
}

export async function fetchBoxPresets() {
  const resp = await fetch(`${BASE}/box-presets`);
  const body = await handleResponse(resp);
  return body.presets;
}

export async function postPlan(requestBody) {
  const resp = await fetch(`${BASE}/plan`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(requestBody),
  });
  return handleResponse(resp);
}

export async function postApprove(requestBody) {
  const resp = await fetch(`${BASE}/approve`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(requestBody),
  });
  return handleResponse(resp);
}

export async function postSend(requestBody) {
  const resp = await fetch(`${BASE}/send`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(requestBody),
  });
  return handleResponse(resp);
}

// 비전(준형)이 만드는 box_scan.json(HMI 설계 가이드라인 4.2절 스키마)을
// 계획 계산에 바로 쓸 수 있는 단순 박스 목록으로 변환한다.
export async function postParseBoxScan(boxScanJson) {
  const resp = await fetch(`${BASE}/parse-box-scan`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(boxScanJson),
  });
  return handleResponse(resp);
}

// 준형이 실제로 넘겨주는 all_boxes_corners_*.json 스키마용.
export async function postParseVisionCorners(visionCornersJson) {
  const resp = await fetch(`${BASE}/parse-vision-corners`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(visionCornersJson),
  });
  return handleResponse(resp);
}

// 로봇(MSI2) 동작 트리거 - cart-scan/trunk-scan/pick-and-place 전부 실제 ROS2
// Action으로 연동되어 있다(routes/robot.py 참고). cart-scan/trunk-scan 성공 시
// filename/url/point_count가 함께 온다. 요청 바디는 필요 없다.
export async function postCartScan() {
  const resp = await fetch(`${BASE}/robot/cart-scan`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({}),
  });
  return handleResponse(resp);
}

export async function postTrunkScan() {
  const resp = await fetch(`${BASE}/robot/trunk-scan`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({}),
  });
  return handleResponse(resp);
}

// 트렁크/카트 스캔 성공 응답의 url(예: "/api/robot/trunk-scan-file/xxx.ply",
// "/api/robot/cart-scan-file/xxx.ply")로 실제 파일 바이너리를 받아온다 -
// PLYLoader.parse()에 그대로 넘길 수 있는 ArrayBuffer.
export async function fetchScanFile(url) {
  const resp = await fetch(url);
  if (!resp.ok) {
    throw new Error("스캔 결과 파일을 불러오지 못했습니다");
  }
  return resp.arrayBuffer();
}

// planId는 선택 - 지금은 로그/정보 제공용일 뿐이다(POST /api/approve 응답의
// plan_id를 넘기면 됨). isaac_task_runner.py의 run_pick_and_place()는 아직 이
// 값을 안 받고 디스크의 placement_result.json을 그대로 읽는다(20_task_export.py
// 참고 - MSI2 실제 전송 경로가 확정되면 이 값으로 어떤 계획을 실행할지 지정하게 될 것).
export async function postPickAndPlace(planId = "") {
  const resp = await fetch(`${BASE}/robot/pick-and-place`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ plan_id: planId }),
  });
  return handleResponse(resp);
}

// POST /api/robot/pick-and-place가 15~20분 동안 응답을 안 주는 동안, 이 GET을
// 주기적으로 폴링해서 박스 단위 진행 상황(box_started/box_done 등)을 받아온다
// - {"events": [{"stage", "box_index", "box_count", "box_id"}, ...]} 형태로,
// 매번 "지금까지의 전체 이벤트 목록"을 그대로 돌려준다(마지막으로 본 개수
// 이후만 새 이벤트로 취급하면 됨).
export async function fetchPickAndPlaceProgress() {
  const resp = await fetch(`${BASE}/robot/pick-and-place/progress`);
  return handleResponse(resp);
}

// 산업현장 시나리오 미리보기 - routes/scenarios.py 참고. 요청 바디는 필요 없다.
export async function postScenarioPlan(scenarioId) {
  const resp = await fetch(`${BASE}/scenarios/${scenarioId}/plan`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({}),
  });
  return handleResponse(resp);
}
