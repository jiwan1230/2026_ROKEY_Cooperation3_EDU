// src/hooks/usePlanActions.js
// ControlPanel.jsx("알고리즘 검증" 탭)와 RealtimeControlPanel.jsx("실시간
// 제어" 탭)가 승인/전송 버튼에 똑같이 쓰는 로직 - 원래 ControlPanel.jsx
// 안에 있던 handleApprove/handleSend를 그대로 옮겼다(동작 변경 없음).
import { usePlannerDispatch, usePlannerState } from "../state/PlannerContext.jsx";
import { postApprove, postSend } from "../api/client.js";

export function usePlanActions() {
  const state = usePlannerState();
  const dispatch = usePlannerDispatch();

  const handleApprove = async () => {
    if (state.planState !== "COMPUTED" || !state.result) return;
    try {
      // 승인 시 감사 기록에 남는 parameters는 useDebouncedPlan.js가
      // postPlan()에 실제로 보낸 요청 바디와 형식이 같아야 한다(snake_case
      // 키 이름 + 마진 필드 ""→null + Number() 변환). state.params를 그대로
      // 넘기면 camelCase/빈 문자열 그대로라 실제 계산에 쓰인 값과 형식이
      // 어긋난다.
      const { params } = state;
      const approvedParameters = {
        mode: params.mode,
        margin: params.margin === "" ? null : Number(params.margin),
        wall_margin: params.wallMargin === "" ? null : Number(params.wallMargin),
        obstacle_margin: params.obstacleMargin === "" ? null : Number(params.obstacleMargin),
        ceiling_margin: params.ceilingMargin === "" ? null : Number(params.ceilingMargin),
        entrance_margin: params.entranceMargin === "" ? null : Number(params.entranceMargin),
        entrance_preference: params.entrancePreference,
        contact_preference: params.contactPreference,
        height_preference: params.heightPreference,
        allow_stacking: params.allowStacking,
        allow_rotation: params.allowRotation,
        fixed_order: params.fixedOrder,
      };
      const resp = await postApprove({
        box_snapshot_id: state.result.box_snapshot_id,
        trunk_map_id: state.result.trunk_map_id,
        parameters: approvedParameters,
        placed: state.result.placed,
        // POST /api/plan 응답 그대로 - 트렁크 로컬 좌표(placed[].position)를
        // m0609_base_link 좌표로 되돌리는 데 필요(algorism_bridge.build_approved_task 참고).
        trunk_offset_base_frame: state.result.trunk_offset_base_frame,
      });
      dispatch({ type: "APPROVE_SUCCESS", payload: resp });
    } catch (err) {
      dispatch({ type: "COMPUTE_ERROR", payload: { error_code: err.error_code, cause: err.cause, action: err.action } });
    }
  };

  const handleSend = async () => {
    if (state.planState !== "APPROVED" || !state.pendingTask) return;
    try {
      const resp = await postSend({ task: state.pendingTask });
      dispatch({ type: "SEND_SUCCESS", payload: resp });
    } catch (err) {
      dispatch({ type: "COMPUTE_ERROR", payload: { error_code: err.error_code, cause: err.cause, action: err.action } });
    }
  };

  const locked = state.planState === "APPROVED";

  return { handleApprove, handleSend, locked };
}
