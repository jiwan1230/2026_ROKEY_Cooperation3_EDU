import { createContext, useContext, useReducer } from "react";
import { initialState, plannerReducer } from "./plannerReducer.js";

const PlannerStateContext = createContext(null);
const PlannerDispatchContext = createContext(null);

export function PlannerProvider({ children }) {
  const [state, dispatch] = useReducer(plannerReducer, initialState);
  return (
    <PlannerStateContext.Provider value={state}>
      <PlannerDispatchContext.Provider value={dispatch}>
        {children}
      </PlannerDispatchContext.Provider>
    </PlannerStateContext.Provider>
  );
}

export function usePlannerState() {
  const ctx = useContext(PlannerStateContext);
  if (ctx === null) throw new Error("usePlannerState는 PlannerProvider 안에서만 사용할 수 있습니다");
  return ctx;
}

export function usePlannerDispatch() {
  const ctx = useContext(PlannerDispatchContext);
  if (ctx === null) throw new Error("usePlannerDispatch는 PlannerProvider 안에서만 사용할 수 있습니다");
  return ctx;
}

// Header.jsx처럼 PlannerProvider 밖(App.jsx의 탭 전환과 무관하게 항상 떠
// 있는 최상단 헤더)에서도 렌더링돼야 하는 컴포넌트를 위한 버전 - 없으면
// 조용히 null을 반환한다("알고리즘 검증"/"실시간 제어" 탭이 각자 독립된
// PlannerProvider를 쓰게 되면서, 둘 다를 감싸지 않는 헤더는 어느 하나에도
// 속하지 않는다).
export function usePlannerDispatchOptional() {
  return useContext(PlannerDispatchContext);
}
