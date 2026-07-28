// src/components/CameraPreviewPanel.jsx
// 로봇에 달린 비전 카메라 실시간 화면 - isaac_task_runner.py가 시작할 때
// 만드는 OmniGraph(Isaac Sim 자체 ROS2 카메라 브리지, /camera/rgb 토픽)를
// MSI2에서 띄운 web_video_server(`ros2 run web_video_server web_video_server`)가
// MJPEG로 그대로 서빙해준다 - <img src="...">가 표준 HTTP MJPEG 스트림을
// 그대로 재생하므로 프론트엔드에 별도 스트리밍 코드가 필요 없다.
//
// [2026-07-28] isaac_task_runner.py의 idle 디스패치 루프는 대기 중엔
// world.step()을 아예 안 해서(RMPflow 발산 버그 재발 방지용 설계) 렌더링이
// 멈춘다 - 그래서 이 스트림은 실제 작업(스캔/픽앤플레이스) 중에만 갱신되고
// 대기 중엔 마지막 프레임에서 자연히 멈춘다(의도된 동작, 안전한 1단계).
import { useState } from "react";
import styles from "./CameraPreviewPanel.module.css";

const MSI2_HOST = import.meta.env.VITE_MSI2_HOST || "10.10.0.2";
const MSI2_WEB_VIDEO_SERVER_PORT = import.meta.env.VITE_MSI2_CAMERA_PORT || "8080";
const STREAM_URL = `http://${MSI2_HOST}:${MSI2_WEB_VIDEO_SERVER_PORT}/stream?topic=/camera/rgb`;

export default function CameraPreviewPanel() {
  const [streamFailed, setStreamFailed] = useState(false);
  const [streamKey, setStreamKey] = useState(0);

  const retry = () => {
    setStreamFailed(false);
    setStreamKey((k) => k + 1);
  };

  return (
    <div className={styles.panel}>
      <span className={styles.title}>로봇 카메라 실시간</span>
      {streamFailed ? (
        <div className={styles.placeholder}>
          <span className={styles.icon}>📷</span>
          <span>카메라 스트림에 연결할 수 없음</span>
          <span className={styles.hint}>MSI2에서 web_video_server가 켜져 있는지 확인하세요</span>
          <button type="button" data-testid="camera-retry" onClick={retry}>다시 시도</button>
        </div>
      ) : (
        <img
          key={streamKey}
          className={styles.stream}
          data-testid="camera-stream"
          src={STREAM_URL}
          alt="로봇 카메라 실시간 스트림"
          onError={() => setStreamFailed(true)}
        />
      )}
    </div>
  );
}
