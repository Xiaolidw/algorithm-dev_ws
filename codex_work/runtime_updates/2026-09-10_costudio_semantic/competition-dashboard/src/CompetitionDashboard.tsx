import { MessageEvent, PanelExtensionContext } from "@coscene/extension";
import { ReactElement, useEffect, useLayoutEffect, useMemo, useState } from "react";
import { createRoot } from "react-dom/client";

type StringMessage = { data?: string };

type MissionTask = {
  sequence?: number;
  object_id?: string;
  color?: string;
  destination?: string;
  status?: string;
  execution_status?: string;
};

type MissionPlan = {
  task_count?: number;
  tasks?: MissionTask[];
};

type MissionStatus = {
  state?: string;
  detail?: string;
  variables?: Record<string, number>;
  task_count?: number;
  current_task_index?: number;
  visual_stable_count?: number;
  active?: boolean;
  completed_task_count?: number;
  tasks?: MissionTask[];
  current_task?: MissionTask | null;
};

type CurrentTask = {
  current_task_index?: number;
  current_task?: MissionTask | null;
};

type DashboardState = {
  question: string;
  status: MissionStatus;
  plan: MissionPlan;
  current: CurrentTask;
};

const TOPICS = [
  "/semantic/question_raw",
  "/mission/status",
  "/mission/plan",
  "/mission/current_task",
  "/mission/execution_status",
];

const INITIAL_STATE: DashboardState = {
  question: "等待赛题生成……",
  status: { state: "IDLE", detail: "等待任务开始" },
  plan: { task_count: 0, tasks: [] },
  current: { current_task_index: 0, current_task: null },
};

const STATE_LABELS: Record<string, string> = {
  IDLE: "等待任务",
  REQUESTING_QUESTION: "正在生成赛题",
  WAITING_SEMANTIC: "大模型解析中",
  SEMANTIC_READY: "语义解析完成",
  PLAN_READY: "前往资源点",
  VERIFYING_CARGO: "识别资源",
  VISUAL_CONFIRMED: "资源已确认",
  NAVIGATING: "导航中",
  NAVIGATING_PICKUP: "前往抓取点",
  PICKUP_TRANSIT: "抓取通道导航",
  PRECISION_DOCKING: "精确靠近",
  GRASPING: "正在抓取",
  OBJECT_GRASPED: "抓取完成",
  NAVIGATING_DROPOFF: "搬运避障中",
  DROPOFF_TRANSIT: "放置通道导航",
  DROPOFF_FINE_APPROACH: "低速进入放置位",
  DROPOFF_ALIGN: "对正放置区",
  DROPOFF_REACHED: "到达放置区",
  DEPARTING_DROPOFF: "离开放置区",
  TASK_COMPLETED: "单件已完成",
  MISSION_COMPLETED: "任务完成",
  RETRY_WAIT: "重新规划中",
  WAIT_DYNAMIC_CLEAR: "等待动态障碍让行",
  PICKING: "正在抓取",
  PLACING: "正在放置",
  COMPLETED: "任务完成",
  ERROR: "发生错误",
};

const TASK_LABELS: Record<string, string> = {
  PENDING: "待执行",
  VERIFYING: "识别中",
  VISUAL_CONFIRMED: "已识别",
  NAVIGATING: "导航中",
  PICKING: "抓取中",
  PICKED: "已抓取",
  NAVIGATING_PICKUP: "前往抓取点",
  PICKUP_TRANSIT: "抓取通道导航",
  PRECISION_DOCKING: "精确靠近",
  GRASPING: "抓取中",
  OBJECT_GRASPED: "已抓取",
  NAVIGATING_DROPOFF: "搬运中",
  DROPOFF_TRANSIT: "放置通道导航",
  DROPOFF_FINE_APPROACH: "低速进入放置位",
  DROPOFF_ALIGN: "对正放置区",
  DROPOFF_REACHED: "准备放置",
  DEPARTING_DROPOFF: "放置完成",
  TASK_COMPLETED: "已完成",
  PLACING: "放置中",
  COMPLETED: "已完成",
};

function parseJson(event: MessageEvent): unknown {
  const message = event.message as StringMessage;
  if (typeof message.data !== "string") {
    return undefined;
  }
  try {
    const parsed: unknown = JSON.parse(message.data);
    return parsed;
  } catch {
    return undefined;
  }
}

function updateFromFrame(previous: DashboardState, frame: readonly MessageEvent[]): DashboardState {
  let next = previous;
  for (const event of frame) {
    if (event.topic === "/semantic/question_raw") {
      const text = (event.message as StringMessage).data?.trim();
      if (text) {
        next = { ...next, question: text };
      }
    } else if (event.topic === "/mission/execution_status") {
      const value = parseJson(event) as MissionStatus | undefined;
      if (value) {
        const executionTasks = value.tasks ?? next.plan.tasks ?? [];
        next = {
          ...next,
          status: value,
          plan: { task_count: value.task_count ?? executionTasks.length, tasks: executionTasks },
          current: {
            current_task_index: value.current_task_index ?? 0,
            current_task: value.current_task ?? null,
          },
        };
      }
    } else if (event.topic === "/mission/status") {
      const value = parseJson(event) as MissionStatus | undefined;
      // Once the executor has published, it is the authoritative progress
      // source.  The semantic coordinator intentionally keeps its own index
      // at zero and must not overwrite the physical pick/place progress.
      if (value && typeof next.status.active !== "boolean") {
        next = { ...next, status: value };
      }
    } else if (event.topic === "/mission/plan") {
      const value = parseJson(event) as MissionPlan | undefined;
      if (value) {
        next = { ...next, plan: value };
      }
    } else if (event.topic === "/mission/current_task") {
      const value = parseJson(event) as CurrentTask | undefined;
      if (value) {
        next = { ...next, current: value };
      }
    }
  }
  return next;
}

function CompetitionDashboard({ context }: { context: PanelExtensionContext }): ReactElement {
  const [dashboard, setDashboard] = useState<DashboardState>(INITIAL_STATE);
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [questionInput, setQuestionInput] = useState(
    "仓库任务中，红色物块数量为x，蓝色物块数量为y。红色物块需要2个，蓝色物块需要3个。",
  );
  const [redDestination, setRedDestination] = useState("A");
  const [blueDestination, setBlueDestination] = useState("B");
  const [blueFirst, setBlueFirst] = useState(true);
  const [publishFeedback, setPublishFeedback] = useState("尚未发布");

  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      const frame = renderState.currentFrame ?? [];
      if (frame.length > 0) {
        setDashboard((previous) => updateFromFrame(previous, frame));
      }
      setRenderDone(() => done);
    };

    context.watch("currentFrame");
    context.subscribe(TOPICS.map((topic) => ({ topic })));
    context.advertise?.("/costudio/mission_request", "std_msgs/msg/String");
  }, [context]);

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  const tasks = useMemo(() => dashboard.plan.tasks ?? [], [dashboard.plan.tasks]);
  const metrics = useMemo(() => {
    const red = tasks.filter((task) => task.color?.toLowerCase() === "red").length;
    const blue = tasks.filter((task) => task.color?.toLowerCase() === "blue").length;
    const first = tasks[0];
    const last = tasks.length > 0 ? tasks[tasks.length - 1] : undefined;
    return { red, blue, first, last };
  }, [tasks]);

  const total = dashboard.plan.task_count ?? tasks.length;
  const rawIndex = dashboard.current.current_task_index ?? dashboard.status.current_task_index ?? 0;
  const currentNumber = total > 0 ? Math.min(rawIndex + 1, total) : 0;
  const currentTask = dashboard.current.current_task;
  const state = dashboard.status.state ?? "IDLE";
  const stateLabel = STATE_LABELS[state] ?? state;
  const currentTaskStatus = currentTask?.execution_status ?? currentTask?.status;
  const taskLabel = currentTaskStatus ? (TASK_LABELS[currentTaskStatus] ?? currentTaskStatus) : stateLabel;
  const destination = currentTask?.destination ? `${currentTask.destination} 区` : "—";

  const publishMission = (): void => {
    const question = questionInput.trim();
    if (!question) {
      setPublishFeedback("题目不能为空");
      return;
    }
    if (!context.publish) {
      setPublishFeedback("当前连接不支持发布；请使用可写 Rosbridge 连接");
      return;
    }
    const red = { variable: "x", color: "red", destination: redDestination };
    const blue = { variable: "y", color: "blue", destination: blueDestination };
    const payload = {
      question,
      mapping: blueFirst ? [blue, red] : [red, blue],
      auto_start: true,
      source: "costudio_competition_dashboard",
    };
    context.publish("/costudio/mission_request", { data: JSON.stringify(payload) });
    setPublishFeedback("已发布，等待本地大模型解析");
  };

  return (
    <div className="competition-dashboard">
      <style>{CSS}</style>
      <header className="dashboard-header">
        <div>
          <div className="eyebrow">2026 大模型技术创新赛</div>
          <h1>智能仓储机器人 · 实时评分看板</h1>
        </div>
        <div className={`state-pill state-${state.toLowerCase()}`}>
          <span className="live-dot" />
          {stateLabel}
        </div>
      </header>

      <section className="question-card">
        <div className="section-label">当前赛题</div>
        <div className="question-text">{dashboard.question}</div>
      </section>

      <section className="publish-card">
        <div className="section-label">发布赛题与映射条件</div>
        <textarea
          aria-label="赛题文本"
          value={questionInput}
          onChange={(event) => { setQuestionInput(event.target.value); }}
        />
        <div className="mapping-row">
          <label>红色 x → <select value={redDestination} onChange={(event) => { setRedDestination(event.target.value); }}><option>A</option><option>B</option><option>C</option></select> 区</label>
          <label>蓝色 y → <select value={blueDestination} onChange={(event) => { setBlueDestination(event.target.value); }}><option>A</option><option>B</option><option>C</option></select> 区</label>
          <label>先执行 <select value={blueFirst ? "blue" : "red"} onChange={(event) => { setBlueFirst(event.target.value === "blue"); }}><option value="blue">蓝色</option><option value="red">红色</option></select></label>
          <button type="button" onClick={publishMission} disabled={dashboard.status.active === true}>发布并执行</button>
          <span className="publish-feedback">{publishFeedback}</span>
        </div>
      </section>

      <section className="metric-grid">
        <MetricCard tone="red" label="红色资源" value={String(metrics.red)} />
        <MetricCard tone="blue" label="蓝色资源" value={String(metrics.blue)} />
        <MetricCard
          tone="cyan"
          label="首次抓取"
          value={formatTask(metrics.first)}
        />
        <MetricCard
          tone="yellow"
          label="最终放置"
          value={formatTask(metrics.last)}
        />
      </section>

      <section className="content-grid">
        <div className="panel-card plan-card">
          <div className="section-label">抓取与放置顺序</div>
          {tasks.length === 0 ? (
            <div className="empty-state">等待大模型生成任务计划……</div>
          ) : (
            <div className="task-flow">
              {tasks.map((task, index) => {
                const active = index === rawIndex;
                const taskStatus = task.execution_status ?? task.status ?? "PENDING";
                const complete = taskStatus === "COMPLETED" || index < rawIndex;
                return (
                  <div
                    className={`task-chip color-${task.color ?? "unknown"} ${active ? "active" : ""} ${complete ? "complete" : ""}`}
                    key={`${task.sequence ?? index}-${task.object_id ?? "task"}`}
                  >
                    <span className="task-index">{index + 1}</span>
                    <span className="task-main">
                      <b>{colorLabel(task.color)}</b>
                      <small>放置到 {task.destination ?? "—"} 区</small>
                    </span>
                    <span className="task-status">{TASK_LABELS[taskStatus] ?? taskStatus}</span>
                  </div>
                );
              })}
            </div>
          )}
        </div>

        <aside className="status-column">
          <StatusCard label="任务进度" value={total > 0 ? `第 ${currentNumber} / ${total} 个` : "等待计划"} />
          <StatusCard label="当前目标" value={currentTask ? `${colorLabel(currentTask.color)} → ${destination}` : "尚未分配"} />
          <StatusCard label="工作状态" value={taskLabel} />
          <StatusCard label="系统详情" value={dashboard.status.detail ?? "—"} compact />
        </aside>
      </section>

      <footer className="dashboard-footer">
        <span>数据源：Rosbridge / ROS2 Humble</span>
        <span>路径：/plan · 图像：/perception/annotated_image</span>
      </footer>
    </div>
  );
}

function MetricCard({ label, value, tone }: { label: string; value: string; tone: string }): ReactElement {
  return (
    <div className={`metric-card metric-${tone}`}>
      <span>{label}</span>
      <strong>{value}</strong>
    </div>
  );
}

function StatusCard({ label, value, compact = false }: { label: string; value: string; compact?: boolean }): ReactElement {
  return (
    <div className={`status-card ${compact ? "compact" : ""}`}>
      <span>{label}</span>
      <strong>{value}</strong>
    </div>
  );
}

function colorLabel(color?: string): string {
  return color?.toLowerCase() === "red" ? "红色资源" : color?.toLowerCase() === "blue" ? "蓝色资源" : "资源";
}

function formatTask(task?: MissionTask): string {
  return task ? `${colorLabel(task.color)}到 ${task.destination ?? "—"} 区` : "—";
}

const CSS = `
  :root { color-scheme: dark; }
  * { box-sizing: border-box; }
  .competition-dashboard {
    min-height: 100%; padding: 18px; overflow: auto; color: #e8eef9;
    background: radial-gradient(circle at 15% 0%, #162d50 0, transparent 38%),
      linear-gradient(145deg, #07101f 0%, #0b1424 58%, #101b2c 100%);
    font-family: "SimSun", "宋体", serif;
    font-size: 14px;
  }
  .dashboard-header { display:flex; align-items:center; justify-content:space-between; gap:18px; margin-bottom:14px; }
  .eyebrow { color:#70b7ff; font-size:13px; font-weight:700; letter-spacing:1.6px; text-transform:uppercase; }
  h1 { margin:4px 0 0; font-size:clamp(24px, 2.5vw, 34px); line-height:1.25; letter-spacing:.5px; }
  .state-pill { display:flex; align-items:center; gap:9px; padding:10px 14px; border:1px solid #2d4d73; border-radius:999px; background:#10223a; font-size:15px; font-weight:700; white-space:nowrap; }
  .live-dot { width:9px; height:9px; border-radius:50%; background:#4ade80; box-shadow:0 0 12px #4ade80; animation:pulse 1.6s infinite; }
  .state-error .live-dot { background:#fb7185; box-shadow:0 0 12px #fb7185; }
  .question-card, .publish-card, .panel-card, .status-card { border:1px solid rgba(136,174,218,.22); background:rgba(13,28,49,.86); box-shadow:0 12px 30px rgba(0,0,0,.18); }
  .question-card { padding:14px 16px; border-radius:12px; margin-bottom:12px; }
  .section-label { color:#8ea6c5; font-size:14px; font-weight:700; letter-spacing:1px; margin-bottom:8px; }
  .question-text { font-size:clamp(16px, 1.55vw, 21px); line-height:1.65; font-weight:600; overflow-wrap:anywhere; }
  .publish-card { padding:12px 16px; border-radius:12px; margin-bottom:12px; }
  .publish-card textarea { width:100%; min-height:64px; resize:vertical; padding:10px 12px; color:#e8eef9; background:#071321; border:1px solid #315171; border-radius:8px; font:600 15px/1.5 "SimSun","宋体",serif; }
  .mapping-row { display:flex; align-items:center; flex-wrap:wrap; gap:10px 14px; margin-top:9px; }
  .mapping-row label { color:#bed1e8; font-weight:700; }
  .mapping-row select { color:#fff; background:#132943; border:1px solid #3b658e; border-radius:6px; padding:5px 8px; }
  .mapping-row button { color:#07101f; background:#61c8ff; border:0; border-radius:8px; padding:8px 16px; font-weight:900; cursor:pointer; }
  .mapping-row button:disabled { opacity:.45; cursor:not-allowed; }
  .publish-feedback { color:#7fcaff; font-size:13px; }
  .metric-grid { display:grid; grid-template-columns:repeat(4,minmax(0,1fr)); gap:10px; margin-bottom:10px; }
  .metric-card { min-height:92px; padding:13px 16px; border-radius:11px; display:flex; flex-direction:column; justify-content:space-between; border:1px solid rgba(255,255,255,.12); }
  .metric-card span { font-size:14px; font-weight:700; opacity:.88; }
  .metric-card strong { font-size:clamp(24px,2.7vw,38px); line-height:1.12; overflow-wrap:anywhere; }
  .metric-red { background:linear-gradient(135deg,#b52f45,#ef5d6f); }
  .metric-blue { background:linear-gradient(135deg,#2247bf,#4064ee); }
  .metric-cyan { background:linear-gradient(135deg,#176e8a,#38a4c6); }
  .metric-yellow { background:linear-gradient(135deg,#806b16,#c4a92e); }
  .content-grid { display:grid; grid-template-columns:minmax(0,3fr) minmax(250px,1fr); gap:10px; min-height:280px; }
  .panel-card { border-radius:12px; padding:14px; }
  .task-flow { display:grid; grid-template-columns:repeat(auto-fit,minmax(178px,1fr)); gap:9px; }
  .task-chip { display:grid; grid-template-columns:32px 1fr auto; align-items:center; gap:9px; min-height:68px; padding:9px; border-radius:10px; border:1px solid #2a4566; background:#10213a; opacity:.82; }
  .task-chip.active { border-color:#65b5ff; box-shadow:0 0 0 2px rgba(101,181,255,.18),0 0 22px rgba(70,158,255,.18); opacity:1; }
  .task-chip.complete { background:#123526; border-color:#277b53; }
  .task-index { display:grid; place-items:center; width:30px; height:30px; border-radius:50%; background:#203a5a; font-weight:800; }
  .color-red .task-index { background:#b73349; } .color-blue .task-index { background:#3158d4; }
  .task-main { display:flex; flex-direction:column; gap:3px; min-width:0; }
  .task-main b { font-size:16px; } .task-main small { color:#aebed2; font-size:14px; line-height:1.3; }
  .task-status { color:#8ec8ff; font-size:13px; line-height:1.3; white-space:normal; overflow-wrap:anywhere; }
  .status-column { display:grid; grid-template-rows:repeat(4,minmax(0,1fr)); gap:9px; }
  .status-card { display:flex; flex-direction:column; justify-content:center; gap:5px; padding:12px 14px; border-radius:11px; background:linear-gradient(135deg,rgba(20,83,61,.88),rgba(24,126,81,.72)); }
  .status-card span { color:#b8dbc9; font-size:14px; font-weight:700; }
  .status-card strong { font-size:clamp(19px,2vw,28px); line-height:1.3; overflow-wrap:anywhere; }
  .status-card.compact strong { font-size:15px; font-weight:600; }
  .empty-state { min-height:210px; display:grid; place-items:center; color:#7f96b2; border:1px dashed #2c4564; border-radius:10px; font-size:15px; }
  .dashboard-footer { display:flex; justify-content:space-between; gap:12px; color:#6f87a5; font-size:12px; line-height:1.4; padding:10px 3px 0; }
  .metric-card strong, .task-index, .status-card strong, .state-pill,
  .dashboard-footer {
    font-family: "Times New Roman", "SimSun", "宋体", serif;
    font-variant-numeric: tabular-nums lining-nums;
  }
  @keyframes pulse { 0%,100%{opacity:1;transform:scale(1)} 50%{opacity:.48;transform:scale(.8)} }
  @media (max-width:850px) { .metric-grid{grid-template-columns:repeat(2,1fr)} .content-grid{grid-template-columns:1fr} .status-column{grid-template-columns:repeat(2,1fr);grid-template-rows:auto} }
  @media (max-width:520px) { .competition-dashboard{padding:10px} .dashboard-header{align-items:flex-start;flex-direction:column} .metric-grid{grid-template-columns:1fr 1fr} .metric-card{min-height:78px;padding:10px} .status-column{grid-template-columns:1fr} .dashboard-footer{flex-direction:column} }
`;

export function initCompetitionDashboard(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<CompetitionDashboard context={context} />);
  return () => {
    root.unmount();
  };
}
