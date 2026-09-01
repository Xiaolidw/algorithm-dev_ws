import { MessageEvent, PanelExtensionContext } from "@coscene/extension";
import { ReactElement, useEffect, useLayoutEffect, useMemo, useState } from "react";
import { createRoot } from "react-dom/client";

type StringMessage = { data?: string };
type MissionTask = { sequence?: number; object_id?: string; color?: string; destination?: string; status?: string };
type MissionPlan = { task_count?: number; tasks?: MissionTask[] };
type MissionStatus = { state?: string; detail?: string; variables?: Record<string, number>; task_count?: number; current_task_index?: number; visual_stable_count?: number };
type CurrentTask = { current_task_index?: number; current_task?: MissionTask | null };
type DashboardState = { question: string; status: MissionStatus; plan: MissionPlan; current: CurrentTask };

const TOPICS = ["/semantic/question_raw", "/mission/status", "/mission/plan", "/mission/current_task"];
const INITIAL_STATE: DashboardState = {
  question: "等待赛题生成……",
  status: { state: "IDLE", detail: "系统已就绪，等待任务开始" },
  plan: { task_count: 0, tasks: [] },
  current: { current_task_index: 0, current_task: null },
};
const STATE_LABELS: Record<string, string> = { IDLE: "待命中", REQUESTING_QUESTION: "正在生成赛题", WAITING_SEMANTIC: "正在理解赛题", SEMANTIC_READY: "任务已解析", PLAN_READY: "计划已生成", VERIFYING_CARGO: "正在确认资源", VISUAL_CONFIRMED: "资源确认完成", NAVIGATING: "正在导航", PICKING: "机械臂抓取中", PLACING: "正在投放", COMPLETED: "本轮任务完成", ERROR: "需要人工处理" };
const TASK_LABELS: Record<string, string> = { PENDING: "等待执行", VERIFYING: "识别确认中", VISUAL_CONFIRMED: "识别完成", NAVIGATING: "导航中", PICKING: "抓取中", PICKED: "已抓取", PLACING: "放置中", COMPLETED: "已完成" };

function parseJson(event: MessageEvent): unknown {
  const message = event.message as StringMessage;
  if (typeof message.data !== "string") {
    return undefined;
  }
  try { return JSON.parse(message.data) as unknown; } catch { return undefined; }
}

function updateFromFrame(previous: DashboardState, frame: readonly MessageEvent[]): DashboardState {
  let next = previous;
  for (const event of frame) {
    if (event.topic === "/semantic/question_raw") {
      const question = (event.message as StringMessage).data?.trim();
      if (question) {
        next = { ...next, question };
      }
    } else if (event.topic === "/mission/status") {
      const status = parseJson(event) as MissionStatus | undefined;
      if (status) {
        next = { ...next, status };
      }
    } else if (event.topic === "/mission/plan") {
      const plan = parseJson(event) as MissionPlan | undefined;
      if (plan) {
        next = { ...next, plan };
      }
    } else if (event.topic === "/mission/current_task") {
      const current = parseJson(event) as CurrentTask | undefined;
      if (current) {
        next = { ...next, current };
      }
    }
  }
  return next;
}

function CompetitionDashboard({ context }: { context: PanelExtensionContext }): ReactElement {
  const [dashboard, setDashboard] = useState<DashboardState>(INITIAL_STATE);
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
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
  }, [context]);
  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  const tasks = useMemo(() => dashboard.plan.tasks ?? [], [dashboard.plan.tasks]);
  const redCount = tasks.filter((task) => task.color?.toLowerCase() === "red").length;
  const blueCount = tasks.filter((task) => task.color?.toLowerCase() === "blue").length;
  const total = dashboard.plan.task_count ?? tasks.length;
  const rawIndex = dashboard.current.current_task_index ?? dashboard.status.current_task_index ?? 0;
  const activeIndex = total > 0 ? Math.min(Math.max(rawIndex, 0), total - 1) : 0;
  const current = dashboard.current.current_task ?? tasks[activeIndex];
  const completed = tasks.filter((task, index) => task.status === "COMPLETED" || index < rawIndex).length;
  const state = dashboard.status.state ?? "IDLE";
  const stateLabel = STATE_LABELS[state] ?? state;
  const currentLabel = current ? `${colorLabel(current.color)} → ${zoneLabel(current.destination)}` : "等待任务分配";
  const actionLabel = current?.status ? TASK_LABELS[current.status] ?? current.status : stateLabel;
  const confidence = dashboard.status.visual_stable_count ?? 0;

  return <main className="command-board">
    <style>{CSS}</style>
    <header className="board-header"><div className="brand-block"><span className="brand-kicker">2026 · INTELLIGENT WAREHOUSE</span><h1>机器人任务指挥台</h1></div><div className={`system-state state-${state.toLowerCase()}`}><i /><span>{stateLabel}</span></div></header>
    <section className="briefing" aria-label="当前赛题"><div className="panel-heading"><span>MISSION BRIEF</span><b>当前赛题</b></div><p>{dashboard.question}</p></section>
    <section className="overview-grid" aria-label="任务概况">
      <Overview label="资源总数" value={total > 0 ? `${total} 件` : "待解析"} note="大模型任务计划" tone="violet" />
      <Overview label="颜色构成" value={`${redCount} 红 · ${blueCount} 蓝`} note="已纳入抓取顺序" tone="blue" />
      <Overview label="当前目标" value={currentLabel} note={actionLabel} tone="amber" />
      <Overview label="执行进度" value={total > 0 ? `${completed} / ${total}` : "—"} note={total > 0 ? `正在处理第 ${activeIndex + 1} 项` : "等待计划下发"} tone="green" />
    </section>
    <section className="mission-grid">
      <div className="route-panel"><div className="panel-heading"><span>EXECUTION ROUTE</span><b>抓取与投放队列</b></div>{tasks.length > 0 ? <ol className="route-list">{tasks.map((task, index) => {
        const isCurrent = index === activeIndex; const isComplete = task.status === "COMPLETED" || index < rawIndex;
        return <li className={`route-item ${task.color?.toLowerCase() ?? "neutral"} ${isCurrent ? "current" : ""} ${isComplete ? "done" : ""}`} key={`${task.sequence ?? index}-${task.object_id ?? "resource"}`}><span className="route-number">{index + 1}</span><span className="route-copy"><b>{colorLabel(task.color)}</b><small>投放至 {zoneLabel(task.destination)}</small></span><span className="route-state">{TASK_LABELS[task.status ?? "PENDING"] ?? task.status ?? "等待执行"}</span></li>;
      })}</ol> : <div className="route-empty">赛题解析完成后，将在这里展示每一项抓取与投放任务。</div>}</div>
      <aside className="telemetry-panel" aria-label="实时状态"><div className="panel-heading"><span>LIVE TELEMETRY</span><b>执行状态</b></div><Telemetry label="当前动作" value={actionLabel} /><Telemetry label="识别稳定帧" value={confidence > 0 ? `${confidence} 帧` : "等待视觉确认"} /><Telemetry label="系统说明" value={dashboard.status.detail ?? "暂无补充信息"} soft /></aside>
    </section>
    <footer className="board-footer"><span>ROS 2 HUMBLE · ROSBRIDGE 9090</span><span>导航路径 /plan · 目标识别 /perception/annotated_image</span></footer>
  </main>;
}

function Overview({ label, value, note, tone }: { label: string; value: string; note: string; tone: string }): ReactElement { return <article className={`overview-card tone-${tone}`}><span>{label}</span><strong>{value}</strong><small>{note}</small></article>; }
function Telemetry({ label, value, soft = false }: { label: string; value: string; soft?: boolean }): ReactElement { return <div className={`telemetry-row ${soft ? "soft" : ""}`}><span>{label}</span><strong>{value}</strong></div>; }
function colorLabel(color?: string): string { if (color?.toLowerCase() === "red") { return "红色物块"; } if (color?.toLowerCase() === "blue") { return "蓝色物块"; } return "待确认物块"; }
function zoneLabel(destination?: string): string { return destination ? `${destination} 区` : "目标区域待定"; }

const CSS = `
  :root { color-scheme:dark; } * { box-sizing:border-box; }
  .command-board { min-height:100%; padding:16px 18px 12px; overflow:auto; color:#edf4ff; background:linear-gradient(135deg,#081423 0%,#0d1d31 52%,#10243d 100%); font-family:"Microsoft YaHei UI","PingFang SC",system-ui,sans-serif; }
  .board-header { display:flex; align-items:center; justify-content:space-between; gap:18px; margin-bottom:12px; }.brand-kicker,.panel-heading span { color:#75b8ff; font-size:10px; font-weight:800; letter-spacing:1.65px; } h1 { margin:3px 0 0; font-size:clamp(19px,2vw,28px); line-height:1.2; letter-spacing:.4px; }
  .system-state { display:flex; align-items:center; gap:8px; flex:0 0 auto; padding:8px 12px; border:1px solid #315376; border-radius:999px; background:#102a45; color:#d9edff; font-size:13px; font-weight:800; white-space:nowrap; }.system-state i { width:8px; height:8px; border-radius:50%; background:#50e39a; box-shadow:0 0 12px #50e39a; animation:pulse 1.7s infinite; }.state-error i { background:#ff7182; box-shadow:0 0 12px #ff7182; }
  .briefing,.route-panel,.telemetry-panel { min-width:0; border:1px solid rgba(139,185,231,.22); border-radius:12px; background:rgba(8,23,41,.68); box-shadow:0 9px 25px rgba(0,0,0,.17); }.briefing { padding:11px 14px; margin-bottom:10px; }.panel-heading { display:flex; align-items:baseline; justify-content:space-between; gap:10px; }.panel-heading b { color:#f0f6ff; font-size:13px; white-space:nowrap; }.briefing p { margin:6px 0 0; color:#dce8f7; font-size:clamp(13px,1.35vw,16px); line-height:1.5; font-weight:600; white-space:pre-wrap; overflow-wrap:anywhere; }
  .overview-grid { display:grid; grid-template-columns:1.1fr 1.2fr 1.65fr 1.1fr; gap:9px; margin-bottom:10px; }.overview-card { min-width:0; min-height:94px; padding:11px 13px; display:flex; flex-direction:column; justify-content:space-between; border-radius:11px; border:1px solid rgba(255,255,255,.12); overflow:hidden; }.overview-card span { font-size:11px; font-weight:800; opacity:.85; }.overview-card strong { font-size:clamp(17px,1.8vw,26px); line-height:1.18; overflow-wrap:anywhere; }.overview-card small { font-size:11px; line-height:1.25; opacity:.88; overflow-wrap:anywhere; }.tone-violet { background:linear-gradient(140deg,#5637a8,#7b59d0); }.tone-blue { background:linear-gradient(140deg,#155e9c,#2586c8); }.tone-amber { background:linear-gradient(140deg,#886123,#c18b30); }.tone-green { background:linear-gradient(140deg,#1d6f59,#2d9971); }
  .mission-grid { display:grid; grid-template-columns:minmax(0,1.9fr) minmax(245px,.9fr); gap:10px; }.route-panel,.telemetry-panel { padding:12px; }.route-list { list-style:none; margin:9px 0 0; padding:0; display:grid; grid-template-columns:repeat(auto-fit,minmax(200px,1fr)); gap:8px; }.route-item { min-width:0; display:grid; grid-template-columns:29px minmax(0,1fr); gap:8px; align-items:center; padding:8px; border-radius:10px; border:1px solid #2b496b; background:#10243d; }.route-item.current { border-color:#86c9ff; box-shadow:0 0 0 2px rgba(109,190,255,.17); }.route-item.done { opacity:.7; background:#133b35; }.route-item.red .route-number { background:#c3445d; }.route-item.blue .route-number { background:#3473d1; }.route-number { width:28px; height:28px; display:grid; place-items:center; border-radius:50%; background:#3c526e; font-size:13px; font-weight:900; }.route-copy { min-width:0; display:flex; flex-direction:column; gap:2px; }.route-copy b { font-size:13px; overflow-wrap:anywhere; }.route-copy small { color:#b9cce0; font-size:11px; line-height:1.25; overflow-wrap:anywhere; }.route-state { grid-column:2; color:#8ed3ff; font-size:11px; line-height:1.25; overflow-wrap:anywhere; }.route-empty { min-height:96px; margin-top:9px; padding:16px; display:grid; place-items:center; text-align:center; color:#97abc0; border:1px dashed #365574; border-radius:9px; font-size:13px; line-height:1.5; }
  .telemetry-panel { display:grid; align-content:start; gap:8px; }.telemetry-row { min-width:0; padding:9px 10px; border-radius:9px; background:rgba(29,102,78,.75); border:1px solid rgba(94,212,164,.18); }.telemetry-row.soft { background:rgba(33,63,96,.7); border-color:rgba(132,185,244,.18); }.telemetry-row span { display:block; margin-bottom:4px; color:#bddcca; font-size:11px; font-weight:800; }.telemetry-row.soft span { color:#b8d4f1; }.telemetry-row strong { display:block; font-size:14px; line-height:1.35; overflow-wrap:anywhere; white-space:pre-wrap; }
  .board-footer { display:flex; justify-content:space-between; gap:12px; margin-top:9px; color:#7894b2; font-size:10px; line-height:1.35; }.board-footer span { overflow-wrap:anywhere; } @keyframes pulse { 50% { opacity:.42; transform:scale(.78); } }
  @media (max-width:880px) { .overview-grid { grid-template-columns:repeat(2,minmax(0,1fr)); }.mission-grid { grid-template-columns:1fr; }.telemetry-panel { grid-template-columns:repeat(3,minmax(0,1fr)); }.telemetry-panel .panel-heading { grid-column:1/-1; }.route-list { grid-template-columns:repeat(2,minmax(0,1fr)); } } @media (max-width:560px) { .command-board { padding:12px; }.board-header { align-items:flex-start; flex-direction:column; gap:8px; }.overview-grid,.route-list,.telemetry-panel { grid-template-columns:1fr; }.telemetry-panel .panel-heading { grid-column:auto; }.overview-card { min-height:82px; }.board-footer { flex-direction:column; gap:3px; } }
`;

export function initCompetitionDashboard(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<CompetitionDashboard context={context} />);
  return () => {
    root.unmount();
  };
}
