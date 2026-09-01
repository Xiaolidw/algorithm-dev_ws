import { ExtensionContext } from "@coscene/extension";

import { initCompetitionDashboard } from "./CompetitionDashboard";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({
    name: "大模型技术创新赛 · 比赛看板",
    initPanel: initCompetitionDashboard,
  });
}
