# 大模型技术创新赛 · 比赛看板

面向 2026 大模型技术创新赛智能仓储赛题的 coStudio 自定义面板。

实时订阅：

- `/semantic/question_raw`：当前赛题；
- `/mission/plan`：抓取顺序、红蓝资源数量与放置区域；
- `/mission/current_task`：当前执行对象和任务进度；
- `/mission/status`：系统阶段与详细状态。

面板可直接编辑赛题，并设置变量 `x`（红色）、`y`（蓝色）的目标区域和执行先后顺序。点击“发布并执行”后，面板向 `/costudio/mission_request` 发布 `std_msgs/msg/String` JSON；虚拟机本地 Qwen 模型完成变量求解，随后自动生成并执行最多 5 个物块的批任务。任务 active 时按钮自动禁用，避免重复发布。

“赛题用时”位于“系统详情”下方，从网关收到 CoStudio 发布请求开始每 0.25 秒刷新；任务完成或失败后冻结为网关记录的端到端时间，刷新面板后也能从 `/mission/execution_status` 恢复。

## Develop

Extension development uses the `npm` package manager to install development dependencies and run build scripts.

To install extension dependencies, run `npm` from the root of the extension package.

```sh
npm install
```

To build and install the extension into your coStudio, run:

```sh
npm run local-install
```

重新打开 coStudio（或按 `Ctrl+R`），在“添加面板”中选择“比赛看板”。

也可以右键以 PowerShell 运行 `install-and-open.ps1`，脚本会自动安装扩展并通过桌面快捷方式打开 coStudio，避免 Electron 输出管道导致的 EPIPE 错误。

首次迁移到其他电脑时，在 coStudio 的布局菜单选择“本地文件导入”，导入 `2026-competition-dashboard-layout.json`。

使用 Windows SSH 隧道时，Rosbridge 连接地址为 `ws://localhost:9090`；直连虚拟机时使用 `ws://192.168.64.128:9090`。

## Package

Extensions are packaged into `.coe` files. These files contain the metadata (package.json) and the build code for the extension.

Before packaging, make sure to set `name`, `publisher`, `version`, and `description` fields in _package.json_. When ready to distribute the extension, run:

```sh
npm run package
```

This command will package the extension into a `.coe` file in the local directory.
