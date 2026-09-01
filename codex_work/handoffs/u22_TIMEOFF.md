# u22 远程连接交接文档

## 1. 项目总任务、技术栈、整体架构

### 总任务

恢复 Codex Desktop 对远程 SSH 主机 `u22` 的稳定连接，消除应用内持续“思考 / 重连”状态，并让远程环境可运行 Codex CLI。

### 技术栈与环境

- 本机：Windows；Codex Desktop（Windows Store 包 `OpenAI.Codex_26.825.6671.0_x64`）；本地 Codex CLI。
- 远程：SSH 别名 `u22`，目标 `192.168.64.128:22`，用户 `ros`，使用本机 `~/.ssh/id_ed25519`。
- 远端已识别为 Ubuntu，SSHD 标识为 `OpenSSH_8.9p1 Ubuntu-3ubuntu0.16`。
- 网络：Clash Verge / mihomo，本地 HTTP 混合代理端口 `127.0.0.1:7897`。

### 整体架构

Codex Desktop 同时维护本地 App Server 和远程 App Server。远程路径为：Codex Desktop → Windows OpenSSH 客户端 → `u22` 的 SSH → 远程 shell → 远程 `codex` CLI / App Server。任一远程 SSH 握手或远程 CLI 检查失败，Desktop 都会自动退避并重连；这就是界面反复显示重连的来源。

## 2. 已全部完成的工作

- 已定位并修正本机 Codex 代理配置：`C:\Users\奶龙\.codex.env` 的 `HTTP_PROXY`、`HTTPS_PROXY`、`ALL_PROXY` 已从旧端口 `12334` 更新为 `http://127.0.0.1:7897`。
- 已验证 Clash 的 `127.0.0.1:7897` 正在监听，且经该 HTTP 代理请求 `http://example.com/` 返回 `200`。
- 已确认 Codex Desktop 本地连接状态为 `connected`，本地 Codex/桌面进程也能连接到 `7897`；代理不是当前 `u22` 重连的直接根因。
- 已检查 Windows 时间、hosts 文件和 Chrome 企业策略；未发现它们导致本次 Codex 远程 SSH 问题的证据。
- 已从 Codex Desktop 最新日志确认重连对象、目标地址、错误类型和重连节奏。

## 3. 当前阶段、现存 BUG 与卡点

### 当前阶段

本机代理已可用；当前卡在远程 `u22` 的 SSH / 远程 CLI 准备阶段。

### 确认的 BUG / 阻塞

1. `u22` 当前在 SSH 密钥交换阶段主动断开连接：

   ```text
   kex_exchange_identification: Connection closed by remote host
   Connection closed by 192.168.64.128 port 22
   ```

   Codex Desktop 会自动重试，日志中已出现 `reconnectAttempt=2` 至 `reconnectAttempt=6`，退避后继续循环。

2. 在 SSH 偶尔能成功认证的较早一次检查中，远程环境返回：

   ```text
   No `codex` found in PATH. Please install the Codex CLI on the remote machine.
   ```

   即使 SSH 稳定，远程机也必须具备可执行的 `codex` 命令才能启动远程 App Server。

### 关键日志位置

`C:\Users\奶龙\AppData\Local\Packages\OpenAI.Codex_2p2nqsd0c76g0\LocalCache\Local\Codex\Logs\2026\08\31\codex-desktop-129e4f2c-44e0-49a7-ac33-5281e5591fbd-2100-t0-i1-141059-0.log`

## 4. 待完成任务清单

- [ ] 在远程 `u22` 上修复 SSH 服务，使本机连续执行 `ssh u22` 不再在密钥交换阶段被断开。
- [ ] 检查远程 SSHD 日志、连接限额/防护规则和虚拟机网络状态，查明主动断开的原因。
- [ ] 在远程 `u22` 安装 Codex CLI，并确保非交互登录 shell 的 `PATH` 可找到它。
- [ ] 从本机验证：`ssh u22 'command -v codex && codex --version'`。
- [ ] 回到 Codex Desktop 测试 `u22` 远程连接，确认状态变为 `connected` 且不再出现自动重连。
- [ ] 若近期不需要远程环境，可在 Codex Desktop 中断开或移除 `u22`，立即停止无意义的后台重连。

## 5. 踩坑记录 / 需要规避的错误

- 不要把本次“持续重连”归因于 OpenAI 服务、模型思考或本机代理：日志明确指向 `u22` 的 SSH 连接失败。
- 不要继续使用旧代理端口 `12334`；当前有效端口是 `7897`。修改 `.codex.env` 后应重启 Codex，确保后台进程重新读取配置。
- 不要只修复 SSH 而忽略远程 CLI：远程 `codex` 不在 `PATH` 会导致下一步仍失败。
- 不要直接删除 `~/.ssh/config`、私钥或 `known_hosts` 来“重置连接”；目前没有主机密钥不匹配证据，删除会扩大问题并可能丢失其他 SSH 配置。
- 不要因单次成功 TCP 连接就判断 SSH 正常；本问题发生在 SSH 密钥交换阶段，必须用完整的 `ssh u22` 命令验证。
- 排查远端时不要在未确认影响前重启或重置整台虚拟机；优先读取 `sshd` 状态与日志、确认连接限额和网络服务。
