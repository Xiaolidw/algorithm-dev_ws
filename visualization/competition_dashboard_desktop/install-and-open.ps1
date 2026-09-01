$ErrorActionPreference = "Stop"

$projectDirectory = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location -LiteralPath $projectDirectory

if (-not (Test-Path -LiteralPath (Join-Path $projectDirectory "node_modules"))) {
    npm install
    if ($LASTEXITCODE -ne 0) {
        throw "npm install failed with exit code $LASTEXITCODE"
    }
}

npm run local-install
if ($LASTEXITCODE -ne 0) {
    throw "coStudio extension installation failed with exit code $LASTEXITCODE"
}

$desktopShortcut = Join-Path ([Environment]::GetFolderPath("Desktop")) "coStudio.lnk"
if (-not (Test-Path -LiteralPath $desktopShortcut)) {
    throw "coStudio desktop shortcut was not found: $desktopShortcut"
}

# Explorer launches coStudio without inheriting this script's stdout pipe.
# This avoids Electron's EPIPE/broken-pipe error.
Start-Process -FilePath "explorer.exe" -ArgumentList ('"' + $desktopShortcut + '"')

Write-Host "Competition dashboard installed."
Write-Host "Open layout file: 2026-competition-dashboard-layout.json"
Write-Host "Rosbridge URL: ws://192.168.64.128:9090"
