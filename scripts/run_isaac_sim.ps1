<#
.SYNOPSIS
    Launch Isaac Sim on Windows with the isaac.sim.mcp_extension enabled.

.DESCRIPTION
    Windows counterpart to scripts/run_isaac_sim.sh. Resolves the Isaac Sim
    launcher, verifies the extension manifest is present, then starts Isaac Sim
    with this repo registered as an extension search folder.

    The MCP extension opens a TCP socket (default localhost:8766) that the
    isaacsim-mcp-server process connects to. Because that socket accepts
    arbitrary Python via the execute_script command, it is bound to localhost
    and must never be exposed to another host.

.PARAMETER IsaacSimRoot
    Directory containing isaac-sim.bat. Defaults to $env:ISAACSIM_ROOT, then to
    the local source build at <repo>/_build/windows-x86_64/release.

.PARAMETER Port
    TCP port for the extension socket. Defaults to $env:ISAAC_MCP_PORT or 8766.

.PARAMETER ExtraArgs
    Any remaining arguments are forwarded verbatim to isaac-sim.bat.

.EXAMPLE
    .\run_isaac_sim.ps1

.EXAMPLE
    .\run_isaac_sim.ps1 -Port 8767 -- --no-window
#>
[CmdletBinding()]
param(
    [string] $IsaacSimRoot,
    [int]    $Port,
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]] $ExtraArgs
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$repoRoot     = Split-Path -Parent $PSScriptRoot
$extensionId  = 'isaac.sim.mcp_extension'
$manifestPath = Join-Path $repoRoot "$extensionId\config\extension.toml"

if (-not (Test-Path -LiteralPath $manifestPath)) {
    throw "Extension manifest not found at: $manifestPath`nRun this script from inside the isaacsim-mcp-server checkout."
}

if (-not $Port) {
    $Port = if ($env:ISAAC_MCP_PORT) { [int] $env:ISAAC_MCP_PORT } else { 8766 }
}

# Resolve the Isaac Sim install: explicit param, then env var, then local build.
$candidates = @()
if ($IsaacSimRoot)          { $candidates += $IsaacSimRoot }
if ($env:ISAACSIM_ROOT)     { $candidates += $env:ISAACSIM_ROOT }
$candidates += (Join-Path $repoRoot '..\IsaacSim\_build\windows-x86_64\release')
$candidates += (Join-Path $env:USERPROFILE 'isaacsim')

$launcher = $null
foreach ($candidate in $candidates) {
    if (-not $candidate) { continue }
    $resolved = Join-Path $candidate 'isaac-sim.bat'
    if (Test-Path -LiteralPath $resolved) {
        $launcher = (Resolve-Path -LiteralPath $resolved).Path
        break
    }
}

if (-not $launcher) {
    throw @"
Isaac Sim launcher (isaac-sim.bat) not found. Checked:
$($candidates -join "`n")

Set ISAACSIM_ROOT to your Isaac Sim directory, or pass -IsaacSimRoot.
"@
}

# Warn early instead of letting the extension fail to bind inside Isaac Sim.
function Test-PortInUse {
    param([int] $ProbePort)
    $client = [System.Net.Sockets.TcpClient]::new()
    try {
        return $client.ConnectAsync('127.0.0.1', $ProbePort).Wait(300)
    } catch {
        return $false
    } finally {
        $client.Dispose()
    }
}

if (Test-PortInUse -ProbePort $Port) {
    Write-Warning "Port $Port already has a listener. The MCP extension will fail to bind. Use -Port to pick another, and set ISAAC_MCP_PORT to match for the MCP server."
}

# Give USD/3D-generation helpers a writable working dir on Windows.
if (-not $env:USD_WORKING_DIR) {
    $env:USD_WORKING_DIR = Join-Path $repoRoot '.cache\usd'
}
New-Item -ItemType Directory -Path $env:USD_WORKING_DIR -Force | Out-Null

Write-Host "Repo root:  $repoRoot"
Write-Host "Isaac Sim:  $launcher"
Write-Host "Extension:  $extensionId"
Write-Host "MCP port:   $Port (localhost only)"
Write-Host "USD workdir: $env:USD_WORKING_DIR"
Write-Host ''

$argList = @(
    '--ext-folder', $repoRoot
    '--enable',     $extensionId
    "--/exts/isaac.sim.mcp/server.port=$Port"
    '--/exts/isaac.sim.mcp/server.host=localhost'
)
if ($ExtraArgs) {
    $argList += ($ExtraArgs | Where-Object { $_ -ne '--' })
}

& $launcher @argList
exit $LASTEXITCODE
