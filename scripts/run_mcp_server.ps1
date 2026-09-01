<#
.SYNOPSIS
    Start the Isaac Sim MCP server on Windows (counterpart to run_mcp_server.sh).

.DESCRIPTION
    Prefers the installed console script in the project virtual environment
    (.venv\Scripts\isaacsim-mcp-server.exe), falling back to running the server
    module from source with the venv Python.

    The MCP server speaks JSON-RPC over stdio, so this script prints nothing to
    stdout on the success path -- diagnostics go to stderr only, or the client
    would see corrupted protocol frames. It is meant to be spawned by an MCP
    client; .ps1 files are not directly spawnable, so point the client at
    `powershell -File <this script>` (see the README) or run it in a terminal.

.PARAMETER ServerArgs
    Any arguments are forwarded verbatim to the server.

.EXAMPLE
    .\run_mcp_server.ps1

.EXAMPLE
    $env:ISAAC_MCP_PORT = '8767'; .\run_mcp_server.ps1
#>
[CmdletBinding()]
param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]] $ServerArgs
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$repoRoot     = Split-Path -Parent $PSScriptRoot
$installedCli = Join-Path $repoRoot '.venv\Scripts\isaacsim-mcp-server.exe'
$pythonBin    = Join-Path $repoRoot '.venv\Scripts\python.exe'
$serverModule = Join-Path $repoRoot 'isaac_mcp\server.py'

if (-not $env:ISAAC_MCP_PORT) {
    $env:ISAAC_MCP_PORT = '8766'
}

# $ServerArgs is $null (not an empty array) when nothing trails the script name;
# normalise so splatting below is always an array.
$forward = @()
if ($ServerArgs) { $forward = $ServerArgs }

# Prefer the installed CLI entry point (pip install isaacsim-mcp-server).
if (Test-Path -LiteralPath $installedCli) {
    & $installedCli @forward
    exit $LASTEXITCODE
}

# Fall back to running from source. `-m isaac_mcp.server` resolves the package
# from the working directory, so run from the repo root. Push/Pop keeps that
# change scoped -- PowerShell has no exec to hand the process off like the .sh,
# so a bare Set-Location would leave the launcher's own location mutated. Use
# the call operator (not Start-Process) so the server keeps the parent's stdio.
if ((Test-Path -LiteralPath $pythonBin) -and (Test-Path -LiteralPath $serverModule)) {
    Push-Location -LiteralPath $repoRoot
    & $pythonBin -m isaac_mcp.server @forward
    $code = $LASTEXITCODE
    Pop-Location
    exit $code
}

[Console]::Error.WriteLine('Error: isaacsim-mcp-server not found.')
[Console]::Error.WriteLine('Install via: pip install isaacsim-mcp-server')
[Console]::Error.WriteLine('Or set up the source venv: uv sync')
exit 1
