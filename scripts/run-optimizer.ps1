param(
    [ValidateRange(1,10000)][int]$Budget = 30,
    [string]$RunDirectory = '',
    [switch]$Resume
)
$ErrorActionPreference='Stop'
$projectRoot=Split-Path -Parent $PSScriptRoot
# dotnet: RFR_DOTNET, then PATH, then the default install folder.
$dotnetExe=$env:RFR_DOTNET
if(-not $dotnetExe){$dotnetExe=(Get-Command dotnet -CommandType Application -ErrorAction SilentlyContinue | Select-Object -First 1).Source}
if(-not $dotnetExe){$dotnetExe='C:/Program Files/dotnet/dotnet.exe'}
# Python: RFR_PYTHON, then the Codex runtime this script has always used (if present), then PATH.
$pythonExe=$env:RFR_PYTHON
if(-not $pythonExe){
    $pythonExe='C:/Users/adarz/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe'
    if(-not (Test-Path -LiteralPath $pythonExe)){$pythonExe=(Get-Command python -CommandType Application -ErrorAction SilentlyContinue | Select-Object -First 1).Source}
}
if(-not $pythonExe){throw 'Python not found; set RFR_PYTHON to the full path of python.exe'}
Push-Location -LiteralPath $projectRoot
try {
    if($Resume -and -not $RunDirectory){throw 'Resume requires the existing RunDirectory.'}
    if(-not $RunDirectory){$RunDirectory=Join-Path $projectRoot ('cad/tests/optimizer-'+[DateTime]::UtcNow.ToString('yyyyMMdd-HHmmss-fff'))}
    if(-not $Resume){
        & $dotnetExe build scripts/native/RfrEngineering.csproj --no-restore --nologo
        if($LASTEXITCODE -ne 0){throw 'Native helper build failed'}
        & $pythonExe -X utf8 scripts/make-optimizer-spec.py
        if($LASTEXITCODE -ne 0){throw 'Spec preparation failed'}
    }
    Write-Host 'Running bounded native chassis optimization. Each candidate must pass geometry and fit checks, then native torsion, mesh and reversal checks.'
    & $pythonExe -X utf8 scripts/optimize-chassis.py --spec design/optimizer-baseline.json --run $RunDirectory --budget $Budget
    $runExit=$LASTEXITCODE
    Write-Host "Results: $RunDirectory/summary.json"
    if($runExit -eq 2){Write-Host 'No eligible chassis score; see gate results. This is not a completed optimization.'}
    exit $runExit
} finally {Pop-Location}
