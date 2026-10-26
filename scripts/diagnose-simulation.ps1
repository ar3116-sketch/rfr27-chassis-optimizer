$ErrorActionPreference = 'Stop'
Add-Type @'
using System;
using System.Text;
using System.Runtime.InteropServices;
public static class RfrMsiDiagnostics {
 [DllImport("msi.dll",CharSet=CharSet.Unicode)] public static extern uint MsiEnumFeatures(string product,uint index,StringBuilder feature,StringBuilder parent);
 [DllImport("msi.dll",CharSet=CharSet.Unicode)] public static extern int MsiQueryFeatureState(string product,string feature);
}
'@
$rfrRoot = Split-Path $PSScriptRoot -Parent
$rfrProduct = Get-ChildItem 'HKLM:\SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall' | Get-ItemProperty | Where-Object DisplayName -EQ 'SOLIDWORKS 2026 SP04.1'
if (@($rfrProduct).Count -ne 1) { throw 'Expected one installed SOLIDWORKS product' }
$rfrFeatures = @()
for ($rfrIndex=0; $rfrIndex -lt 1000; $rfrIndex++) {
 $rfrName = [Text.StringBuilder]::new(256); $rfrParent = [Text.StringBuilder]::new(256)
 $rfrResult = [RfrMsiDiagnostics]::MsiEnumFeatures($rfrProduct.PSChildName,$rfrIndex,$rfrName,$rfrParent)
 if ($rfrResult -eq 259) { break }; if ($rfrResult -ne 0) { throw "MsiEnumFeatures error $rfrResult" }
 if ($rfrName.ToString() -match 'Simulation|Cosmos|Motion') {
  $rfrState = [RfrMsiDiagnostics]::MsiQueryFeatureState($rfrProduct.PSChildName,$rfrName.ToString())
  $rfrFeatures += [pscustomobject]@{feature=$rfrName.ToString();parent=$rfrParent.ToString();state=$rfrState;meaning=(@{1='advertised';2='absent';3='local';4='source';5='default'})[$rfrState]}
 }
}
$rfrSolidWorksDir = if ($env:SolidWorksDir) { $env:SolidWorksDir } else { 'C:\Program Files\SOLIDWORKS Corp\SOLIDWORKS' }
$rfrDll = Join-Path $rfrSolidWorksDir 'Simulation\cosworks.dll'
$rfrSignature = Get-AuthenticodeSignature -LiteralPath $rfrDll
$rfrRegistration = @('Registry::HKEY_CLASSES_ROOT\SldWorks.Simulation','Registry::HKEY_CLASSES_ROOT\SldWorks.Simulation.19','HKLM:\SOFTWARE\SolidWorks\SOLIDWORKS 2026\Applications\SOLIDWORKS Simulation') | ForEach-Object { @{path=$_;present=(Test-Path -LiteralPath $_)} }
$rfrEvidence=Join-Path $rfrRoot ('handoff/evidence/simulation-installation-'+(Get-Date -Format 'yyyyMMdd-HHmmss')+'.json')
$rfrRecord=[ordered]@{observedUtc=[DateTime]::UtcNow.ToString('o');product=$rfrProduct.DisplayName;productCode=$rfrProduct.PSChildName;installSource=$rfrProduct.InstallSource;sourcePresent=(Test-Path -LiteralPath $rfrProduct.InstallSource);features=$rfrFeatures;registration=$rfrRegistration;dll=@{path=$rfrDll;signatureStatus=$rfrSignature.Status.ToString();publisher=$rfrSignature.SignerCertificate.Subject;sha256=(Get-FileHash -LiteralPath $rfrDll -Algorithm SHA256).Hash};changesMade=$false}
$rfrRecord | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $rfrEvidence -Encoding utf8
$rfrRecord | ConvertTo-Json -Depth 8
