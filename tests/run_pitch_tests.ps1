$ErrorActionPreference = 'Stop'
$repoPath = Split-Path $PSScriptRoot -Parent
$outPath = Join-Path $repoPath 'build/pitch-host-tests'
New-Item -ItemType Directory -Path $outPath -Force | Out-Null
$vswherePath = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio/Installer/vswhere.exe'
$vsPath = & $vswherePath -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $vsPath) { throw 'Visual Studio C++ Build Tools required.' }
$vcvarsPath = Join-Path $vsPath 'VC/Auxiliary/Build/vcvars64.bat'
$batch = @"
@echo off
call "$vcvarsPath" >nul
if errorlevel 1 exit /b 1
cl /nologo /utf-8 /std:c++17 /EHsc /W4 /WX /Iapplications /Itests/stubs tests/pitch_cascade_test.cpp applications/pitch_cascade.cpp /Febuild/pitch-host-tests/pitch_test.exe /Fobuild/pitch-host-tests/ /link /INCREMENTAL:NO
if errorlevel 1 exit /b 1
build\pitch-host-tests\pitch_test.exe
exit /b %errorlevel%
"@
$batchPath = Join-Path $outPath 'run.cmd'
Set-Content -LiteralPath $batchPath -Value $batch -Encoding ascii
Push-Location $repoPath
try {
  & $env:ComSpec /d /c $batchPath
  if ($LASTEXITCODE -ne 0) { throw "Pitch cascade behavior tests failed: $LASTEXITCODE" }
} finally { Pop-Location }
