param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path $PSScriptRoot -Parent
$outputDir = Join-Path $PSScriptRoot 'review_output'
New-Item -ItemType Directory -Force $outputDir | Out-Null
Get-Content (Join-Path $projectRoot 'User/inc/seize_sky.h') |
    Where-Object { $_ -match '^#define ARM_(U1|U2|DJ)_\w+_POS\s' } |
    Set-Content (Join-Path $outputDir 'preset_snapshot.h')
$exePath = Join-Path $outputDir 'kinematics_review.exe'
& $Compiler -std=c11 -Wall -Wextra -I (Join-Path $projectRoot 'User/inc') -I (Join-Path $projectRoot 'Algorithm/inc') -I $outputDir (Join-Path $PSScriptRoot 'kinematics_review.c') (Join-Path $projectRoot 'User/src/kinematics.c') -lm -o $exePath
if ($LASTEXITCODE -ne 0) { throw 'Host compilation failed.' }
& $exePath | Tee-Object -FilePath (Join-Path $outputDir 'kinematics_review_results.txt')
if ($LASTEXITCODE -ne 0) { throw 'Kinematics audit failed.' }
