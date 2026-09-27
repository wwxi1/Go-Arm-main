$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
# 同时检查关闭和启用Event Recorder的分支；输出放到日志目录，不覆盖已跟踪的旧exe。
$testOutputDir = Join-Path $root 'Tools/logs'
New-Item -ItemType Directory -Path $testOutputDir -Force | Out-Null
foreach ($eventEnabled in 0, 1) {
    $out = Join-Path $testOutputDir "test_diag_event_$eventEnabled.exe"
    & gcc -std=c11 -Wall -Wextra -Werror "-DARM_DIAG_EVENT_ENABLE=$eventEnabled" `
        "-I$PSScriptRoot/stubs" "-I$root/User/inc" "-I$root/Motor/inc" "-I$root/Communication/VOFA" "-I$root/ThirdParty/EventRecorder" `
        "$PSScriptRoot/test_diag.c" "$root/User/src/arm_diag.c" "$root/User/src/kinematics.c" "$root/Communication/VOFA/vofa.c" -lm -o $out
    if ($LASTEXITCODE -ne 0) { throw '本地测试编译失败' }
    & $out
    if ($LASTEXITCODE -ne 0) { throw '本地测试未通过' }
}
