$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$out = Join-Path $PSScriptRoot 'test_diag.exe'
& gcc -std=c11 -Wall -Wextra -Werror -DARM_DIAG_EVENT_ENABLE=0 `
    "-I$PSScriptRoot/stubs" "-I$root/User/inc" "-I$root/Communication/VOFA" `
    "$PSScriptRoot/test_diag.c" "$root/User/src/arm_diag.c" "$root/Communication/VOFA/vofa.c" -o $out
if ($LASTEXITCODE -ne 0) { throw 'Host compilation failed' }
& $out
if ($LASTEXITCODE -ne 0) { throw 'Host tests failed' }
