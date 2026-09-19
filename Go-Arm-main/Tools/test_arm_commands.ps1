param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path $PSScriptRoot -Parent
$outputDir = Join-Path ([IO.Path]::GetTempPath()) ('go-arm-command-test-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $outputDir | Out-Null
$source = Get-Content -Raw (Join-Path $projectRoot 'User/src/seize_sky.c')
$header = Get-Content -Raw (Join-Path $projectRoot 'User/inc/seize_sky.h')
$parts = @()
foreach ($type in @('ArmCommand_t', 'ArmState_t')) {
    $match = [regex]::Match($header, '(?s)typedef enum\s*\{[^}]*\}\s*' + $type + ';')
    if (!$match.Success) { throw "Missing enum: $type" }
    $parts += $match.Value
}
$parts += @'
typedef struct { unsigned IdType, Identifier; } FDCAN_RxHeaderTypeDef;
#define FDCAN_EXTENDED_ID 1
struct { ArmState_t state; } ArmControl;
volatile ArmCommand_t arm_cmd = ARM_CMD_NONE;
volatile uint8_t level_flag = 1, Is_on, Is_open, Is_Sys_reset;
static uint32_t irq_mask;
static bool inject_command;
static uint32_t __get_PRIMASK(void) { return irq_mask; }
static void __disable_irq(void) { irq_mask = 1; }
static void __set_PRIMASK(uint32_t value) {
    irq_mask = value;
    if (inject_command && value == 0) {
        inject_command = false;
        arm_cmd = ARM_CMD_KEEP;
    }
}
'@
foreach ($name in @('Arm_State_Update', 'Arm_Receive')) {
    # Extract real functions; hardware and interrupt intrinsics are mocked above.
    $match = [regex]::Match($source, '(?ms)^void ' + $name + '\([^\n]*\)\s*\{.*?^\}')
    if (!$match.Success) { throw "Missing function: $name" }
    $parts += $match.Value
}
$parts -join "`n" | Set-Content -Encoding utf8 (Join-Path $outputDir 'arm_command_under_test.h')
$exePath = Join-Path $outputDir 'arm_command_test.exe'
& $Compiler -std=c11 -Wall -Wextra -Werror -I $outputDir (Join-Path $PSScriptRoot 'arm_command_test.c') -o $exePath
if ($LASTEXITCODE -ne 0) { throw 'Command test compilation failed.' }
& $exePath
if ($LASTEXITCODE -ne 0) { throw 'Command tests failed.' }
