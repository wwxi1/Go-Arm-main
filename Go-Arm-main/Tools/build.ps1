<#
============================================================================
 build.ps1 —— Keil uVision 命令行编译脚本(移植自 agent-work/build.ps1)
============================================================================
用法(在仓库根目录执行):

  powershell -File Tools\build.ps1              # 增量编译本工程(默认)
  powershell -File Tools\build.ps1 -Rebuild     # 全部重新编译
  powershell -File Tools\build.ps1 -Project "E:\...\xxx.uvprojx"  # 编译其他工程
  powershell -File Tools\build.ps1 -UV4 "D:\Keil_v5\UV4\UV4.exe"  # 指定 uVision 路径

说明:
  - 编译日志写到 Tools\logs\<工程名>_build.log;
  - 唯一环境依赖是 uVision 的 UV4.exe 路径(默认 E:\keil5\UV4\UV4.exe);
  - UV4 批处理退出码:0 = 无错误无警告, 1 = 有警告(均视为成功,脚本退出码为 0),
    >= 2 = 编译失败(脚本退出码非 0)。
============================================================================
#>
param(
    [string]$Project = "",   # 留空时默认本工程 MDK-ARM 下的 uvprojx
    [switch]$Rebuild,        # 存在则全部重新编译(-r),否则增量(-b)
    [string]$UV4 = "E:\keil5\UV4\UV4.exe",
    [string]$LogDir = ""     # 留空时默认本脚本同目录 logs
)

# 默认工程路径:脚本所在目录的上一级 MDK-ARM
if (-not $Project) {
    $Project = Join-Path $PSScriptRoot '..\MDK-ARM\27RC_Proj_Template.uvprojx'
}
# 默认日志目录:脚本同目录 logs
if (-not $LogDir) {
    $LogDir = Join-Path $PSScriptRoot 'logs'
}

if (-not (Test-Path $Project)) {
    Write-Host "[build] 找不到工程文件: $Project"
    exit 2
}
if (-not (Test-Path $UV4)) {
    Write-Host "[build] 找不到 UV4.exe: $UV4(用 -UV4 参数指定 uVision 安装路径)"
    exit 2
}

$mode = if ($Rebuild) { '-r' } else { '-b' }
$projName = [System.IO.Path]::GetFileNameWithoutExtension($Project)
$log = Join-Path $LogDir "$projName`_build.log"

New-Item -ItemType Directory -Path $LogDir -Force | Out-Null
Remove-Item $log -ErrorAction SilentlyContinue

Write-Host "[build] $mode $Project"
Write-Host "[build] log -> $log"

$p = Start-Process -FilePath $UV4 `
    -ArgumentList @($mode, $Project, '-j0', '-o', $log) `
    -Wait -PassThru

if (Test-Path $log) {
    Write-Host "[build] --- log tail ---"
    Get-Content $log -Tail 30
}

# UV4 退出码:0/1 都算成功(0 = 干净,1 = 仅有警告),>=2 为失败
if ($p.ExitCode -le 1) {
    Write-Host "[build] SUCCESS (UV4 exit=$($p.ExitCode); 0=clean, 1=warnings-only)"
    exit 0
} else {
    Write-Host "[build] FAILED (UV4 exit=$($p.ExitCode))"
    exit $p.ExitCode
}
