param([string]$Gcc = 'D:\msys64\mingw64\bin\gcc.exe')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$testExe = Join-Path ([IO.Path]::GetTempPath()) ('rm_auto_flow_' + [Guid]::NewGuid().ToString('N') + '.exe')
try
{
    $includes = @("-I$projectRoot\application", "-I$projectRoot\components", "-I$projectRoot\bsp")
    $sources = @('motor', 'pid', 'ramp', 'mecanum') | ForEach-Object { Join-Path $projectRoot "components\$_.c" }
    & $Gcc -std=c11 -Wall -Wextra -Werror @includes (Join-Path $PSScriptRoot 'test_flow.c') @sources -lm -o $testExe
    if ($LASTEXITCODE -ne 0) { throw 'Host test compile failed' }
    & $testExe
    if ($LASTEXITCODE -ne 0) { throw 'Host flow test failed' }
}
finally
{
    if (Test-Path -LiteralPath $testExe) { Remove-Item -LiteralPath $testExe }
}
