param(
    [string]$Gcc = 'D:\msys64\mingw64\bin\gcc.exe',
    [string]$Gpp
)
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
if (-not $Gpp)
{
    $Gpp = Join-Path (Split-Path -Parent (Get-Command $Gcc).Source) 'g++.exe'
}
$testDir = Join-Path ([IO.Path]::GetTempPath()) ('rm_auto_flow_' + [Guid]::NewGuid().ToString('N'))
$testExe = Join-Path $testDir 'test_flow.exe'
$null = New-Item -ItemType Directory -Path $testDir
$objects = @()
try
{
    $flags = @('-Wall', '-Wextra', '-Werror', "-I$projectRoot\application", "-I$projectRoot\components", "-I$projectRoot\bsp")
    $cSources = @((Join-Path $PSScriptRoot 'test_flow.c'), (Join-Path $projectRoot 'application\app_chassis_io.c'), (Join-Path $projectRoot 'application\app_chassis_control.c')) + @('motor', 'pid', 'ramp', 'mecanum' | ForEach-Object { Join-Path $projectRoot "components\$_.c" })
    $cppSources = @((Join-Path $projectRoot 'components\wheel.cpp'), (Join-Path $projectRoot 'application\app_chassis_wheels.cpp'))

    # 各自按 C 和 C++ 编译，最后用 C++ 编译器链接到同一个程序。
    foreach ($source in ($cSources + $cppSources))
    {
        $object = Join-Path $testDir ([IO.Path]::GetFileName($source) + '.o')
        $objects += $object
        if ($source.EndsWith('.cpp'))
        {
            & $Gpp -std=c++11 @flags -c $source -o $object
        }
        else
        {
            & $Gcc -std=c11 @flags -c $source -o $object
        }
        if ($LASTEXITCODE -ne 0) { throw "Host test compile failed: $source" }
    }
    & $Gpp @objects -lm -o $testExe
    if ($LASTEXITCODE -ne 0) { throw 'Host test link failed' }
    & $testExe
    if ($LASTEXITCODE -ne 0) { throw 'Host flow test failed' }
}
finally
{
    foreach ($file in ($objects + $testExe))
    {
        if (Test-Path -LiteralPath $file) { Remove-Item -LiteralPath $file }
    }
    Remove-Item -LiteralPath $testDir
}
