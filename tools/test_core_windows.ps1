param([Parameter(Mandatory=$true)][string]$ToolRoot)
$ErrorActionPreference = 'Stop'
$repo = Split-Path $PSScriptRoot -Parent
$compilerBin = Join-Path $ToolRoot 'w64devkit/bin'
$ipoptDir = Join-Path $ToolRoot 'casadi/casadi'
$env:Path = $compilerBin + ';' + $ipoptDir + ';' + $env:Path
$pcDir = Join-Path $repo 'build/pkgconfig'
New-Item -ItemType Directory -Force -Path $pcDir | Out-Null
$prefix = $ipoptDir.Replace('\', '/')
$pc = @"
Name: ipopt
Description: Temporary native Ipopt test library
Version: 3.14
Libs: -L$prefix -lipopt
Cflags: -I$prefix/include/coin-or
"@
[IO.File]::WriteAllText((Join-Path $pcDir 'ipopt.pc'), $pc)
$env:PKG_CONFIG_PATH = $pcDir
& (Join-Path $compilerBin 'cmake.exe') -S (Join-Path $repo 'src/obca_navigation') -B (Join-Path $repo 'build/core') -G 'MinGW Makefiles' -DOBCA_STANDALONE=ON -DCMAKE_BUILD_TYPE=Release
if ($LASTEXITCODE -ne 0) { throw 'CMake configure failed' }
& (Join-Path $compilerBin 'cmake.exe') --build (Join-Path $repo 'build/core') -j 2
if ($LASTEXITCODE -ne 0) { throw 'C++ build failed' }
& (Join-Path $compilerBin 'ctest.exe') --test-dir (Join-Path $repo 'build/core') --output-on-failure -V
if ($LASTEXITCODE -ne 0) { throw 'Core tests failed' }
