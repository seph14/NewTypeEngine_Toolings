@echo off
rem Build + run the Stage-1 tetrahedral-cage builder/validator (no engine deps).
rem Usage: run.bat [input.(obj|vat)] [extra args...]
rem   no args -> first .vat or .obj found in assets\ (Gate-1 run)
setlocal
for /f "usebackq tokens=*" %%i in (`"%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSPATH=%%i"
call "%VSPATH%\VC\Auxiliary\Build\vcvars64.bat" >nul || exit /b 1
cl /nologo /std:c++20 /EHsc /O2 /utf-8 /DNOMINMAX /W3 ^
  /Fo"%TEMP%\tet_cage.obj" "%~dp0src\tet_cage.cpp" /Fe:"%~dp0tet_cage.exe" || exit /b 1
if not "%~1"=="" (
  "%~dp0tet_cage.exe" %*
  endlocal & exit /b %errorlevel%
)
if not exist "%~dp0assets" mkdir "%~dp0assets"
set "INPUT="
for %%f in ("%~dp0assets\*.vat") do if not defined INPUT set "INPUT=%%~f"
if not defined INPUT for %%f in ("%~dp0assets\*.obj") do if not defined INPUT set "INPUT=%%~f"
if defined INPUT (
  echo tet_cage: no input given, using %INPUT%
  "%~dp0tet_cage.exe" "%INPUT%"
) else (
  echo tet_cage: drop a source .vat or .obj into TetCage\assets\ ^(or pass one: run.bat model.vat^)
  endlocal & exit /b 2
)
endlocal
