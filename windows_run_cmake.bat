@echo off

set CMAKE_CONFIGURATION_FILE=%1
if '%1'=='' (
	set CMAKE_CONFIGURATION_FILE=..\user.cmake
	set filename=user
) else (
	set filename=%~n1
)

REM echo CMAKE_CONFIGURATION_FILE is %filename%

set PATH_TO_BUILD_TREE=%2
if '%2'=='' (
	set PATH_TO_BUILD_TREE="%~dp1build_%filename%"
)
REM echo PATH_TO_BUILD_TREE is '%PATH_TO_BUILD_TREE%'

set PATH_TO_SOURCE_TREE=%3
if '%3'=='' (
	if '%1'=='' (
		set PATH_TO_SOURCE_TREE=..
	) else (
		set PATH_TO_SOURCE_TREE="%~dp1"
	)
	
)
REM echo PATH_TO_SOURCE_TREE is '%PATH_TO_SOURCE_TREE%'

set SUPERFLUOUS_ARGS=%4
if NOT '%SUPERFLUOUS_ARGS%'=='' (
	echo WARNING: superfluous arguments '%SUPERFLUOUS_ARGS%' ignored.
	pause
)


mkdir %PATH_TO_BUILD_TREE% 2> NUL
chdir /D %PATH_TO_BUILD_TREE%
if ERRORLEVEL 1 (
	echo ERROR: Change to '%PATH_TO_BUILD_TREE%' failed!
	pause
	exit /B 2
)


echo on
cmake -C %CMAKE_CONFIGURATION_FILE% %PATH_TO_SOURCE_TREE%
@echo off
if ERRORLEVEL 1 (
	echo ERROR: Cmake failed!
	pause
	exit /B 3
)

pause
for %%i in (*.sln) do set SLN_FILE="%%~dpnxi"
REM echo SLN_FILE is %SLN_FILE%
if NOT DEFINED SLN_FILE (
	goto NO_SHORTCUT
)
set TARGET=%SLN_FILE%
set SHORTCUT=..\start_vs_%filename%.lnk
set PWS=powershell.exe -ExecutionPolicy Bypass -NoLogo -NonInteractive -NoProfile
%PWS% -Command "$ws=New-Object -ComObject WScript.Shell; $sc=$ws.CreateShortcut('%SHORTCUT%'); $sc.TargetPath='%TARGET%'; $sc.Description='Open project in Visual Studio'; $sc.Save()"
if ERRORLEVEL 1 (
	echo ERROR: Creation of the shortcut to the "Visual Studio Solution (sln)" file failed!
	pause
	exit /B 4
)
:NO_SHORTCUT

