echo off

IF "%1%" == "" (
    set url="ws://192.168.137.29:8080/ws"
) ELSE (
    set url="ws://%1/ws"
) 

echo %url%

cd /d %~dp0
@REM cd .\_build\windows_x86_64\Release
cd .\cbuild\Release
.\momo ayame --signaling-url %url% --channel-id test