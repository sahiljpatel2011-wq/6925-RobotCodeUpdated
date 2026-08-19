@echo off
set JAVA_HOME=C:\Users\Public\wpilib\2025\jdk
set PATH=%JAVA_HOME%\bin;%PATH%
cd /d "%~dp0"
call gradlew.bat simulateJava
