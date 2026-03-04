@echo off
setlocal enabledelayedexpansion

REM Parse command-line arguments
set "LocalFile="
set "Remote="
set "RemotePath="
set "RunCommand="
set "Port=22"
set "IdentityFile="
set "RemoteWorkingDir="

:parse_args
if "%~1"=="" goto done_parsing
if "%~1"=="-LocalFile" (
    set "LocalFile=%~2"
    shift & shift
    goto parse_args
)
if "%~1"=="-Remote" (
    set "Remote=%~2"
    shift & shift
    goto parse_args
)
if "%~1"=="-RemotePath" (
    set "RemotePath=%~2"
    shift & shift
    goto parse_args
)
if "%~1"=="-RunCommand" (
    set "RunCommand=%~2"
    shift & shift
    goto parse_args
)
if "%~1"=="-Port" (
    set "Port=%~2"
    shift & shift
    goto parse_args
)
if "%~1"=="-IdentityFile" (
    set "IdentityFile=%~2"
    shift & shift
    goto parse_args
)
if "%~1"=="-RemoteWorkingDir" (
    set "RemoteWorkingDir=%~2"
    shift & shift
    goto parse_args
)
shift
goto parse_args

:done_parsing

REM Validate required arguments
if "!LocalFile!"=="" (
    echo Error: -LocalFile is required
    exit /b 1
)
if "!Remote!"=="" (
    echo Error: -Remote is required
    exit /b 1
)
if "!RemotePath!"=="" (
    echo Error: -RemotePath is required
    exit /b 1
)
if "!RunCommand!"=="" (
    echo Error: -RunCommand is required
    exit /b 1
)

REM Check if file exists
if not exist "!LocalFile!" (
    echo Error: Local file not found: !LocalFile!
    exit /b 1
)

REM Resolve full path
for %%A in ("!LocalFile!") do set "LocalFile=%%~fA"

REM Extract remote directory from RemotePath
for %%A in ("!RemotePath:/=\!") do set "FileName=%%~nxA"
set "RemoteDir=!RemotePath:%FileName%=!"
if "!RemoteDir!"=="" set "RemoteDir=."

REM Set default working directory
if "!RemoteWorkingDir!"=="" set "RemoteWorkingDir=!RemoteDir!"

REM Build SSH/SCP arguments
set "SSH_ARGS=-p !Port!"
set "SCP_ARGS=-P !Port!"

if not "!IdentityFile!"=="" (
    set "SSH_ARGS=!SSH_ARGS! -i !IdentityFile!"
    set "SCP_ARGS=!SCP_ARGS! -i !IdentityFile!"
)

REM Step 1: Prepare remote directory
echo [1/3] Preparing remote directory on !Remote! ...
ssh !SSH_ARGS! !Remote! "mkdir -p '!RemoteDir!'"
if errorlevel 1 (
    echo Error: Remote directory preparation failed
    exit /b 1
)

REM Step 2: Copy file via SCP
echo [2/3] Copying !LocalFile! to !Remote!:!RemotePath! ...
scp !SCP_ARGS! "!LocalFile!" "!Remote!:!RemotePath!"
if errorlevel 1 (
    echo Error: SCP copy failed
    exit /b 1
)

REM Step 3: show first line of the program on the remote host
if not "!RemotePath!"=="" (
    echo [3/4] Remote program first line:
    ssh !SSH_ARGS! !Remote! "head -n 1 '!RemotePath!' || echo '(could not read)'"
)

REM Step 4: Run remote command
echo [4/4] Running remote command on !Remote! ...
ssh !SSH_ARGS! !Remote! "cd '!RemoteWorkingDir!' && !RunCommand!"
if errorlevel 1 (
    echo Error: Remote command failed
    exit /b 1
)

echo Completed successfully.
exit /b 0


REM Example usage:
REM deploy-scp-run.bat -LocalFile ".\scratch\sionna-rt-channel-example.cc" -Remote "ledirachh@10.108.140.148" -RemotePath "/home/ledirachh/j1/scratch/sionna-rt-channel-example.cc" -RunCommand "cd /home/ledirachh/j1; ./ns3 run scratch/sionna-rt-channel-example"

deploy-scp-run.bat                           ← the helper script you are invoking
    -LocalFile ".\scratch\sionna-rt-channel-example.cc"
                                               ← path of the file on **your Windows PC**
    -Remote "ledirachh@10.108.140.148"
                                               ← SSH target in user@host form
    -RemotePath "/home/ledirachh/j1/scratch/sionna-rt-channel-example.cc"
                                               ← where to copy the file to on the Linux box
    -RunCommand "cd /home/ledirachh/j1; ./ns3 run scratch/sionna-rt-channel-example"
                                               ← shell command executed **after** the copy