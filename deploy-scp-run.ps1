param(
    [Parameter(Mandatory = $true)]
    [string]$LocalFile,

    [Parameter(Mandatory = $true)]
    [string]$Remote,

    [Parameter(Mandatory = $true)]
    [string]$RemotePath,

    [Parameter(Mandatory = $true)]
    [string]$RunCommand,

    [int]$Port = 22,

    [string]$IdentityFile,

    [string]$RemoteWorkingDir
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

function Test-CommandExists {
    param([string]$Name)
    return $null -ne (Get-Command $Name -ErrorAction SilentlyContinue)
}

function Escape-BashSingleQuoted {
    param([string]$Value)
    $replacement = "'" + '"' + "'" + '"' + "'"
    return $Value -replace "'", $replacement
}

if (-not (Test-Path -LiteralPath $LocalFile -PathType Leaf)) {
    throw "Local file not found: $LocalFile"
}

if (-not (Test-CommandExists -Name "scp")) {
    throw "scp was not found in PATH. Install OpenSSH Client on Windows."
}

if (-not (Test-CommandExists -Name "ssh")) {
    throw "ssh was not found in PATH. Install OpenSSH Client on Windows."
}

$resolvedLocalFile = (Resolve-Path -LiteralPath $LocalFile).Path

$remoteDir = "."
$lastSlash = $RemotePath.LastIndexOf('/')
if ($lastSlash -gt 0) {
    $remoteDir = $RemotePath.Substring(0, $lastSlash)
} elseif ($lastSlash -eq 0) {
    $remoteDir = "/"
}

if ([string]::IsNullOrWhiteSpace($RemoteWorkingDir)) {
    $RemoteWorkingDir = $remoteDir
}

$sshBaseArgs = @("-p", "$Port")
$scpBaseArgs = @("-P", "$Port")

if (-not [string]::IsNullOrWhiteSpace($IdentityFile)) {
    $sshBaseArgs += @("-i", $IdentityFile)
    $scpBaseArgs += @("-i", $IdentityFile)
}

$escapedRemoteDir = Escape-BashSingleQuoted $remoteDir
$prepareRemoteCmd = "set -e; mkdir -p '$escapedRemoteDir'"

Write-Host "[1/3] Preparing remote directory on $Remote ..."
& ssh @sshBaseArgs $Remote $prepareRemoteCmd
if ($LASTEXITCODE -ne 0) {
    throw "Remote directory preparation failed with exit code $LASTEXITCODE"
}

Write-Host "[2/3] Copying $resolvedLocalFile to ${Remote}:$RemotePath ..."
& scp @scpBaseArgs $resolvedLocalFile "${Remote}:$RemotePath"
if ($LASTEXITCODE -ne 0) {
    throw "scp failed with exit code $LASTEXITCODE"
}

$escapedWorkDir = Escape-BashSingleQuoted $RemoteWorkingDir
$remoteExec = "set -e; cd '$escapedWorkDir'; $RunCommand"

Write-Host "[3/3] Running remote command on $Remote ..."
& ssh @sshBaseArgs $Remote $remoteExec
if ($LASTEXITCODE -ne 0) {
    throw "Remote command failed with exit code $LASTEXITCODE"
}

Write-Host "Completed successfully."

# Example usage:
# .\deploy-scp-run.ps1 -LocalFile ".\scratch\sionna-rt-channel-example.cc" -Remote "ledirachh@10.108.140.148" -RemotePath "/home/ledirachh/j1/scratch/sionna-rt-channel-example.cc" -RunCommand "cd /home/ledirachh/j1; ./ns3 run scratch/sionna-rt-channel-example"
# powershell -ExecutionPolicy Bypass -File ".\deploy-scp-run.ps1" -LocalFile ".\scratch\sionna-rt-channel-example.cc" -Remote "ledirachh@10.108.140.148" -RemotePath "/home/ledirachh/j1/scratch/sionna-rt-channel-example.cc" -RunCommand "cd /home/ledirachh/j1; ./ns3 run scratch/sionna-rt-channel-example"
