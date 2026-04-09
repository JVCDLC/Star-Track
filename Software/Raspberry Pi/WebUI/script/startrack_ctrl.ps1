param(
    [ValidateSet("start", "sim", "stop", "status", "logs")]
    [string]$Command = "start",
    [ValidateSet("hardware", "sim")]
    [string]$Mode = "hardware"
)

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RepoRoot = Resolve-Path (Join-Path $ScriptDir "..\..\..\..")
$AppDir = Join-Path $RepoRoot "Software\Raspberry Pi\WebUI"
$EntryPoint = Join-Path $AppDir "main.py"
$LogFile = Join-Path $AppDir "system.log"
$PidFile = Join-Path $AppDir "startrack_win.pid"

$ServerPort = if ($env:STARTRACK_PORT) { $env:STARTRACK_PORT } else { "8443" }
$ArduinoPort = if ($env:STARTRACK_ARDUINO_PORT) { $env:STARTRACK_ARDUINO_PORT } else { "auto" }
$ArduinoBaud = if ($env:STARTRACK_ARDUINO_BAUD) { $env:STARTRACK_ARDUINO_BAUD } else { "115200" }

function Resolve-Python {
    $candidates = @(
        (Join-Path $RepoRoot ".venv\Scripts\python.exe"),
        (Join-Path $AppDir ".venv\Scripts\python.exe")
    )
    foreach ($candidate in $candidates) {
        if (Test-Path $candidate) {
            return $candidate
        }
    }
    throw "Python venv not found. Checked: $($candidates -join ', ')"
}

function Get-RunningProcess {
    if (!(Test-Path $PidFile)) { return $null }
    $pid = Get-Content $PidFile -ErrorAction SilentlyContinue
    if (!$pid) { return $null }
    try {
        return Get-Process -Id [int]$pid -ErrorAction Stop
    } catch {
        return $null
    }
}

function Stop-System {
    $proc = Get-RunningProcess
    if ($proc) {
        Write-Host "Stopping StarTrack PID $($proc.Id)..."
        Stop-Process -Id $proc.Id -Force
    }
    if (Test-Path $PidFile) {
        Remove-Item $PidFile -Force
    }
}

function Start-System([string]$runMode) {
    if (!(Test-Path $EntryPoint)) {
        throw "Entrypoint not found: $EntryPoint"
    }
    $existing = Get-RunningProcess
    if ($existing) {
        throw "StarTrack already running (PID $($existing.Id)). Run stop first."
    }

    $py = Resolve-Python
    if (!(Test-Path $LogFile)) { New-Item -ItemType File -Path $LogFile -Force | Out-Null }

    $simArduino = if ($runMode -eq "sim") { "1" } else { "0" }
    Write-Host "Starting StarTrack ($runMode)..."
    Write-Host "Port=$ServerPort SimArduino=$simArduino ArduinoPort=$ArduinoPort Baud=$ArduinoBaud"

    $psi = New-Object System.Diagnostics.ProcessStartInfo
    $psi.FileName = $py
    $psi.WorkingDirectory = $AppDir
    $psi.Arguments = "-u `"$EntryPoint`" --mode $runMode --port $ServerPort"
    $psi.RedirectStandardOutput = $true
    $psi.RedirectStandardError = $true
    $psi.UseShellExecute = $false
    $psi.CreateNoWindow = $true
    $psi.Environment["STARTRACK_SIM_ARDUINO"] = $simArduino
    $psi.Environment["STARTRACK_ARDUINO_PORT"] = $ArduinoPort
    $psi.Environment["STARTRACK_ARDUINO_BAUD"] = $ArduinoBaud

    $proc = New-Object System.Diagnostics.Process
    $proc.StartInfo = $psi
    $null = $proc.Start()

    Start-Job -Name "StarTrackStdout" -ScriptBlock {
        param($p, $log)
        while (!$p.HasExited) {
            $line = $p.StandardOutput.ReadLine()
            if ($line) { Add-Content -Path $log -Value $line }
        }
    } -ArgumentList $proc, $LogFile | Out-Null

    Start-Job -Name "StarTrackStderr" -ScriptBlock {
        param($p, $log)
        while (!$p.HasExited) {
            $line = $p.StandardError.ReadLine()
            if ($line) { Add-Content -Path $log -Value $line }
        }
    } -ArgumentList $proc, $LogFile | Out-Null

    Set-Content -Path $PidFile -Value $proc.Id
    Write-Host "Started with PID $($proc.Id)"
}

switch ($Command) {
    "start" { Start-System $Mode; break }
    "sim" { Start-System "sim"; break }
    "stop" { Stop-System; break }
    "status" {
        $proc = Get-RunningProcess
        if ($proc) {
            Write-Host "RUNNING PID=$($proc.Id)"
        } else {
            Write-Host "STOPPED"
        }
        break
    }
    "logs" {
        if (!(Test-Path $LogFile)) { New-Item -ItemType File -Path $LogFile -Force | Out-Null }
        Get-Content -Path $LogFile -Wait -Tail 200
        break
    }
}
