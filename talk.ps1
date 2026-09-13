# Send a scripted sequence of serial commands to the running firmware and
# print everything it says back.
#
#   .\talk.ps1 -Script "s|1500"
#   .\talk.ps1 -Script "r|300|m 800|8000|r|300"
#
# Entries are separated by "|". A bare number is a wait in milliseconds;
# anything else is a command line sent to the board.
#
# Why this exists: this board's USB CDC needs DTR asserted before it will
# send anything, and dropping DTR when the session closes knocks the ESP32-S3
# into its ROM download mode, where the app is not running at all. So this
# script resets the board into the app before talking, and again afterwards,
# using the esptool that ships with the ESP32 core. A physical RESET press
# does the same thing by hand.
param(
  [string]$Script = "s|1500",
  [int]$OpenSettleMs = 600
)
$ErrorActionPreference = "Continue"
$esptool = "$env:LOCALAPPDATA\Arduino15\packages\esp32\tools\esptool_py\5.3.1\esptool.exe"

# 239A = Adafruit TinyUSB, the app is running. 303A:1001 = ESP32-S3 ROM
# USB-Serial/JTAG, i.e. the board is sitting in download mode.
function Get-BoardState {
  $rows = Get-CimInstance Win32_PnPEntity | Where-Object { $_.Name -match 'COM(\d+)' }
  foreach ($r in $rows) {
    if ($r.Name -match 'COM(\d+)') { $com = "COM" + $Matches[1] } else { continue }
    if ($r.DeviceID -match 'VID_239A') { return @{ Port = $com; Mode = "app" } }
    if ($r.DeviceID -match 'VID_303A&PID_1001') { return @{ Port = $com; Mode = "bootloader" } }
  }
  return $null
}

function Resume-App {
  $st = Get-BoardState
  if (-not $st) { throw "No board found on USB." }
  if ($st.Mode -eq "app") { return $st.Port }
  & $esptool --port $st.Port run *> $null
  foreach ($i in 1..30) {
    Start-Sleep -Milliseconds 500
    $st = Get-BoardState
    if ($st -and $st.Mode -eq "app") { return $st.Port }
  }
  throw "Board did not come back as the running app. Press RESET on the Feather."
}

$port = Resume-App
Write-Host "# talking to $port"

$p = New-Object System.IO.Ports.SerialPort $port,115200,None,8,One
$p.ReadTimeout = 200
$p.DtrEnable = $true
$p.Open()
Start-Sleep -Milliseconds $OpenSettleMs
$p.DiscardInBuffer()
$script:log = ""
function Pump([int]$ms) {
  $sw = [Diagnostics.Stopwatch]::StartNew()
  while ($sw.ElapsedMilliseconds -lt $ms) {
    try { $script:log += $p.ReadExisting() } catch {}
    Start-Sleep -Milliseconds 50
  }
}
foreach ($entry in $Script.Split("|")) {
  $e = $entry.Trim()
  if ($e -eq "") { continue }
  $n = 0
  if ([int]::TryParse($e, [ref]$n)) { Pump $n } else { $p.WriteLine($e); Pump 250 }
}
Pump 400
$p.Close()
$script:log

# Leave the board running the app rather than parked in the bootloader.
Start-Sleep -Seconds 2
try { Resume-App | Out-Null; Write-Host "# app running" } catch { Write-Host "# $_" }
