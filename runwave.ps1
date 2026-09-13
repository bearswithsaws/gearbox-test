# Arm the horizon-to-horizon wave, reboot the board into it, and watch.
# Runs `Cycles` round trips at each of Speed, 1.5x Speed and 2x Speed, so the
# arm visibly gets faster. Acceleration always equals speed, meaning one
# second to full speed at any tier.
#
#   .\runwave.ps1                 # 30 / 45 / 60 deg/s
#   .\runwave.ps1 -Speed 70       # 70 / 105 / 140 deg/s, to find the ceiling
#
# Each leg's wall time is compared against the theoretical trapezoid time. A
# leg materially slower than predicted means the motor is not keeping up with
# its pulse train. A stall trip is harmless here: the AS5048A is absolute, so
# position is recovered on the spot rather than lost.
param(
  [double]$Speed = 30,
  [int]$Watch = 240
)
$ErrorActionPreference = "Continue"
$esptool = "$env:LOCALAPPDATA\Arduino15\packages\esp32\tools\esptool_py\5.3.1\esptool.exe"

function Get-BoardState {
  foreach ($r in (Get-CimInstance Win32_PnPEntity | Where-Object { $_.Name -match 'COM\d+' })) {
    if ($r.Name -match 'COM(\d+)') { $com = "COM" + $Matches[1] } else { continue }
    if ($r.DeviceID -match 'VID_239A') { return @{ Port = $com; Mode = "app" } }
    if ($r.DeviceID -match 'VID_303A&PID_1001') { return @{ Port = $com; Mode = "bootloader" } }
  }
  return $null
}
function Resume-App {
  $st = Get-BoardState
  if (-not $st) { throw "No board on USB." }
  if ($st.Mode -eq "app") { return $st.Port }
  & $esptool --port $st.Port run *> $null
  foreach ($i in 1..30) {
    Start-Sleep -Milliseconds 500
    $st = Get-BoardState
    if ($st -and $st.Mode -eq "app") { return $st.Port }
  }
  throw "Board did not return to the app. Press RESET on the Feather."
}

$port = Resume-App
$p = New-Object System.IO.Ports.SerialPort $port,115200,None,8,One
$p.ReadTimeout = 200; $p.DtrEnable = $true; $p.Open()
Start-Sleep -Milliseconds 600
$p.DiscardInBuffer()
$p.WriteLine("V $Speed")
Start-Sleep -Milliseconds 800
Write-Host ($p.ReadExisting())
$p.Close()

Start-Sleep -Seconds 2
$port = Resume-App
Write-Host "# waving on $port"
Start-Sleep -Milliseconds 1200
$p = New-Object System.IO.Ports.SerialPort $port,115200,None,8,One
$p.ReadTimeout = 200; $p.DtrEnable = $true; $p.Open()
$log = ""
$sw = [Diagnostics.Stopwatch]::StartNew()
while ($sw.Elapsed.TotalSeconds -lt $Watch) {
  try { $log += $p.ReadExisting() } catch {}
  if ($log -match 'wave finished|ABORTED') {
    Start-Sleep -Milliseconds 700
    try { $log += $p.ReadExisting() } catch {}
    break
  }
  Start-Sleep -Milliseconds 100
}
$p.WriteLine("s"); Start-Sleep -Milliseconds 1500
try { $log += $p.ReadExisting() } catch {}
$p.Close()
$log

Start-Sleep -Seconds 2
try { Resume-App | Out-Null; Write-Host "# app running" } catch { Write-Host "# $_" }
