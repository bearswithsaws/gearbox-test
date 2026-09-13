# Find the axis's top-speed ceiling: walk the speed up in 10 deg/s steps until
# it loses sync, and report the highest clean round trip.
#
#   .\runceiling.ps1              # from 60 deg/s up
#   .\runceiling.ps1 -Start 40
#
# One number per run, so belt tension attempts, grub screw tightening and
# current settings can be compared against each other. Acceleration tracks
# speed but is capped at 100 deg/s^2, which is measured clean, so this isolates
# top speed. Losing sync is harmless: the encoder is absolute and after the
# reduction, so position is recovered rather than lost.
param(
  [double]$Start = 60,
  [int]$Watch = 260
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
$p.WriteLine("C $Start")
Start-Sleep -Milliseconds 800
Write-Host ($p.ReadExisting())
$p.Close()

Start-Sleep -Seconds 2
$port = Resume-App
Write-Host "# searching on $port"
Start-Sleep -Milliseconds 1200
$p = New-Object System.IO.Ports.SerialPort $port,115200,None,8,One
$p.ReadTimeout = 200; $p.DtrEnable = $true; $p.Open()
$log = ""
$sw = [Diagnostics.Stopwatch]::StartNew()
while ($sw.Elapsed.TotalSeconds -lt $Watch) {
  try { $log += $p.ReadExisting() } catch {}
  if ($log -match 'CEILING:') {
    Start-Sleep -Milliseconds 700
    try { $log += $p.ReadExisting() } catch {}
    break
  }
  Start-Sleep -Milliseconds 100
}
$p.Close()
($log -split "`n" | Select-String -Pattern 'ceiling|deg/s|LOST SYNC|CEILING|unreachable' | ForEach-Object { $_.Line })

Start-Sleep -Seconds 2
try { Resume-App | Out-Null; Write-Host "# app running" } catch { Write-Host "# $_" }
