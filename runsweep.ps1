# Arm the level-and-staged-sweep boot action, reboot the board into it, then
# attach and capture the run live. The sweep runs from setup(), so it does not
# depend on this session staying open; attaching is only to watch. Opening a
# port does not disturb the board, closing one resets it, which is why the
# sweep is started by a boot action rather than by a serial command.
#
#   .\runsweep.ps1                 # 0 -> 180 -> 0
#   .\runsweep.ps1 -To 90 -Watch 150
param(
  [double]$To = 180,
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
function Talk([string]$port, [scriptblock]$body) {
  $p = New-Object System.IO.Ports.SerialPort $port,115200,None,8,One
  $p.ReadTimeout = 200
  $p.DtrEnable = $true
  $p.Open()
  Start-Sleep -Milliseconds 600
  try { & $body $p } finally { $p.Close() }
}

# --- 1. record the up direction and arm the sweep -------------------------
$port = Resume-App
Write-Host "# arming on $port"
Talk $port {
  param($p)
  $p.DiscardInBuffer()
  $p.WriteLine("y"); Start-Sleep -Milliseconds 400
  $p.WriteLine("W $To"); Start-Sleep -Milliseconds 600
  Write-Host ($p.ReadExisting())
}

# --- 2. reboot into the sweep and watch ----------------------------------
Start-Sleep -Seconds 2
$port = Resume-App
Write-Host "# sweeping, watching $port for ${Watch}s"
Start-Sleep -Milliseconds 1500
$log = ""
Talk $port {
  param($p)
  $sw = [Diagnostics.Stopwatch]::StartNew()
  while ($sw.Elapsed.TotalSeconds -lt $Watch) {
    try { $script:log += $p.ReadExisting() } catch {}
    if ($script:log -match 'staged sweep complete|ABORTED|refused at') {
      Start-Sleep -Milliseconds 500
      try { $script:log += $p.ReadExisting() } catch {}
      break
    }
    Start-Sleep -Milliseconds 100
  }
  $p.WriteLine("P"); Start-Sleep -Milliseconds 2500
  try { $script:log += $p.ReadExisting() } catch {}
  $p.WriteLine("s"); Start-Sleep -Milliseconds 2000
  try { $script:log += $p.ReadExisting() } catch {}
}
$log

Start-Sleep -Seconds 2
try { Resume-App | Out-Null; Write-Host "# app running" } catch { Write-Host "# $_" }
