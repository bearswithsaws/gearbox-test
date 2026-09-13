# Build / flash / monitor the ElevationSweep sketch with the arduino-cli that
# ships inside Arduino IDE 2.x. Usage:
#   .\build.ps1                 compile only
#   .\build.ps1 upload          compile + upload (auto-detects the Feather's COM port)
#   .\build.ps1 upload COM7     compile + upload to a specific port
#   .\build.ps1 monitor [COM7]  serial monitor at 115200
param(
  [string]$Action = "compile",
  [string]$Port = ""
)
$ErrorActionPreference = "Continue"   # native tools print warnings on stderr; PS 5.1 would treat those as fatal
$cli = "C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
$cfg = "$env:USERPROFILE\.arduinoIDE\arduino-cli.yaml"
$fqbn = "esp32:esp32:adafruit_feather_esp32s3"
$sketch = Join-Path $PSScriptRoot "ElevationSweep"
$lib = Join-Path $PSScriptRoot "GearboxAxis"

function Find-Port {
  $rows = & $cli --config-file $cfg board list --format json | ConvertFrom-Json
  $list = if ($rows.detected_ports) { $rows.detected_ports } else { $rows }
  foreach ($r in $list) {
    $addr = $r.port.address
    $ids = $r.port.properties
    if ($ids -and (($ids.vid -match '239A') -or ($ids.vid -match '303A'))) { return $addr }
    if ($r.matching_boards) { return $addr }
  }
  return $null
}

switch ($Action) {
  "compile" {
    & $cli --config-file $cfg compile --fqbn $fqbn --library $lib $sketch
    exit $LASTEXITCODE
  }
  "upload" {
    if (-not $Port) { $Port = Find-Port }
    if (-not $Port) { throw "No Feather found. Plug it in, or pass the COM port." }
    & $cli --config-file $cfg compile --fqbn $fqbn --library $lib $sketch
    if ($LASTEXITCODE -ne 0) { throw "compile failed" }
    & $cli --config-file $cfg upload --fqbn $fqbn -p $Port $sketch
    exit $LASTEXITCODE
  }
  "monitor" {
    if (-not $Port) { $Port = Find-Port }
    if (-not $Port) { throw "No Feather found." }
    & $cli --config-file $cfg monitor -p $Port -c baudrate=115200
  }
  default { throw "unknown action $Action" }
}
