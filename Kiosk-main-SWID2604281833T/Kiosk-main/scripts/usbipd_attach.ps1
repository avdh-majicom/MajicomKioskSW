param(
  [string]$BusId,
  [string]$Distribution = "Debian"
)

$ErrorActionPreference = "Stop"

function Get-UsbIpdDevices {
  $lines = & usbipd list
  $devices = @()
  foreach ($line in $lines) {
    if ($line -match '^\s*(\d+-\d+)\s+([0-9A-Fa-f]{4}:[0-9A-Fa-f]{4})\s+(.+?)\s+(Shared|Not shared|Attached|Auto attached)\s*$') {
      $devices += [pscustomobject]@{
        BusId = $Matches[1]
        VidPid = $Matches[2]
        Description = $Matches[3].Trim()
        State = $Matches[4]
      }
    }
  }
  return $devices
}

if (-not $BusId) {
  $devices = Get-UsbIpdDevices
  if (-not $devices -or $devices.Count -eq 0) {
    Write-Host "No usbipd devices found. Is usbipd installed?" -ForegroundColor Red
    exit 1
  }

  $serialRegex = 'Arduino|USB Serial|USB-SERIAL|CDC|CH340|FTDI|Silicon Labs|CP210|Serial'
  $candidates = $devices | Where-Object { $_.Description -match $serialRegex }
  if ($candidates.Count -eq 0) { $candidates = $devices }

  if ($candidates.Count -eq 1) {
    $BusId = $candidates[0].BusId
  } else {
    Write-Host "Multiple USB devices detected:" -ForegroundColor Yellow
    $candidates | ForEach-Object { Write-Host ("  {0}  {1}  [{2}]" -f $_.BusId, $_.Description, $_.State) }
    $BusId = Read-Host "Enter BUSID to attach"
  }
}

if (-not $BusId) {
  Write-Host "No BUSID selected." -ForegroundColor Red
  exit 1
}

Write-Host "Binding BUSID $BusId..."
& usbipd bind --busid $BusId
if ($LASTEXITCODE -ne 0) {
  Write-Host "Bind failed or already shared; continuing." -ForegroundColor Yellow
}

Write-Host "Attaching BUSID $BusId to WSL distro '$Distribution'..."
& usbipd attach --wsl --busid $BusId --distribution $Distribution
if ($LASTEXITCODE -ne 0) {
  Write-Host "Attach failed." -ForegroundColor Red
  exit $LASTEXITCODE
}

Write-Host "Attached. In WSL, check: ls /dev/ttyACM* /dev/ttyUSB*"
