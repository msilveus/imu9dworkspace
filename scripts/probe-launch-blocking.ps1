$ErrorActionPreference = "Stop"
$VerbosePreference = "Continue"

Write-Host "Launching probe-rs gdb..."

# Direct inline call, no Start-Process, no -FilePath
& probe-rs gdb --chip STM32F401RETx target\thumbv7em-none-eabihf\debug\imu_app -- -vv
