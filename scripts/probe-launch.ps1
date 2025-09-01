# Kill any existing probe-rs process
Get-Process -Name "probe-rs" -ErrorAction SilentlyContinue | ForEach-Object { $_.Kill() }

# Start a new probe-rs gdb session in the background
Start-Process -FilePath "probe-rs" -ArgumentList "gdb", "--chip", "STM32F401RETx"

# Exit immediately so CLion can continue
exit 0
