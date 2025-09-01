# flash.ps1

# Assume script is run from project root, or working directory is set by CLion
$configPath = ".\.cargo\config.toml"

if (-not (Test-Path $configPath)) {
    Write-Error "Cannot find config file at $configPath"
    exit 1
}

# Extract chip name
$chip = Select-String -Path $configPath -Pattern 'chip\s*=\s*"([^"]+)"' | ForEach-Object {
    if ($_ -match 'chip\s*=\s*"([^"]+)"') { $matches[1] }
}

if (-not $chip) {
    Write-Error "Chip name not found in config.toml"
    exit 1
}

# Flash with chip name
cargo flash --chip $chip --release
