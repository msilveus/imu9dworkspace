[CmdletBinding()]
param()

$projectRoot = Get-Location
Write-Verbose "Project root: $projectRoot"

$cargoToml = Join-Path $projectRoot "Cargo.toml"
Write-Verbose "Reading Cargo.toml from $cargoToml"

$crateName = $null
$lines = Get-Content $cargoToml

foreach ($line in $lines) {
    Write-Verbose "Checking line: $line"
    if ($line -match '^\s*name\s*=\s*"(.*?)"\s*$') {
        $crateName = $matches[1]
        Write-Verbose "Found crate name: $crateName"
        Write-Host "DEBUG: Crate name is '$crateName'"
        break  # Now this works as expected, breaking out of the loop only
    }
}

Write-Host "DEBUG: Crate name after loop is '$crateName'"

if (-not $crateName) {
    Write-Error "Could not determine crate name from Cargo.toml"
    exit 1
}

# After crate name found...

$targetTriple = "thumbv7em-none-eabihf"
$tempBuildPath = "D:\work\RustTempBuilds\$targetTriple\debug\$crateName"
Write-Host "DEBUG: Looking for ELF file at: '$tempBuildPath'"
if (Test-Path $tempBuildPath) {
    Write-Host "DEBUG: File found!"
} else {
    Write-Host "DEBUG: File NOT found!"
}

Write-Verbose "Looking for ELF file at: $tempBuildPath"

if (Test-Path $tempBuildPath) {
    Write-Verbose "ELF file found!"
    $finalTargetPath = Join-Path $projectRoot "target\$targetTriple\debug"
    Write-Verbose "Destination folder: $finalTargetPath"

    if (-not (Test-Path $finalTargetPath)) {
        Write-Verbose "Creating destination folder..."
        New-Item -ItemType Directory -Path $finalTargetPath | Out-Null
    }

    Copy-Item -Path $tempBuildPath -Destination $finalTargetPath -Force
    Write-Host "[OK] ELF copied to $finalTargetPath"
} else {
    Write-Warning "ELF file NOT found at: $tempBuildPath"
}
