$probes = Get-Process -Name "probe-rs" -ErrorAction SilentlyContinue
if ($probes) {
    $probes | ForEach-Object { $_.Kill() }
}
exit 0
