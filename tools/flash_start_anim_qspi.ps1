param(
    [string]$BinPath = "assets/start_anim_qspi.bin",
    [string]$ExternalLoader,
    [string]$Port = "SWD",
    [string]$Address = "0x90000000"
)

$ErrorActionPreference = "Stop"

if (-not $ExternalLoader) {
    throw "Please pass -ExternalLoader <path-to-.stldr>"
}

if (-not (Test-Path -LiteralPath $BinPath)) {
    throw "Animation bin not found: $BinPath"
}

if (-not (Test-Path -LiteralPath $ExternalLoader)) {
    throw "External loader not found: $ExternalLoader"
}

$cli = (Get-Command STM32_Programmer_CLI.exe -ErrorAction Stop).Source

& $cli -c "port=$Port" -el "$ExternalLoader" -w "$BinPath" "$Address" -v -rst
