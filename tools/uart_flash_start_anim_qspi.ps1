param(
    [string]$BinPath = "assets/start_anim_qspi.bin",
    [string]$ComPort = "COM8",
    [int]$BaudRate = 115200,
    [int]$ReadTimeoutMs = 10000,
    [int]$FinalWaitMs = 180000,
    [int]$ChunkSize = 4096,
    [int]$WriteDelayMs = 2
)

$ErrorActionPreference = "Stop"

function Get-Crc32 {
    param([byte[]]$Bytes)

    $crc = [uint32]::MaxValue
    $poly = [uint32]::Parse('EDB88320', [System.Globalization.NumberStyles]::HexNumber)
    foreach ($b in $Bytes) {
        $crc = [uint32](($crc -bxor [uint32]$b))
        for ($i = 0; $i -lt 8; $i++) {
            if (($crc -band 1) -ne 0) {
                $crc = [uint32](($crc -shr 1) -bxor $poly)
            }
            else {
                $crc = [uint32]($crc -shr 1)
            }
        }
    }

    return [uint32]($crc -bxor [uint32]::MaxValue)
}

function Read-SerialLine {
    param(
        [System.IO.Ports.SerialPort]$Port,
        [int]$TimeoutMs
    )

    $end = (Get-Date).AddMilliseconds($TimeoutMs)
    $sb = New-Object System.Text.StringBuilder

    while ((Get-Date) -lt $end) {
        try {
            $ch = $Port.ReadByte()
        }
        catch [System.TimeoutException] {
            continue
        }

        if ($ch -lt 0) {
            continue
        }

        if ($ch -eq 10) {
            break
        }

        if ($ch -ne 13) {
            [void]$sb.Append([char]$ch)
        }
    }

    return $sb.ToString().Trim()
}

function Read-SerialLineStrict {
    param(
        [System.IO.Ports.SerialPort]$Port,
        [int]$TimeoutMs
    )

    $end = (Get-Date).AddMilliseconds($TimeoutMs)

    while ((Get-Date) -lt $end) {
        $line = Read-SerialLine -Port $Port -TimeoutMs 1000
        if (-not $line) {
            continue
        }

        if ($line.Length -lt 2) {
            continue
        }

        return $line
    }

    return $null
}

function Read-SerialResult {
    param(
        [System.IO.Ports.SerialPort]$Port,
        [int]$TimeoutMs
    )

    $end = (Get-Date).AddMilliseconds($TimeoutMs)

    while ((Get-Date) -lt $end) {
        $line = Read-SerialLineStrict -Port $Port -TimeoutMs $TimeoutMs

        if (-not $line) {
            continue
        }

        if ($line -eq 'OK') {
            continue
        }

        if ($line -eq 'RASE_OK') {
            $line = 'ERASE_OK'
        }

        Write-Host $line

        if ($line.StartsWith('DONE')) {
            return $line
        }

        if ($line.StartsWith('ERR')) {
            return $line
        }
    }

    return $null
}

function Drain-SerialDuringTx {
    param(
        [System.IO.Ports.SerialPort]$Port
    )

    while ($Port.BytesToRead -gt 0) {
        $line = Read-SerialLine -Port $Port -TimeoutMs 5
        if (-not $line) {
            break
        }

        if ($line -eq 'OK') {
            continue
        }

        if ($line -eq 'RASE_OK') {
            $line = 'ERASE_OK'
        }

        Write-Host $line

        if ($line.StartsWith('ERR')) {
            throw "MCU returned error during payload: $line"
        }
    }
}

function Wait-SerialPrefix {
    param(
        [System.IO.Ports.SerialPort]$Port,
        [string]$Prefix,
        [int]$TimeoutMs
    )

    $end = (Get-Date).AddMilliseconds($TimeoutMs)

    while ((Get-Date) -lt $end) {
        $line = Read-SerialLineStrict -Port $Port -TimeoutMs $TimeoutMs

        if (-not $line) {
            continue
        }

        if ($line -eq 'OK') {
            continue
        }

        if ($line -eq 'RASE_OK') {
            $line = 'ERASE_OK'
        }

        Write-Host $line

        if ($line.StartsWith($Prefix)) {
            return $line
        }

        if ($line.StartsWith('ERR')) {
            return $line
        }
    }

    return $null
}

if (-not (Test-Path -LiteralPath $BinPath)) {
    throw "Animation bin not found: $BinPath"
}

[byte[]]$payload = [System.IO.File]::ReadAllBytes((Resolve-Path -LiteralPath $BinPath))
if ($payload.Length -le 0) {
    throw "Animation bin is empty"
}

if ($payload.Length -gt 4MB) {
    throw "Animation bin too large for download mode (max 4MB payload)"
}

$crc32 = Get-Crc32 -Bytes $payload
Write-Host "Payload size: $($payload.Length) bytes"
Write-Host "Payload CRC32: 0x$('{0:X8}' -f $crc32)"

$header = New-Object byte[] 12
[BitConverter]::GetBytes([uint32]0x314C4451).CopyTo($header, 0)
[BitConverter]::GetBytes([uint32]$payload.Length).CopyTo($header, 4)
[BitConverter]::GetBytes([uint32]$crc32).CopyTo($header, 8)

$port = [System.IO.Ports.SerialPort]::new($ComPort, $BaudRate, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$port.ReadTimeout = $ReadTimeoutMs
$port.WriteTimeout = 5000
$port.DtrEnable = $true
$port.RtsEnable = $true

try {
    $opened = $false
    for ($attempt = 1; $attempt -le 20; $attempt++) {
        try {
            $port.Open()
            $opened = $true
            break
        }
        catch {
            Start-Sleep -Milliseconds 500
        }
    }

    if (-not $opened) {
        throw "Failed to open $ComPort after multiple attempts"
    }

    Start-Sleep -Milliseconds 300
    $port.DiscardInBuffer()
    $port.DiscardOutBuffer()

    Write-Host "Sending header to $ComPort ..."
    $port.Write($header, 0, $header.Length)

    $line = Read-SerialLineStrict -Port $port -TimeoutMs 15000
    if (-not $line) {
        throw "Timed out waiting for MCU header response"
    }
    if (-not $line.StartsWith("OKH")) {
        throw "Header not accepted by MCU. Response: '$line'"
    }

    $eraseDone = Wait-SerialPrefix -Port $port -Prefix 'ERASE_OK' -TimeoutMs $FinalWaitMs
    if (-not $eraseDone) {
        throw "Timed out waiting for MCU erase completion"
    }

    if ($eraseDone.StartsWith('ERR')) {
        throw "MCU returned error during erase: $eraseDone"
    }

    Write-Host "Header accepted, streaming payload ..."

    $offset = 0
    while ($offset -lt $payload.Length) {
        $len = [Math]::Min($ChunkSize, $payload.Length - $offset)
        $port.Write($payload, $offset, $len)
        $offset += $len

        Drain-SerialDuringTx -Port $port

        if ($WriteDelayMs -gt 0) {
            Start-Sleep -Milliseconds $WriteDelayMs
        }

        if (($offset % 1048576 -eq 0) -or ($offset -eq $payload.Length)) {
            $progress = [Math]::Round(($offset * 100.0) / $payload.Length, 1)
            Write-Host "TX $offset/$($payload.Length) ($progress%)"
        }
    }

    Write-Host "Payload sent, waiting for MCU final result ..."
    $final = Read-SerialResult -Port $port -TimeoutMs $FinalWaitMs

    if ($final -and $final.StartsWith("DONE")) {
        Write-Host "SUCCESS: start_anim_qspi.bin programmed to W25Q64"
    }
    elseif ($final -and $final.StartsWith("ERR")) {
        throw "MCU returned error: $final"
    }
    else {
        throw "Timed out waiting for MCU final result"
    }
}
finally {
    if ($port -and $port.IsOpen) {
        $port.Close()
    }
}
