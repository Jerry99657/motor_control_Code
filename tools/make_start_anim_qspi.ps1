param(
    [string]$InputGif = "start.gif",
    [string]$OutputBin = "assets/start_anim_qspi.bin",
    [int]$Fps = 10,
    [uint32]$MaxPayloadBytes = 4MB
)

$ErrorActionPreference = "Stop"

if (-not (Test-Path -LiteralPath $InputGif)) {
    throw "Input gif not found: $InputGif"
}

if ($Fps -le 0) {
    throw "Fps must be > 0"
}

$ffprobe = "ffprobe"
$ffmpeg = "ffmpeg"

$streamInfo = & $ffprobe -v error -select_streams v:0 -show_entries stream=width,height -of csv=p=0:s=x -- "$InputGif"
if (-not $streamInfo) {
    throw "ffprobe failed to read width/height from $InputGif"
}

$parts = $streamInfo.Trim().Split("x")
if ($parts.Count -ne 2) {
    throw "Unexpected ffprobe output: $streamInfo"
}

$width = [int]$parts[0]
$height = [int]$parts[1]
if ($width -le 0 -or $height -le 0) {
    throw "Invalid dimensions: ${width}x${height}"
}

$outputDir = Split-Path -Parent $OutputBin
if ($outputDir -and -not (Test-Path -LiteralPath $outputDir)) {
    New-Item -ItemType Directory -Path $outputDir | Out-Null
}

$tmpRaw = [System.IO.Path]::ChangeExtension($OutputBin, ".rgb565.raw")

$workingFps = $Fps
$frameSize = [uint32]($width * $height * 2)
$rawSize = [uint64]0
$frameCount = [uint32]0

while ($workingFps -ge 1)
{
    & $ffmpeg -y -i "$InputGif" -vf "fps=$workingFps,scale=${width}:${height}:flags=lanczos" -pix_fmt rgb565le -f rawvideo "$tmpRaw" | Out-Null
    if (-not (Test-Path -LiteralPath $tmpRaw)) {
        throw "ffmpeg failed to generate raw frame file"
    }

    $rawSize = [uint64](Get-Item -LiteralPath $tmpRaw).Length
    if ($rawSize -eq 0 -or ($rawSize % $frameSize) -ne 0) {
        Remove-Item -LiteralPath $tmpRaw -Force -ErrorAction SilentlyContinue
        throw "Raw frame size mismatch: rawSize=$rawSize frameSize=$frameSize"
    }

    if ($rawSize -le $MaxPayloadBytes) {
        $frameCount = [uint32]($rawSize / $frameSize)
        break
    }

    Remove-Item -LiteralPath $tmpRaw -Force -ErrorAction SilentlyContinue
    $workingFps--
}

if ($workingFps -lt 1) {
    throw "Unable to fit animation payload under ${MaxPayloadBytes} bytes even at 1 fps"
}

if ($workingFps -ne $Fps) {
    Write-Output "Adjusted FPS from $Fps to $workingFps to fit under ${MaxPayloadBytes} bytes"
}

$frameDelay = [uint16]([Math]::Round(1000.0 / $workingFps))

$magic = [uint32]0x314E4151    # 'QAN1'
$version = [uint16]1
$headerSize = [uint16]32
$payloadSize = [uint32]$rawSize
$dataOffset = [uint32]$headerSize
$reserved = [uint32]0

$rawBytes = [System.IO.File]::ReadAllBytes($tmpRaw)
$fs = [System.IO.File]::Open($OutputBin, [System.IO.FileMode]::Create, [System.IO.FileAccess]::Write)
$bw = New-Object System.IO.BinaryWriter($fs)

try {
    $bw.Write($magic)
    $bw.Write($version)
    $bw.Write($headerSize)
    $bw.Write([uint16]$width)
    $bw.Write([uint16]$height)
    $bw.Write([uint16]$frameCount)
    $bw.Write($frameDelay)
    $bw.Write($frameSize)
    $bw.Write($payloadSize)
    $bw.Write($dataOffset)
    $bw.Write($reserved)
    $bw.Write($rawBytes)
}
finally {
    $bw.Close()
    $fs.Close()
}

Remove-Item -LiteralPath $tmpRaw -Force -ErrorAction SilentlyContinue

Write-Output "Generated: $OutputBin"
Write-Output "FPS      : $workingFps"
Write-Output "Frames   : $frameCount"
Write-Output "Size     : ${width}x${height}"
Write-Output "Delay(ms): $frameDelay"
Write-Output "Payload  : $payloadSize bytes"
