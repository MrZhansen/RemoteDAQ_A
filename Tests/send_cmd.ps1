# Send one V1.1 control frame to the A board and decode what comes back.
# A only accepts control frames whose source address is 192.168.1.20 and
# replies to that address on UDP 5005, so the machine running this script
# must own 192.168.1.20 (or B must be disconnected while it does).
#
# Example (status request):
#   powershell -ExecutionPolicy Bypass -File Tests\send_cmd.ps1 -Command 6
# Example (switch to GEAR3):
#   powershell -ExecutionPolicy Bypass -File Tests\send_cmd.ps1 -Command 1 -Parameter 3
param(
    [int]$Command = 6,
    [int]$Parameter = 0,
    [string]$TargetAddress = '192.168.1.10',
    [int]$TargetPort = 5006,
    [string]$LocalAddress = '192.168.1.20',
    [int]$LocalPort = 5005,
    [int]$ListenSeconds = 5
)

$ErrorActionPreference = 'Stop'

function Get-Crc16CcittFalse([byte[]]$Data) {
    $crc = 0xFFFF
    foreach ($byte in $Data) {
        $crc = $crc -bxor ([int]$byte -shl 8)
        for ($bit = 0; $bit -lt 8; $bit++) {
            if (($crc -band 0x8000) -ne 0) {
                $crc = (($crc -shl 1) -bxor 0x1021) -band 0xFFFF
            } else {
                $crc = ($crc -shl 1) -band 0xFFFF
            }
        }
    }
    return $crc
}

$frame = New-Object byte[] 9
$frame[0] = 0xAA
$frame[1] = 0x55
$frame[2] = 0x02
$frame[3] = [byte]$Command
$frame[4] = [byte]($Parameter -band 0xFF)
$frame[5] = [byte](($Parameter -shr 8) -band 0xFF)
$frame[6] = 0x00
$crc = Get-Crc16CcittFalse -Data $frame[0..6]
$frame[7] = [byte]($crc -band 0xFF)
$frame[8] = [byte](($crc -shr 8) -band 0xFF)
$hex = ($frame | ForEach-Object { '{0:X2}' -f $_ }) -join ' '
Write-Output "TX $hex -> ${TargetAddress}:$TargetPort (from ${LocalAddress}:$LocalPort)"

try {
    $local = New-Object System.Net.IPEndPoint ([System.Net.IPAddress]::Parse($LocalAddress)), $LocalPort
    $udp = New-Object System.Net.Sockets.UdpClient $local
} catch {
    Write-Output "Cannot bind ${LocalAddress}:$LocalPort - is that address on a local NIC, and is port $LocalPort free?"
    Write-Output $_.Exception.Message
    exit 2
}

$udp.Client.ReceiveBufferSize = 4MB
$udp.Client.ReceiveTimeout = 200
$target = New-Object System.Net.IPEndPoint ([System.Net.IPAddress]::Parse($TargetAddress)), $TargetPort
[void]$udp.Send($frame, $frame.Length, $target)

$statusCount = 0
$dataCount = 0
$badCrc = 0
$lastStatus = $null
$lastData = $null
$deadline = (Get-Date).AddSeconds($ListenSeconds)

while ((Get-Date) -lt $deadline) {
    try {
        $remote = New-Object System.Net.IPEndPoint ([System.Net.IPAddress]::Any), 0
        $payload = $udp.Receive([ref]$remote)
    } catch [System.Net.Sockets.SocketException] {
        continue
    }

    if ($payload.Length -lt 4) { continue }
    $crcOk = $false
    if ($payload.Length -ge 4) {
        $recvCrc = [int]$payload[$payload.Length - 2] -bor ([int]$payload[$payload.Length - 1] -shl 8)
        $crcOk = ((Get-Crc16CcittFalse -Data $payload[0..($payload.Length - 3)]) -eq $recvCrc)
    }
    if (-not $crcOk) { $badCrc++ }

    if ($payload.Length -eq 12 -and $payload[2] -eq 3) {
        $statusCount++
        $lastStatus = [pscustomobject]@{
            Peer       = $remote.ToString()
            Header     = ('{0:X2} {1:X2} {2:X2}' -f $payload[0], $payload[1], $payload[2])
            Gear       = $payload[3]
            Mask       = ('0x{0:X2}' -f $payload[4])
            TriggerSrc = $payload[5]
            TriggerMod = $payload[6]
            TriggerLvl = $payload[7]
            WaveHz     = ([int]$payload[8] -bor ([int]$payload[9] -shl 8))
            CrcOk      = $crcOk
            Raw        = ($payload | ForEach-Object { '{0:X2}' -f $_ }) -join ' '
        }
    } elseif ($payload.Length -eq 260 -and $payload[2] -eq 1) {
        $dataCount++
        $lastData = [pscustomobject]@{
            Peer  = $remote.ToString()
            Gear  = $payload[5]
            Mask  = ('0x{0:X2}' -f $payload[6])
            Seq   = ([int]$payload[3] -bor ([int]$payload[4] -shl 8))
            N     = $payload[7]
            CrcOk = $crcOk
        }
    }
}

$udp.Close()

Write-Output ("Received {0} status frame(s), {1} data frame(s), {2} bad CRC." -f $statusCount, $dataCount, $badCrc)
if ($lastStatus) {
    Write-Output 'Last status frame:'
    $lastStatus | Format-List
} else {
    Write-Output 'No status frame received.'
}
if ($lastData) {
    Write-Output 'Last data frame:'
    $lastData | Format-List
} else {
    Write-Output 'No data frame received.'
}

if (($statusCount -gt 0 -or $dataCount -gt 0) -and ($badCrc -eq 0)) {
    exit 0
}
exit 1
