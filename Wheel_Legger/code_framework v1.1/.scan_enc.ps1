$ErrorActionPreference = 'SilentlyContinue'
$root = 'e:\ROBOT_NEW\wheel_legger\code_framework v1.1'
$files = Get-ChildItem -Path $root -Recurse -File | Where-Object { $_.Extension -in '.c','.h','.txt','.md','.s','.ld' }
$utf8strict = New-Object System.Text.UTF8Encoding($false, $true)
$gbk = [System.Text.Encoding]::GetEncoding(936)
foreach ($f in $files) {
    $bytes = [System.IO.File]::ReadAllBytes($f.FullName)
    try { $null = $utf8strict.GetString($bytes) }
    catch {
        try { $null = $gbk.GetString($bytes); $ok = 'GBK-OK' }
        catch { $ok = 'GBK-FAIL' }
        Write-Output ("{0}  [{1}]" -f $f.FullName, $ok)
    }
}
