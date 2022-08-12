$file_path = "C:\data\Archive\11AUG2022"
$file_name = Get-ChildItem -File $file_path
$file = Join-Path -Path $file_path -ChildPath $file_name
(Get-Content $file) -replace ':','=' | Set-Content $file
$file_content = Get-Content $file | Out-String
$hash_table = ConvertFrom-StringData -StringData $file_content
#$hash_table.GetEnumerator() | Sort-Object -Property {[double]$_.Value}
$bad_paths = ($hash_table.GetEnumerator() | ? {[double]$_.Value -lt 0.0}).Key
$good_paths = ($hash_table.GetEnumerator() | ? {[double]$_.Value -gt 0.0 -and [double]$_.Value -lt 500}).Key


$sub_files = Get-ChildItem -Path "C:\data\Adversary" -Recurse
#$sub_files = Get-ChildItem -Path "C:\data\Archive\20JUL2022\Normal_path_-9_758824_2000" -Recurse
$to = Join-Path $file_path -ChildPath "bad_paths" 
Foreach($bad_file_name in $bad_paths)
{
    $from_file_obj = $sub_files| Where {$_.Name -match $bad_file_name}
    $from = Join-Path -Path $from_file_obj.Directory -ChildPath $from_file_obj.Name
    Copy-Item $from -Destination $to
}
Write-Host "Just moved " $bad_paths.Count " bad paths to the archive"

$to = Join-Path $file_path -ChildPath "good_paths" 
Foreach($good_file_name in $good_paths)
{
    $from_file_obj = $sub_files| Where {$_.Name -match $good_file_name}
    $from = Join-Path -Path $from_file_obj.Directory -ChildPath $from_file_obj.Name
    Copy-Item $from -Destination $to
}
Write-Host "Just moved " $good_paths.Count " good paths to the archive"


