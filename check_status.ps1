$number_completed = (Get-Content C:\data\Adversary\ScoresTest2.txt,C:\data\Adversary\ScoresTest1.txt).Count
$first_ten =  Get-Content C:\data\Adversary\ScoresTest2.txt,C:\data\Adversary\ScoresTest1.txt | foreach{[double]($_.split(":")[1])} | sort | select -First 10
$good_paths = (Get-Content C:\data\Adversary\ScoresTest2.txt,C:\data\Adversary\ScoresTest1.txt | Where {[double]($_.split(":")[1]) -ge 0.0 -and [double]($_.split(":")[1]) -le 2}).Count
$bad_paths = (Get-Content C:\data\Adversary\ScoresTest2.txt,C:\data\Adversary\ScoresTest1.txt | Where {[double]($_.split(":")[1]) -lt 0.0}).Count
$accidents = (Get-Content C:\data\Adversary\ScoresTest2.txt,C:\data\Adversary\ScoresTest1.txt | Where {[double]($_.split(":")[1]) -eq -1.0}).Count
$feature_count = (Get-ChildItem C:\data\Features\ -File).Count
$uncaught = Get-Content C:\data\Adversary\ScoresTest2.txt,C:\data\Adversary\ScoresTest1.txt | Select-String -Pattern "-666666"


Write-Host "Currently there are" $number_completed "simulations completed"
if(((Get-ChildItem C:\data\Adversary\1\ -File) | select -First 1).Name -match "Normal")
{
    Write-Host "There are" $good_paths "good paths and" $bad_paths "bad paths (including" $accidents "accidents)"
    Write-Host "Size of Features folder is" $feature_count
}
elseif(((Get-ChildItem C:\data\Adversary\1\ -File) | select -First 1).Name -match "Hybrid")
{
    Write-Host "There are" $good_paths "good paths and" $bad_paths "bad paths"
    Write-Host "Size of Features folder is" $feature_count
    Write-Host "Uncaught paths are:" $uncaught
}
Write-Host "Here are the top 10: " 
ForEach($score in $first_ten)
{
   Get-Content C:\data\Adversary\ScoresTest2.txt,C:\data\Adversary\ScoresTest1.txt | Select-String -Pattern $score | select -First 1
}