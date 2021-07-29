<#
This script install precompiled dependencies to build python-fcl on Windows:
 * fcl
 * libccd
 * eigen
 * octomap
#>

# Remember starting location for future usage
$base_dir = Get-Location
# Binaries folder
$install_dir = "$base_dir\deps\install"


$file_name = "PrecompiledDependenciesWindows.zip"
Invoke-WebRequest -Uri "https://github.com/CyrilWaechter/python-fcl/releases/download/v0.6.1/$file_name" -Outfile $file_name
Expand-Archive $file_name -DestinationPath deps


# ------------------------------------------------------------------------------
# Python-FCL

Write-Host "Copying dependent DLLs"
Copy-Item $install_dir\bin\octomap.dll $base_dir\src\fcl
Copy-Item $install_dir\bin\octomath.dll $base_dir\src\fcl
Copy-Item $install_dir\bin\ccd.dll $base_dir\src\fcl

Set-Location $base_dir
Write-Host "All done!"
