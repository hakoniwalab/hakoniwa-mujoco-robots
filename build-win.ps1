[CmdletBinding()]
param(
    [switch]$Clean,
    [string]$BuildDirName = "build-win",
    [string]$Generator = "",
    [string]$Platform = "",
    [string]$Configuration = "Release",
    [string]$ToolchainFile = "",
    [string]$MuJoCoRoot = "",
    [string]$HakoniwaCoreRoot = "",
    [string]$HakoniwaPduEndpointRoot = "",
    [string[]]$ExtraPrefixPaths = @(),
    [string]$Glfw3Dir = "",
    [switch]$DisableViewer,
    [switch]$UseThirdpartyHakoniwa,
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$AdditionalCMakeArgs
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

function Is-SingleConfigGenerator {
    param([string]$Name)

    if ([string]::IsNullOrWhiteSpace($Name)) {
        return $false
    }

    return $Name -match "Ninja|Makefiles"
}

function Resolve-ExistingPath {
    param(
        [Parameter(Mandatory = $true)]
        [string]$PathValue,
        [Parameter(Mandatory = $true)]
        [string]$Label
    )
    if (-not (Test-Path -LiteralPath $PathValue)) {
        throw "$Label does not exist: $PathValue"
    }
    return (Resolve-Path -LiteralPath $PathValue).Path
}

$RepoRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$SourceDir = Join-Path $RepoRoot "src"
$BuildDir = Join-Path $RepoRoot $BuildDirName
$UseViewerValue = if ($DisableViewer) { "OFF" } else { "ON" }

$ResolvedGenerator = $Generator
if ([string]::IsNullOrWhiteSpace($ResolvedGenerator) -and $env:CMAKE_GENERATOR) {
    $ResolvedGenerator = $env:CMAKE_GENERATOR
}

if ($Clean -and (Test-Path -LiteralPath $BuildDir)) {
    Write-Host "Cleaning build directory: $BuildDir"
    Remove-Item -LiteralPath $BuildDir -Recurse -Force
}

$ConfigureArgs = @(
    "-S", $SourceDir,
    "-B", $BuildDir,
    "-DUSE_VIEWER=$UseViewerValue"
)

if (-not [string]::IsNullOrWhiteSpace($ResolvedGenerator)) {
    $ConfigureArgs += @("-G", $ResolvedGenerator)
}

if ($UseThirdpartyHakoniwa) {
    $ConfigureArgs += "-DHAKO_USE_THIRDPARTY_HAKONIWA=ON"
}

if ((-not [string]::IsNullOrWhiteSpace($Platform)) -and (-not (Is-SingleConfigGenerator $ResolvedGenerator))) {
    $ConfigureArgs += @("-A", $Platform)
}

if (-not [string]::IsNullOrWhiteSpace($ToolchainFile)) {
    $ResolvedToolchain = Resolve-ExistingPath -PathValue $ToolchainFile -Label "Toolchain file"
    $ConfigureArgs += "-DCMAKE_TOOLCHAIN_FILE=$ResolvedToolchain"
}

if (-not [string]::IsNullOrWhiteSpace($MuJoCoRoot)) {
    $ResolvedMuJoCoRoot = Resolve-ExistingPath -PathValue $MuJoCoRoot -Label "MuJoCo root"
    $ConfigureArgs += "-DMUJOCO_ROOT=$ResolvedMuJoCoRoot"
}

$PrefixList = New-Object System.Collections.Generic.List[string]
if (-not [string]::IsNullOrWhiteSpace($HakoniwaCoreRoot)) {
    $ResolvedCoreRoot = Resolve-ExistingPath -PathValue $HakoniwaCoreRoot -Label "Hakoniwa core root"
    $ConfigureArgs += "-DHAKONIWA_INSTALL_PREFIX=$ResolvedCoreRoot"
    $PrefixList.Add($ResolvedCoreRoot)
}

if (-not [string]::IsNullOrWhiteSpace($HakoniwaPduEndpointRoot)) {
    $ResolvedEndpointRoot = Resolve-ExistingPath -PathValue $HakoniwaPduEndpointRoot -Label "Hakoniwa PDU endpoint root"
    $ConfigureArgs += "-DHAKONIWA_PDU_ENDPOINT_PREFIX=$ResolvedEndpointRoot"
    $PrefixList.Add($ResolvedEndpointRoot)
}

foreach ($PrefixPath in $ExtraPrefixPaths) {
    if ([string]::IsNullOrWhiteSpace($PrefixPath)) {
        continue
    }
    $PrefixList.Add((Resolve-ExistingPath -PathValue $PrefixPath -Label "Extra prefix path"))
}

if ($PrefixList.Count -gt 0) {
    $UniquePrefixes = [System.Linq.Enumerable]::ToArray(
        [System.Linq.Enumerable]::Distinct([string[]]$PrefixList)
    )
    $JoinedPrefixes = [string]::Join(";", $UniquePrefixes)
    $ConfigureArgs += "-DCMAKE_PREFIX_PATH=$JoinedPrefixes"
    $ConfigureArgs += "-DHAKONIWA_EXTRA_PREFIX_PATH=$JoinedPrefixes"
}

if (-not [string]::IsNullOrWhiteSpace($Glfw3Dir)) {
    $ResolvedGlfw3Dir = Resolve-ExistingPath -PathValue $Glfw3Dir -Label "glfw3_DIR"
    $ConfigureArgs += "-Dglfw3_DIR=$ResolvedGlfw3Dir"
}

if ($null -ne $AdditionalCMakeArgs -and $AdditionalCMakeArgs.Count -gt 0) {
    $ConfigureArgs += $AdditionalCMakeArgs
}

if (Is-SingleConfigGenerator $ResolvedGenerator) {
    $ConfigureArgs += "-DCMAKE_BUILD_TYPE=$Configuration"
}

Write-Host "Configuring with CMake..."
& cmake @ConfigureArgs
if ($LASTEXITCODE -ne 0) {
    throw "CMake configure failed."
}

$BuildArgs = @(
    "--build", $BuildDir
)

if (-not (Is-SingleConfigGenerator $ResolvedGenerator)) {
    $BuildArgs += @("--config", $Configuration)
}

Write-Host "Building with CMake..."
& cmake @BuildArgs
if ($LASTEXITCODE -ne 0) {
    throw "CMake build failed."
}
