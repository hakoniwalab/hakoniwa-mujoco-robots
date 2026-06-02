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

function Add-Issue {
    param(
        [Parameter(Mandatory = $true)]
        [System.Collections.Generic.List[string]]$Issues,
        [Parameter(Mandatory = $true)]
        [string]$Message
    )
    $Issues.Add($Message)
}

function Test-CMakeConfig {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Prefix,
        [Parameter(Mandatory = $true)]
        [string[]]$RelativePaths
    )
    foreach ($RelativePath in $RelativePaths) {
        if (Test-Path -LiteralPath (Join-Path $Prefix $RelativePath)) {
            return $true
        }
    }
    return $false
}

function Write-PreflightHelp {
    Write-Host ""
    Write-Host "Install hints:"
    Write-Host "  hakoniwa-core-pro:"
    Write-Host "    Build and install it first, then pass:"
    Write-Host "      -HakoniwaCoreRoot C:\project\hakoniwa-core-pro\install"
    Write-Host ""
    Write-Host "  hakoniwa-pdu-endpoint:"
    Write-Host "    Build and install it first, then pass:"
    Write-Host "      -HakoniwaPduEndpointRoot C:\project\hakoniwa-pdu-endpoint\install"
    Write-Host ""
    Write-Host "  glfw3 / vcpkg:"
    Write-Host "    Install glfw3 with vcpkg, then pass one of:"
    Write-Host "      -ExtraPrefixPaths C:\project\vcpkg\installed\x64-windows"
    Write-Host "      -Glfw3Dir C:\project\vcpkg\installed\x64-windows\share\glfw3"
    Write-Host ""
    Write-Host "  Vendored Hakoniwa mode:"
    Write-Host "    Use -UseThirdpartyHakoniwa after git submodule update --init --recursive."
}

function Invoke-Preflight {
    $Issues = New-Object System.Collections.Generic.List[string]

    if (-not (Get-Command cmake -ErrorAction SilentlyContinue)) {
        Add-Issue -Issues $Issues -Message "cmake is not installed or not in PATH."
    }

    if ($UseThirdpartyHakoniwa) {
        $ThirdpartyCore = Join-Path $RepoRoot "thirdparty\hakoniwa-core-pro"
        $ThirdpartyEndpoint = Join-Path $RepoRoot "thirdparty\hakoniwa-pdu-endpoint"
        if (-not (Test-Path -LiteralPath (Join-Path $ThirdpartyCore "CMakeLists.txt"))) {
            Add-Issue -Issues $Issues -Message "vendored hakoniwa-core-pro was not found. Run: git submodule update --init --recursive"
        }
        if (-not (Test-Path -LiteralPath (Join-Path $ThirdpartyEndpoint "CMakeLists.txt"))) {
            Add-Issue -Issues $Issues -Message "vendored hakoniwa-pdu-endpoint was not found. Run: git submodule update --init --recursive"
        }
    }
    else {
        if ([string]::IsNullOrWhiteSpace($HakoniwaCoreRoot)) {
            Add-Issue -Issues $Issues -Message "Hakoniwa core root is not set. Pass -HakoniwaCoreRoot <install-prefix>."
        }
        elseif (-not (Test-Path -LiteralPath $HakoniwaCoreRoot)) {
            Add-Issue -Issues $Issues -Message "Hakoniwa core root does not exist: $HakoniwaCoreRoot"
        }
        elseif (-not (Test-CMakeConfig -Prefix $HakoniwaCoreRoot -RelativePaths @("lib\cmake\hakoniwa-core\hakoniwa-coreConfig.cmake"))) {
            Add-Issue -Issues $Issues -Message "hakoniwa-core package config was not found under: $HakoniwaCoreRoot"
        }

        if ([string]::IsNullOrWhiteSpace($HakoniwaPduEndpointRoot)) {
            Add-Issue -Issues $Issues -Message "Hakoniwa PDU endpoint root is not set. Pass -HakoniwaPduEndpointRoot <install-prefix>."
        }
        elseif (-not (Test-Path -LiteralPath $HakoniwaPduEndpointRoot)) {
            Add-Issue -Issues $Issues -Message "Hakoniwa PDU endpoint root does not exist: $HakoniwaPduEndpointRoot"
        }
        elseif (-not (Test-CMakeConfig -Prefix $HakoniwaPduEndpointRoot -RelativePaths @("lib\cmake\hakoniwa_pdu_endpoint\hakoniwa_pdu_endpointConfig.cmake"))) {
            Add-Issue -Issues $Issues -Message "hakoniwa_pdu_endpoint package config was not found under: $HakoniwaPduEndpointRoot"
        }
    }

    if (-not $DisableViewer) {
        $GlfwFound = $false
        if (-not [string]::IsNullOrWhiteSpace($Glfw3Dir)) {
            if (Test-CMakeConfig -Prefix $Glfw3Dir -RelativePaths @("glfw3Config.cmake")) {
                $GlfwFound = $true
            }
            else {
                Add-Issue -Issues $Issues -Message "glfw3Config.cmake was not found in -Glfw3Dir: $Glfw3Dir"
            }
        }

        foreach ($PrefixPath in $ExtraPrefixPaths) {
            if ([string]::IsNullOrWhiteSpace($PrefixPath)) {
                continue
            }
            if (Test-CMakeConfig -Prefix $PrefixPath -RelativePaths @("share\glfw3\glfw3Config.cmake", "lib\cmake\glfw3\glfw3Config.cmake")) {
                $GlfwFound = $true
            }
        }

        if (-not $GlfwFound -and [string]::IsNullOrWhiteSpace($Glfw3Dir) -and $ExtraPrefixPaths.Count -eq 0) {
            Add-Issue -Issues $Issues -Message "glfw3 is not configured. Pass -ExtraPrefixPaths <vcpkg-installed-triplet> or -Glfw3Dir <glfw3-cmake-dir>."
        }
    }

    if ($Issues.Count -gt 0) {
        [Console]::Error.WriteLine("ERROR: build prerequisites are missing:")
        foreach ($Issue in $Issues) {
            [Console]::Error.WriteLine("  - $Issue")
        }
        Write-PreflightHelp
        exit 1
    }
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

Invoke-Preflight

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
