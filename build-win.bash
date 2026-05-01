#!/bin/bash

  powershell.exe -ExecutionPolicy Bypass -File ./build-win.ps1 \
    -Clean \
    -UseThirdpartyHakoniwa \
    -ExtraPrefixPaths "C:/project/vcpkg/installed/x64-windows" 
