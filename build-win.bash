#!/bin/bash

  powershell.exe -ExecutionPolicy Bypass -File ./build-win.ps1 \
    -Clean \
    -HakoniwaCoreRoot "../hakoniwa-core-pro/install" \
    -HakoniwaPduEndpointRoot "../hakoniwa-pdu-endpoint/install" \
    -ExtraPrefixPaths "C:/project/vcpkg/installed/x64-windows" 