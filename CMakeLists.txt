name: Build and Release

on:
  workflow_dispatch:

jobs:
  build:
    runs-on: ${{ matrix.os }}
    strategy:
      matrix:
        os: [ubuntu-latest, windows-latest]
    steps:
    - uses: actions/checkout@v3

    # Linux: Build using Makefile
    - name: Build on Linux
      if: runner.os == 'Linux'
      run: |
        sudo apt-get update
        sudo apt-get install -y gcc g++ make
        make
        tar -czvf sysguage-linux.tar.gz sysguage

    # Windows: Build using g++
    - name: Build on Windows
      if: runner.os == 'Windows'
      run: |
        choco install mingw -y
        "C:\ProgramData\chocolatey\bin\g++" -o sysguage.exe src/sysguage.cpp
        Compress-Archive -Path sysguage.exe -DestinationPath sysguage-windows.zip

    # Upload artifacts
    - name: Upload Linux Binary
      if: runner.os == 'Linux'
      uses: actions/upload-artifact@v3
      with:
        name: sysguage-linux
        path: sysguage-linux.tar.gz

    - name: Upload Windows Binary
      if: runner.os == 'Windows'
      uses: actions/upload-artifact@v3
      with:
        name: sysguage-windows
        path: sysguage-windows.zip
