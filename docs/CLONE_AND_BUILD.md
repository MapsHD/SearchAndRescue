# Download, build and run:

Requires a GPU and driver with OpenGL 4.1 core.

## Build and run from sources:

### Requirements:

- Git
- CMake 3.20 or newer
- A C++23 compiler. CI builds with Apple Clang on macOS 26, GCC on Ubuntu 24.04 and MSVC (Visual Studio 2022, "Desktop development with C++" workload) on Windows Server 2025.
- Linux only, X11 and OpenGL development packages :

``` bash
sudo apt-get install build-essential cmake git \
    libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev libxext-dev libgl-dev
```

All other dependencies are git submodules in [external](../external) and are built together with the project.

### Download:

``` bash
git clone --recursive https://github.com/MapsHD/SearchAndRescue.git
cd SearchAndRescue

# Already cloned without --recursive, or after pulling a submodule update :
git submodule update --init --recursive
```

### Configure and build:

``` bash
# Configure (Debug or Release) :
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release

# Build, replace 4 with about half of your CPU cores to keep memory use in check :
cmake --build build --config Release -j 4
```

`CMAKE_BUILD_TYPE` is used by single-configuration generators (Makefiles, Ninja). `--config` is used by multi-configuration generators (Visual Studio, Xcode). Passing both works with any generator.

### Faster builds with Ninja:

Installing [Ninja](https://ninja-build.org) and selecting it with `-G Ninja` speeds up both clean and incremental builds. CI uses it on all platforms.

``` bash
# Install :
brew install ninja                 # macOS
sudo apt-get install ninja-build   # Linux
winget install Ninja-build.Ninja   # Windows, also bundled with Visual Studio's C++ CMake tools

# Configure and build :
cmake -B build -S . -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build -j 4
```

On Windows, run these from a "Developer PowerShell for VS" (or "x64 Native Tools Command Prompt"), so CMake can find the MSVC compiler.

A build directory remembers its generator. To switch generators, delete `build` or configure into a different directory.

### Run:

``` bash
# Linux, macOS (Makefiles, Ninja) :
./build/app/HDMapping-SearchAndRescue

# Windows (Ninja) :
.\build\app\HDMapping-SearchAndRescue.exe

# Windows (Visual Studio generator) :
.\build\app\Release\HDMapping-SearchAndRescue.exe
```

## Run prebuilt:

``` bash
# Download :
git clone https://github.com/MapsHD/SearchAndRescue.git

# Enter project directory :
cd SearchAndRescue

# Open *binary* in Windows explorer :
explorer.exe .

# Double click on HDMapping-SearchAndRescue.exe
```
## Formatting:

Formatting follows [.clang-format](../.clang-format). The scripts only use the clang-format pinned in [requirements.txt](../requirements.txt), installed into a local `.venv`. A clang-format found on `PATH` is never used.

``` bash
# Set up once, from the project directory :
python -m venv .venv
.venv/bin/python -m pip install -r requirements.txt

# Format app/ and lib/ in place :
.venv/bin/python scripts/run_clang_format.py

# Check only, changes nothing (the same check CI runs) :
.venv/bin/python scripts/run_clang_format_check.py
```

On Windows use `.venv\Scripts\python` instead of `.venv/bin/python`.

CI ([ci.yml](../.github/workflows/ci.yml)) runs the clang-format check first. The Linux, macOS and Windows builds start only when it passes.
