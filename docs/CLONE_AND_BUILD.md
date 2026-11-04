# Download, build and run:

Requires a GPU and driver with OpenGL 4.1 core.

## Build and run from sources:

``` bash
# Download :
git clone --recursive https://github.com/MapsHD/SearchAndRescue.git

# Enter project directory :
cd SearchAndRescue

# Configure project :
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release

# Build :
cmake --build build --config Release -j 8

# Run (Windows) :
.\build\app\Release\HDMapping-SearchAndRescue.exe

# Run (Linux, macOS) :
./build/app/HDMapping-SearchAndRescue
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
