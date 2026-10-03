# Download, build and run:

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

# Run :
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