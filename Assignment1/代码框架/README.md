# Rasterizer (GAMES101 Assignment 1)

## Requirements
- C++17 compiler and CMake >= 3.10  
- OpenCV (C++ dev package)  
- Eigen

### Quick install
- **macOS (Homebrew)**: `brew install opencv eigen`
- **Ubuntu/Debian**: `sudo apt-get install build-essential cmake libopencv-dev libeigen3-dev`
- **Windows (vcpkg)**: `vcpkg install opencv[eigen]` then configure with `-DCMAKE_TOOLCHAIN_FILE=.../vcpkg.cmake`

## Configure & Build
This repo includes `CMakePresets.json`.

```bash
cd 代码框架
cmake --preset default   # generates build/ and compile_commands.json
cmake --build --preset default
```

If OpenCV is not under `/usr/local/opt/opencv`, edit `OpenCV_DIR` in `CMakePresets.json` to your installed path (e.g., `/opt/homebrew/opt/opencv/lib/cmake/opencv4` on Apple Silicon) and rerun `cmake --preset default`.

## Run
```bash
cd 代码框架/build
./Rasterizer          # interactive; press a/d to rotate, ESC to quit
```
To render once via CLI: `./Rasterizer -r 45 output.png`

## IDE Hints
- VS Code: `.vscode/settings.json` points to `build/compile_commands.json`; after configuring, red squiggles for OpenCV/Eigen should disappear.
- clangd users can symlink at repo root: `ln -sf 代码框架/build/compile_commands.json compile_commands.json` (optional, not committed).

## Git Ignore (recommended)
Add to `.gitignore` if not already:  
```
/代码框架/build/
/compile_commands.json
```
