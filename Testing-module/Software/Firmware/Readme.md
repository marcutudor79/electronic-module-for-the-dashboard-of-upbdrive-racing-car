# ECU Emulator Firmware

## How to build the project (on Windows)
1. Open a console (cmd) with the path set to 'Firmware' directory

2. Change directory into the 'main' folder

    ```
    cd main
    ```

3. Create a build directory

    ```
    mkdir build
    cd build
    ```

4. Use 'cmake' to generate the build files
    ```
    cmake ..
    ```

5. Build the project using 'cmake'
    ```
    cmake --build .
    ```

6. Copy main.uf2 file on the Raspberry Pi Pico board

## Prerequisites (on Windows)

Follow this tutorial [https://shawnhymel.com/2096/how-to-set-up-raspberry-pi-pico-c-c-toolchain-on-windows-with-vs-code/.](https://shawnhymel.com/2096/how-to-set-up-raspberry-pi-pico-c-c-toolchain-on-windows-with-vs-code/)

! 'Download Pico SDK and Examples' part is not needed for this project

! 'Install VS Code' part is not needed for this project

! 'Build Blink Example' part is needed for this project