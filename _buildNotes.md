# Bootloader build notes

VSC build does not work, CMake needs to be configured differently.

VSC Cmake interferes with the command-line cmake, so needs to be turned off
in `.vscode/settings.json`, add 
```
{
    "cmake.automaticReconfigure": false,
    "cmake.configureOnEdit": false
}
```

Delete the build folder in case of failures.

Manual build from command line:

cd into the project root dir (e.g. D:\bootloader\rp2040-serial-bootloader\)
```
mkdir build
cd build
cmake -G "MinGW Makefiles" ..
make
```
