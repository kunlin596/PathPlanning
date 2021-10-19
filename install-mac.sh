#! /bin/bash

echo
echo ----------------------
echo Installing uWebSockets
echo ----------------------
echo

brew install cmake openssl libuv zlib

echo
echo ----------------------
echo Installing uWebSockets
echo ----------------------
echo

git clone https://github.com/uWebSockets/uWebSockets 
cd uWebSockets
git checkout e94b6e1
patch CMakeLists.txt < ../cmakepatch.txt
mkdir build
export PKG_CONFIG_PATH=/usr/local/opt/openssl/lib/pkgconfig
cd build
cmake ..
make 
sudo make install
cd ..
cd ..
sudo rm -r uWebSockets
