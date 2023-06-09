mkdir build && cd build
cmake ..
make -j8
sudo make install

cd ../applications
mkdir build && cd build
cmake ..
make -j8