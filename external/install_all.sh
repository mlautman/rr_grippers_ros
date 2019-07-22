# install libsoc
cd libsoc
autoreconf -i
./configure --enable-board=echo236fe\
make
make install
cd ../

# install librevpi
cd librevpi
mkdir build
cd build
cmake ../
make
make install
cd ../../
rm -r librevpi/build

# install mcp
# cd mcp23017_libsoc
# ./configure
# make
# make install
