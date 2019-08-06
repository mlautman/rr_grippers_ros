# install libsoc
cd libsoc
autoreconf -i
./configure --enable-board=echo236fe
make
sudo make install
cd ../
