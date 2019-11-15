Step 1. :
- install libzmq:  
`sudo apt-get install libzmq3-dev`

- install cppzmq:
```bash
git clone https://github.com/zeromq/cppzmq.git
cd cppzmq
mkdir build
cd build
cmake ..
sudo make -j4 install
```

- install eigen 3.3.7
```bash
wget http://bitbucket.org/eigen/eigen/get/3.3.7.tar.gz
tar -xvzf 3.3.7.tar.gz
cd eigen-eigen-323c052e1731
mkdir build
cd build 
cmake ..
sudo make install
```


- Install flatbuffers
```bash
git clone https://github.com/google/flatbuffers.git
cd flatbuffers
cmake -G "Unix Makefiles" (install cmake if need)
make
sudo make install
```


Step 2. :
- Compile libSensorFusion:
```bash
cd libSensorFusion
mkdir build
cd build
cmake ..
make
```

