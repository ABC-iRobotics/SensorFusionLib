cmake -B ../build -DBUILD_ZMQ_TESTS:BOOL=TRUE -DPolicy_ALL_3RD_PARTIES="Download all" -DWITH_DOCS=FALSE .
cd ../build
cmake --build .
ctest .
cmake --install .
