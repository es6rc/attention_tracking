cmake -DOpenFace_DIR=$OPENFACE/install/lib/cmake/OpenFace/ \
CMAKE_BUILD_TYPE=RELEASE  -DCMAKE_CXX_FLAGS="-std=c++11" \
-DCMAKE_EXE_LINKER_FLAGS="-std=c++11"  ../
