# Patch
patch -d ORB_SLAM3/include -p2 --binary < patch/System.h.patch
sed -i 's/++11/++14/g' ORB_SLAM3/CMakeLists.txt
