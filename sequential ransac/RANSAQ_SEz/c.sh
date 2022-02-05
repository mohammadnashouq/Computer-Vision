g++ -c MatrixReaderWriter.cpp
g++ -c PLYWriter.cpp
g++ -c PlaneEstimation.cpp
g++ PlaneRANSAC.cpp MatrixReaderWriter.o PLYWriter.o PlaneEstimation.o -lopencv_core -o PlaneRANSAC
