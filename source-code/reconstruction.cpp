#include "poisson.cpp"
#include "gp3.cpp"

int main(int argc, char** argv){
    if (argc < 4) {
        std::cerr << argv[0] << "poisson reconstruction:\npoisson xxx.obj(input point cloud) xxx.ply(output mesh)" << std::endl;
        std::cerr << argv[0] << "gp3 reconstruction:\ngp3 xxx.obj(input point cloud) xxx.ply(output mesh)" << std::endl;
        return -1;
    }
    if (strcmp(argv[1],"poisson") == 0){
        poisson_reconstruction(argv[2], argv[3]);
    }
    if (strcmp(argv[1],"gp3") == 0){
        gp3_reconstruction(argv[2], argv[3]);
    }
    return 0;
}