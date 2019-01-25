#include <cslibs_mesh_map/mesh_map.h>

using namespace cslibs_mesh_map;

int main(int argc, char *argv[])
{
    MeshMap map;
    if(!map.loadMeshWithNormals(argv[1])){
        std::cerr << "Invalid input file: " << argv[1] << std::endl;
        return 42;
    }
    std::cout << "File successfully loaded!" << std::endl;
    return 0;
}
