#include <MeshDefinition.h>
#include <MeshSlicer.h>
#include <iostream>
#include <list>
#include <vector>
#include <MeshMarker.h>

int main(int argc, char **argv)
{

    SurfaceMesh mesh;
    SurfaceMesh sliced_mesh;

    OpenMesh::IO::read_mesh(mesh, argv[1]);
    std::cout << "find vertices: " << mesh.n_vertices() << std::endl;
	MeshMarker marker;
    marker.SetObject(mesh);
    marker.LoadFromFile(argv[2]);

    MeshSlicer slicer(mesh);
    slicer.SetOnCutEdges(marker.slice_flag());
    slicer.ConstructWedge();
    slicer.SliceAccordingToWedge(sliced_mesh);

    OpenMesh::IO::write_mesh(sliced_mesh, argv[3]);
    return 0;
}