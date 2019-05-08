#include <MeshDefinition.h>
#include <MeshMarker.h>
#include <MeshSlicer.h>
#include <iostream>
#include <list>
#include <vector>

int main(int argc, char **argv)
{

    SurfaceMesh mesh;
    SurfaceMesh sliced_mesh;

    OpenMesh::IO::read_mesh(mesh, argv[1]);
    std::cout << "find vertices: " << mesh.n_vertices() << std::endl;
    MeshMarker marker(mesh);
    MeshSlicer slicer(mesh);
    marker.ComputeCutGraph();

    slicer.SetOnCutEdges(marker.slice_flag());
    slicer.ConstructWedge();
    slicer.SliceAccordingToWedge(sliced_mesh);
    OpenMesh::IO::write_mesh(sliced_mesh, argv[2]);
    return 0;
}