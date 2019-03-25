#include "MeshMarker.h"

void MeshMarker::SetObject(SurfaceMesh &mesh)
{
    p_mesh_ = &mesh;
    mesh.add_property(slice_);
}

void MeshMarker::ResetMarker()
{
    SurfaceMesh &mesh = *p_mesh_;
    using namespace OpenMesh;
    for (auto eiter = mesh.edges_begin(); eiter != mesh.edges_end(); ++eiter)
    {
        OpenMesh::EdgeHandle e = *eiter;
        mesh.property(slice_, e) = false;
    }
}

// Compute the shortest path between two vertex and set slice flag.
void MeshMarker::ComputeAndSetSlice(OpenMesh::VertexHandle v0, OpenMesh::VertexHandle v1)
{
    SurfaceMesh &mesh = *p_mesh_;
    using namespace OpenMesh;
    VPropHandleT<double> dist;
    VPropHandleT<VertexHandle> parent;

    DijkstraShortestDist(mesh, v0, dist, parent);
    std::vector<VertexHandle> slice_vertices;

    VertexHandle cptr = v1;
    slice_vertices.push_back(v1);
    while (mesh.property(parent, cptr).is_valid())
    {
        cptr = mesh.property(parent, cptr);
        slice_vertices.push_back(cptr);
    }

    std::reverse(slice_vertices.begin(), slice_vertices.end());

    for (int i = 0; i < slice_vertices.size() - 1; ++i)
    {
        EdgeHandle e = mesh.edge_handle(mesh.find_halfedge(slice_vertices[i], slice_vertices[i + 1]));
        if (!mesh.property(slice_, e))
        {
            mesh.property(slice_, e) = true;
        }
    }
}

void MeshMarker::LoadFromFile(std::string filename)
{
    using namespace OpenMesh;
    SurfaceMesh &mesh = *p_mesh_;
    ResetMarker();

    std::ifstream ins(filename);
    if (ins.is_open())
    {
        std::string line;
        while (std::getline(ins, line))
        {
            std::stringstream ss(line);
            std::string mode;
            ss >> mode;

            if (mode != "v" && mode != "e")
                continue;

            std::vector<int> path;
            int index;
            while (ss >> index)
            {
                path.push_back(index);
            }

            if (mode == "v") // vertex sequence on the cut curve, may not be consecutive
            {
                for (int i = 0; i < path.size() - 1; ++i)
                    ComputeAndSetSlice(mesh.vertex_handle(path[i]), mesh.vertex_handle(path[i + 1]));
            }

            else if (mode == "e")
            {
                for (auto i : path)
                {
                    EdgeHandle e = mesh.edge_handle(i);
                    mesh.property(slice_, e) = true;
                }
            }
        }
    }
}

void MeshMarker::SaveToFile(std::string filename)
{
    using namespace OpenMesh;
    SurfaceMesh &mesh = *p_mesh_;

    std::ofstream f(filename);
    f << "e ";
    for (auto eiter = mesh.edges_begin(); eiter != mesh.edges_end(); ++eiter)
    {
        f << " " << eiter->idx();
    }
    f.close();
}
