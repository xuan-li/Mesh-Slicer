#include "MeshMarker.h"
#include "queue"

MeshMarker::MeshMarker(SurfaceMesh &mesh) : mesh_(mesh)
{
    mesh.add_property(slice_);
    ResetMarker();
}

void MeshMarker::ResetMarker()
{
    SurfaceMesh &mesh = mesh_;
    using namespace OpenMesh;
    for (auto eiter = mesh.edges_begin(); eiter != mesh.edges_end(); ++eiter)
    {
        OpenMesh::EdgeHandle e = *eiter;
        mesh.property(slice_, e) = false;
    }
}

// Compute the shortest path between two vertex and set slice flag.
void MeshMarker::ConnectVertexPair(OpenMesh::VertexHandle v0, OpenMesh::VertexHandle v1)
{
    SurfaceMesh &mesh = mesh_;
    using namespace OpenMesh;
    VPropHandleT<VertexHandle> parent;
    VPropHandleT<bool> visited;
    mesh.add_property(parent);
    mesh.add_property(visited);

    // reset visited
    for (auto viter = mesh_.vertices_begin(); viter != mesh_.vertices_end(); ++viter)
    {
        VertexHandle v = *viter;
        mesh_.property(visited, v) = false;
    }

    mesh_.property(visited, v0) = true;

    std::queue<VertexHandle> q;
    q.push(v0);

    bool found = false;
    while (!q.empty() && !found)
    {
        VertexHandle front = q.front();
        q.pop();
        for (auto vviter = mesh_.vv_iter(front); vviter.is_valid(); ++vviter)
        {
            VertexHandle neighbor = *vviter;
            if (!mesh_.property(visited, neighbor))
            {
                mesh_.property(visited, neighbor) = true;
                q.push(neighbor);
                mesh_.property(parent, neighbor) = front;
                if (neighbor == v1)
                {
                    found = true;
                    break;
                }
            }
        }
    }
    mesh_.remove_property(visited);

    std::vector<VertexHandle> slice_vertices;
    slice_vertices.push_back(v1);
    while (slice_vertices.back() != v0)
    {
        slice_vertices.push_back(mesh_.property(parent, slice_vertices.back()));
    }
    mesh_.remove_property(parent);

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

void MeshMarker::ComputeCutGraph()
{
    if (cut_graph_.is_valid())
        return;
    mesh_.add_property(cut_graph_);
    int euler = mesh_.n_vertices() - mesh_.n_edges() + mesh_.n_faces();
    if (euler == 2)
        FindAndMarkCutGraphSphere();
    else
        FindAndMarkCutGraphNonSphere();
}

void MeshMarker::ConnectVertices(std::vector<OpenMesh::VertexHandle> vertices) 
{
    using namespace OpenMesh;
    for (int i = 0; i < vertices.size() - 1; ++i)
    {
        ConnectVertexPair(vertices[i], vertices[i + 1]);
    }
}

void MeshMarker::ConnectVerticesToCutGraph(std::vector<OpenMesh::VertexHandle> vertices)
{
    for (auto v : vertices)
    {
        ConnectVertexToCutGraph(v);
    }
}

void MeshMarker::FindAndMarkCutGraphSphere()
{
    using namespace OpenMesh;
    auto base_point = *(mesh_.vertices_begin());
    OpenMesh::VPropHandleT<double> dist;
    OpenMesh::VPropHandleT<OpenMesh::VertexHandle> parent;
    DijkstraShortestDist(mesh_, base_point, dist, parent);

    // Find the biggest dist
    double max_dist = 0;
    VertexHandle end;
    for (auto viter = mesh_.vertices_begin(); viter != mesh_.vertices_end(); ++viter)
    {
        VertexHandle v = *viter;
        if (mesh_.property(dist, v) > max_dist)
        {
            max_dist = mesh_.property(dist, v);
            end = v;
        }
    }

    std::vector<VertexHandle> longest_path_vertices;

    VertexHandle cptr = end;
    longest_path_vertices.push_back(end);
    while (mesh_.property(parent, cptr).is_valid())
    {
        cptr = mesh_.property(parent, cptr);
        longest_path_vertices.push_back(cptr);
    }

    std::reverse(longest_path_vertices.begin(), longest_path_vertices.end());

    longest_path_vertices;
    for (auto eiter = mesh_.edges_begin(); eiter != mesh_.edges_end(); ++eiter)
    {
        OpenMesh::EdgeHandle e = *eiter;
        mesh_.property(cut_graph_, e) = false;
    }
    for (int i = 0; i < longest_path_vertices.size() - 1; ++i)
    {
        EdgeHandle e = mesh_.edge_handle(mesh_.find_halfedge(longest_path_vertices[i], longest_path_vertices[i + 1]));
        mesh_.property(cut_graph_, e) = true;
    }
}

void MeshMarker::FindAndMarkCutGraphNonSphere()
{
    using namespace OpenMesh;
    for (auto eiter = mesh_.edges_begin(); eiter != mesh_.edges_end(); ++eiter)
    {
        OpenMesh::EdgeHandle e = *eiter;
        mesh_.property(cut_graph_, e) = true;
    }

    FPropHandleT<bool> visited;
    mesh_.add_property(visited);
    for (auto fiter = mesh_.faces_begin(); fiter != mesh_.faces_end(); ++fiter)
    {
        FaceHandle f = *fiter;
        mesh_.property(visited, f) = false;
    }

    std::queue<FaceHandle> q;
    auto root_face = *mesh_.faces_begin();
    q.push(root_face);
    mesh_.property(visited, root_face) = true;
    while (!q.empty())
    {
        auto front = q.front();
        q.pop();
        for (auto ffiter = mesh_.ff_iter(front); ffiter.is_valid(); ++ffiter)
        {
            auto neighbor = *ffiter;
            if (!mesh_.property(visited, neighbor))
            {
                mesh_.property(visited, neighbor) = true;
                auto intersect = mesh_.FaceFaceIntersection(front, neighbor);
                mesh_.property(cut_graph_, intersect) = false;
                q.push(neighbor);
            }
        }
    }
    mesh_.remove_property(visited);
    PruneCut();
}

void MeshMarker::PruneCut()
{
    using namespace OpenMesh;
    bool cont;
    do
    {
        cont = false;
        for (auto viter = mesh_.vertices_begin(); viter != mesh_.vertices_end(); ++viter)
        {
            VertexHandle v = *viter;
            std::vector<HalfedgeHandle> he_on_cut;
            for (auto vhiter = mesh_.vih_iter(v); vhiter.is_valid(); ++vhiter)
            {
                HalfedgeHandle h = *vhiter;
                EdgeHandle e = mesh_.edge_handle(h);
                if (mesh_.property(cut_graph_, e))
                {
                    he_on_cut.push_back(h);
                }
            }
            if (he_on_cut.size() == 1)
            {
                mesh_.property(cut_graph_, mesh_.edge_handle(he_on_cut.front())) = false;
                cont = true;
            }
            if (he_on_cut.size() == 2)
            {
                auto h1 = he_on_cut.front();
                auto h2 = he_on_cut.back();
                if (mesh_.next_halfedge_handle(h1) == mesh_.opposite_halfedge_handle(h2))
                {
                    mesh_.property(cut_graph_, mesh_.edge_handle(h1)) = false;
                    mesh_.property(cut_graph_, mesh_.edge_handle(h2)) = false;
                    mesh_.property(cut_graph_, mesh_.edge_handle(mesh_.prev_halfedge_handle(h1))) = true;
                    cont = true;
                }
                if (mesh_.next_halfedge_handle(h2) == mesh_.opposite_halfedge_handle(h1))
                {
                    mesh_.property(cut_graph_, mesh_.edge_handle(h1)) = false;
                    mesh_.property(cut_graph_, mesh_.edge_handle(h2)) = false;
                    mesh_.property(cut_graph_, mesh_.edge_handle(mesh_.prev_halfedge_handle(h2))) = true;
                    cont = true;
                }
            }
        }
    } while (cont);
}

void MeshMarker::ConnectVertexToCutGraph(OpenMesh::VertexHandle v0)
{
    SurfaceMesh &mesh = mesh_;
    using namespace OpenMesh;
    VPropHandleT<VertexHandle> parent;
    VPropHandleT<bool> visited;
    mesh.add_property(parent);
    mesh.add_property(visited);

    // reset visited
    for (auto viter = mesh_.vertices_begin(); viter != mesh_.vertices_end(); ++viter)
    {
        VertexHandle v = *viter;
        mesh_.property(visited, v) = false;
    }

    mesh_.property(visited, v0) = true;

    std::queue<VertexHandle> q;
    q.push(v0);

    bool found = false;
    VertexHandle connector;
    while (!q.empty() && !found)
    {
        VertexHandle front = q.front();
        q.pop();
        for (auto vviter = mesh_.vv_iter(front); vviter.is_valid(); ++vviter)
        {
            VertexHandle neighbor = *vviter;
            if (!mesh_.property(visited, neighbor))
            {
                if (OnSelectedEdge(neighbor, cut_graph_))
                {
                    found = true;
                    mesh_.property(parent, neighbor) = front;
                    connector = neighbor;
                    break;
                }
                else if (OnSelectedEdge(neighbor, slice_))
                {
                    continue;
                }

                mesh_.property(visited, neighbor) = true;
                q.push(neighbor);
                mesh_.property(parent, neighbor) = front;
            }
        }
    }
    mesh_.remove_property(visited);

    std::vector<VertexHandle> slice_vertices;
    slice_vertices.push_back(connector);
    while (slice_vertices.back() != v0)
    {
        slice_vertices.push_back(mesh_.property(parent, slice_vertices.back()));
    }
    mesh_.remove_property(parent);

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

bool MeshMarker::OnSelectedEdge(OpenMesh::VertexHandle v, const OpenMesh::EPropHandleT<bool> flag)
{
    for (auto veiter = mesh_.ve_iter(v); veiter.is_valid(); ++veiter)
    {
        if (mesh_.property(flag, *veiter))
        {
            return true;
        }
    }
    return false;
}
