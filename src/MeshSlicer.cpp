#include "MeshSlicer.h"
#include <fstream>
#include <list>
#include <map>
#include <queue>

MeshSlicer::MeshSlicer(SurfaceMesh &mesh) : mesh_(mesh)
{

    mesh_.add_property(wedge_);
    mesh_.add_property(on_cut_);
    mesh_.add_property(split_to_);
    mesh_.add_property(convert_to_);
}

void MeshSlicer::ResetFlags()
{
    for (auto eiter = mesh_.edges_begin(); eiter != mesh_.edges_end(); ++eiter)
    {
        mesh_.property(on_cut_, *eiter) = false;
    }
}

std::vector<OpenMesh::VertexHandle> MeshSlicer::SplitTo(OpenMesh::VertexHandle v) { return mesh_.property(split_to_, v); }

OpenMesh::HalfedgeHandle MeshSlicer::ConvertTo(OpenMesh::HalfedgeHandle h) { return mesh_.property(convert_to_, h); }

void MeshSlicer::ConstructWedge()
{
    using namespace OpenMesh;
    for (auto hiter = mesh_.halfedges_begin(); hiter != mesh_.halfedges_end(); ++hiter)
    {
        OpenMesh::HalfedgeHandle h = *hiter;
        mesh_.property(wedge_, h) = 0;
    }
    for (SurfaceMesh::VertexIter viter = mesh_.vertices_begin(); viter != mesh_.vertices_end(); ++viter)
    {
        VertexHandle v = *viter;
        std::list<HalfedgeHandle> halfedges_around;
        for (SurfaceMesh::VertexIHalfedgeCWIter vihiter = mesh_.vih_cwiter(v); vihiter.is_valid(); ++vihiter)
        {
            HalfedgeHandle h = *vihiter;
            if (mesh_.is_boundary(h))
                continue;
            halfedges_around.push_back(h);
        }
        for (std::list<HalfedgeHandle>::iterator it = halfedges_around.begin(); it != halfedges_around.end(); ++it)
        {
            HalfedgeHandle h = *it;
            if (mesh_.is_boundary(v))
            {
                if (mesh_.is_boundary(mesh_.opposite_halfedge_handle(h)))
                {
                    halfedges_around.insert(halfedges_around.end(), halfedges_around.begin(), it);
                    halfedges_around.erase(halfedges_around.begin(), it);
                    break;
                }
            }
            else
            {
                if (mesh_.property(on_cut_, mesh_.edge_handle(h)))
                {
                    halfedges_around.insert(halfedges_around.end(), halfedges_around.begin(), it);
                    halfedges_around.erase(halfedges_around.begin(), it);
                    break;
                }
            }
        }

        /*construct wedge around vertex v*/
        int w = 0;
        std::list<HalfedgeHandle>::iterator it = halfedges_around.begin();
        mesh_.property(wedge_, *it) = 0;
        ++it;
        for (; it != halfedges_around.end(); ++it)
        {
            HalfedgeHandle h = *it;
            if (mesh_.property(on_cut_, mesh_.edge_handle(h)) || mesh_.is_boundary(mesh_.edge_handle(h)))
            {
                w++;
            }
            mesh_.property(wedge_, h) = w;
        }
    }
}

void MeshSlicer::SliceAccordingToWedge(SurfaceMesh &new_mesh)
{
    using namespace OpenMesh;
    HPropHandleT<VertexHandle> new_end; // store the new end of an halfedge after being cutted.
    mesh_.add_property(new_end);

    HPropHandleT<HalfedgeHandle> halfedge_split_to;
    mesh_.add_property(halfedge_split_to);

    int max_vid = mesh_.n_vertices() - 1;

    for (SurfaceMesh::VertexIter viter = mesh_.vertices_begin(); viter != mesh_.vertices_end(); ++viter)
    {
        VertexHandle v = *viter;
        // triverse around in-halfedges, add new vertices if needed and bind halfedges to their new ends.
        std::map<int, VertexHandle> whether_new_vert_exists;
        for (SurfaceMesh::VertexIHalfedgeIter vihiter = mesh_.vih_iter(v); vihiter.is_valid(); ++vihiter)
        {
            HalfedgeHandle h = *vihiter;
            int wedge = mesh_.property(wedge_, h);
            if (whether_new_vert_exists[wedge].is_valid())
                mesh_.property(new_end, h) = whether_new_vert_exists[wedge];
            else
            {
                VertexHandle new_vertex = new_mesh.add_vertex(mesh_.point(v));
                mesh_.property(split_to_, v).push_back(new_vertex);
                mesh_.property(new_end, h) = new_vertex;
                whether_new_vert_exists[wedge] = new_vertex;
            }
        }
    }

    /*construct face*/
    for (SurfaceMesh::FaceIter fiter = mesh_.faces_begin(); fiter != mesh_.faces_end(); ++fiter)
    {
        FaceHandle f = *fiter;
        std::vector<VertexHandle> verts;
        std::vector<VertexHandle> old_verts;
        for (SurfaceMesh::FaceHalfedgeIter fhiter = mesh_.fh_iter(f); fhiter.is_valid(); ++fhiter)
        {
            HalfedgeHandle h = *fhiter;
            VertexHandle v = mesh_.to_vertex_handle(h);
            old_verts.push_back(v);
            VertexHandle nv = mesh_.property(new_end, h);
            verts.push_back(nv);
        }
        FaceHandle new_f = new_mesh.add_face(verts);
        for (int i = 0; i < 3; ++i)
        {
            HalfedgeHandle he = new_mesh.find_halfedge(verts[i], verts[(i + 1) % 3]);
            HalfedgeHandle ohe = mesh_.find_halfedge(old_verts[i], old_verts[(i + 1) % 3]);
            mesh_.property(convert_to_, ohe) = he;
            EdgeHandle e = new_mesh.edge_handle(he);
            EdgeHandle oe = mesh_.edge_handle(ohe);
        }
    }

    new_mesh.RequestBoundary();
}
