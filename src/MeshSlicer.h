#ifndef MESH_SLICER
#define MESH_SLICER

#include "Dijkstra.h"
#include "MeshDefinition.h"

// This class is modified from my previous research codes.
// The algorithm is in Gu's book Computational Conformal Geometry.
class MeshSlicer
{
  public:
    MeshSlicer(SurfaceMesh &mesh);
    void ResetFlags();
    void SliceMeshToDisk(SurfaceMesh &sliced_mesh);
    std::vector<OpenMesh::VertexHandle> SplitTo(OpenMesh::VertexHandle v);
    OpenMesh::HalfedgeHandle ConvertTo(OpenMesh::HalfedgeHandle h);
    std::vector<OpenMesh::VertexHandle> GetLongestPath();
    void FindAndMarkCutGraphSphere();
    void FindAndMarkCutGraphNonSphere();
    void ConstructWedge();
    void SliceAccordingToWedge(SurfaceMesh &sliced_mesh);
    void AddOnCutEdge(OpenMesh::EdgeHandle e) { mesh_.property(on_cut_, e) = true; }
    void SetOnCutEdges(OpenMesh::EPropHandleT<bool> &on_cut) { on_cut_ = on_cut; }
    OpenMesh::VPropHandleT<std::vector<OpenMesh::VertexHandle>> split_to() { return split_to_; }
    void OutputVertexCorrespondences(std::string filename);
  protected:
    SurfaceMesh &mesh_;
    OpenMesh::VertexHandle base_point_;
    OpenMesh::HPropHandleT<int> wedge_;
    OpenMesh::EPropHandleT<bool> on_cut_;
    OpenMesh::VPropHandleT<std::vector<OpenMesh::VertexHandle>> split_to_;
    // one halfedge on the original mesh will appear once and only once on sliced mesh;
    OpenMesh::HPropHandleT<OpenMesh::HalfedgeHandle> convert_to_;
    std::vector<OpenMesh::VertexHandle> longest_path_;
    OpenMesh::HPropHandleT<OpenMesh::HalfedgeHandle> original_reflection_;
};

#endif // !MESH_SLICER
