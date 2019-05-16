#ifndef MESH_MARKER_H_
#define MESH_MARKER_H_

#include "Dijkstra.h"
#include <MeshDefinition.h>

#ifndef PI
#define PI 3.14159254
#endif // !PI

// This class is to store user-defined cones, cone angles and slices.
class MeshMarker
{
  public:
    MeshMarker(SurfaceMesh &mesh);
    void ResetMarker();
    void ComputeCutGraph();
    void ConnectVertices(std::vector<OpenMesh::VertexHandle> vertices);
    void ConnectVerticesToCutGraph(std::vector<OpenMesh::VertexHandle> vertices);
    OpenMesh::EPropHandleT<bool> slice_flag() const { return slice_; }
    void SetSliceFlagByCutGraph() { slice_ = cut_graph_; }
    void AddSlice(OpenMesh::EdgeHandle e) { mesh_.property(slice_, e) = true; }

  protected:
    void FindAndMarkCutGraphSphere();
    void FindAndMarkCutGraphNonSphere();
    void PruneCut();
    void ConnectVertexPair(OpenMesh::VertexHandle v0, OpenMesh::VertexHandle v1);
    void ConnectVertexToCutGraph(OpenMesh::VertexHandle v);
    bool OnSelectedEdge(OpenMesh::VertexHandle v, const OpenMesh::EPropHandleT<bool> flag);
    SurfaceMesh &mesh_;
    OpenMesh::EPropHandleT<bool> slice_;
    OpenMesh::EPropHandleT<bool> cut_graph_;
};

#endif // !MESH_MARKER_H_
