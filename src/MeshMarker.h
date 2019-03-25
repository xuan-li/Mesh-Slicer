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
    void SetObject(SurfaceMesh &mesh);
    void ResetMarker();
    void ComputeAndSetSlice(OpenMesh::VertexHandle v0, OpenMesh::VertexHandle v1);
    void LoadFromFile(std::string filename);
    void SaveToFile(std::string filename);
    OpenMesh::EPropHandleT<bool> slice_flag() const { return slice_; }

  protected:
    SurfaceMesh *p_mesh_;
    OpenMesh::EPropHandleT<bool> slice_;
};

#endif // !MESH_MARKER_H_
