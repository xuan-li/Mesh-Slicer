# Mesh Slicer

A tool to slice a mesh.

## Dependency
- OpenMesh

## Compile

```
mkdir build
cd build
cmake .. -G "Visual Studio 15 2017 Win64" -DOpenMesh_DIR=your_installation_path
```

```OpenMesh_DIR``` is the directory contains the ```include``` and ```lib``` folder.

## Usage

1. Use ```MeshMarker``` to mark the edges to be split.
   - ```ComputeCutGraph```: mark the cut graph, the cut graph will be stored in the class to be reused.
   - ```SetSliceFlagByCutGraph```: mark the cut graph to be the edges to be split.
   - ```ConnectVerticesToCutGraph```: connect some vertices to the cut graph, and add paths to the edges to be split.
   -   ```ConnectVertices```: connect vertex sequence, and the path to the edges to be split.
2. Use ```MeshSlicer``` to slice the mesh according to the marker stored in a ```MeshMarker```.

```c++
MeshSlicer slicer(mesh);
slicer.SetOnCutEdges(marker.slice_flag());
slicer.ConstructWedge();
slicer.SliceAccordingToWedge(sliced_mesh);
```
