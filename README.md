# Mesh Slicer

A tool to slice a mesh according to vertices sequences or edge sequences.

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

```
MeshSlicer.exe model_file path_file output_path
```
Note that the program has only been tested on Windows.

### Path File Format

This program accept both vertices sequences and edge sequences. Multiple paths are supported. 

The vertex path starts with the letter ```v``` , and the edge path starts with the letter ```e```. The example format is included in the ```data``` folder. 