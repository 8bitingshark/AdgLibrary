# 🧮 Adg library
---
This is a small C++ library developed as part of the **Advanced Graphics** course.  

It is designed as a simple educational framework for understanding 3D mathematics and its practical use in graphics applications.
### 📌 Key Components
---
- **Vector3** — A lightweight class for representing 3D vectors, points, and versors. Includes basic vector algebra operations (dot, cross), normalization, scaling, and operator overloading.

- **Quaternion** — A class for representing and manipulating 3D rotations using unit quaternions. Supports:
  - Arithmetic operations (addition, scaling, multiplication...)
  - Norm, normalization, conjugation
  - Conversion to/from axis-angle/euler angles/matrix3x3
  - Rotation application (standard and optimized)
  - Dot product 
  - spherical/linear interpolation (slerp/lerp)

- **Matrix3x3** — A minimal matrix class representing a 3×3 matrix. Currently includes:
  - Matrix-vector multiplication (row/column representation)
  - Matrix-matrix multiplication
  - Conversion to/from axis-angle/euler angles
  - Transposition
  - Compatible with Quaternion conversions

- **Transform** — Represents a 3D transformation using translation, rotation (quaternion), and uniform scale. Currently includes:
  - Apply transformation to points, vectors, and versors
  - Cumulation with other transforms
  - Inverse computation (for uniform scaling)
  - Linear interpolation (mixing) between transforms

- **AffineTransform** — A compact representation of an affine transformation using a 3×3 matrix and translation vector. Currently includes:
  - Apply transformation to points, vectors, and versors
  - Construct from translation, quaternion, and scale
  - Export as row-major or column-major 4×4 matrix

- **Mesh** — Represents a 3D mesh, including vertices, normals, and faces. It supports applying transformations and preparing the mesh for export.

- **ObjHandler** — Manages loading and saving of `.obj` meshes for use with the `Mesh` class. Features include:  
  - Simple parsing of vertices, normals, faces, and materials
  - Export meshes to OBJ/MTL format
  - Supports standard OBJ indexing conventions (vertex, UV, normal)

### Example
---
![Png Example](ExampleOfTransformation.png)
“A duo of Suzannes, just a tad transformed.”

## ⚙️ How to Run
---

> 🖥 **Platform:** Windows 10/11  
> 🧰 **IDE:** Visual Studio 2022  
> 🧾 **Language Standard:** C++20

> ⚠️ Currently, the project only supports Windows and Visual Studio 2022.  
> Multi-platform and build-system support (e.g., CMake, Linux) is not yet implemented.

### 🛠 Prerequisites
---
- Visual Studio 2022 installed with:
  - *C++ Desktop Development* workload
- Git LFS installed (for large asset files):
  ```bash
  git lfs install
