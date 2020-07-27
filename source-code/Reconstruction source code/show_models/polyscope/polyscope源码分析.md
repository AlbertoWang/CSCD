[TOC]

# 宏观结构
所有的 shader 都在 include\polyscope\gl\shaders 下的头文件中。


# Shader 分析
## 通用
### Vertex Shader
看MVP 矩阵的 传递，structure.cpp 中的 getModelView() 函数。

相机的 View 矩阵取决于，view.cpp 文件的 viewMat 变量。