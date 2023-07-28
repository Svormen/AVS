/**
 * @file    loop_mesh_builder.cpp
 *
 * @author  Slavomir Svorada <xsvora02@stud.fit.vutbr.cz>
 *
 * @brief   Parallel Marching Cubes implementation using OpenMP loops
 *
 * @date    29 November 2022
 **/

#include <iostream>
#include <math.h>
#include <limits>

#include "loop_mesh_builder.h"

LoopMeshBuilder::LoopMeshBuilder(unsigned gridEdgeSize)
    : BaseMeshBuilder(gridEdgeSize, "OpenMP Loop")
{

}

unsigned LoopMeshBuilder::marchCubes(const ParametricScalarField &field)
{
    // Compute total number of cubes in the grid
    size_t totalCubesCount = mGridSize*mGridSize*mGridSize;

    unsigned loopTotalTriangles = 0;

    // Loop over each coordinate in the 3D grid and pragma
    #pragma omp parallel for schedule(static, 16) reduction(+: loopTotalTriangles)
    for(size_t i = 0; i < totalCubesCount; ++i)
    {
        // Compute 3D position in the grid
        Vec3_t<float> cubeOffset( i % mGridSize,
                                 (i / mGridSize) % mGridSize,
                                  i / (mGridSize*mGridSize));

        // Evaluate "Marching Cube" at given position in the grid and store the number of triangles generated
        loopTotalTriangles += buildCube(cubeOffset, field);
    }
    return loopTotalTriangles;
}

float LoopMeshBuilder::evaluateFieldAt(const Vec3_t<float> &pos, const ParametricScalarField &field)
{
    // Store pointer to and number of 3D points in the field (to avoid "data()" and "size()" call in the loop)
    const Vec3_t<float> *pPoints = field.getPoints().data();
    const unsigned count = unsigned(field.getPoints().size());

    float loopValue = std::numeric_limits<float>::max();

    // Find minimum square distance from points "pos" to any point in the field
    for(unsigned i = 0; i < count; ++i)
    {
        float distanceSquared  = (pos.x - pPoints[i].x) * (pos.x - pPoints[i].x);
        distanceSquared       += (pos.y - pPoints[i].y) * (pos.y - pPoints[i].y);
        distanceSquared       += (pos.z - pPoints[i].z) * (pos.z - pPoints[i].z);

        // Comparing squares instead of real distance to avoid unnecessary "sqrt"s in the loop
        loopValue = std::min(loopValue, distanceSquared);
    }

    // Finally take square root of the minimal square distance to get the real distance
    return sqrt(loopValue);
}

void LoopMeshBuilder::emitTriangle(const BaseMeshBuilder::Triangle_t &triangle)
{
    #pragma omp critical(loopTriangle)
    mTriangles.push_back(triangle);
}
