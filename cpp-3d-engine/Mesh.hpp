#pragma once
#include "stl_reader/stl_reader.h"

class Mesh{
    public:
        Mesh();
        void calculate_normals();
};

typedef union Triangle
{
    Vector3D array[3];
    struct vertex{
        Vector3D a;
        Vector3D b;
        Vector3D c;
    };
};

typedef union Vector3D{
    float array[3];
    struct
    {
        float x;
        float y;
        float z;
    }axis;
};
