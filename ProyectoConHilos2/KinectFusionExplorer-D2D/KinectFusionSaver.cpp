#include "stdafx.h"
#include "ppl.h"
#include "KinectFusionProcessor.h"
#include "KinecFusionSaver.h"


#define AssertOwnThread() \
    _ASSERT_EXPR(GetCurrentThreadId() == threadId, __FUNCTIONW__ L" called on wrong thread!");

#define AssertOtherThread() \
    _ASSERT_EXPR(GetCurrentThreadId() != threadId, __FUNCTIONW__ L" called on wrong thread!");

CKinectFusionSaver::CKinectFusionSaver():
    mesh(nullptr),
    threadId(0),
    hThread(nullptr),
    numero(0),
    hWnd(nullptr),
    msgSaved(WM_NULL)
    {}

CKinectFusionSaver::~CKinectFusionSaver() {
    mesh->Release();
}

HRESULT CKinectFusionSaver::SetWindow(HWND hWnd, UINT msgSaved) {
    AssertOtherThread();

    this->hWnd = hWnd;
    this->msgSaved = msgSaved;

    return S_OK;
}
HRESULT CKinectFusionSaver::StartProcessing() {
    AssertOtherThread();

    if (hThread == nullptr)
    {
        hThread = CreateThread(nullptr, 0, ThreadProc, this, 0, &threadId);
    }
    return (hThread != nullptr) ? S_OK : HRESULT_FROM_WIN32(GetLastError());
}

DWORD WINAPI CKinectFusionSaver::ThreadProc(LPVOID lpParameter) {
    return reinterpret_cast<CKinectFusionSaver*>(lpParameter)->MainLoop();
}

DWORD CKinectFusionSaver::MainLoop()
{
    AssertOwnThread();
    while (1) {
        // Si tengo tengo mesh disponble, que me lo han enviado desde KFusionExplorer, ejecuto save mesh internamente.
            SaveMesh(mesh);
            delete mesh;
            if (hWnd != nullptr && msgSaved != WM_NULL) PostMessage(hWnd, msgSaved, 0, 0);
            
    }
    return 0;
}
HRESULT CKinectFusionSaver::SaveMesh(INuiFusionColorMesh* pMesh) {
    AssertOwnThread();
    HRESULT hr = S_OK;
    hr = WriteAsciiPlyMeshFile2(pMesh, numero + 1, true, false);
    numero += 1;
    return hr;

}
HRESULT CKinectFusionSaver::WriteAsciiPlyMeshFile2(INuiFusionColorMesh* mesh, int num, bool flipYZ, bool outputColor) {
    AssertOwnThread();

    HRESULT hr = S_OK;

    if (NULL == mesh)
    {
        return E_INVALIDARG;
    }

    unsigned int numVertices = mesh->VertexCount();
    unsigned int numTriangleIndices = mesh->TriangleVertexIndexCount();
    unsigned int numTriangles = numVertices / 3;
    unsigned int numColors = mesh->ColorCount();

    if (0 == numVertices || 0 == numTriangleIndices || 0 != numVertices % 3
        || numVertices != numTriangleIndices || (outputColor && numVertices != numColors))
    {
        return E_INVALIDARG;
    }

    const Vector3* vertices = NULL;
    hr = mesh->GetVertices(&vertices);
    if (FAILED(hr))
    {
        return hr;
    }

    const int* triangleIndices = NULL;
    hr = mesh->GetTriangleIndices(&triangleIndices);
    if (FAILED(hr))
    {
        return hr;
    }

    const int* colors = NULL;
    if (outputColor)
    {
        hr = mesh->GetColors(&colors);
        if (FAILED(hr))
        {
            return hr;
        }
    }
    /*************************************************************************************************************************************/
    // Open File
    std::string filename("C:\\Users\\Usuario\\3D Objects\\PruebaFusionConHilos\\");
    filename.append(std::to_string(num) + ".ply");
    FILE* meshFile = NULL;
    errno_t err = fopen_s(&meshFile, filename.c_str(), "wt");

    // Could not open file for writing - return
    if (0 != err || NULL == meshFile)
    {
        return E_ACCESSDENIED;
    }

    // Write the header line
    std::string header = "ply\nformat ascii 1.0\ncomment file created by Microsoft Kinect Fusion\n";
    fwrite(header.c_str(), sizeof(char), header.length(), meshFile);

    const unsigned int bufSize = MAX_PATH * 3;
    char outStr[bufSize];
    int written = 0;

    if (outputColor)
    {
        // Elements are: x,y,z, r,g,b
        written = sprintf_s(outStr, bufSize, "element vertex %u\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\n", numVertices);
        fwrite(outStr, sizeof(char), written, meshFile);
    }
    else
    {
        // Elements are: x,y,z
        written = sprintf_s(outStr, bufSize, "element vertex %u\nproperty float x\nproperty float y\nproperty float z\n", numVertices);
        fwrite(outStr, sizeof(char), written, meshFile);
    }

    written = sprintf_s(outStr, bufSize, "element face %u\nproperty list uchar int vertex_index\nend_header\n", numTriangles);
    fwrite(outStr, sizeof(char), written, meshFile);

    if (flipYZ)
    {
        if (outputColor)
        {
            // Sequentially write the 3 vertices of the triangle, for each triangle
            for (unsigned int t = 0, vertexIndex = 0; t < numTriangles; ++t, vertexIndex += 3)
            {
                unsigned int color0 = colors[vertexIndex];
                unsigned int color1 = colors[vertexIndex + 1];
                unsigned int color2 = colors[vertexIndex + 2];

                written = sprintf_s(outStr, bufSize, "%f %f %f %u %u %u\n%f %f %f %u %u %u\n%f %f %f %u %u %u\n",
                    vertices[vertexIndex].x, -vertices[vertexIndex].y, -vertices[vertexIndex].z,
                    ((color0 >> 16) & 255), ((color0 >> 8) & 255), (color0 & 255),
                    vertices[vertexIndex + 1].x, -vertices[vertexIndex + 1].y, -vertices[vertexIndex + 1].z,
                    ((color1 >> 16) & 255), ((color1 >> 8) & 255), (color1 & 255),
                    vertices[vertexIndex + 2].x, -vertices[vertexIndex + 2].y, -vertices[vertexIndex + 2].z,
                    ((color2 >> 16) & 255), ((color2 >> 8) & 255), (color2 & 255));

                fwrite(outStr, sizeof(char), written, meshFile);
            }
        }
        else
        {
            // Sequentially write the 3 vertices of the triangle, for each triangle
            for (unsigned int t = 0, vertexIndex = 0; t < numTriangles; ++t, vertexIndex += 3)
            {
                written = sprintf_s(outStr, bufSize, "%f %f %f\n%f %f %f\n%f %f %f\n",
                    vertices[vertexIndex].x, -vertices[vertexIndex].y, -vertices[vertexIndex].z,
                    vertices[vertexIndex + 1].x, -vertices[vertexIndex + 1].y, -vertices[vertexIndex + 1].z,
                    vertices[vertexIndex + 2].x, -vertices[vertexIndex + 2].y, -vertices[vertexIndex + 2].z);
                fwrite(outStr, sizeof(char), written, meshFile);
            }
        }
    }
    else
    {
        if (outputColor)
        {
            // Sequentially write the 3 vertices of the triangle, for each triangle
            for (unsigned int t = 0, vertexIndex = 0; t < numTriangles; ++t, vertexIndex += 3)
            {
                unsigned int color0 = colors[vertexIndex];
                unsigned int color1 = colors[vertexIndex + 1];
                unsigned int color2 = colors[vertexIndex + 2];

                written = sprintf_s(outStr, bufSize, "%f %f %f %u %u %u\n%f %f %f %u %u %u\n%f %f %f %u %u %u\n",
                    vertices[vertexIndex].x, vertices[vertexIndex].y, vertices[vertexIndex].z,
                    ((color0 >> 16) & 255), ((color0 >> 8) & 255), (color0 & 255),
                    vertices[vertexIndex + 1].x, vertices[vertexIndex + 1].y, vertices[vertexIndex + 1].z,
                    ((color1 >> 16) & 255), ((color1 >> 8) & 255), (color1 & 255),
                    vertices[vertexIndex + 2].x, vertices[vertexIndex + 2].y, vertices[vertexIndex + 2].z,
                    ((color2 >> 16) & 255), ((color2 >> 8) & 255), (color2 & 255));

                fwrite(outStr, sizeof(char), written, meshFile);
            }
        }
        else
        {
            // Sequentially write the 3 vertices of the triangle, for each triangle
            for (unsigned int t = 0, vertexIndex = 0; t < numTriangles; ++t, vertexIndex += 3)
            {
                written = sprintf_s(outStr, bufSize, "%f %f %f\n%f %f %f\n%f %f %f\n",
                    vertices[vertexIndex].x, vertices[vertexIndex].y, vertices[vertexIndex].z,
                    vertices[vertexIndex + 1].x, vertices[vertexIndex + 1].y, vertices[vertexIndex + 1].z,
                    vertices[vertexIndex + 2].x, vertices[vertexIndex + 2].y, vertices[vertexIndex + 2].z);
                fwrite(outStr, sizeof(char), written, meshFile);
            }
        }
    }

    // Sequentially write the 3 vertex indices of the triangle face, for each triangle (0-referenced in PLY)
    for (unsigned int t = 0, baseIndex = 0; t < numTriangles; ++t, baseIndex += 3)
    {
        written = sprintf_s(outStr, bufSize, "3 %u %u %u\n", baseIndex, baseIndex + 1, baseIndex + 2);
        fwrite(outStr, sizeof(char), written, meshFile);
    }

    fflush(meshFile);
    fclose(meshFile);

    return hr;
}