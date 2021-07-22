#pragma once
#include <Windows.h>

class CKinectFusionSaver
{
	DWORD threadId;
	HANDLE hThread;

	int numero;

public:
	CKinectFusionSaver();
	~CKinectFusionSaver();
	HRESULT	SetWindow(HWND hWnd, UINT msgSaved);
	HRESULT SaveMesh(INuiFusionColorMesh* pMesh);
	HRESULT WriteAsciiPlyMeshFile2(INuiFusionColorMesh* mesh, int num, bool flipYZ = true, bool outputColor = false);
	HRESULT StartProcessing();
	INuiFusionColorMesh* mesh;

private:
	HWND                        hWnd;
	UINT                        msgSaved;
	static DWORD WINAPI ThreadProc(LPVOID lpParameter);
	DWORD MainLoop();
};
