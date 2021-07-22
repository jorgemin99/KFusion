// ConsoleApplication1.cpp : Este archivo contiene la función "main". La ejecución del programa comienza y termina ahí.
//

#include <iostream>
#include <sstream>
#include <vector>
#include <array>
#define _USE_MATH_DEFINES
#include <cmath>
#include <atlbase.h>
#include <ppl.h> // for Concurrency::parallel_for

#include <Windows.h>
#include <Kinect.h>
#include <NuiKinectFusionApi.h>
#include <fstream>

// Quote from Kinect for Windows SDK v2.0 - Samples/Native/KinectFusionExplorer-D2D, and Partial Modification
// KinectFusionHelper is: Copyright (c) Microsoft Corporation. All rights reserved.

#include "KinectFusionHelper.h"

#include <opencv2/opencv.hpp>

#define ERROR_CHECK( ret )											\
	if( FAILED( ret ) ){											\
		std::stringstream ss;										\
		ss << "failed " #ret " " << std::hex << ret << std::endl;	\
		throw std::runtime_error( ss.str().c_str() );				\
		}

class KinectApp
{
private:

	CComPtr<IKinectSensor> kinect;

	CComPtr<IColorFrameReader> colorFrameReader;
	CComPtr<IDepthFrameReader> depthFrameReader;
	CComPtr<ICoordinateMapper> coordinateMapper;
	std::vector<BYTE> colorBuffer;
	std::vector<UINT16> depthBuffer;
	int colorWidth;
	int colorHeight;
	int depthWidth;
	int depthHeight;
	int num;
	unsigned int colorBytesPerPixel;
	unsigned int depthBytesPerPixel;
	ColorImageFormat colorFormat = ColorImageFormat::ColorImageFormat_Bgra;

	CComPtr<INuiFusionColorReconstruction> reconstruction;
	NUI_FUSION_IMAGE_FRAME* depthImageFrame;
	NUI_FUSION_IMAGE_FRAME* smoothDepthImageFrame;
	NUI_FUSION_IMAGE_FRAME* colorImageFrame;
	NUI_FUSION_IMAGE_FRAME* pointCloudImageFrame;
	NUI_FUSION_IMAGE_FRAME* surfaceImageFrame;
	/*NUI_FUSION_IMAGE_FRAME* normalImageFrame;*/
	NUI_FUSION_RECONSTRUCTION_PARAMETERS reconstructionParameters;
	NUI_FUSION_CAMERA_PARAMETERS cameraParameters;
	Matrix4 worldToCameraTransform;
	Matrix4 mthAnt;
	Matrix4 mthGlobal;
	cv::Mat surfaceImage;
	/*cv::Mat normalImage;*/
	std::ofstream filePoses;

public:

	void initialize()
	{
		// Sensor
		ERROR_CHECK(GetDefaultKinectSensor(&kinect));

		ERROR_CHECK(kinect->Open());

		BOOLEAN isOpen;
		ERROR_CHECK(kinect->get_IsOpen(&isOpen));
		if (!isOpen) {
			throw std::runtime_error("failed IKinectSensor::get_IsOpen( &isOpen )");
		}

		ERROR_CHECK(kinect->get_CoordinateMapper(&coordinateMapper));

		// Color Frame Source Color Frame Reader
		CComPtr<IColorFrameSource> colorFrameSource;
		ERROR_CHECK(kinect->get_ColorFrameSource(&colorFrameSource));
		ERROR_CHECK(colorFrameSource->OpenReader(&colorFrameReader));

		CComPtr<IFrameDescription> colorFrameDescription;
		ERROR_CHECK(colorFrameSource->CreateFrameDescription(colorFormat, &colorFrameDescription));
		ERROR_CHECK(colorFrameDescription->get_Width(&colorWidth));
		ERROR_CHECK(colorFrameDescription->get_Height(&colorHeight));
		ERROR_CHECK(colorFrameDescription->get_BytesPerPixel(&colorBytesPerPixel));

		colorBuffer.resize(colorWidth * colorHeight * colorBytesPerPixel);

		// Depth Frame Source Depth Frame Reader
		CComPtr<IDepthFrameSource> depthFrameSource;
		ERROR_CHECK(kinect->get_DepthFrameSource(&depthFrameSource));
		ERROR_CHECK(depthFrameSource->OpenReader(&depthFrameReader));

		CComPtr<IFrameDescription> depthFrameDescription;
		ERROR_CHECK(depthFrameSource->get_FrameDescription(&depthFrameDescription));
		ERROR_CHECK(depthFrameDescription->get_Width(&depthWidth));
		ERROR_CHECK(depthFrameDescription->get_Height(&depthHeight));
		ERROR_CHECK(depthFrameDescription->get_BytesPerPixel(&depthBytesPerPixel));

		depthBuffer.resize(depthWidth * depthHeight);

		// Fusion
		initializeFusion();
	}

	void run()
	{
		while (1) {
			update();
			draw();
			if (CheckForReset()) {
				SavePose("filePoses4.txt");
				saveMesh();
				reset();
			}
			int key = cv::waitKey(10);
			if (key == VK_ESCAPE) {
				release();
				cv::destroyAllWindows();
				break;
			}
			else if (key == 'r') {
				std::cout << "Reset Reconstruction" << std::endl;
				reset();
			}
			else if (key == 's') {
				std::cout << "Save Mesh File" << std::endl;
				saveMesh();
			}
		}
	}

private:

	void initializeFusion()
	{
		num = 0;

		reconstructionParameters.voxelsPerMeter = 128;//256
		reconstructionParameters.voxelCountX = 640;//512
		reconstructionParameters.voxelCountY = 384;//384
		reconstructionParameters.voxelCountZ = 640;//512

		// Reconstruction
		SetIdentityMatrix(worldToCameraTransform);
		SetIdentityMatrix(mthGlobal);
		SetIdentityMatrix(mthAnt);

		ERROR_CHECK(NuiFusionCreateColorReconstruction(&reconstructionParameters, NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE::NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_AMP, -1, &worldToCameraTransform, &reconstruction));

		//
		cameraParameters.focalLengthX = NUI_KINECT_DEPTH_NORM_FOCAL_LENGTH_X;
		cameraParameters.focalLengthY = NUI_KINECT_DEPTH_NORM_FOCAL_LENGTH_Y;
		cameraParameters.principalPointX = NUI_KINECT_DEPTH_NORM_PRINCIPAL_POINT_X;
		cameraParameters.principalPointY = NUI_KINECT_DEPTH_NORM_PRINCIPAL_POINT_Y;

		// Image Frame
		ERROR_CHECK(NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE::NUI_FUSION_IMAGE_TYPE_FLOAT, depthWidth, depthHeight, &cameraParameters, &depthImageFrame));
		ERROR_CHECK(NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE::NUI_FUSION_IMAGE_TYPE_FLOAT, depthWidth, depthHeight, &cameraParameters, &smoothDepthImageFrame));
		ERROR_CHECK(NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE::NUI_FUSION_IMAGE_TYPE_COLOR, depthWidth, depthHeight, &cameraParameters, &colorImageFrame));
		ERROR_CHECK(NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE::NUI_FUSION_IMAGE_TYPE_POINT_CLOUD, depthWidth, depthHeight, &cameraParameters, &pointCloudImageFrame));
		ERROR_CHECK(NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE::NUI_FUSION_IMAGE_TYPE_COLOR, depthWidth, depthHeight, &cameraParameters, &surfaceImageFrame));
		/*ERROR_CHECK( NuiFusionCreateImageFrame( NUI_FUSION_IMAGE_TYPE::NUI_FUSION_IMAGE_TYPE_COLOR, depthWidth, depthHeight, &cameraParameters, &normalImageFrame ) );*/
	}

	void update()
	{
		updateColorFrame();
		updateDepthFrame();
		updateFusionFrame();
	}

	void updateColorFrame()
	{
		CComPtr<IColorFrame> colorFrame;
		HRESULT ret = colorFrameReader->AcquireLatestFrame(&colorFrame);
		if (FAILED(ret)) {
			return;
		}

		ERROR_CHECK(colorFrame->CopyConvertedFrameDataToArray(static_cast<UINT>(colorBuffer.size()), &colorBuffer[0], colorFormat));
	}

	void updateDepthFrame()
	{
		CComPtr<IDepthFrame> depthFrame;
		HRESULT ret = depthFrameReader->AcquireLatestFrame(&depthFrame);
		if (FAILED(ret)) {
			return;
		}

		ERROR_CHECK(depthFrame->CopyFrameDataToArray(static_cast<UINT>(depthBuffer.size()), &depthBuffer[0]));
	}

	void updateFusionFrame()
	{
		// Depth Float Image
		ERROR_CHECK(reconstruction->DepthToDepthFloatFrame(&depthBuffer[0], static_cast<UINT>(depthBuffer.size() * depthBytesPerPixel), depthImageFrame, NUI_FUSION_DEFAULT_MINIMUM_DEPTH/* 0.5[m] */, NUI_FUSION_DEFAULT_MAXIMUM_DEPTH/* 8.0[m] */, true));
		ERROR_CHECK(reconstruction->SmoothDepthFloatFrame(depthImageFrame, smoothDepthImageFrame, NUI_FUSION_DEFAULT_SMOOTHING_KERNEL_WIDTH, NUI_FUSION_DEFAULT_SMOOTHING_DISTANCE_THRESHOLD));

		// Color Image
		std::vector<ColorSpacePoint> points(depthWidth * depthHeight);
		ERROR_CHECK(coordinateMapper->MapDepthFrameToColorSpace(depthWidth * depthHeight, &depthBuffer[0], depthWidth * depthHeight, &points[0]));
		NUI_FUSION_BUFFER* colorImageFrameBuffer = colorImageFrame->pFrameBuffer;
		RGBQUAD* src = reinterpret_cast<RGBQUAD*>(&colorBuffer[0]);
		RGBQUAD* dst = reinterpret_cast<RGBQUAD*>(colorImageFrameBuffer->pBits);
		Concurrency::parallel_for(0, depthHeight, [&](int y) {
			for (int x = 0; x < depthWidth; x++) {
				unsigned int index = y * depthWidth + x;
				const ColorSpacePoint point = points[index];
				int colorX = static_cast<int>(std::ceil(point.X));
				int colorY = static_cast<int>(std::ceil(point.Y));
				if ((colorX >= 0) && (colorX < colorWidth) && (colorY >= 0) && (colorY < colorHeight)) {
					dst[index] = src[colorY * colorWidth + colorX];
				}
			}
			});

		ERROR_CHECK(reconstruction->GetCurrentWorldToCameraTransform(&worldToCameraTransform));

		HRESULT ret = reconstruction->ProcessFrame(smoothDepthImageFrame, colorImageFrame, NUI_FUSION_DEFAULT_ALIGN_ITERATION_COUNT, NUI_FUSION_DEFAULT_INTEGRATION_WEIGHT, NUI_FUSION_DEFAULT_COLOR_INTEGRATION_OF_ALL_ANGLES, nullptr, &worldToCameraTransform);
		if (FAILED(ret)) {
			static unsigned int errorCount = 0;
			if (++errorCount >= 100) {
				errorCount = 0;
				reset();
			}
		}

		result();
	}

	inline void result()
	{
		// Point Cloud
		ERROR_CHECK(reconstruction->CalculatePointCloud(pointCloudImageFrame, surfaceImageFrame, &worldToCameraTransform));

		/*// Shading Color Transform Matrix
		Matrix4 worldToBGRTransform = { 0.0f };
		worldToBGRTransform.M11 = reconstructionParameters.voxelsPerMeter / reconstructionParameters.voxelCountX;
		worldToBGRTransform.M22 = reconstructionParameters.voxelsPerMeter / reconstructionParameters.voxelCountY;
		worldToBGRTransform.M33 = reconstructionParameters.voxelsPerMeter / reconstructionParameters.voxelCountZ;
		worldToBGRTransform.M41 = 0.5f;
		worldToBGRTransform.M42 = 0.5f;
		worldToBGRTransform.M43 = 0.0f;
		worldToBGRTransform.M44 = 1.0f;

		// Shading Point Cloud
		ERROR_CHECK( NuiFusionShadePointCloud( pointCloudImageFrame, &worldToCameraTransform, &worldToBGRTransform, surfaceImageFrame, normalImageFrame ) );*/
	}

	inline void reset()
	{
		// Reconstruction
		SetIdentityMatrix(worldToCameraTransform);
		ERROR_CHECK(reconstruction->ResetReconstruction(&worldToCameraTransform, nullptr));
	}

	void release()
	{
		// Image Frame
		ERROR_CHECK(NuiFusionReleaseImageFrame(depthImageFrame));
		ERROR_CHECK(NuiFusionReleaseImageFrame(smoothDepthImageFrame));
		ERROR_CHECK(NuiFusionReleaseImageFrame(colorImageFrame));
		ERROR_CHECK(NuiFusionReleaseImageFrame(pointCloudImageFrame));
		ERROR_CHECK(NuiFusionReleaseImageFrame(surfaceImageFrame));
		/*ERROR_CHECK( NuiFusionReleaseImageFrame( normalImageFrame ) );*/
	}

	inline void saveMesh()
	{
		// Mesh
		CComPtr<INuiFusionColorMesh> mesh;
		ERROR_CHECK(reconstruction->CalculateMesh(1, &mesh));

		// Mesh File
		//L"mesh.ply"
		//wchar_t* fileName = L"mesh.ply";
		ERROR_CHECK(WriteAsciiPlyMeshFile2(mesh, num + 1, true, true));

		/*wchar_t* fileName = L"mesh.stl";
		ERROR_CHECK( WriteBinarySTLMeshFile( mesh, W2OLE( fileName ), true ) );*/

		///*wchar_t* fileName = L"mesh.obj";
		//ERROR_CHECK( WriteAsciiObjMeshFile2( mesh, num+1, true ) );
		num++;
	}

	void draw()
	{
		drawFusionFrame();
	}

	void drawFusionFrame()
	{
		// Fusion
		NUI_FUSION_BUFFER* surfaceImageFrameBuffer = surfaceImageFrame->pFrameBuffer;
		surfaceImage = cv::Mat(depthHeight, depthWidth, CV_8UC4, surfaceImageFrameBuffer->pBits);
		cv::imshow("Surface", surfaceImage);
		/*NUI_FUSION_BUFFER* normalImageFrameBuffer = normalImageFrame->pFrameBuffer;
		normalImage = cv::Mat( depthHeight, depthWidth, CV_8UC4, normalImageFrameBuffer->pBits );
		cv::imshow( "Normal", normalImage );*/
	}
	bool CheckForReset()
	{
		float roll, pitch, yaw, trans;

		yaw = atan2(worldToCameraTransform.M12, worldToCameraTransform.M11);
		pitch = atan2(-worldToCameraTransform.M13, sqrt(pow(worldToCameraTransform.M23, 2) + pow(worldToCameraTransform.M33, 2)));
		roll = atan2(worldToCameraTransform.M23, worldToCameraTransform.M33);
		trans = sqrt(pow(worldToCameraTransform.M41, 2) + pow(worldToCameraTransform.M42, 2) + pow(worldToCameraTransform.M43, 2));
		if (abs(yaw) >= 1.6 | abs(pitch) >= 1.6 | abs(roll) >= 1.6 | abs(trans) >= 0.4)
		{
			multiplica(&mthAnt.M11, &worldToCameraTransform.M11, &mthGlobal.M11);
			mthAnt = mthGlobal;
			return true;
		}
		return false;
	}
	void multiplica(float* mthAnt, float* mth, float* mthRes) 
	{
		int row, col, i;
		int index, index1, index2;
		float sum;
		for (row = 0; row < 4; row++) {
			for (col = 0; col < 4; col++) {
				sum = 0;
				index = col + (row * 4);
				for (i = 0; i < 4; i++) {
					index1 = col + (i * 4);
					index2 = i + (row * 4);
					sum += (*(mthAnt + index1)) * (*(mth + index2));
				}
				*(mthRes + index) = sum;
			}
		}
	}
	void SavePose(const char* filename)
	{
		filePoses.open(filename, std::ios_base::app);//app -> append
		filePoses << "[" << mthGlobal.M11 << " " << mthGlobal.M21 << " " << mthGlobal.M31 << " " << mthGlobal.M41 << "; ";
		filePoses << mthGlobal.M12 << " " << mthGlobal.M22 << " " << mthGlobal.M32 << " " << mthGlobal.M42 << "; ";
		filePoses << mthGlobal.M13 << " " << mthGlobal.M23 << " " << mthGlobal.M33 << " " << mthGlobal.M43 << "; ";
		filePoses << mthGlobal.M14 << " " << mthGlobal.M24 << " " << mthGlobal.M34 << " " << mthGlobal.M44 << "]\n";
		filePoses.close();
	}
};

void main()
{
	try {
		KinectApp app;
		app.initialize();
		app.run();
	}
	catch (std::exception& ex) {
		std::cout << ex.what() << std::endl;
	}
}
