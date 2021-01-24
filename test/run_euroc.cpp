
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <thread>
#include <iomanip>

#include <cv.h>
#include <opencv2/opencv.hpp>
#include <highgui.h>
#include <eigen3/Eigen/Dense>
#include "System.h"

using namespace std;
using namespace cv;
using namespace Eigen;

const int nDelayTimes = 2;
std::shared_ptr<System> pSystem;

void PubImuData()
{
//    string sImu_data_file = DATA_PATH + "MH_05_imu0.txt";
    string sImu_data_file = DATA_PATH + "MH_02_imu0.txt";
    if (SIMULATE_OPEN) {
        sImu_data_file = DATA_PATH + "imu_pose.txt";
    }
	cout << "1 PubImuData start sImu_data_filea: " << sImu_data_file << endl;

	ifstream fsImu;
	fsImu.open(sImu_data_file.c_str());
	if (!fsImu.is_open())
	{
		cerr << "Failed to open imu file! " << sImu_data_file << endl;
		return;
	}

	std::string sImu_line;
	double dStampNSec = 0.0;
	Vector3d vAcc;
	Vector3d vGyr;
    Quaterniond q;
    Vector3d t;
	while (std::getline(fsImu, sImu_line) && !sImu_line.empty()) // read imu data
	{
		std::istringstream ssImuData(sImu_line);
        if (SIMULATE_OPEN) {
            ssImuData >> dStampNSec
                      >> q.w() >> q.x() >> q.y() >> q.z() >> t(0) >> t(1) >> t(2)
                      >> vGyr.x() >> vGyr.y() >> vGyr.z() >> vAcc.x() >> vAcc.y() >> vAcc.z();
            // cout << "Imu t: " << fixed << dStampNSec << " gyr: " << vGyr.transpose() << " acc: " << vAcc.transpose() << endl;
            pSystem->PubImuData(dStampNSec, vGyr, vAcc);
        } else {
            ssImuData >> dStampNSec >> vGyr.x() >> vGyr.y() >> vGyr.z() >> vAcc.x() >> vAcc.y() >> vAcc.z();
            // cout << "Imu t: " << fixed << dStampNSec << " gyr: " << vGyr.transpose() << " acc: " << vAcc.transpose() << endl;
            pSystem->PubImuData(dStampNSec / 1e9, vGyr, vAcc);
        }
		usleep(5000*nDelayTimes);
	}
	fsImu.close();
}

void PubImageData()
{
    if (SIMULATE_OPEN) {
        string sImage_file = DATA_PATH + "cam_pose.txt";
        cout << "1 PubImageData start sImage_file: " << sImage_file << endl;

        ifstream fsImage;
        fsImage.open(sImage_file.c_str());
        if (!fsImage.is_open())
        {
            cerr << "Failed to open image file! " << sImage_file << endl;
            return;
        }

        std::string sImage_line;
        double dStampSec;
        std::vector<double> vdStampSec;
        while (std::getline(fsImage, sImage_line) && !sImage_line.empty())
        {
            std::istringstream ssImgData(sImage_line);
            ssImgData >> dStampSec;
            vdStampSec.push_back(dStampSec);
        }
        fsImage.close();

        for(int i=0; i<vdStampSec.size(); i++) {
            dStampSec = vdStampSec[i];

            std::stringstream filename1;
            filename1 << DATA_PATH << "keyframe/all_points_" << i << ".txt";

            Mat img;
            pSystem->PubImageData(dStampSec, img, filename1.str());

            usleep(50000*nDelayTimes);
        }
    } else {
//        string sImage_file = DATA_PATH + "MH_05_cam0.txt";
        string sImage_file = DATA_PATH + "MH_02_cam0.txt";
        cout << "1 PubImageData start sImage_file: " << sImage_file << endl;

        ifstream fsImage;
        fsImage.open(sImage_file.c_str());
        if (!fsImage.is_open())
        {
            cerr << "Failed to open image file! " << sImage_file << endl;
            return;
        }

        std::string sImage_line;
        double dStampNSec;
        string sImgFileName;

        // cv::namedWindow("SOURCE IMAGE", CV_WINDOW_AUTOSIZE);
        while (std::getline(fsImage, sImage_line) && !sImage_line.empty())
        {
            std::istringstream ssImuData(sImage_line);
            ssImuData >> dStampNSec >> sImgFileName;
            // cout << "Image t : " << fixed << dStampNSec << " Name: " << sImgFileName << endl;
            string imagePath = DATA_PATH + "cam0/data/" + sImgFileName;

            Mat img = imread(imagePath.c_str(), 0);
            if (img.empty())
            {
                cerr << "image is empty! path: " << imagePath << endl;
                return;
            }
            pSystem->PubImageData(dStampNSec / 1e9, img, "");
            // cv::imshow("SOURCE IMAGE", img);
            // cv::waitKey(0);
            usleep(50000*nDelayTimes);
        }
        fsImage.close();
    }
}

#ifdef __APPLE__
// support for MacOS
void DrawIMGandGLinMainThrd(){
//    string sImage_file = DATA_PATH + "MH_05_cam0.txt";
    string sImage_file = DATA_PATH + "MH_02_cam0.txt";

	cout << "1 PubImageData start sImage_file: " << sImage_file << endl;
    if (SIMULATE_OPEN) {
        pSystem->InitDrawGL();
        while(1) {
            pSystem->DrawGLFrame();
            usleep(50000*nDelayTimes);
        }
        return;
    }

	ifstream fsImage;
	fsImage.open(sImage_file.c_str());
	if (!fsImage.is_open())
	{
		cerr << "Failed to open image file! " << sImage_file << endl;
		return;
	}

	std::string sImage_line;
	double dStampNSec;
	string sImgFileName;

	pSystem->InitDrawGL();
	while (std::getline(fsImage, sImage_line) && !sImage_line.empty())
	{
		std::istringstream ssImuData(sImage_line);
		ssImuData >> dStampNSec >> sImgFileName;
		// cout << "Image t : " << fixed << dStampNSec << " Name: " << sImgFileName << endl;
		string imagePath = DATA_PATH + "cam0/data/" + sImgFileName;

		Mat img = imread(imagePath.c_str(), 0);
		if (img.empty())
		{
			cerr << "image is empty! path: " << imagePath << endl;
			return;
		}
		//pSystem->PubImageData(dStampNSec / 1e9, img);
		cv::Mat show_img;
		cv::cvtColor(img, show_img, CV_GRAY2RGB);
		if (SHOW_TRACK)
		{
			for (unsigned int j = 0; j < pSystem->trackerData[0].cur_pts.size(); j++)
			{
				double len = min(1.0, 1.0 *  pSystem->trackerData[0].track_cnt[j] / WINDOW_SIZE);
				cv::circle(show_img,  pSystem->trackerData[0].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
			}

			cv::namedWindow("IMAGE", CV_WINDOW_AUTOSIZE);
			cv::imshow("IMAGE", show_img);
		  // cv::waitKey(1);
		}

		pSystem->DrawGLFrame();
		usleep(50000*nDelayTimes);
	}
	fsImage.close();

} 
#endif

int main(int argc, char **argv)
{
	if(argc != 2)
	{
		cerr << "./run_euroc PATH_TO_CONFIG/config_file \n"
			<< "For example: ./run_euroc ../config/euroc_config.yaml"<< endl;
		return -1;
	}

	pSystem.reset(new System(argv[1]));
	std::thread thd_BackEnd(&System::ProcessBackEnd, pSystem);//后端优化最重要的线程

	// sleep(5);
    std::thread thd_PubImuData(PubImuData);//获取IMU数据线程
    std::thread thd_PubImageData(PubImageData);//获取图像数据线程
#ifdef __linux__
    std::thread thd_Draw(&System::Draw, pSystem);//显示轨迹线程
#elif __APPLE__
    DrawIMGandGLinMainThrd();
#endif

    thd_PubImuData.join();
    thd_PubImageData.join();

    // thd_BackEnd.join();
#ifdef __linux__
    thd_Draw.join();
#endif

	cout << "main end... see you ..." << endl;
	return 0;
}
