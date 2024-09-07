/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>
#include<curl/curl.h>

#include<System.h>

using namespace std;
using namespace cv;

const int cell_size = 800;
atomic<unsigned long long> atomic_cnts[2][cell_size][cell_size]; // 0:visited, 1:occupied
bool flag = 1;
Sophus::SE3f now;
bool check_boundary(int r, int c);
void bresenham(int r1, int c1, int r2, int c2);
void drawOccupancyMap(Mat &canvas);
void occupancy_grid(ORB_SLAM3::System &SLAM);
void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

// libcurl 콜백 함수: 데이터를 메모리에 저장
size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp) {
    std::vector<uchar>* buffer = static_cast<std::vector<uchar>*>(userp);
    size_t totalSize = size * nmemb;
    buffer->insert(buffer->end(), (uchar*)contents, (uchar*)contents + totalSize);
    return totalSize;
}

// HTTP 요청으로 이미지를 받아 cv::Mat으로 변환하는 함수
cv::Mat downloadImage(const std::string& url) {
    CURL* curl;
    CURLcode res;
    std::vector<uchar> buffer;  // 이미지 데이터를 담을 버퍼

    curl = curl_easy_init();
    if(curl) {
        // URL 설정
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &buffer);

        // HTTP 요청 보내기
        res = curl_easy_perform(curl);
        if(res != CURLE_OK) {
            std::cerr << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
            curl_easy_cleanup(curl);
            return cv::Mat();
        }

        // curl 세션 종료
        curl_easy_cleanup(curl);

        // 받은 데이터를 OpenCV의 cv::Mat 형식으로 변환
        cv::Mat img = cv::imdecode(buffer, cv::IMREAD_COLOR);
        if(img.empty()) {
            std::cerr << "Failed to decode image" << std::endl;
        }
        return img;
    }

    return cv::Mat();  // 실패 시 빈 Mat 반환
}

int main(int argc, char **argv)
{  
    if(argc != 4)
    {
        cerr << endl << "Usage: ./slam_http path_to_vocabulary path_to_settings server_addr" << endl;
        return 1;
    }

    string url = string("http://") + argv[3] + "/capture";
    cout << "URL: " << url << endl;

    cout << endl << "-------" << endl;
    cout.precision(17);
    
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR, true);
    float imageScale = SLAM.GetImageScale();

    double t_resize = 0.f;
    double t_track = 0.f;
    int tframe = 1;

    thread thPoints(occupancy_grid, ref(SLAM));

    for (;;) {

        // Main loop
        cv::Mat im;

        // Read image
        im = downloadImage(url);

        if(im.empty()) {
            cerr << "Image download or decode failed." << endl;
        }

        if(imageScale != 1.f)
        {
#ifdef REGISTER_TIMES
#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
#else
            std::chrono::monotonic_clock::time_point t_Start_Resize = std::chrono::monotonic_clock::now();
#endif
#endif
            int width = im.cols * imageScale;
            int height = im.rows * imageScale;
            cv::resize(im, im, cv::Size(width, height));
#ifdef REGISTER_TIMES
#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
#else
            std::chrono::monotonic_clock::time_point t_End_Resize = std::chrono::monotonic_clock::now();
#endif
            t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
            SLAM.InsertResizeTime(t_resize);
#endif
        }

        /*
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif
        */

        // Pass the image to the SLAM system
        // cout << "tframe = " << tframe << endl;
        const auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        double stamp = std::chrono::duration<double>(duration).count()*1e-3;
        SLAM.TrackMonocular(im, stamp); // TODO change to monocular_inertial
        tframe++;

        /*
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

#ifdef REGISTER_TIMES
        t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
        SLAM.InsertTrackTime(t_track);
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        */
        
        /* 네트워크 딜레이로 인한 자동 프레임 드랍이 있기에 건너뜀
        // Wait to load the next frame
        double T=0;
        if(ni<nImages[seq]-1)
            T = vTimestampsCam[seq][ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestampsCam[seq][ni-1];

        //std::cout << "T: " << T << std::endl;
        //std::cout << "ttrack: " << ttrack << std::endl;

        if(ttrack<T) {
            //std::cout << "usleep: " << (dT-ttrack) << std::endl;
            usleep((T-ttrack)*1e6); // 1e6
        }
        */
    }
    // Stop all threads
    thPoints.join();
    SLAM.Shutdown();

    SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");

    return 0;
}

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t*1e-9);

        }
    }
}

bool check_boundary(int r, int c)
{
    if (r < 0 || c < 0 || r >= cell_size || c >= cell_size)
        return false;
    return true;
}

void bresenham(int r1, int c1, int r2, int c2)
{
    if (check_boundary(r2, c2))
        atomic_cnts[1][r2][c2].fetch_add(1);
    if (c1 == c2)
    {
        if (r1 > r2)
            swap(r1, r2);

        while (r1 <= r2)
        {
            if (!check_boundary(r1, c1))
                break;
            atomic_cnts[0][r1][c1].fetch_add(1);
            r1++;
        }
    }
    else
    {
        if (c1 > c2)
        {
            swap(c1, c2);
            swap(r1, r2);
        }
        if (r1 == r2)
        {
            while (c1 <= c2)
            {
                if (!check_boundary(r1, c1))
                    break;
                atomic_cnts[0][r1][c1].fetch_add(1);
                c1++;
            }
        }
        else
        {
            if (r1 > r2)
            {
                r2 = r1 + (r1 - r2);

                int dr = r2 - r1;
                int dc = c2 - c1;

                if (dr <= dc)
                {
                    const int r0 = r1;
                    int p = 2 * dr - dc;
                    while (c1 <= c2)
                    {
                        if (!check_boundary(r0 - (r1 - r0), c1))
                            break;
                        atomic_cnts[0][r0 - (r1 - r0)][c1].fetch_add(1);
                        c1++;
                        if (p < 0)
                            p = p + 2 * dr;
                        else
                        {
                            p = p + 2 * dr - 2 * dc;
                            r1++;
                        }
                    }
                }
                else
                {
                    swap(dr, dc);
                    swap(c1, r1);
                    swap(c2, r2);
                    int p = 2 * dr - dc;
                    const int c0 = c1;
                    while (c1 <= c2)
                    {
                        if (!check_boundary(c0 - (c1 - c0), r1))
                            break;
                        atomic_cnts[0][c0 - (c1 - c0)][r1].fetch_add(1);
                        c1++;
                        if (p < 0)
                            p = p + 2 * dr;
                        else
                        {
                            p = p + 2 * dr - 2 * dc;
                            r1++;
                        }
                    }
                }
            }
            else
            {
                int dr = r2 - r1;
                int dc = c2 - c1;

                if (dc >= dr)
                {
                    int p = 2 * dr - dc;
                    while (c1 <= c2)
                    {
                        if (!check_boundary(r1, c1))
                            break;
                        atomic_cnts[0][r1][c1].fetch_add(1);
                        c1++;
                        if (p < 0)
                            p = p + 2 * dr;
                        else
                        {
                            p = p + 2 * dr - 2 * dc;
                            r1++;
                        }
                    }
                }
                else
                {
                    swap(dr, dc);
                    swap(c1, r1);
                    swap(c2, r2);
                    int p = 2 * dr - dc;
                    while (c1 <= c2)
                    {
                        if (!check_boundary(c1, r1))
                            break;
                        atomic_cnts[0][c1][r1].fetch_add(1);
                        c1++;
                        if (p < 0)
                            p = p + 2 * dr;
                        else
                        {
                            p = p + 2 * dr - 2 * dc;
                            r1++;
                        }
                    }
                }
            }
        }
    }
}

void drawOccupancyMap(Mat &canvas)
{

#pragma omp parallel for schedule(dynamic, 1) collapse(4)
    for (int i = 0; i < cell_size; ++i)
    {
        for (int j = 0; j < cell_size; ++j)
        {
            int visit_cnt = 0;
            int occupy_cnt = 0;
            for (int dr = -1; dr <= 1; ++dr)
            {
                for (int dc = -1; dc <= 1; ++dc)
                {
                    if (!check_boundary(i + dr, j + dc))
                        continue;
                    visit_cnt += atomic_cnts[0][i + dr][j + dc];
                    occupy_cnt += atomic_cnts[1][i + dr][j + dc];
                }
            }
            if (visit_cnt < 5)
                continue;

            const int percent = (occupy_cnt * 100) / visit_cnt;
            if (percent >= 15)
            {
                circle(canvas, Point(j, i), 0, Scalar(0, 0, 0), 3);
            }
            else
            {
                circle(canvas, Point(j, i), 0, Scalar(255, 255, 255));
            }
        }
    }
}

void occupancy_grid(ORB_SLAM3::System &SLAM)
{
    Mat canvas(cell_size, cell_size, CV_8UC3, cv::Scalar(120, 120, 120)); // Creating a blank canvas
    const float res = 0.01;                                               // 0.01 m/cell

    while (flag)
    {
#pragma omp parallel for schedule(dynamic, 1) collapse(2)
        for (int i = 0; i < cell_size; ++i)
        {
            for (int j = 0; j < cell_size; ++j)
            {
                atomic_cnts[0][i][j].store(0);
                atomic_cnts[1][i][j].store(0);
            }
        }

        canvas.setTo(cv::Scalar(120, 120, 120));

        const auto mps = SLAM.mpAtlas->GetAllMapPoints();
        const int mps_len = mps.size();

#pragma omp parallel for schedule(dynamic, 1)
        for (int i = 0; i < mps_len; ++i)
        {
            const auto p = mps[i]->GetWorldPos();
            const int c = cell_size / 2 + (int)(p(0) / res); // x
            const int r = cell_size / 2 - (int)(p(2) / res); // y
            const int z = (int)(p(1) / res); // z

            const auto p0 = mps[i]->GetReferenceKeyFrame()->GetPose().inverse().translation();
            const int c0 = cell_size / 2 + (int)(p0(0) / res); // x
            const int r0 = cell_size / 2 - (int)(p0(2) / res); // y
            const int z0 = (int)(p0(1) / res); // z

            // cut height above 1 meter
            if(abs(z - z0) > (int)(1.f / res))
                continue;

            bresenham(r0, c0, r, c);
        }

        drawOccupancyMap(canvas);

        const Sophus::Vector3f trans = now.translation();
        const Sophus::Vector3f dir = now.rotationMatrix().col(2);
        const int c = cell_size / 2 + (int)(trans(0) / res); // x
        const int r = cell_size / 2 - (int)(trans(2) / res); // y
        const int ratio[2] = {(int)(dir(0) / res), -(int)(dir(2) / res)}; // y
        const int dc = (20*ratio[0]) / (abs(ratio[0]) + abs(ratio[1]));
        const int dr = (20*ratio[1]) / (abs(ratio[0]) + abs(ratio[1]));
        if (check_boundary(r, c))
        {
            circle(canvas, Point(c, r), 0, Scalar(0, 0, 255), 10);
            arrowedLine(canvas, Point(c,r), Point(c + dc, r + dr), Scalar(0,0,255),2, LINE_8, 0, 0.5);
        }

        imshow("Canvas", canvas);
        waitKey(1);

        usleep(100 * 1000);
    }
}
