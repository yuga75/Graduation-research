#include <iostream>
#include <iomanip>
#include <cmath>
#include <Kinect.h>
#include <NtKinect.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <conio.h>
#include <thread>
#include <chrono>
#include <Windows.h>

using namespace std;

// 定数の定義
#define M_PI 3.141592653589793238462643383279502884L

float rotateCenterX, rotateCenterY, rotateCenterZ;

int SoundCount = 0;
double TotalCount = 0.0;
double GoodCount = 0.0;
double BadCount = 0.0;
double Score = 0.0;
bool CountSwitch = false;

int SlideTextX = 50;

int SlideHumanX = 100;
int SlideHumanY = 200;

int MaxDegree = 80;
int MinDegree = 20;

int FirstMaxDegree = 0;
int NextMaxDegree = 0;
int NextUserDegree = 0;
double descentRate = 0.0;

int CountMemory = 0;

// 3D座標系での回転
void rotateCoordinates3D(float& x, float& y, float& z, float degreesX, float degreesY, float degreesZ) {

    // 度数法からラジアンに変換
    float radiansX = degreesX * M_PI / 180.0;
    float radiansY = degreesY * M_PI / 180.0;
    float radiansZ = degreesZ * M_PI / 180.0;

    float tempX, tempY, tempZ;

    x -= 2.0;
    //y -= 0.5;
    z -= 3.0;

    // X軸周りの回転
    tempY = y * cos(radiansX) - z * sin(radiansX);
    tempZ = y * sin(radiansX) + z * cos(radiansX);
    y = tempY;
    z = tempZ;

    // Y軸周りの回転
    tempX = x * cos(radiansY) + z * sin(radiansY);
    tempZ = -x * sin(radiansY) + z * cos(radiansY);
    x = tempX;
    z = tempZ;

    // Z軸周りの回転
    //tempX = x * cos(radiansZ) - y * sin(radiansZ);
    //tempY = x * sin(radiansZ) + y * cos(radiansZ);
    //x = tempX;
    //y = tempY;
}

void metronome(bool& MetronomeSwitch){
    printf("%d\n", MetronomeSwitch);

    while (true) {
        // メトロノームの周期を設定（例: 1秒ごとに）
        std::this_thread::sleep_for(std::chrono::seconds(1));

        SoundCount += 1;

        LPCWSTR metronome1 = L"wav/metronome1.wav";
        LPCWSTR metronome2 = L"wav/metronome2.wav";

        if (SoundCount % 4 == 0) {
            PlaySound(metronome2, NULL, SND_ASYNC | SND_NODEFAULT);
            printf("鳴った\n");
        }
        else {
            PlaySound(metronome1, NULL, SND_ASYNC | SND_NODEFAULT);
            printf("鳴った２\n");

        }
        printf("%d\n\n", MetronomeSwitch);
        if (MetronomeSwitch == false) { // フラグがfalseに設定された場合はループを抜ける
            SoundCount = 0;
            break;
        }
    }
}

void DegreeManager(int UpperBody, bool MetronomeSwitchDegreeManager) {
    if (MetronomeSwitchDegreeManager == true) {
        if (SoundCount % 4 == 0
            && UpperBody >= 45) {
            CountMemory = SoundCount;
            NextUserDegree = UpperBody;
        }
        
        if (SoundCount == CountMemory + 1
            && FirstMaxDegree == 0) {
            FirstMaxDegree = NextUserDegree;
            NextUserDegree == 0;
        }
        else if (SoundCount == CountMemory + 1
            && FirstMaxDegree != 0) {
            NextMaxDegree = NextUserDegree;
            int AngleChange = FirstMaxDegree - NextMaxDegree;
            //printf("最初の角度:%d\n", FirstMaxDegree);
            //printf("現在の角度:%d\n", NextMaxDegree);
            descentRate = (double)(AngleChange * (-100) / FirstMaxDegree);
            //printf("変化率:%lf\n\n", descentRate);
            NextUserDegree == 0;
        }
    }
}

// Kinectデータを表示および処理する関数
void doJob(bool MetronomeSwitchEnter) {
    NtKinect kinect;

    // 関節座標を格納する変数
    float
        HeadX, HeadY, HeadZ,
        ShoulderX, ShoulderY, ShoulderZ,
        SpineShoulderX, SpineShoulderY, SpineShoulderZ,
        SpineBaseX, SpineBaseY, SpineBaseZ,
        HipRightX, HipRightY, HipRightZ,
        HipLeftX, HipLeftY, HipLeftZ,
        KneeRightX, KneeRightY, KneeRightZ,
        KneeLeftX, KneeLeftY, KneeLeftZ,
        AnkleRightX, AnkleRightY, AnkleRightZ,
        AnkleLeftX, AnkleLeftY, AnkleLeftZ;

    // 関節タイプ
    JointType
        JointHead = JointType_Head,
        JointSpineShoulder = JointType_SpineShoulder,
        JointSpineBase = JointType_SpineBase,
        JointKneeRight = JointType_KneeRight,
        JointKneeLeft = JointType_KneeLeft,
        JointAnkleRight = JointType_AnkleRight,
        JointAnkleLeft = JointType_AnkleLeft;

    bool bodyInAngleRange = false;
    int counter = 0;
    int radius = 50;

    int ellipseX = 15;
    int ellipseY = 15;
    cv::Mat frame; // 空のMatオブジェクトを作成

    // warningスレッドを管理するフラグ
    std::thread warning_thread;

    while (1) {
        kinect.setRGB();
        kinect.setSkeleton();
        frame = kinect.rgbImage.clone();

        // Enterキーが押されたかどうかを検出
        if (GetAsyncKeyState(VK_RETURN) & 0x8000) {
            // Enterキーが押された時の処理
            MetronomeSwitchEnter = !MetronomeSwitchEnter;

            Sleep(100); // キーが連続して押されるのを防ぐための短い待機

            if (MetronomeSwitchEnter == true) {
                // メトロノームを新しいスレッドで開始
                std::thread metronome_thread(metronome,std::ref(MetronomeSwitchEnter));
                metronome_thread.detach();
            }
            else {

            }
        }

        // Rキーが押されたかどうかを検出
        if (GetAsyncKeyState('R') & 0x8000
            && MetronomeSwitchEnter == false) {
            // Rキーが押された時の処理
            Score = 0.0;
            TotalCount = 0.0;
            GoodCount = 0.0;
            BadCount = 0.0;

            FirstMaxDegree = 0;
            NextMaxDegree = 0;
            NextUserDegree = 0;
            descentRate = 0.0;
        }

        // 矩形の領域を暗くする
        cv::Rect Rect(0, 0, 1920, 1080); // 例として (x座標, y座標, 幅, 高さ) を指定します
        cv::rectangle(kinect.rgbImage, Rect, cv::Scalar(0, 0, 0), -1); // 長方形を描画

        // スコアを表示
        std::stringstream TextScore;
        TextScore << "Score: " << Score;
        cv::putText(kinect.rgbImage, TextScore.str(), cv::Point(SlideTextX, 100), cv::FONT_HERSHEY_SIMPLEX, 2.0, cv::Scalar(255, 255, 255), 2);

        // 腹筋の回数を表示
        std::stringstream TextTotalCount;
        TextTotalCount << std::fixed << std::setprecision(1);
        TextTotalCount << "Total: " << TotalCount;
        cv::putText(kinect.rgbImage, TextTotalCount.str(), cv::Point(SlideTextX, 200), cv::FONT_HERSHEY_SIMPLEX, 2.0, cv::Scalar(255, 255, 255), 2);

        std::stringstream TextGoodCount;
        TextGoodCount << std::fixed << std::setprecision(1);
        TextGoodCount << "Good: " << GoodCount;
        cv::putText(kinect.rgbImage, TextGoodCount.str(), cv::Point(SlideTextX, 250), cv::FONT_HERSHEY_SIMPLEX, 2.0, cv::Scalar(255, 255, 255), 2);

        std::stringstream TextBadCount;
        TextBadCount << std::fixed << std::setprecision(1);
        TextBadCount << "Bad: " << BadCount;
        cv::putText(kinect.rgbImage, TextBadCount.str(), cv::Point(SlideTextX, 300), cv::FONT_HERSHEY_SIMPLEX, 2.0, cv::Scalar(255, 255, 255), 2);

        // 角度の表示
        std::stringstream TextFirstDegree;
        TextFirstDegree << "First: " << FirstMaxDegree;
        cv::putText(kinect.rgbImage, TextFirstDegree.str(), cv::Point(SlideTextX, 400), cv::FONT_HERSHEY_SIMPLEX, 2.0, cv::Scalar(255, 255, 255), 2);

        std::stringstream TextCurrentDegree;
        TextCurrentDegree << "Now: " << NextMaxDegree;
        cv::putText(kinect.rgbImage, TextCurrentDegree.str(), cv::Point(SlideTextX, 450), cv::FONT_HERSHEY_SIMPLEX, 2.0, cv::Scalar(255, 255, 255), 2);

        std::stringstream TextDescentRate;
        TextDescentRate << "Rate: " << descentRate << "%";
        cv::putText(kinect.rgbImage, TextDescentRate.str(), cv::Point(SlideTextX, 500), cv::FONT_HERSHEY_SIMPLEX, 2.0, cv::Scalar(255, 255, 255), 2);

        // テキスト表示
        if (MetronomeSwitchEnter == true) {
            std::stringstream TextCheer;
            if (descentRate * 100 > 30) {
                TextCheer << "Hang in there!";
                cv::putText(kinect.rgbImage, TextCheer.str(), cv::Point(500, 200), cv::FONT_HERSHEY_SIMPLEX, 2.0, cv::Scalar(0, 0, 255), 2);
            }
            else {
                TextCheer << "Keep it up!";
                cv::putText(kinect.rgbImage, TextCheer.str(), cv::Point(500, 200), cv::FONT_HERSHEY_SIMPLEX, 2.0, cv::Scalar(0, 255, 255), 2);
            }
        }

        for (auto person : kinect.skeleton) {
            for (auto joint : person) {
                if (joint.TrackingState == TrackingState_Tracked) {
                    // 関節の絞り込み（頭、肩、背骨、尻）
                    if (joint.JointType == JointType_Head
                        || joint.JointType == JointType_SpineShoulder
                        || joint.JointType == JointType_SpineBase
                        || joint.JointType == JointType_KneeRight
                        || joint.JointType == JointType_KneeLeft
                        || joint.JointType == JointType_AnkleRight
                        || joint.JointType == JointType_AnkleLeft) {


                        // 関節座標の取得
                        HeadX = person[JointHead].Position.X;
                        HeadY = person[JointHead].Position.Y;
                        HeadZ = person[JointHead].Position.Z;

                        SpineShoulderX = person[JointSpineShoulder].Position.X;
                        SpineShoulderY = person[JointSpineShoulder].Position.Y;
                        SpineShoulderZ = person[JointSpineShoulder].Position.Z;

                        SpineBaseX = person[JointSpineBase].Position.X;
                        SpineBaseY = person[JointSpineBase].Position.Y;
                        SpineBaseZ = person[JointSpineBase].Position.Z;

                        KneeRightX = person[JointKneeRight].Position.X;
                        KneeRightY = person[JointKneeRight].Position.Y;
                        KneeRightZ = person[JointKneeRight].Position.Z;

                        KneeLeftX = person[JointKneeLeft].Position.X;
                        KneeLeftY = person[JointKneeLeft].Position.Y;
                        KneeLeftZ = person[JointKneeLeft].Position.Z;

                        AnkleRightX = person[JointAnkleRight].Position.X;
                        AnkleRightY = person[JointAnkleRight].Position.Y;
                        AnkleRightZ = person[JointAnkleRight].Position.Z;

                        AnkleLeftX = person[JointAnkleLeft].Position.X;
                        AnkleLeftY = person[JointAnkleLeft].Position.Y;
                        AnkleLeftZ = person[JointAnkleLeft].Position.Z;

                        rotateCenterX = SpineBaseX;
                        rotateCenterY = SpineBaseY;
                        rotateCenterZ = SpineBaseZ;

                        // 座標変換
                        rotateCoordinates3D(HeadX, HeadY, HeadZ, 50.0, 90.0, 0.0);
                        rotateCoordinates3D(SpineShoulderX, SpineShoulderY, SpineShoulderZ, 50.0, 90.0, 0.0);
                        rotateCoordinates3D(SpineBaseX, SpineBaseY, SpineBaseZ, 50.0, 90.0, 0.0);
                        rotateCoordinates3D(HipRightX, HipRightY, HipRightZ, 50.0, 90.0, 0.0);
                        rotateCoordinates3D(HipLeftX, HipLeftY, HipLeftZ, 50.0, 90.0, 0.0);
                        rotateCoordinates3D(KneeRightX, KneeRightY, KneeRightZ, 50.0, 90.0, 0.0);
                        rotateCoordinates3D(KneeLeftX, KneeLeftY, KneeLeftZ, 50.0, 90.0, 0.0);
                        rotateCoordinates3D(AnkleRightX, AnkleRightY, AnkleRightZ, 50.0, 90.0, 0.0);
                        rotateCoordinates3D(AnkleLeftX, AnkleLeftY, AnkleLeftZ, 50.0, 90.0, 0.0);

                        // 関節座標をカラースペースにマッピング
                        ColorSpacePoint cp;
                        kinect.coordinateMapper->MapCameraPointToColorSpace(joint.Position, &cp);
                        ColorSpacePoint CpHead;
                        kinect.coordinateMapper->MapCameraPointToColorSpace({ HeadX, HeadY, HeadZ}, & CpHead);
                        ColorSpacePoint CpSpineShoulder;
                        kinect.coordinateMapper->MapCameraPointToColorSpace({ SpineShoulderX, SpineShoulderY , SpineShoulderZ }, & CpSpineShoulder);
                        ColorSpacePoint CpSpineBase;
                        kinect.coordinateMapper->MapCameraPointToColorSpace({ SpineBaseX, SpineBaseY , SpineBaseZ}, & CpSpineBase);
                        ColorSpacePoint CpHipRight;
                        kinect.coordinateMapper->MapCameraPointToColorSpace({ HipRightX, HipRightY, HipRightZ }, & CpHipRight);
                        ColorSpacePoint CpHipLeft;
                        kinect.coordinateMapper->MapCameraPointToColorSpace({ HipLeftX, HipLeftY, HipLeftZ }, & CpHipLeft);
                        ColorSpacePoint CpKneeRight;
                        kinect.coordinateMapper->MapCameraPointToColorSpace({ KneeRightX, KneeRightY, KneeRightZ }, &CpKneeRight);
                        ColorSpacePoint CpKneeLeft;
                        kinect.coordinateMapper->MapCameraPointToColorSpace({ KneeLeftX, KneeLeftY, KneeLeftZ }, &CpKneeLeft);
                        ColorSpacePoint CpAnkleRight;
                        kinect.coordinateMapper->MapCameraPointToColorSpace({ AnkleRightX, AnkleRightY, AnkleRightZ }, &CpAnkleRight);
                        ColorSpacePoint CpAnkleLeft;
                        kinect.coordinateMapper->MapCameraPointToColorSpace({ AnkleLeftX, AnkleLeftY, AnkleLeftZ }, &CpAnkleLeft);

                        // カメラからの適切な距離に関節があるか確認
                        if ((person[JointHead].Position.Z >= 0)
                            && (person[JointSpineBase].Position.Z >= 0)) {
                            // フレームの枠を緑に設定
                            cv::rectangle(kinect.rgbImage, cv::Rect(0, 0, kinect.rgbImage.cols, kinect.rgbImage.rows), cv::Scalar(0, 255, 0), 50);

                            // 肩から尻への角度を計算
                            float AngleShouderToSpineBase = std::atan2(SpineShoulderY - SpineBaseY, SpineShoulderX - SpineBaseX) * 180.0 / M_PI;
                            // 肩と尻のZ座標からなる角度を計算
                            float VecZ1 = SpineShoulderZ - SpineBaseZ;
                            float VecX1 = SpineShoulderX - SpineBaseX;
                            float AngleZ1 = atan2(VecZ1, VecX1) * 180.0 / M_PI;
                            // 新しく計算したZ座標からの角度を角度を考慮した上で更新
                            if (AngleShouderToSpineBase > 90) {
                                AngleShouderToSpineBase = 180 - AngleShouderToSpineBase;
                                AngleZ1 = -AngleZ1;
                            }
                            int IntAngleShouderToSpineBase = (int)AngleShouderToSpineBase;
                            DegreeManager(IntAngleShouderToSpineBase, MetronomeSwitchEnter);

                            // 尻の近くに角度を表示
                            std::stringstream angleTextSpineBase;
                            angleTextSpineBase << IntAngleShouderToSpineBase;
                            cv::putText(kinect.rgbImage, angleTextSpineBase.str(), cv::Point((int)CpSpineBase.X- SlideHumanX, (int)CpSpineBase.Y+SlideHumanY), cv::FONT_HERSHEY_SIMPLEX, 2.0, cv::Scalar(255, 0, 255), 2);
                            

                            // 肩から頭へのベクトルと肩から尻へのベクトルのなす角を計算
                            cv::Point3f Vec1(HeadX - SpineShoulderX, HeadY - SpineShoulderY, HeadZ - SpineShoulderZ);
                            cv::Point3f Vec2(SpineBaseX - SpineShoulderX, SpineBaseY - SpineShoulderY, SpineBaseZ - SpineShoulderZ);

                            float DotProduct = Vec1.x * Vec2.x + Vec1.y * Vec2.y + Vec1.z * Vec2.z;
                            float MagnitudeVec1 = sqrt(Vec1.x * Vec1.x + Vec1.y * Vec1.y + Vec1.z * Vec1.z);
                            float MagnitudeVec2 = sqrt(Vec2.x * Vec2.x + Vec2.y * Vec2.y + Vec2.z * Vec2.z);

                            float CosineValue = DotProduct / (MagnitudeVec1 * MagnitudeVec2);
                            float AngleInRadians = acos(CosineValue);

                            float AngleInDegrees = AngleInRadians * (180.0 / M_PI);
                            int AngleUpperBody = (int)AngleInDegrees;
                            

                            // 肩の近くに角度を表示
                            std::stringstream angleText;
                            angleText << AngleUpperBody;
                            cv::putText(kinect.rgbImage, angleText.str(), cv::Point((int)CpSpineShoulder.X-250, (int)CpSpineShoulder.Y+SlideHumanY), cv::FONT_HERSHEY_SIMPLEX, 2.0, cv::Scalar(255, 0, 255), 2);
                            
                            // 身体が適切な位置にある場合の処理
                            if ((AngleUpperBody >= 165 && AngleUpperBody <= 180)
                                && (AngleShouderToSpineBase > MinDegree && AngleShouderToSpineBase < MaxDegree)) {

                                // 関節にマーカーを配置
                                cv::circle(kinect.rgbImage, cv::Point((int)CpHead.X-SlideHumanX, (int)CpHead.Y+ SlideHumanY), 15, cv::Scalar(0, 255, 0), -1);
                                cv::circle(kinect.rgbImage, cv::Point((int)CpSpineShoulder.X-SlideHumanX, (int)CpSpineShoulder.Y+ SlideHumanY), 15, cv::Scalar(0, 255, 0), -1);
                                cv::circle(kinect.rgbImage, cv::Point((int)CpSpineBase.X-SlideHumanX, (int)CpSpineBase.Y+ SlideHumanY), 15, cv::Scalar(0, 255, 0), -1);
                                cv::circle(kinect.rgbImage, cv::Point((int)CpKneeRight.X-SlideHumanX, (int)CpKneeRight.Y+ SlideHumanY), 15, cv::Scalar(0, 255, 0), -1);
                                cv::circle(kinect.rgbImage, cv::Point((int)CpKneeLeft.X-SlideHumanX, (int)CpKneeLeft.Y+ SlideHumanY), 15, cv::Scalar(0, 255, 0), -1);
                                cv::circle(kinect.rgbImage, cv::Point((int)CpAnkleRight.X-SlideHumanX, (int)CpAnkleRight.Y+ SlideHumanY), 15, cv::Scalar(0, 255, 0), -1);
                                cv::circle(kinect.rgbImage, cv::Point((int)CpAnkleLeft.X-SlideHumanX, (int)CpAnkleLeft.Y+ SlideHumanY), 15, cv::Scalar(0, 255, 0), -1);

                                // 関節を線で接続
                                cv::line(kinect.rgbImage, cv::Point((int)CpHead.X-SlideHumanX, (int)CpHead.Y+ SlideHumanY), cv::Point((int)CpSpineShoulder.X-SlideHumanX, (int)CpSpineShoulder.Y+ SlideHumanY), cv::Scalar(0, 255, 0), 10);
                                cv::line(kinect.rgbImage, cv::Point((int)CpSpineShoulder.X-SlideHumanX, (int)CpSpineShoulder.Y+ SlideHumanY), cv::Point((int)CpSpineBase.X-SlideHumanX, (int)CpSpineBase.Y+ SlideHumanY), cv::Scalar(0, 255, 0), 10);
                                cv::line(kinect.rgbImage, cv::Point((int)CpSpineBase.X-SlideHumanX, (int)CpSpineBase.Y+ SlideHumanY), cv::Point((int)CpKneeRight.X-SlideHumanX, (int)CpKneeRight.Y+ SlideHumanY), cv::Scalar(0, 255, 0), 10);
                                cv::line(kinect.rgbImage, cv::Point((int)CpSpineBase.X-SlideHumanX, (int)CpSpineBase.Y+ SlideHumanY), cv::Point((int)CpKneeLeft.X-SlideHumanX, (int)CpKneeLeft.Y+ SlideHumanY), cv::Scalar(0, 255, 0), 10);
                                cv::line(kinect.rgbImage, cv::Point((int)CpKneeRight.X-SlideHumanX, (int)CpKneeRight.Y+ SlideHumanY), cv::Point((int)CpAnkleRight.X-SlideHumanX, (int)CpAnkleRight.Y+ SlideHumanY), cv::Scalar(0, 255, 0), 10);
                                cv::line(kinect.rgbImage, cv::Point((int)CpKneeLeft.X-SlideHumanX, (int)CpKneeLeft.Y+ SlideHumanY), cv::Point((int)CpAnkleLeft.X-SlideHumanX, (int)CpAnkleLeft.Y+ SlideHumanY), cv::Scalar(0, 255, 0), 10);

                                if (90 - IntAngleShouderToSpineBase > 15) {
                                    ellipseX = (90 - IntAngleShouderToSpineBase) * 2;
                                    ellipseY = 90 - IntAngleShouderToSpineBase - 10;
                                }
                                
                                // 黒い背景の画像を生成
                                cv::Mat img = cv::Mat::zeros(300, 300, CV_8UC3);
                                // 回転する楕円を描画する回転矩形の情報を作成
                                cv::Point center(((int)CpSpineShoulder.X-SlideHumanX + (int)CpSpineBase.X- SlideHumanX) / 2, ((int)CpSpineShoulder.Y+ SlideHumanY + (int)CpSpineBase.Y+SlideHumanY) / 2);  // 中心座標
                                cv::Size axes(ellipseX, ellipseY);     // 長軸と短軸の長さ
                                double angle = -IntAngleShouderToSpineBase;         // 回転角度

                                // 回転する楕円を描画するための回転矩形（RotatedRect）を作成
                                cv::RotatedRect rotatedRect(center, axes, angle);

                                // 回転楕円を描画
                                cv::ellipse(kinect.rgbImage, rotatedRect, cv::Scalar(0, 0, 255), -1);

                                if (((SoundCount % 4 == 0 && IntAngleShouderToSpineBase >= 60)
                                    || (SoundCount % 4 == 0 && IntAngleShouderToSpineBase <= 40))
                                    && SoundCount != 0
                                    && !CountSwitch) {
                                    TotalCount += 0.5;
                                    GoodCount += 0.5;
                                    CountSwitch = true;
                                }
                                else if ((SoundCount % 4 != 0 && IntAngleShouderToSpineBase < 60)
                                    || (SoundCount % 4 != 0 && IntAngleShouderToSpineBase > 40)) {
                                    CountSwitch = false;
                                }

                                if (ellipseX >= 90 && MetronomeSwitchEnter == true) {
                                    Score += 0.5;
                                }
                            }
                            else {
                                if (AngleUpperBody < 165) {
                                    std::stringstream Warning1;
                                    Warning1 << "Straighten your body!";
                                    cv::putText(kinect.rgbImage, Warning1.str(), cv::Point(500, 100), cv::FONT_HERSHEY_SIMPLEX, 2.0, cv::Scalar(0, 0, 255), 3);
                                }

                                if (IntAngleShouderToSpineBase > MaxDegree) {
                                    std::stringstream Warning2;
                                    Warning2 << "Raise your body!";
                                    cv::putText(kinect.rgbImage, Warning2.str(), cv::Point(500, 150), cv::FONT_HERSHEY_SIMPLEX, 2.0, cv::Scalar(0, 0, 255), 3);
                                }

                                if (IntAngleShouderToSpineBase < MinDegree) {
                                    std::stringstream Warning3;
                                    Warning3 << "Tilt your body down!";
                                    cv::putText(kinect.rgbImage, Warning3.str(), cv::Point(500, 150), cv::FONT_HERSHEY_SIMPLEX, 2.0, cv::Scalar(0, 0, 255), 3);
                                }

                                // 身体が正しい位置にない場合のマーカーと線の色を変更
                                cv::circle(kinect.rgbImage, cv::Point((int)CpHead.X-SlideHumanX, (int)CpHead.Y+ SlideHumanY), 15, cv::Scalar(0, 0, 255), -1);
                                cv::circle(kinect.rgbImage, cv::Point((int)CpSpineShoulder.X-SlideHumanX, (int)CpSpineShoulder.Y+ SlideHumanY), 15, cv::Scalar(0, 0, 255), -1);
                                cv::circle(kinect.rgbImage, cv::Point((int)CpSpineBase.X-SlideHumanX, (int)CpSpineBase.Y+ SlideHumanY), 15, cv::Scalar(0, 0, 255), -1);
                                cv::circle(kinect.rgbImage, cv::Point((int)CpKneeRight.X-SlideHumanX, (int)CpKneeRight.Y+ SlideHumanY), 15, cv::Scalar(0, 0, 255), -1);
                                cv::circle(kinect.rgbImage, cv::Point((int)CpKneeLeft.X-SlideHumanX, (int)CpKneeLeft.Y+ SlideHumanY), 15, cv::Scalar(0, 0, 255), -1);
                                cv::circle(kinect.rgbImage, cv::Point((int)CpAnkleRight.X-SlideHumanX, (int)CpAnkleRight.Y+ SlideHumanY), 15, cv::Scalar(0, 0, 255), -1);
                                cv::circle(kinect.rgbImage, cv::Point((int)CpAnkleLeft.X-SlideHumanX, (int)CpAnkleLeft.Y+ SlideHumanY), 15, cv::Scalar(0, 0, 255), -1);

                                cv::line(kinect.rgbImage, cv::Point((int)CpHead.X- SlideHumanX, (int)CpHead.Y+ SlideHumanY), cv::Point((int)CpSpineShoulder.X- SlideHumanX, (int)CpSpineShoulder.Y+ SlideHumanY), cv::Scalar(0, 0, 255), 10);
                                cv::line(kinect.rgbImage, cv::Point((int)CpSpineShoulder.X- SlideHumanX, (int)CpSpineShoulder.Y+ SlideHumanY), cv::Point((int)CpSpineBase.X- SlideHumanX, (int)CpSpineBase.Y+ SlideHumanY), cv::Scalar(0, 0, 255), 10);
                                cv::line(kinect.rgbImage, cv::Point((int)CpSpineBase.X- SlideHumanX, (int)CpSpineBase.Y+ SlideHumanY), cv::Point((int)CpKneeRight.X- SlideHumanX, (int)CpKneeRight.Y+ SlideHumanY), cv::Scalar(0, 0, 255), 10);
                                cv::line(kinect.rgbImage, cv::Point((int)CpSpineBase.X- SlideHumanX, (int)CpSpineBase.Y+ SlideHumanY), cv::Point((int)CpKneeLeft.X- SlideHumanX, (int)CpKneeLeft.Y+ SlideHumanY), cv::Scalar(0, 0, 255), 10);
                                cv::line(kinect.rgbImage, cv::Point((int)CpKneeRight.X- SlideHumanX, (int)CpKneeRight.Y+ SlideHumanY), cv::Point((int)CpAnkleRight.X- SlideHumanX, (int)CpAnkleRight.Y+ SlideHumanY), cv::Scalar(0, 0, 255), 10);
                                cv::line(kinect.rgbImage, cv::Point((int)CpKneeLeft.X- SlideHumanX, (int)CpKneeLeft.Y+ SlideHumanY), cv::Point((int)CpAnkleLeft.X- SlideHumanX, (int)CpAnkleLeft.Y+ SlideHumanY), cv::Scalar(0, 0, 255), 10);

                                if (((SoundCount % 4 == 0 && IntAngleShouderToSpineBase >= 60)
                                    || (SoundCount % 4 == 0 && IntAngleShouderToSpineBase <= 40))
                                    && SoundCount != 0
                                    && !CountSwitch) {
                                    TotalCount += 0.5;
                                    BadCount += 0.5;
                                    CountSwitch = true;
                                }
                                else if ((SoundCount % 4 != 0 && IntAngleShouderToSpineBase < 60)
                                    || (SoundCount % 4 != 0 && IntAngleShouderToSpineBase > 40)) {
                                    CountSwitch = false;
                                }
                            }
                        }
                        else {
                            // カメラからの距離が正しくない場合、フレームの枠を赤に設定
                            cv::rectangle(kinect.rgbImage, cv::Rect(0, 0, kinect.rgbImage.cols, kinect.rgbImage.rows), cv::Scalar(0, 0, 255), 50);
                        }
                    }
                }
            }
        }

        // RGB画像を表示
        cv::imshow("AbdominalSystem", kinect.rgbImage);
        auto key = cv::waitKey(1);
        if (key == 'q') {
            break; 
        }
    }
    cv::destroyAllWindows();
}

int main(int argc, char** argv) {
    
    bool MetronomeSwitch = false;

    try {
        doJob(MetronomeSwitch);
    }
    catch (exception& ex) {
   
        cout << ex.what() << endl;
        string s;
        cin >> s;
    }
    
    return 0;
}
