#include <iostream>
#include <opencv2/opencv.hpp>

int main() {
    // 画像の読み込み
    cv::Mat image = cv::imread("input.jpg", cv::IMREAD_GRAYSCALE);

    if (image.empty()) {
        std::cerr << "Failed to read the image." << std::endl;
        return 1;
    }

    // 2値化の閾値を設定（例：128）
    int threshold = 128;

    // 2値化された画像を格納する行列
    cv::Mat binaryImage(image.rows, image.cols, CV_8UC1);

    // 2値化
    for (int y = 0; y < image.rows; ++y) {
        for (int x = 0; x < image.cols; ++x) {
            if (image.at<uchar>(y, x) >= threshold) {
                binaryImage.at<uchar>(y, x) = 255;  // 白
            }
            else {
                binaryImage.at<uchar>(y, x) = 0;    // 黒
            }
        }
    }

    // ラプラシアンフィルタを自前で適用
    cv::Mat filteredImage(image.rows, image.cols, CV_8UC1);

    for (int y = 1; y < image.rows - 1; ++y) {
        for (int x = 1; x < image.cols - 1; ++x) {
            int sum = 0;
            sum += binaryImage.at<uchar>(y - 1, x);     // 上
            sum += binaryImage.at<uchar>(y + 1, x);     // 下
            sum += binaryImage.at<uchar>(y, x - 1);     // 左
            sum += binaryImage.at<uchar>(y, x + 1);     // 右
            sum -= 4 * binaryImage.at<uchar>(y, x);     // 中央

            // フィルタリング後の値をクリッピングして格納
            filteredImage.at<uchar>(y, x) = cv::saturate_cast<uchar>(sum);
        }
    }

    // 2値化された画像およびフィルタリング後の画像の保存
    cv::imwrite("binary.jpg", binaryImage);
    cv::imwrite("filtered.jpg", filteredImage);

    return 0;
}
