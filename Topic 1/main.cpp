#include <iostream>
#include <vector>
#include "opencv2/highgui.hpp"

int main(int argc, char** argv){

    cv::String FilePath = "../../rgb/";
    std::vector<cv::String> OldName;
    cv::glob(FilePath, OldName);//将图片导入一个容器内

    if (!OldName.empty()){
        for (auto &i : OldName){
            cv::Mat img = cv::imread(i, 1);//设置一个调用imwrite函数的一个参数

            if (i.find(".png") != cv::String::npos){
                char NewName[100];//设置一个sprint的参数
                sprintf(NewName, "%04ld.png", &i - &OldName[0]);
                cv::imwrite(FilePath + (cv::String)NewName, img);
            }
            else{
                std::cerr << "That has a false about path.\n" << std::endl;
                return -1;
            }
        }
        std::cout << "Changed succeed!\n";
    }


}
