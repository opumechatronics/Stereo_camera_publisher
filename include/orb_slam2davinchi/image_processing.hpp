#include <opencv2/opencv.hpp>

class ImageProcessing
{
public:
    ImageProcessing()
    {

    }

    ~ImageProcessing()
    {

    }

    cv::Mat blured(cv::Mat image, cv::Mat mask, int kernel_size)
    {
        cv::Mat mask_bgr;
        if(mask.channels() == 1)
        {
            cv::cvtColor(mask, mask_bgr, cv::COLOR_GRAY2BGR);
        }
        else
        {
            mask_bgr = mask;
        }

        cv::Mat image_blured;

        cv::blur(image, image_blured, cv::Size(kernel_size, kernel_size));   

        cv::Mat mask_blured = image_blured & mask_bgr;

        cv::Mat mask_inv = ~mask_bgr;

        cv::Mat image_without_mask = image & mask_inv;

        return mask_blured | image_without_mask;
    }

    cv::Mat blured_times(cv::Mat image, cv::Mat mask, int kernel_size, int times)
    {
        cv::Mat dilated_mask, tmp, tmp_blured;
        cv::dilate(mask, dilated_mask, cv::Mat(), cv::Point(-1, -1));
        tmp = image.clone();
        

        for(int i = 0; i < (times-1); i++)
        {
            blur(tmp, tmp_blured, cv::Size(kernel_size, kernel_size));
            tmp = tmp_blured.clone();
        }

        cv::Mat mask_only_blured = tmp & dilated_mask;
        cv::Mat image_mask_inv = image & (~dilated_mask);

        return mask_only_blured | image_mask_inv;

    }

    cv::Mat step_blured(cv::Mat image, cv::Mat mask, int kernel_size1, int kernel_size2, int times)
    {
        cv::Mat mask_tmp, tmp;
        cv::dilate(mask, mask_tmp, cv::Mat(), cv::Point(-1, -1));
        tmp = image.clone();

        for(int i = 0; i < (times/2); i++)
        {
            tmp = blured(tmp, mask_tmp, kernel_size1);
            cv::erode(mask_tmp, mask_tmp, cv::Mat(), cv::Point(-1, -1));
        }
        for(int i = 0; i < (times/2); i++)
        {
            tmp = blured(tmp, mask_tmp, kernel_size2);
            cv::erode(mask_tmp, mask_tmp, cv::Mat(), cv::Point(-1, -1));
        }
        return tmp;
    }

};