#include <bagproc/bag2image.h>

bool checkPath_OK(std::string &path)
{
    if(path.back() != '/') {    // Is the end of the string a slash?
        path.append("/");
    }
    DIR *dir = opendir(path.c_str());
    if(dir == NULL)
    {
        ROS_WARN("A nonexistent path, check it. %s", path.c_str());
        return false;
    }
    return true;
}

bool clear_dir(const std::string& path) {
    DIR* dir = opendir(path.c_str());
    if (dir == nullptr) {
        return false;
    }
    bool success = true;
    while (dirent* entry = readdir(dir)) {
        if (entry->d_type != DT_DIR && entry->d_type != DT_REG) {
            continue;
        }
        std::string name = entry->d_name;
        if (name == "." || name == "..") {
            continue;
        }
        std::string entry_path = path + "/" + name;
        if (entry->d_type == DT_DIR) {
            success = clear_dir(entry_path) && success;
        } else {
            success = (unlink(entry_path.c_str()) == 0) && success;
        }
    }
    closedir(dir);
    return success;
}

bool create_dir(const std::string& path) {
    return mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == 0;
}

// Check whether the target folder exists, and create it if it does not
bool check_targetDir(std::string& path)
{
    bool success = true;
    if(!checkPath_OK(path))
    {
        success = create_dir(path);
        if(success) {
            ROS_INFO("create_dir :%s success.",path.c_str());
        }
        else {
            ROS_WARN("create_dir :%s failure.",path.c_str());
        }
    }
    return success;
}

// Check whether the target folder exists, and clear it if it does not
bool check_imageDir(std::string& path)
{
    bool success = true;
    if(checkPath_OK(path))
    {
        success = clear_dir(path);
        if(success) {
            ROS_INFO("clear_dir :%s success.",path.c_str());
        }
        else {
            ROS_WARN("clear_dir :%s failure.",path.c_str());
        }
    }
    else
    {
        success = create_dir(path);
        if(success) {
            ROS_INFO("create_dir :%s success.",path.c_str());
        }
        else {
            ROS_WARN("create_dir :%s failure.",path.c_str());
        }
    }
    return success;
}

std::string removeUnderscores(const std::string& inputStr) {
    std::string str(inputStr);
    str.erase(std::remove(str.begin(), str.end(), '-'), str.end());
    return str;
}

int main(int argc, char **argv)
{
    std::string bagName, bagPath, filter;
    uint8_t interval;

    if(argc < 3)
    {
        ROS_WARN_STREAM("Error grguiments. Todo: --bagName --interval(Image extraction frame interval)");
        std::cout << "Enter the bagfile complete name:" << std::endl;
        std::cin >> bagName;
        std::cout << "Enter the file image extraction frame interval:" << std::endl;
        std::cin >> interval;
    }
    else if(argc == 3)
    {
        bagName.assign(argv[1]);
        interval = atoi(argv[2]);
    }
    else if(argc == 4)
    {
        bagName.assign(argv[1]);
        interval = atoi(argv[2]);
        filter.assign(argv[3]);
    }
    else
    {
        ROS_ERROR_STREAM("Input parameter sequence incorrect, [rs_bag2video] exit.");
        return -1;
    }

    char path_t[1024];
    if(getcwd(path_t, sizeof(path_t)) != NULL)
    {
        bagPath = path_t;
        bagPath += "/" + bagName;
        std::cout << "[rs_bag2image INFO]:" << " --bagPath:" << bagPath << " --interval(frames):" << unsigned(interval) << std::endl;
    }
    else
    {
        std::cerr << "Failed to obtain the file directory." << std::endl;
        return -1;
    }

    bool image_compressed =true;
    if(filter == "raw")
        image_compressed = false;
    std::cout << "[rs_bag2image] INFO:" << "\n --bagPath:" << bagPath << "\n --interval(frames):" << unsigned(interval) << " --compressed_image?:" << std::boolalpha << image_compressed << std::endl;

    std::string subTopics[3];
    if(image_compressed)
    {
        subTopics[0] = "/camera/color/image_raw/compressed";
        subTopics[1] = "/camera/depth/image_rect_raw/compressed";
        subTopics[2] = "/camera/aligned_depth_to_color/image_raw/compressed";
    }
    else
    {
        subTopics[0] = "/camera/color/image_raw";
        subTopics[1] = "/camera/depth/image_rect_raw";
        subTopics[2] = "/camera/aligned_depth_to_color/image_raw";
    }

    ros::init(argc, argv, "rs_bag2image");
    if(boost::filesystem::exists(bagPath) == false)  // Check whether the bagfile exists.
    {
        ROS_ERROR("Specified rosbag file not exist.-- %s \n", bagPath.c_str());
        return -1;
    }

    std::string filePath = boost::filesystem::path(bagPath).parent_path().string();
    std::string filename = boost::filesystem::path(bagPath).stem().string();
    filename = removeUnderscores(filename);

    std::string outPath = filePath + "/output_images";
    if(!check_targetDir(outPath))
    {
        ROS_ERROR("Check output_image DIR error, outPath: %s", outPath.c_str());
        return -1;
    }

    rosbag::Bag bag;
    bag.open(bagPath);

    std::string imgPath;
    imgPath = outPath + filename;
    if (!check_targetDir(imgPath))
    {
        ROS_ERROR("check image DIR error, imgPath: %s", imgPath.c_str());
        return -1;
    }

    std::string colorPath = imgPath +  "color";
    if (!check_imageDir(colorPath))
    {
        ROS_ERROR("check image DIR error, imgPath: %s", colorPath.c_str());
        return -1;
    }

    std::string depthPath = imgPath +  "depth";
    if (!check_imageDir(depthPath))
    {
        ROS_ERROR("check image DIR error, imgPath: %s", depthPath.c_str());
        return -1;
    }

    std::string alignPath = imgPath +  "align";
    if (!check_imageDir(alignPath))
    {
        ROS_ERROR("check image DIR error, imgPath: %s", alignPath.c_str());
        return -1;
    }

    uint8_t color_cap = 0, depth_cap = 0, align_cap = 0;;
    uint16_t color_count = 0, depth_count = 0, align_count = 0;

    setbuf(stdout, NULL);   // Set to no buffering
    std::cout << "\033[?25l\n";   // Hidden cursor
    for (rosbag::MessageInstance const m : rosbag::View(bag))
    {
        if(image_compressed)
        {
            sensor_msgs::CompressedImageConstPtr image_ptr = m.instantiate<sensor_msgs::CompressedImage>();
            if (image_ptr != nullptr)
            {
                std::cout << "\r[rs_bag2image] -Start:---->>  -color:" << color_count << " -depth:" << depth_count << " -align:" << align_count;
                std::stringstream imgName;
                try
                {
                    cv::Mat image = cv::imdecode(cv::Mat(image_ptr->data), cv::IMREAD_COLOR); // IMREAD_ANYDEPTH
                    if (m.getTopic() == subTopics[0])
                    {
                        if ((++color_cap) % interval == 0)
                        {
                            // cv::Mat image = cv_bridge::toCvCopy(image_ptr, sensor_msgs::image_encodings::BGR8)->image;
                            imgName << colorPath << "color_" << std::setw(5) << std::setfill('0') << ++color_count << ".png";
                            cv::imwrite(imgName.str(), image);
                        }
                    }
                    else if (m.getTopic() == subTopics[1])
                    {
                        if ((++depth_cap) % interval == 0)
                        {
                            // cv::Mat image = cv_bridge::toCvCopy(image_ptr, sensor_msgs::image_encodings::TYPE_16UC1)->image;
                            imgName << depthPath << "depth_" << std::setw(5) << std::setfill('0') << ++depth_count << ".png";
                            cv::imwrite(imgName.str(), image);
                        }
                    }
                    else if (m.getTopic() == subTopics[2])
                    {
                        if ((++align_cap) % interval == 0)
                        {
                            // cv::Mat image = cv_bridge::toCvCopy(image_ptr, sensor_msgs::image_encodings::TYPE_16UC1)->image;
                            imgName << alignPath << "align_" << std::setw(5) << std::setfill('0') << ++align_count << ".png";
                            cv::imwrite(imgName.str(), image);
                        }
                    }
                }
                catch (const cv_bridge::Exception &e)
                {
                    ROS_ERROR("cv_bridge exception: %s", e.what());
                }
            }
        }
        else
        {
            sensor_msgs::ImageConstPtr img_ptr = m.instantiate<sensor_msgs::Image>();
            if (img_ptr != nullptr)
            {
                std::cout << "\r[rs_bag2image] RAW -Start:---->>  -color:" << color_count << " -depth:" << depth_count << " -align:" << align_count;
                std::stringstream imgName;
                
                try
                {
                    if (m.getTopic() == subTopics[0])
                    {
                        if ((++color_cap) % interval == 0)
                        {
                            cv::Mat image = cv_bridge::toCvCopy(img_ptr, sensor_msgs::image_encodings::BGR8)->image;
                            // cv::Mat image = cv::imdecode(cv::Mat(img_ptr->data), cv::IMREAD_ANYCOLOR); // IMREAD_ANYDEPTH
                            imgName << colorPath << "color_raw" << std::setw(5) << std::setfill('0') << ++color_count << ".png";
                            cv::imwrite(imgName.str(), image);
                        }
                    }
                    else if (m.getTopic() == subTopics[1])
                    {
                        if ((++depth_cap) % interval == 0)
                        {
                            // cv::Mat image = cv::imdecode(cv::Mat(img_ptr->data), cv::IMREAD_ANYDEPTH); // IMREAD_ANYDEPTH
                            cv::Mat image = cv_bridge::toCvCopy(img_ptr, sensor_msgs::image_encodings::BGR8)->image;  // TYPE_16UC1
                            imgName << depthPath << "depth_raw" << std::setw(5) << std::setfill('0') << ++depth_count << ".png";
                            cv::imwrite(imgName.str(), image);
                        }
                    }
                    else if (m.getTopic() == subTopics[2])
                    {
                        if ((++align_cap) % interval == 0)
                        {
                            // cv::Mat image = cv::imdecode(cv::Mat(img_ptr->data), cv::IMREAD_ANYDEPTH); // IMREAD_ANYDEPTH
                            cv::Mat image = cv_bridge::toCvCopy(img_ptr, sensor_msgs::image_encodings::BGR8)->image;
                            imgName << alignPath << "align_raw" << std::setw(5) << std::setfill('0') << ++align_count << ".png";
                            cv::imwrite(imgName.str(), image);
                        }
                    }
                }
                catch (const cv_bridge::Exception &e)
                {
                    ROS_ERROR("cv_bridge exception: %s", e.what());
                }
            }
        }

    }
    bag.close();
    std::cout << "\nrs_bag2image complished extract: [--color_images " << color_count << "], [--depth_images " << depth_count << "], [--align_images " << align_count << "]." << std::endl
              << "\033[?25h"; // Show cursor.
    ROS_INFO("rosbag convert to image successed.\nOutput path: %s", imgPath.c_str());

    return 0;
}