#include <bagproc/bag2video.h>

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

std::string replaceSlash(std::string str){
    for(int i=0; i<str.length(); i++){
        if(str[i] == '/'){
            str[i] = '_';
        }
    }
    return str;
}

std::string removeStrigula(const std::string& inputStr) {
    std::string str(inputStr);
    str.erase(std::remove(str.begin(), str.end(), '-'), str.end());
    return str;
}

int main(int argc, char **argv)
{
    std::string bagName, bagPath;

    if(argc < 2)
    {
        ROS_WARN_STREAM("Error grguiments. Todo: --bagName");
        std::cout << "Enter the bagfile complete name:" << std::endl;
        std::cin >> bagName;
    }
    else if(argc == 2)
    {
        bagName.assign(argv[1]);
    }
    // else if(argc == 3)
    // {
    //     bagName.assign(argv[1]);
    //     frame_height = atoi(argv[2]);
    // }
    else
    {
        ROS_ERROR_STREAM("Input parameter sequence incorrect, [bag2video_HSV] exit.");
        return -1;
    }

    char path_t[1024];
    if(getcwd(path_t, sizeof(path_t)) != NULL)
    {
        bagPath = path_t;
        bagPath += "/" + bagName;
        std::cout << "[bag2video_HSV INFO]:" << " --bagPath:" << bagPath << std::endl;
    }
    else
    {
        std::cerr << "Failed to obtain the file directory." << std::endl;
        return -1;
    }

    ROS_INFO("[bag2video_HSV] --bagPath: %s", bagPath.c_str());

    if(boost::filesystem::exists(bagPath) == false)  // Check whether the bagfile exists.
    {
        ROS_ERROR("Specified rosbag file not exist.-- %s \n", bagPath.c_str());
        return -1;
    }
    std::string filePath = boost::filesystem::path(bagPath).parent_path().string();
    std::string filename = boost::filesystem::path(bagPath).stem().string();
    filename = removeStrigula(filename);

    ros::init(argc, argv, "bag2video_hsv");

    std::string outPath = filePath + "/output_videos";
    if (!check_targetDir(outPath))
    {
        ROS_ERROR("check output_video DIR error, outPath: %s", outPath.c_str());
        return -1;
    }

    rosbag::Bag bag;
    bag.open(bagPath, rosbag::bagmode::Read);
    uint16_t frame_width, frame_height;

    {
        // get image frame size (width & height)
        std::vector<std::string> topics;
        topics.push_back(std::string("/image_raw/compressed"));
        rosbag::View view(bag, rosbag::TopicQuery(topics));
        rosbag::MessageInstance msg = *view.begin();
        sensor_msgs::CompressedImageConstPtr imgMsg = msg.instantiate<sensor_msgs::CompressedImage>();
        if(imgMsg != nullptr)
        {
            cv::Mat imgData = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8)->image;
            frame_width = imgData.cols;
            frame_height = imgData.rows;
            ROS_INFO("image frame size: %dx%d", frame_width, frame_height);
        }
        else
        {
            ROS_ERROR("get the size of image frames error.");
            return -1;
        }
    }

    setbuf(stdout, NULL);     // Set to no buffering
    std::cout << "\033[?25l"; // Hidden cursor

    uint16_t frames_cpr = 0;
    std::string videoName = outPath + filename + "_hsv.mp4";
    cv::VideoWriter vWriter;
    vWriter.open(videoName, cv::VideoWriter::fourcc('h', '2', '6', '4'), 30, cv::Size(frame_width, frame_height));

    for (rosbag::MessageInstance const m : rosbag::View(bag))
    {
        std::cout << "\r[bag2video_HSV] -Start:---->>  -compressed_frames:" << frames_cpr;

        sensor_msgs::CompressedImageConstPtr c_img_ptr = m.instantiate<sensor_msgs::CompressedImage>();
        if (c_img_ptr != nullptr)
        {
            ++frames_cpr;
            cv::Mat img = cv_bridge::toCvCopy(c_img_ptr, sensor_msgs::image_encodings::BGR8)->image;
            cv::cvtColor(img, img, cv::COLOR_BGR2HSV);
            vWriter << img;
        }
    }
    vWriter.release();
    bag.close();
    std::cout << "\bag2video_HSV complished extract: [--compressed images " << frames_cpr << std::endl
              << "\033[?25h"; // Show cursor.
    if(frames_cpr < 10) // Not enough frames to synthesize video
    {
        boost::filesystem::remove(videoName);
        ROS_INFO("Topic 'image_compressed' not in rosbag file.");
    }
    else ROS_INFO("rosbag convert to compressed video successed.\nOutput path: %s", videoName.c_str());

    return 0;
}