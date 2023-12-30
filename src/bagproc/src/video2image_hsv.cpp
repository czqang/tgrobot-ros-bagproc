#include <bagproc/video2image.h>

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

bool check_image_dir(std::string& path)
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

bool check_orCreate_dir(std::string& path)
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

std::string getTimeStrNow()
{
    time_t now = time(NULL);
    struct tm *local_tm = localtime(&now);
    std::stringstream time_name;
    time_name << local_tm->tm_year + 1900
                << std::setw(2) << std::setfill('0') << local_tm->tm_mon + 1
                << std::setw(2) << std::setfill('0') << local_tm->tm_mday
                << std::setw(2) << std::setfill('0') << local_tm->tm_hour
                << std::setw(2) << std::setfill('0') << local_tm->tm_min
                << std::setw(2) << std::setfill('0') << local_tm->tm_sec;
    return time_name.str();
}

int main(int argc, char **argv)
{
    std::string videoName, videoPath;
    float capRate;

    if(argc != 3)
    {
        ROS_WARN_STREAM("Error grguiments. Todo: --video_Path --captureRate");
        std::cout << "Enter the video file complete path:\n" << std::endl;
        std::cin >> videoName;
        std::cout << "Enter the frame extraction frequency:\n" << std::endl;
        std::cin >> capRate;
    }
    else
    {
        videoName.assign(argv[1]);
        capRate = atof(argv[2]);
    }

    char path_t[1024];
    if(getcwd(path_t, sizeof(path_t)) != NULL)
    {
        videoPath = path_t;
        videoPath += "/" + videoName;
        std::cout << "[video2image_hsv INFO]:" << " --bagPath:" << videoPath << " --interval(frames):" << unsigned(capRate) << std::endl;
    }
    else
    {
        std::cerr << "Failed to obtain the file directory." << std::endl;
        return -1;
    }

    ROS_INFO("[video2image_hsv] --videoPath: %s  --captureRate: %.2f", videoPath.c_str(), capRate);
    ros::init(argc, argv, "video2image_hsv");

    // open video file
    cv::VideoCapture cap(videoPath);
    if (!cap.isOpened()) {
        ROS_ERROR("can't open the video %s", videoPath.c_str());
        return -1;
    }

    // get the video file path
    std::string video_dir = videoPath.substr(0, videoPath.find_last_of("/\\"));

    float frameRate = cap.get(cv::CAP_PROP_FPS);
    if(capRate > frameRate)
    {
        ROS_ERROR("Capture rate(%.2f) cannot be greater than maximum FPS(%.2f) of input video", capRate, frameRate);
        return -1;
    }

    std::string outPath = video_dir + "/extract_images";
    if(!check_orCreate_dir(outPath))
    {
        ROS_ERROR("check output_image DIR error, outPath: %s", outPath.c_str());
        return -1;
    }
    
    uint16_t interval = frameRate / capRate; // frame extraction interval
    std::string nowtime = getTimeStrNow();
    std::string imgPath = outPath + "CAP" + nowtime + "R" + std::to_string(interval);
    std::string hsvPath = outPath + "HSV" + nowtime + "R" + std::to_string(interval);

    if(!check_image_dir(imgPath))
    {
        ROS_ERROR("check image DIR error, imgPath: %s", imgPath.c_str());
        return -1;
    }
    if(!check_image_dir(hsvPath))
    {
        ROS_ERROR("check image DIR error, imgPath: %s", hsvPath.c_str());
        return -1;
    }

    uint16_t factor = cap.get(cv::CAP_PROP_FRAME_COUNT)/100;    // percentage scaling factor
    uint16_t total_frames = 0, capture_num = 0;
    setbuf(stdout, NULL);   // Set to no buffering
    std::cout << "\033[?25l";   // Hidden cursor

    cv::Mat imgFrame;
    while(cap.read(imgFrame))
    {
        uint8_t persent = total_frames/factor;
        persent = persent > 100? 100:persent;
        std::cout << "\r video2image_hsv Start:-->>  " << std::to_string(persent) << "%";
        if(++total_frames % interval == 0)
        {
            std::stringstream ss;
            ss << "cap_" << ++capture_num << ".png";
            cv::imwrite(imgPath + ss.str(), imgFrame);
            cv::cvtColor(imgFrame, imgFrame, cv::COLOR_BGR2HSV);
            cv::imwrite(hsvPath + ss.str(), imgFrame);
        }
    }
    std::cout << "\n video2image_hsv Complished." << std::endl << "\033[?25h"; // Show cursor.
    ROS_INFO("Video source frames: %d,  extraction frames: %d", total_frames, capture_num);
    ROS_INFO("Video extract to images successed.\nOutput path: %s", imgPath.c_str());
    ROS_INFO("Video extract to images successed.\nOutput path: %s", hsvPath.c_str());

    return 0;
}
