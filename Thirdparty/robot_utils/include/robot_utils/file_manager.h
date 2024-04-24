/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/
#ifndef TOOLS_FILE_MANAGER_HPP_
#define TOOLS_FILE_MANAGER_HPP_

#include <string>
#include <iostream>
#include <fstream>

namespace FileManager {

    bool CreateFile(std::ofstream &ofs, std::string file_path);

    bool InitDirectory(std::string directory_path);

    bool CreateDirectory(std::string directory_path);

    int CountFiles(std::string directory_path, std::string suffix);

    std::string JoinPath(std::string path1, std::string path2);
};

#endif
