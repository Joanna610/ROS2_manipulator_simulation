#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <cerrno>
#include <thread>
#include <chrono>

#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace fs = std::filesystem;

// Launch a command as a separate process.
pid_t launch_process(const std::vector<std::string>& args)
{
    pid_t pid = fork();

    if (pid == -1)
    {
        perror("fork failed");
        return -1;
    }

    if (pid == 0)
    {
        std::vector<char*> argv;

        for (const auto& arg : args)
            argv.push_back(const_cast<char*>(arg.c_str()));
        

        argv.push_back(nullptr);

        execvp(argv[0], argv.data());

        perror("execvp failed");
        _exit(127);
    }

    return pid;
}

// Wait for a process to finish.
int wait_for_process(pid_t pid)
{
    int status = 0;

    while (waitpid(pid, &status, 0) == -1)
    {
        if (errno == EINTR)
            continue;
        
        perror("waitpid failed");
        return -1;
    }

    if (WIFEXITED(status))
        return WEXITSTATUS(status);

    return 1;
}

int main(int argc, char* argv[])
{
    setenv("QT_QPA_PLATFORM", "xcb", 1);

    const std::string package_name = "mani_gz_sim";

    try
    {
        const std::string package_share =
            ament_index_cpp::get_package_share_directory(
                package_name);

        const std::string default_world =
            (fs::path(package_share) /
             "world" /
             "world.world").string();

        const std::string xacro_file =
            (fs::path(package_share) /
             "description" /
             "robot.urdf.xacro").string();

        const std::string world =
            (argc > 1) ? argv[1] : default_world;

        if (!fs::exists(world))
        {
            throw std::runtime_error(
                "World file not found: " + world);
        }

        if (!fs::exists(xacro_file))
        {
            throw std::runtime_error(
                "Robot Xacro not found: " + xacro_file);
        }

        std::cout << "World: " << world << '\n';
        std::cout << "Robot: " << xacro_file << '\n';

        std::cout << "Starting Gazebo Sim...\n";

        pid_t gazebo_pid = launch_process({
            "gz",
            "sim",
            "-r",
            "-v4",
            world
        });

        if (gazebo_pid < 0)
            throw std::runtime_error("Failed to start Gazebo");

        // Give Gazebo time to initialize.
        std::this_thread::sleep_for(std::chrono::seconds(1));

        std::cout << "Processing Xacro...\n";

        const std::string urdf_file = "/tmp/mani_gz_sim_robot.urdf";

        const std::string xacro_command =
            "xacro \"" + xacro_file +
            "\" -o \"" + urdf_file + "\"";

        const int xacro_status = std::system(xacro_command.c_str());

        if (xacro_status == -1 ||
            !WIFEXITED(xacro_status) ||
            WEXITSTATUS(xacro_status) != 0)
                throw std::runtime_error("Failed to process Xacro");
        
        std::cout << "Spawning robot in Gazebo...\n";

        pid_t spawn_pid = launch_process({
            "ros2",
            "run",
            "ros_gz_sim",
            "create",
            "-file",
            urdf_file,
            "-name",
            "robot",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0.2"
        });

        if (spawn_pid < 0)
            throw std::runtime_error("Failed to start robot spawning");
        
        const int spawn_status =
            wait_for_process(spawn_pid);

        if (spawn_status != 0)
        {
            throw std::runtime_error(
                "Robot spawning failed. Check Gazebo and ros_gz_sim.");
        }

        std::cout << "Robot spawned successfully.\n";

        // --------------------------------------------
        // 5. Keep Gazebo running.
        // --------------------------------------------

        std::cout
            << "Gazebo and robot are running. "
            << "Close Gazebo to exit.\n";

        const int gazebo_status =
            wait_for_process(gazebo_pid);

        std::cout << "Gazebo has closed.\n";

        return gazebo_status == 0 ? 0 : 1;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }
}