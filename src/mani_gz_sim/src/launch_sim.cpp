#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
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
        // Prepare arguments for execvp().
        std::vector<char*> argv;

        for (const auto& arg : args)
        {
            argv.push_back(const_cast<char*>(arg.c_str()));
        }

        argv.push_back(nullptr);

        execvp(argv[0], argv.data());

        // Reached only if execvp fails.
        perror("execvp failed");
        _exit(127);
    }

    return pid;
}

int main(int argc, char* argv[])
{
    // --------------------------------------------------
    // 1. Set the Qt environment variable.
    // --------------------------------------------------

    setenv("QT_QPA_PLATFORM", "xcb", 1);

    const std::string package_name = "mani_gz_sim";

    // --------------------------------------------------
    // 2. Get package paths.
    // --------------------------------------------------

    std::string package_share;

    try
    {
        package_share =
            ament_index_cpp::get_package_share_directory(
                package_name);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Could not find package "
                  << package_name << ": "
                  << e.what() << '\n';

        return 1;
    }

    const std::string default_world =
        (fs::path(package_share) /
         "worlds" /
         "world.world").string();

    const std::string bridge_config =
        (fs::path(package_share) /
         "config" /
         "ros_gz_bridge.yaml").string();

    // --------------------------------------------------
    // 3. Read the world argument.
    //
    // Usage:
    // ./launch_sim
    // ./launch_sim /path/to/world.sdf
    // --------------------------------------------------

    const std::string world =
        (argc > 1) ? argv[1] : default_world;

    // --------------------------------------------------
    // 4. Start the robot state publisher.
    // --------------------------------------------------

    pid_t rsp_pid = launch_process({
        "ros2",
        "launch",
        package_name,
        "rsp_launch.cpp",
        "use_sim_time:=true"
    });

    if (rsp_pid < 0)
    {
        return 1;
    }

    // --------------------------------------------------
    // 5. Start Gazebo Sim.
    // --------------------------------------------------

    pid_t gazebo_pid = launch_process({
        "ros2",
        "launch",
        "mani_gz_sim",
        "gz_sim.launch.py",
        "gz_args:=-r -v4 " + world,
        "on_exit_shutdown:=true"
    });

    if (gazebo_pid < 0)
    {
        kill(rsp_pid, SIGTERM);
        return 1;
    }

    // --------------------------------------------------
    // 6. Spawn the robot.
    // --------------------------------------------------

    pid_t spawn_pid = launch_process({
        "ros2",
        "run",
        "ros_gz_sim",
        "create",
        "-topic",
        "robot_description",
        "-name",
        "my_bot",
        "-z",
        "0.1"
    });

    if (spawn_pid < 0)
    {
        kill(rsp_pid, SIGTERM);
        kill(gazebo_pid, SIGTERM);
        return 1;
    }

    // --------------------------------------------------
    // 7. Start the ROS-Gazebo bridge.
    // --------------------------------------------------

    pid_t bridge_pid = launch_process({
        "ros2",
        "run",
        "ros_gz_bridge",
        "parameter_bridge",
        "--ros-args",
        "-p",
        "config_file:=" + bridge_config
    });

    if (bridge_pid < 0)
    {
        kill(rsp_pid, SIGTERM);
        kill(gazebo_pid, SIGTERM);
        kill(spawn_pid, SIGTERM);
        return 1;
    }

    std::cout << "ROS 2 and Gazebo processes started.\n";
    std::cout << "World: " << world << '\n';

    // --------------------------------------------------
    // 8. Keep the launcher alive.
    // --------------------------------------------------

    // Wait for the first child process to exit.
    int status = 0;
    pid_t finished_pid = wait(&status);

    std::cout << "Process " << finished_pid
              << " exited.\n";

    // Terminate the remaining child processes.
    kill(rsp_pid, SIGTERM);
    kill(gazebo_pid, SIGTERM);
    kill(spawn_pid, SIGTERM);
    kill(bridge_pid, SIGTERM);

    // Reap the remaining children.
    waitpid(rsp_pid, nullptr, 0);
    waitpid(gazebo_pid, nullptr, 0);
    waitpid(spawn_pid, nullptr, 0);
    waitpid(bridge_pid, nullptr, 0);

    return 0;
}