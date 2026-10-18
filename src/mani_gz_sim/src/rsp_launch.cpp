#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

namespace fs = std::filesystem;

// Read a file into a string.
std::string read_file(const std::string& filepath)
{
    std::ifstream file(filepath);

    if (!file.is_open())
    {
        throw std::runtime_error(
            "Failed to open file: " + filepath);
    }

    std::stringstream buffer;
    buffer << file.rdbuf();

    return buffer.str();
}

// Execute a command and capture its standard output.
std::string execute_command(
    const std::vector<std::string>& args)
{
    int pipe_fd[2];

    if (pipe(pipe_fd) == -1)
    {
        throw std::runtime_error("Failed to create pipe");
    }

    pid_t pid = fork();

    if (pid == -1)
    {
        close(pipe_fd[0]);
        close(pipe_fd[1]);
        throw std::runtime_error("Failed to fork process");
    }

    if (pid == 0)
    {
        close(pipe_fd[0]);

        dup2(pipe_fd[1], STDOUT_FILENO);
        close(pipe_fd[1]);

        std::vector<char*> argv;

        for (const auto& arg : args)
        {
            argv.push_back(const_cast<char*>(arg.c_str()));
        }

        argv.push_back(nullptr);

        execvp(argv[0], argv.data());

        perror("execvp");
        _exit(127);
    }

    close(pipe_fd[1]);

    std::string output;
    char buffer[4096];

    ssize_t bytes_read;

    while ((bytes_read = read(
                pipe_fd[0], buffer, sizeof(buffer))) > 0)
    {
        output.append(buffer, bytes_read);
    }

    close(pipe_fd[0]);

    int status = 0;

    if (waitpid(pid, &status, 0) == -1)
    {
        throw std::runtime_error("Failed to wait for xacro");
    }

    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0)
    {
        throw std::runtime_error("Xacro processing failed");
    }

    return output;
}

int main(int argc, char* argv[])
{
    try
    {
        // ----------------------------------------------
        // 1. Read the use_sim_time argument.
        // ----------------------------------------------

        bool use_sim_time = true;

        for (int i = 1; i < argc; ++i)
        {
            const std::string arg = argv[i];

            if (arg == "use_sim_time:=false")
            {
                use_sim_time = false;
            }
            else if (arg == "use_sim_time:=true")
            {
                use_sim_time = true;
            }
        }

        // ----------------------------------------------
        // 2. Find the robot description package.
        // ----------------------------------------------

        const std::string package_path =
            ament_index_cpp::get_package_share_directory(
                "ros_gazebo_simulation");

        const std::string xacro_file =
            (fs::path(package_path) /
             "description" /
             "robot.urdf.xacro").string();

        // ----------------------------------------------
        // 3. Process the Xacro file.
        // ----------------------------------------------

        std::cout << "Processing Xacro: "
                  << xacro_file << '\n';

        const std::string robot_description =
            execute_command({"xacro", xacro_file});

        if (robot_description.empty())
        {
            throw std::runtime_error(
                "Robot description is empty");
        }

        // ----------------------------------------------
        // 4. Start robot_state_publisher.
        // ----------------------------------------------

        const std::string sim_time =
            use_sim_time ? "true" : "false";

        pid_t pid = fork();

        if (pid == -1)
        {
            throw std::runtime_error(
                "Failed to start robot_state_publisher");
        }

        if (pid == 0)
        {
            execlp(
                "ros2",
                "ros2",
                "run",
                "robot_state_publisher",
                "robot_state_publisher",
                "--ros-args",
                "-p",
                ("robot_description:=" +
                 robot_description).c_str(),
                "-p",
                ("use_sim_time:=" +
                 sim_time).c_str(),
                "-r",
                "__node:=node_robot_state_publisher",
                static_cast<char*>(nullptr));

            perror("Failed to launch robot_state_publisher");
            _exit(127);
        }

        std::cout
            << "robot_state_publisher started.\n"
            << "use_sim_time: " << sim_time << '\n';

        // Keep the launcher alive while the node runs.
        int status = 0;

        if (waitpid(pid, &status, 0) == -1)
        {
            throw std::runtime_error(
                "Failed to wait for robot_state_publisher");
        }

        return WIFEXITED(status)
            ? WEXITSTATUS(status)
            : 1;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }
}