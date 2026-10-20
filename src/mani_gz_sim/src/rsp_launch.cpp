#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include <stdexcept>
#include <cerrno>
#include <fstream>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

namespace fs = std::filesystem;

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

        if (dup2(pipe_fd[1], STDOUT_FILENO) == -1)
        {
            _exit(126);
        }

        close(pipe_fd[1]);

        std::vector<char*> argv;

        for (const auto& arg : args)
        {
            argv.push_back(
                const_cast<char*>(arg.c_str()));
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
                pipe_fd[0],
                buffer,
                sizeof(buffer))) > 0)
    {
        output.append(buffer, bytes_read);
    }

    close(pipe_fd[0]);

    if (bytes_read == -1)
    {
        throw std::runtime_error(
            "Failed to read command output");
    }

    int status = 0;

    while (waitpid(pid, &status, 0) == -1)
    {
        if (errno == EINTR)
        {
            continue;
        }

        throw std::runtime_error(
            "Failed to wait for command");
    }

    if (!WIFEXITED(status) ||
        WEXITSTATUS(status) != 0)
    {
        throw std::runtime_error(
            "Command failed: " + args.front());
    }

    return output;
}

// Start a process without waiting for it to finish.
pid_t start_process(
    const std::vector<std::string>& args)
{
    pid_t pid = fork();

    if (pid == -1)
    {
        throw std::runtime_error(
            "Failed to create process");
    }

    if (pid == 0)
    {
        std::vector<char*> argv;

        for (const auto& arg : args)
        {
            argv.push_back(
                const_cast<char*>(arg.c_str()));
        }

        argv.push_back(nullptr);

        execvp(argv[0], argv.data());

        perror("execvp");
        _exit(127);
    }

    return pid;
}

int main(int argc, char* argv[])
{
    try
    {
        // --------------------------------------------
        // 1. Read parameters.
        // --------------------------------------------

        bool use_sim_time = true;

        // Default spawn position.
        std::string x = "0";
        std::string y = "0";
        std::string z = "0.2";

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

        const std::string sim_time =
            use_sim_time ? "true" : "false";

        // --------------------------------------------
        // 2. Locate the robot description.
        // --------------------------------------------

        const std::string package_path =
            ament_index_cpp::get_package_share_directory(
                "mani_gz_sim");

        const std::string xacro_file =
            (fs::path(package_path) /
             "description" /
             "robot.urdf.xacro").string();

        if (!fs::exists(xacro_file))
        {
            throw std::runtime_error(
                "Xacro file not found: " + xacro_file);
        }

        std::cout
            << "Processing Xacro: "
            << xacro_file << '\n';

        // --------------------------------------------
        // 3. Generate the robot description.
        // --------------------------------------------

        const std::string robot_description =
            execute_command({
                "xacro",
                xacro_file
            });

        if (robot_description.empty())
        {
            throw std::runtime_error(
                "Robot description is empty");
        }

        // --------------------------------------------
        // 4. Start robot_state_publisher.
        // --------------------------------------------

        std::cout
            << "Starting robot_state_publisher...\n";

        pid_t rsp_pid = start_process({
            "ros2",
            "run",
            "robot_state_publisher",
            "robot_state_publisher",
            "--ros-args",
            "-p",
            "robot_description:=" + robot_description,
            "-p",
            "use_sim_time:=" + sim_time,
            "-r",
            "__node:=node_robot_state_publisher"
        });

        // --------------------------------------------
        // 5. Spawn the robot in Gazebo.
        // --------------------------------------------

        std::cout
            << "Spawning robot in Gazebo...\n";

        const std::string xacro_output =
            robot_description;

        // Write the generated URDF to a temporary file.
        const std::string urdf_file =
            "/tmp/mani_gz_sim_robot.urdf";

        {
            std::ofstream file(urdf_file);

            if (!file)
            {
                throw std::runtime_error(
                    "Failed to write temporary URDF");
            }

            file << xacro_output;
        }

        // Spawn the model into the running Gazebo world.
        const int spawn_status = std::system(
            ("ros2 run ros_gz_sim create "
             "-file " + urdf_file +
             " -name robot "
             " -x " + x +
             " -y " + y +
             " -z " + z).c_str());

        if (spawn_status == -1 ||
            !WIFEXITED(spawn_status) ||
            WEXITSTATUS(spawn_status) != 0)
        {
            throw std::runtime_error(
                "Failed to spawn robot in Gazebo");
        }

        std::cout
            << "Robot spawned successfully.\n"
            << "use_sim_time: " << sim_time << '\n';

        // Keep the launcher alive while RSP runs.
        int status = 0;

        while (waitpid(rsp_pid, &status, 0) == -1)
        {
            if (errno == EINTR)
            {
                continue;
            }

            throw std::runtime_error(
                "Failed to wait for robot_state_publisher");
        }

        return WIFEXITED(status)
            ? WEXITSTATUS(status)
            : 1;
    }
    catch (const std::exception& e)
    {
        std::cerr
            << "Error: " << e.what() << '\n';

        return 1;
    }
}