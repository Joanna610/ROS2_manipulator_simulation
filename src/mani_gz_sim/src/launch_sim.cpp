#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <signal.h>

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
        {
            argv.push_back(const_cast<char*>(arg.c_str()));
        }

        argv.push_back(nullptr);

        execvp(argv[0], argv.data());

        perror("execvp failed");
        _exit(127);
    }

    return pid;
}

int main(int argc, char* argv[])
{
    // 1. Configure Qt.
    setenv("QT_QPA_PLATFORM", "xcb", 1);

    // 2. Locate the package.
    const std::string package_name = "mani_gz_sim";

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

    // 3. Set the default world.
    const std::string default_world =
        (fs::path(package_share) /
         "world" /
         "world.world").string();

    // 4. Allow a custom world path.
    const std::string world =
        (argc > 1) ? argv[1] : default_world;

    // 5. Check that the world file exists.
    if (!fs::exists(world))
    {
        std::cerr << "World file not found: "
                  << world << '\n';

        return 1;
    }

    // 6. Start Gazebo Sim with the selected world.
    pid_t gazebo_pid = launch_process({
        "gz",
        "sim",
        "-r",
        "-v4",
        world
    });

    if (gazebo_pid < 0)
    {
        std::cerr << "Failed to start Gazebo.\n";
        return 1;
    }

    std::cout << "Gazebo Sim started.\n";
    std::cout << "World: " << world << '\n';
    std::cout << "No robot will be spawned.\n";

    // 7. Wait for Gazebo to exit.
    int status = 0;

    while (waitpid(gazebo_pid, &status, 0) == -1)
    {
        if (errno == EINTR)
        {
            continue;
        }

        perror("waitpid failed");
        return 1;
    }

    std::cout << "Gazebo has closed.\n";

    return WIFEXITED(status)
        ? WEXITSTATUS(status)
        : 1;
}