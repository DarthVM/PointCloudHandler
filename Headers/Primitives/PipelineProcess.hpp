

#ifndef CGALUBUNTU_PIPELINEPROCESS_HPP
#define CGALUBUNTU_PIPELINEPROCESS_HPP

#include <iostream>
#include <string>
#include <vector>

#include <windows.h>
#include <cstdio>


std::string call_python_script(std::string script_name, std::vector<std::string> args) {
    // Create a pipe for the child process's STDOUT.
    HANDLE hReadPipe, hWritePipe;
    SECURITY_ATTRIBUTES sa;
    sa.nLength = sizeof(SECURITY_ATTRIBUTES);
    sa.bInheritHandle = TRUE;  // Pipe handles are inheritable
    sa.lpSecurityDescriptor = NULL;

    if (!CreatePipe(&hReadPipe, &hWritePipe, &sa, 0)) {
        std::cerr << "Error creating pipe." << std::endl;
        return "";
    }

    // Ensure the write handle to the pipe is not inherited
    SetHandleInformation(hWritePipe, HANDLE_FLAG_INHERIT, 0);

    // Set up the process information and startup info
    PROCESS_INFORMATION processInfo;
    ZeroMemory(&processInfo, sizeof(processInfo));

    STARTUPINFO startupInfo;
    ZeroMemory(&startupInfo, sizeof(startupInfo));
    startupInfo.cb = sizeof(startupInfo);
    startupInfo.hStdOutput = hWritePipe;  // Redirect the output to the pipe
    startupInfo.hStdError = hWritePipe;    // Redirect error output to the same pipe
    startupInfo.dwFlags |= STARTF_USESTDHANDLES;

    // Command to execute
    std::string command = "python" + script_name;
    for (const auto& arg : args) {
        command += " " + arg;  // Wrap arguments in quotes
    }

    // Start the child process
    if (CreateProcess(
            NULL,           // No module name (use command line)
            const_cast<LPSTR>(command.c_str()), // Command line
            NULL,           // Process handle not inheritable
            NULL,           // Thread handle not inheritable
            TRUE,           // Set handle inheritance to TRUE
            0,              // No creation flags
            NULL,           // Use parent's environment block
            NULL,           // Use parent's starting directory
            &startupInfo,   // Pointer to STARTUPINFO structure
            &processInfo)   // Pointer to PROCESS_INFORMATION structure
            ) {
        // Close the write end of the pipe before reading from the read end
        CloseHandle(hWritePipe);

        // Read the output from the pipe
        char buffer[128];
        std::string output;
        DWORD bytesRead;

        while (ReadFile(hReadPipe, buffer, sizeof(buffer) - 1, &bytesRead, NULL) && bytesRead > 0) {
            buffer[bytesRead] = '\0'; // Null-terminate the string
            output += buffer;         // Append the output
        }

        // Close handles
        CloseHandle(hReadPipe);
        CloseHandle(processInfo.hProcess);
        CloseHandle(processInfo.hThread);

        return output; // Return the captured output
    } else {
        std::cerr << "Failed to create process. Error: " << GetLastError() << std::endl;
        CloseHandle(hWritePipe);
        CloseHandle(hReadPipe);
        return "";
    }
}



#endif //CGALUBUNTU_PIPELINEPROCESS_HPP
