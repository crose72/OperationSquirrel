#include <iostream>
#include <fstream>
#include <string>

int main() {
    // Replace with the actual terminal device
    std::string terminal = "/dev/pts/3"; // Change this to the output of tty command
    std::ofstream out(terminal);

    if (!out.is_open()) {
        std::cerr << "Failed to open terminal: " << terminal << std::endl;
        return 1;
    }

    out << "This is a message to the secondary terminal." << std::endl;

    // Example of writing more detailed logs
    for (int i = 0; i < 10; ++i) {
        out << "Log message " << i << std::endl;
    }

    out.close();
    return 0;
}
