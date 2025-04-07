#include <iostream>
#include <thread>
#include <atomic>
#include <csignal>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <chrono>
#include <iomanip>
#include <sstream>

std::atomic<bool> running(true);

void signalHandler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\nCtrl+C detected. Shutting down sender...\n";
        running = false;
    }
}

// Function to format the current timestamp as a string
std::string getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
    auto epoch = now_ms.time_since_epoch();
    auto value = std::chrono::duration_cast<std::chrono::milliseconds>(epoch);
    return std::to_string(value.count());
}

void sendData(const char* interfaceIp) {
    int sockfd;
    struct sockaddr_in multicastAddr;

    // Create socket
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        std::cerr << "Socket creation failed\n";
        return;
    }

    // Set the outgoing network interface for multicast
    struct in_addr localInterface;
    localInterface.s_addr = inet_addr(interfaceIp); // Specify the interface IP
    if (setsockopt(sockfd, IPPROTO_IP, IP_MULTICAST_IF, &localInterface, sizeof(localInterface)) < 0) {
        perror("setsockopt(IP_MULTICAST_IF) failed");
        close(sockfd);
        return;
    }

    // Set up multicast address
    memset(&multicastAddr, 0, sizeof(multicastAddr));
    multicastAddr.sin_family = AF_INET;
    multicastAddr.sin_port = htons(12345); // Multicast port
    multicastAddr.sin_addr.s_addr = inet_addr("239.255.255.250"); // Multicast group

    uint32_t data_id = 0; // Counter for data ID
    while (running) {
        // Create message with data ID and timestamp
        std::string timestamp = getCurrentTimestamp();
        std::ostringstream messageStream;
        messageStream << "ID:" << data_id << ";TS:" << timestamp;
        std::string message = messageStream.str();

        // Send the message
        sendto(sockfd, message.c_str(), message.size(), 0,
               (struct sockaddr*)&multicastAddr, sizeof(multicastAddr));
        std::cout << "Sent: " << message << " via interface " << interfaceIp << "\n";

        // Increment data ID and wait before sending the next packet
        ++data_id;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    close(sockfd);
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <interface-ip>\n";
        return 1;
    }

    const char* interfaceIp = argv[1]; // Get the interface IP from command line

    // Register signal handler for Ctrl+C
    signal(SIGINT, signalHandler);

    std::cout << "Starting multicast sender on interface " << interfaceIp << "...\n";

    // Start sending data in a separate thread
    std::thread senderThread(sendData, interfaceIp);

    // Wait for the thread to finish
    senderThread.join();

    std::cout << "Sender terminated.\n";
    return 0;
}