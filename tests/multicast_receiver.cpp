#include <iostream>
#include <thread>
#include <atomic>
#include <csignal>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <chrono>
#include <sstream>

std::atomic<bool> running(true);

void signalHandler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\nCtrl+C detected. Shutting down receiver...\n";
        running = false;
    }
}

// Function to parse the received message
void parseMessage(const std::string& message) {
    size_t idPos = message.find("ID:");
    size_t tsPos = message.find("TS:");

    if (idPos != std::string::npos && tsPos != std::string::npos) {
        // Extract data ID
        uint32_t data_id = std::stoul(message.substr(idPos + 3, tsPos - (idPos + 3)));

        // Extract sent timestamp
        uint64_t sent_timestamp = std::stoull(message.substr(tsPos + 3));

        // Get current timestamp
        auto now = std::chrono::system_clock::now();
        auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
        auto epoch = now_ms.time_since_epoch();
        auto received_timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(epoch).count();

        // Calculate time lag
        uint64_t time_lag = received_timestamp - sent_timestamp;

        // Print parsed data
        std::cout << "Received: Data ID = " << data_id
                  << ", Sent Timestamp = " << sent_timestamp
                  << ", Received Timestamp = " << received_timestamp
                  << ", Time Lag = " << time_lag << " ms\n";
    } else {
        std::cerr << "Invalid message format: " << message << "\n";
    }
}

void receiveData(const char* interfaceIp) {
    int sockfd;
    struct sockaddr_in localAddr, multicastAddr;
    char buffer[1024];

    // Create socket
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        std::cerr << "Socket creation failed\n";
        return;
    }

    // Bind to local address
    memset(&localAddr, 0, sizeof(localAddr));
    localAddr.sin_family = AF_INET;
    localAddr.sin_port = htons(12345); // Multicast port
    localAddr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(sockfd, (struct sockaddr*)&localAddr, sizeof(localAddr)) < 0) {
        perror("Bind failed");
        close(sockfd);
        return;
    }

    // Join multicast group on the specified interface
    struct ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = inet_addr("239.255.255.250"); // Multicast group
    mreq.imr_interface.s_addr = inet_addr(interfaceIp);       // Specify the interface IP

    if (setsockopt(sockfd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) {
        perror("Failed to join multicast group");
        close(sockfd);
        return;
    }

    std::cout << "Receiving multicast data on interface " << interfaceIp << "...\n";

    while (running) {
        socklen_t addrLen = sizeof(multicastAddr);
        ssize_t recvLen = recvfrom(sockfd, buffer, sizeof(buffer) - 1, 0,
                                   (struct sockaddr*)&multicastAddr, &addrLen);
        if (recvLen > 0) {
            buffer[recvLen] = '\0';
            parseMessage(buffer);
        } else if (recvLen < 0) {
            perror("recvfrom() failed");
        }
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

    std::cout << "Starting multicast receiver on interface " << interfaceIp << "...\n";

    // Start receiving data in a separate thread
    std::thread receiverThread(receiveData, interfaceIp);

    // Wait for the thread to finish
    receiverThread.join();

    std::cout << "Receiver terminated.\n";
    return 0;
}