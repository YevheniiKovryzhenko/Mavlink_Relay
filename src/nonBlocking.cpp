#include "nonBlocking.hpp"
#include <stdio.h> 
#include <string.h> 
#include <iostream>
#include <string>
#include <sstream>
#include <vector>

enum word_parsing_defines
{
    unknown,
    vehicle,
    arm,
    disarm
};

using namespace std;

string input;


//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                                                                          //
//                         Non-blocking CLI                                 //
//                                                                          //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////
bool _kbhit()
{
    termios term;
    tcgetattr(0, &term);

    termios term2 = term;
    term2.c_lflag &= ~ICANON;
    tcsetattr(0, TCSANOW, &term2);

    int byteswaiting;
    ioctl(0, FIONREAD, &byteswaiting);

    tcsetattr(0, TCSANOW, &term);

    return byteswaiting > 0;
}

vector<string> splitString(const string& str) 
{
    istringstream iss(str);
    vector<string> tokens;
    string token;
    while (iss >> token) {
        if (!token.empty()) {
            tokens.push_back(token);
        }
    }
    return tokens;
}

bool pressed_enter(char in)
{
    return (in == '\n' || in == '\r');
}

word_parsing_defines matched_str(string &buff)
{
    if (buff.compare("vehicle") == 0) return vehicle;
    else if (buff.compare("arm") == 0) return arm;
    else if (buff.compare("disarm") == 0) return disarm;
    else return unknown;
}

int non_blocking_client_update(Autopilot_Interface* cli) {
    if (_kbhit()) {
        char c = getchar();
        if (pressed_enter(c))
        {
            //printf("pressed ENTER!\n");

            //printf("User input %lu chars: \t %s\n", input.size(), input.c_str());

            vector<string> words = splitString(input);
            
            for (size_t i = 0; i < words.size(); i++) 
            {
                switch (matched_str(words[i]))
                {
                case vehicle:
                    if (cli->settings.enable_target && words.size() > i+1)
                    {
                        switch (matched_str(words[i + 1]))
                        {
                            case arm:
                            printf("Trying to arm the vehicle...\n");
                            cli->arm_disarm(true);
                            input.clear();
                            return 0;

                        case disarm:
                            printf("Trying to disarm the vehicle...\n");
                            cli->arm_disarm(false);
                            input.clear();
                            return 0;
                        
                        default:
                            printf("Unrecognized command!\n");
                            input.clear();
                            return 0;
                        }
                    }
                    else
                    {
                        printf("Error: must enable target for vehicle commands to be active and specify command\n");
                        input.clear();
                        return 0;
                    }
                                    

                default:
                    printf("Unrecognized command!\n");
                    input.clear();
                    return 0;
                }
            }
            

            input.clear();
        }
        else
        {
           input.push_back(c);
        }
    }
    return 0;
}