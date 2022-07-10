/* Copyright (C) 2012-2017 Ultraleap Limited. All rights reserved.
 *
 * <RELEASE-SPECIFIC-EULA>
 */

#define _WINSOCK_DEPRECATED_NO_WARNINGS

#include <stdio.h>
#include <winsock2.h>
#include <stdlib.h>
#include <math.h>

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

// Need to link with Ws2_32.lib, Mswsock.lib, and Advapi32.lib
#pragma comment (lib, "Ws2_32.lib")

#define DEFAULT_BUFLEN 512
#define DEFAULT_PORT "27015"

#include "LeapC.h"
#include "LeapConnection.h"

// Define PI
float pi;

// The last frame received
int64_t lastFrameID = 0; 

// Hand Position and Orientation Variable Initialization
//------------------------------------------------

struct velocity
{
    float x;
    float y;
    float z;
};

struct orientation
{
    float w;
    float x;
    float y;
    float z;
};

struct cartesian
{
    float x;
    float y;
    float z;
};

struct hand
{
    float pinch;
    float grab;
};

struct leapmotion
{
    struct cartesian position;
    struct orientation orientation;
    struct velocity velocity;
    struct hand hand;
};

struct leapmotion RightHand = { { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0.0, 0.0 };
struct leapmotion LeftHand = { { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, 0.0, 0.0 };

    // Hand Position
        // Hand.position.x
        // Hand.position.y
        // Hand.position.z

    // Hand Orientation
        // Hand.orientation.u
        // Hand.orientation.v
        // Hand.orientation.w

    // Hand Velocity
        // Hand.velocity.x
        // Hand.velocity.y 
        // Hand.velocity.z

//------------------------------------------------

//------------------------------------------------
int float_to_int(float fl)
{
    int in = (int)(fl);
    return in;
}

char int_to_byte_array(int in)
{
    char byte[5];
    byte[0] = (in >> 32) & 0xFF;
    byte[1] = (in >> 24) & 0xFF;
    byte[2] = (in >> 16) & 0xFF;
    byte[3] = (in >> 8) & 0xFF;
    byte[4] = (in >> 0) & 0xFF;
    return byte;
}
//------------------------------------------------

//------------------------------------------------

void checkHostName(int hostname)
{
    if (hostname == -1)
    {
        perror("gethostname");
        exit(1);
    }
}

// Returns host information corresponding to host name
void checkHostEntry(struct hostent* hostentry)
{
    if (hostentry == NULL)
    {
        perror("gethostbyname");
        exit(1);
    }
}

// Converts space-delimited IPv4 addresses
// to dotted-decimal format
void checkIPbuffer(char* IPbuffer)
{
    if (NULL == IPbuffer)
    {
        perror("inet_ntoa");
        exit(1);
    }
}

//------------------------------------------------

// Hand Position/Orientation/Gesture Data Acquisition Loop
//------------------------------------------------

    int main(int argc, char** argv) 
    {
    //unsigned short int a, b, c, d;
    //char IPAddress[256];
    //int ip[4];

    signed char data[92] = { '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00', '0x00' };

    // IPAddress Request
    //-------------------------------------------------------------
    char hostbuffer[256];
    char* IPbuffer;
    struct hostent* host_entry;
    int hostname;

    // To retrieve hostname
    hostname = gethostname(hostbuffer, sizeof(hostbuffer));
    checkHostName(hostname);

    // To retrieve host information
    host_entry = gethostbyname(hostbuffer);
    checkHostEntry(host_entry);

    // To convert an Internet network
    // address into ASCII string
    IPbuffer = inet_ntoa(*((struct in_addr*)host_entry->h_addr_list[0]));

    printf("Host IP: %s", IPbuffer);
    //-------------------------------------------------------------


    // Data Socket
    //-------------------------------------------------------------
    
    
    WSADATA wsa;
    SOCKET s;
    struct sockaddr_in server;
    char* message;

    printf("\nInitialising Winsock...");
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
    {
        printf("Failed. Error Code : %d", WSAGetLastError());
        return 1;
    }

    printf("Initialised.\n");

    //Create a socket
    if ((s = socket(AF_INET, SOCK_STREAM, 0)) == INVALID_SOCKET)
    {
        printf("Could not create socket : %d", WSAGetLastError());
    }

    printf("Socket created.\n");

    //192.168.56.1
    //192.168.1.254

    server.sin_addr.s_addr = inet_addr(IPbuffer);
    server.sin_family = AF_INET;
    server.sin_port = htons(2048);

    //Connect to remote server
    if (connect(s, (struct sockaddr*)&server, sizeof(server)) < 0)
    {
        puts("connect error");
        return 1;
    }

    puts("Connected");
    

    //----------------------------------------------------------------------

    //Leap Motion Data Aquisition
    //----------------------------------------------------------------------
    
    // Connect to Leap Motion
    OpenConnection();

    // Let Connection Complete Before Running Data Acquisition Loop
    while(!IsConnected)
    millisleep(100);

    printf("Connected.");

    LEAP_DEVICE_INFO* deviceProps = GetDeviceProperties();

    if(deviceProps)
    printf("Using device %s.\n", deviceProps->serial);

      for(;;)
      {
        LEAP_TRACKING_EVENT *frame = GetFrame();

        if(frame && (frame->tracking_frame_id > lastFrameID))
        {
          lastFrameID = frame->tracking_frame_id;

          //printf("Frame %lli with %i hands.\n", (long long int)frame->tracking_frame_id, frame->nHands);

          for(uint32_t h = 0; h < frame->nHands; h++)
          {
            LEAP_HAND* hand = &frame->pHands[h];

            // Right Hand
            if (h == 0)
            {
                RightHand.position.x = hand->palm.position.x;
                RightHand.position.y = hand->palm.position.y;
                RightHand.position.z = hand->palm.position.z;
                RightHand.orientation.w = hand->palm.orientation.w;
                RightHand.orientation.x = hand->palm.orientation.x;
                RightHand.orientation.y = hand->palm.orientation.y;
                RightHand.orientation.z = hand->palm.orientation.z;
                RightHand.velocity.x = hand->palm.velocity.x;
                RightHand.velocity.y = hand->palm.velocity.y;
                RightHand.velocity.z = hand->palm.velocity.z;
                RightHand.hand.pinch = hand->pinch_strength;
                RightHand.hand.grab = hand->grab_strength;
            }

            // Left Hand
            if (h == 1)
            {
                LeftHand.position.x = hand->palm.position.x;
                LeftHand.position.y = hand->palm.position.y;
                LeftHand.position.z = hand->palm.position.z;
                LeftHand.orientation.w = hand->palm.orientation.w;
                LeftHand.orientation.x = hand->palm.orientation.x;
                LeftHand.orientation.y = hand->palm.orientation.y;
                LeftHand.orientation.z = hand->palm.orientation.z;
                LeftHand.velocity.x = hand->palm.velocity.x;
                LeftHand.velocity.y = hand->palm.velocity.y;
                LeftHand.velocity.z = hand->palm.velocity.z;
                LeftHand.hand.pinch = hand->pinch_strength;
                LeftHand.hand.grab = hand->grab_strength;
            }
          }
          if (RightHand.position.x / abs(RightHand.position.x) > 0)
          {
              data[0] = (0) & 0xFF;
          }
          if (RightHand.position.x / abs(RightHand.position.x) < 0)
          {
              data[0] = (1) & 0xFF;
          }
          data[1] = ((int)(abs(RightHand.position.x * 10)) >> 16) & 0xFF;
          data[2] = ((int)(abs(RightHand.position.x * 10)) >> 8) & 0xFF;
          data[3] = ((int)(abs(RightHand.position.x * 10)) >> 0) & 0xFF;

          if (RightHand.position.y / abs(RightHand.position.y) > 0)
          {
              data[4] = (0) & 0xFF;
          }
          if (RightHand.position.y / abs(RightHand.position.y) < 0)
          {
              data[4] = (1) & 0xFF;
          }
          data[5] = ((int)(abs(RightHand.position.y * 10)) >> 16) & 0xFF;
          data[6] = ((int)(abs(RightHand.position.y * 10)) >> 8) & 0xFF;
          data[7] = ((int)(abs(RightHand.position.y * 10)) >> 0) & 0xFF;

          if (RightHand.position.z / abs(RightHand.position.z) > 0)
          {
              data[8] = (0) & 0xFF;
          }
          if (RightHand.position.z / abs(RightHand.position.z) < 0)
          {
              data[8] = (1) & 0xFF;
          }
          data[9] = ((int)(abs(RightHand.position.z * 10)) >> 16) & 0xFF;
          data[10] = ((int)(abs(RightHand.position.z * 10)) >> 8) & 0xFF;
          data[11] = ((int)(abs(RightHand.position.z * 10)) >> 0) & 0xFF;

          if (RightHand.orientation.x / abs(RightHand.orientation.x) > 0)
          {
              data[12] = (0) & 0xFF;
          }
          if (RightHand.orientation.x / abs(RightHand.orientation.x) < 0)
          {
              data[12] = (1) & 0xFF;
          }
          data[13] = ((int)(abs(RightHand.orientation.x * 1000)) >> 16) & 0xFF;
          data[14] = ((int)(abs(RightHand.orientation.x * 1000)) >> 8) & 0xFF;
          data[15] = ((int)(abs(RightHand.orientation.x * 1000)) >> 0) & 0xFF;

          if (RightHand.orientation.y / abs(RightHand.orientation.y) > 0)
          {
              data[16] = (0) & 0xFF;
          }
          if (RightHand.orientation.y / abs(RightHand.orientation.y) < 0)
          {
              data[16] = (1) & 0xFF;
          }
          data[17] = ((int)(abs(RightHand.orientation.y * 1000)) >> 16) & 0xFF;
          data[18] = ((int)(abs(RightHand.orientation.y * 1000)) >> 8) & 0xFF;
          data[19] = ((int)(abs(RightHand.orientation.y * 1000)) >> 0) & 0xFF;

          if (RightHand.orientation.z / abs(RightHand.orientation.z) > 0)
          {
              data[20] = (0) & 0xFF;
          }
          if (RightHand.orientation.z / abs(RightHand.orientation.z) < 0)
          {
              data[20] = (1) & 0xFF;
          }
          data[21] = ((int)(abs(RightHand.orientation.z * 1000)) >> 16) & 0xFF;
          data[22] = ((int)(abs(RightHand.orientation.z * 1000)) >> 8) & 0xFF;
          data[23] = ((int)(abs(RightHand.orientation.z * 1000)) >> 0) & 0xFF;

          if (RightHand.velocity.x / abs(RightHand.velocity.x) > 0)
          {
              data[24] = (0) & 0xFF;
          }
          if (RightHand.velocity.x / abs(RightHand.velocity.x) < 0)
          {
              data[24] = (1) & 0xFF;
          }
          data[25] = ((int)(abs(RightHand.velocity.x * 10)) >> 16) & 0xFF;
          data[26] = ((int)(abs(RightHand.velocity.x * 10)) >> 8) & 0xFF;
          data[27] = ((int)(abs(RightHand.velocity.x * 10)) >> 0) & 0xFF;

          if (RightHand.velocity.y / abs(RightHand.velocity.y) > 0)
          {
              data[28] = (0) & 0xFF;
          }
          if (RightHand.velocity.y / abs(RightHand.velocity.y) < 0)
          {
              data[28] = (1) & 0xFF;
          }
          data[29] = ((int)(abs(RightHand.velocity.y * 10)) >> 16) & 0xFF;
          data[30] = ((int)(abs(RightHand.velocity.y * 10)) >> 8) & 0xFF;
          data[31] = ((int)(abs(RightHand.velocity.y * 10)) >> 0) & 0xFF;

          if (RightHand.velocity.z / abs(RightHand.velocity.z) > 0)
          {
              data[32] = (0) & 0xFF;
          }
          if (RightHand.velocity.z / abs(RightHand.velocity.z) < 0)
          {
              data[32] = (1) & 0xFF;
          }
          data[33] = ((int)(abs(RightHand.velocity.z * 10)) >> 16) & 0xFF;
          data[34] = ((int)(abs(RightHand.velocity.z * 10)) >> 8) & 0xFF;
          data[35] = ((int)(abs(RightHand.velocity.z * 10)) >> 0) & 0xFF;

          if (LeftHand.position.x / abs(LeftHand.position.x) > 0)
          {
              data[36] = (0) & 0xFF;
          }
          if (LeftHand.position.x / abs(LeftHand.position.x) < 0)
          {
              data[36] = (1) & 0xFF;
          }
          data[37] = ((int)(abs(LeftHand.position.x * 10)) >> 16) & 0xFF;
          data[38] = ((int)(abs(LeftHand.position.x * 10)) >> 8) & 0xFF;
          data[39] = ((int)(abs(LeftHand.position.x * 10)) >> 0) & 0xFF;

          if (LeftHand.position.y / abs(LeftHand.position.y) > 0)
          {
              data[40] = (0) & 0xFF;
          }
          if (LeftHand.position.y / abs(LeftHand.position.y) < 0)
          {
              data[40] = (1) & 0xFF;
          }
          data[41] = ((int)(abs(LeftHand.position.y * 10)) >> 16) & 0xFF;
          data[42] = ((int)(abs(LeftHand.position.y * 10)) >> 8) & 0xFF;
          data[43] = ((int)(abs(LeftHand.position.y * 10)) >> 0) & 0xFF;

          if (LeftHand.position.z / abs(LeftHand.position.z) > 0)
          {
              data[44] = (0) & 0xFF;
          }
          if (LeftHand.position.z / abs(LeftHand.position.z) < 0)
          {
              data[44] = (1) & 0xFF;
          }
          data[45] = ((int)(abs(LeftHand.position.z * 10)) >> 16) & 0xFF;
          data[46] = ((int)(abs(LeftHand.position.z * 10)) >> 8) & 0xFF;
          data[47] = ((int)(abs(LeftHand.position.z * 10)) >> 0) & 0xFF;

          if (LeftHand.orientation.x / abs(LeftHand.orientation.x) > 0)
          {
              data[48] = (0) & 0xFF;
          }
          if (LeftHand.orientation.x / abs(LeftHand.orientation.x) < 0)
          {
              data[48] = (1) & 0xFF;
          }
          data[49] = ((int)(abs(LeftHand.orientation.x * 1000)) >> 16) & 0xFF;
          data[50] = ((int)(abs(LeftHand.orientation.x * 1000)) >> 8) & 0xFF;
          data[51] = ((int)(abs(LeftHand.orientation.x * 1000)) >> 0) & 0xFF;

          if (LeftHand.orientation.y / abs(LeftHand.orientation.y) > 0)
          {
              data[52] = (0) & 0xFF;
          }
          if (LeftHand.orientation.y / abs(LeftHand.orientation.y) < 0)
          {
              data[52] = (1) & 0xFF;
          }
          data[53] = ((int)(abs(LeftHand.orientation.y * 1000)) >> 16) & 0xFF;
          data[54] = ((int)(abs(LeftHand.orientation.y * 1000)) >> 8) & 0xFF;
          data[55] = ((int)(abs(LeftHand.orientation.y * 1000)) >> 0) & 0xFF;

          if (LeftHand.orientation.z / abs(LeftHand.orientation.z) > 0)
          {
              data[56] = (0) & 0xFF;
          }
          if (LeftHand.orientation.z / abs(LeftHand.orientation.z) < 0)
          {
              data[56] = (1) & 0xFF;
          }
          data[57] = ((int)(abs(LeftHand.orientation.z * 1000)) >> 16) & 0xFF;
          data[58] = ((int)(abs(LeftHand.orientation.z * 1000)) >> 8) & 0xFF;
          data[59] = ((int)(abs(LeftHand.orientation.z * 1000)) >> 0) & 0xFF;

          if (LeftHand.velocity.x / abs(LeftHand.velocity.x) > 0)
          {
              data[60] = (0) & 0xFF;
          }
          if (LeftHand.velocity.x / abs(LeftHand.velocity.x) < 0)
          {
              data[60] = (1) & 0xFF;
          }
          data[61] = ((int)(abs(LeftHand.velocity.x * 10)) >> 16) & 0xFF;
          data[62] = ((int)(abs(LeftHand.velocity.x * 10)) >> 8) & 0xFF;
          data[63] = ((int)(abs(LeftHand.velocity.x * 10)) >> 0) & 0xFF;

          if (LeftHand.velocity.y / abs(LeftHand.velocity.y) > 0)
          {
              data[64] = (0) & 0xFF;
          }
          if (LeftHand.velocity.y / abs(LeftHand.velocity.y) < 0)
          {
              data[64] = (1) & 0xFF;
          }
          data[65] = ((int)(abs(LeftHand.velocity.y * 10)) >> 16) & 0xFF;
          data[66] = ((int)(abs(LeftHand.velocity.y * 10)) >> 8) & 0xFF;
          data[67] = ((int)(abs(LeftHand.velocity.y * 10)) >> 0) & 0xFF;

          if (LeftHand.velocity.z / abs(LeftHand.velocity.z) > 0)
          {
              data[68] = (0) & 0xFF;
          }
          if (LeftHand.velocity.z / abs(LeftHand.velocity.z) < 0)
          {
              data[68] = (1) & 0xFF;
          }
          data[69] = ((int)(abs(LeftHand.velocity.z * 10)) >> 16) & 0xFF;
          data[70] = ((int)(abs(LeftHand.velocity.z * 10)) >> 8) & 0xFF;
          data[71] = ((int)(abs(LeftHand.velocity.z * 10)) >> 0) & 0xFF;

          data[72] = ((int)(abs(RightHand.hand.pinch * 1000)) >> 16) & 0xFF;
          data[73] = ((int)(abs(RightHand.hand.pinch * 1000)) >> 8) & 0xFF;
          data[74] = ((int)(abs(RightHand.hand.pinch * 1000)) >> 0) & 0xFF;

          data[75] = ((int)(abs(RightHand.hand.grab * 1000)) >> 16) & 0xFF;
          data[76] = ((int)(abs(RightHand.hand.grab * 1000)) >> 8) & 0xFF;
          data[77] = ((int)(abs(RightHand.hand.grab * 1000)) >> 0) & 0xFF;

          data[78] = ((int)(abs(LeftHand.hand.pinch * 1000)) >> 16) & 0xFF;
          data[79] = ((int)(abs(LeftHand.hand.pinch * 1000)) >> 8) & 0xFF;
          data[80] = ((int)(abs(LeftHand.hand.pinch * 1000)) >> 0) & 0xFF;

          data[81] = ((int)(abs(LeftHand.hand.grab * 1000)) >> 16) & 0xFF;
          data[82] = ((int)(abs(LeftHand.hand.grab * 1000)) >> 8) & 0xFF;
          data[83] = ((int)(abs(LeftHand.hand.grab * 1000)) >> 0) & 0xFF;

          if (RightHand.orientation.w / abs(RightHand.orientation.w) > 0)
          {
              data[84] = (0) & 0xFF;
          }
          if (RightHand.orientation.w / abs(RightHand.orientation.w) < 0)
          {
              data[84] = (1) & 0xFF;
          }
          data[85] = ((int)(abs(RightHand.orientation.w * 1000)) >> 16) & 0xFF;
          data[86] = ((int)(abs(RightHand.orientation.w * 1000)) >> 8) & 0xFF;
          data[87] = ((int)(abs(RightHand.orientation.w * 1000)) >> 0) & 0xFF;

          if (LeftHand.orientation.w / abs(LeftHand.orientation.w) > 0)
          {
              data[88] = (0) & 0xFF;
          }
          if (LeftHand.orientation.w / abs(LeftHand.orientation.w) < 0)
          {
              data[88] = (1) & 0xFF;
          }
          data[89] = ((int)(abs(LeftHand.orientation.w * 1000)) >> 16) & 0xFF;
          data[90] = ((int)(abs(LeftHand.orientation.w * 1000)) >> 8) & 0xFF;
          data[91] = ((int)(abs(LeftHand.orientation.w * 1000)) >> 0) & 0xFF;
            
          //Send Data
          send(s, data, sizeof(data), 0);
          
        }
      } //ctrl-c to exit
      return 0;
    }
//------------------------------------------------


