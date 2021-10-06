/**
 * Copyright (c) 2021 John Robinson.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

//Special thanks to https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
// for providing code to open the serial port.
#include <iostream>
#include <fstream> // For file I/O (reading/writing to COM port)
#include <sstream>
// #include <termios.h> // POSIX terminal control definitions (struct termios)
// #include <asm/termios.h> // Terminal control definitions (struct termios)
#include <vector>
//#include <asm/ioctls.h>
//#include <asm/termbits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <list>
#include <ctime>
#include <time.h>       /* time_t, struct tm, time, localtime, strftime */
#include <chrono>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>// write(), read(), close()
#endif

using namespace std;
using namespace std::chrono;

int openPort(const char* port, char* err) {
  int serial_port = open(port, O_RDWR);

  // Check for errors
  if (serial_port < 0) {
      sprintf(err, "Error %i from open: %s", errno, strerror(errno));
      return serial_port;
  }
  // Create new termios struct, we call it 'tty' for convention
  // No need for "= {0}" at the end as we'll immediately write the existing
  // config to this struct
  struct termios tty;

  // Read in existing settings, and handle any error
  // NOTE: This is important! POSIX states that the struct passed to tcsetattr()
  // must have been initialized with a call to tcgetattr() overwise behaviour
  // is undefined
  if(tcgetattr(serial_port, &tty) != 0) {
      sprintf(err, "Error %i from tcgetattr: %s", errno, strerror(errno));
  }

  //tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag |= PARENB;  // Set parity bit, enabling parity
  //tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag |= CSTOPB;  // Set stop field, two stop bits used in communication
  tty.c_cflag &= ~CSIZE; // Clear all the size bits, then use one of the statements below
  //tty.c_cflag |= CS5; // 5 bits per byte
  //tty.c_cflag |= CS6; // 6 bits per byte
  //tty.c_cflag |= CS7; // 7 bits per byte
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  //tty.c_cflag |= CRTSCTS;  // Enable RTS/CTS hardware flow control
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT IN LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT IN LINUX)
  tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;
  // Set in/out baud rate to be one of 
  //B0,  B50,  B75,  B110,  B134,  B150,  B200, B300, B600, B1200, B1800, B2400, 
  //B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);
  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
  }
  return serial_port;
}

int readPort(int serial_port, char* read_buff, int numChar){
  // Read bytes. The behaviour of read() (e.g. does it block?,
  // how long does it block for?) depends on the configuration
  // settings above, specifically VMIN and VTIME
  int n = read(serial_port, read_buff, numChar-1);
  read_buff[n] = 0;
  return n;
}

// this class assembles incoming characters into a list of "\n" dlimited lines
class lineFifo {
  list<string> lineList;
  char partial[256];

public:
  lineFifo() {
    //lineList = new std::list<string>;
    partial[0] = 0;
  }

  void addLines (char in[]) {
    int cn = strlen(partial);
    for (int i = 0; i < 256; i++) {
      if (in[i] == 0) break;
      if (in[i] == '\n') {
        partial[cn] = 0;
        lineList.push_front(string(partial));
        partial[0] = 0;
        cn = 0;
      } else if (cn == 253) { // prevent overflow
        partial[cn++] = in[i];
        partial[cn++] = '\n';
        partial[cn] = 0;
        lineList.push_front(string(partial));
        partial[0] = 0;
        cn = 0;
      } else {
        partial[cn++] = in[i];
      }
    }
    partial[cn] = 0;
  }

  bool empty(){
    return(lineList.empty());
  }

  string nextLine(){
    if (lineList.empty()) {
      return NULL;
    }
    string nextString = lineList.back();
    lineList.pop_back();
    return nextString;
  } 
};

  
int main(int argc, char* argv[]) {

  bool ts = true;
  bool of = false;
  string ofns;
  char ofnc[256];
  ofstream ofs;
#ifdef __APPLE__
  const char* d_port = "/dev/cu.usbmodem0000000000001";
#else
  const char* d_port = "/dev/ttyACM0";
#endif
  char port[256];
  char err[256], last_err[256];
  strcpy(port, d_port);

  for (int i = 1; i < argc; ++i) {
      if (0 == strcmp(argv[i], "-ts")) ts = true;
      if (0 == strcmp(argv[i], "-nts")) ts = false;
      if (0 == strcmp(argv[i], "-f")) {
          of = true;
          ofns = string(argv[i]);
	        strcpy(ofnc,argv[++i]);
      }
      if (0 == strcmp(argv[i], "-p")) {
          strcpy(port, argv[++i]);
      }
      if (0 == strcmp(argv[i], "-h")) {
        cout << "Usage:" << endl;
        cout << "Logger [-ts -nts] [-h] [-f <filename>] [-p <port>]" << endl;
        cout << "    -ts  add a time stamp to every lint printed (default)" << endl;
        cout << "    -nts do not add a timestamp" << endl;
        cout << "    -f <filename> send output to 'filename' " << endl;
        cout << "    -p <port> open specified port" << endl;
        cout << "    -h   print this message" << endl;
        cout << "Note: if no port is specified, the default port name is used" << endl;
        cout << "      on linux the default name is /dev/ttyACM0" << endl;
        cout << "      on the mac, the default name is  dev/cu.usbmodem0000000000001" << endl;
        return 0;
      }
  }

  if (of) {
      ofs.open(ofnc);
      if (!ofs.is_open()) {
          cout << "failed to open " << ofnc << "." << endl;
          exit(1);
      }
  }

  lineFifo* lf = new lineFifo();
  time_t rawtime;
  struct tm * timeinfo;
  char time_buf [80];
  time (&rawtime);
  timeinfo = localtime (&rawtime);
  strftime (time_buf,80,"%D %r",timeinfo);
  cout << "At " << time_buf << " waiting for port " << port << std::endl;
  while (1) {
      //Open the serial port
      int serial_port = -1;
      while (1) {
          for (int i = 0; i < 60; i++) {
              serial_port = openPort(port, err);
              if (serial_port > 0) break;
              else if (0 != strcmp(err, last_err)){
                cout << err << flush;
                strcpy(last_err, err);
              } else {
                if (i%1 == 0) cout << "." << flush;
              }
              sleep(1);
          }
          cout << endl;
          last_err[0] = 0;
          if (serial_port > 0) {
            break;
          } else {
              cout << "Waiting for port " << port << std::endl;
          }
      }

      cout << "Port " << port << " is open." << std::endl;
      
      high_resolution_clock::time_point last_time = high_resolution_clock::now();


      while (1) {
          char read_buff[256];
          int nc = readPort(serial_port, read_buff, 256);
          //printf(" %d, %s\n", nc, read_buff);
          if (nc > 0) {
              lf->addLines(read_buff);
              while (!lf->empty()) {
                  char dt[256];
                  if (ts) {
                    time (&rawtime);
                    timeinfo = localtime (&rawtime);
                    strftime (time_buf,80,"%D %r",timeinfo);
                    strcat(time_buf, ", \t");
                  } else{
                      strcpy(time_buf,"");
                  }
                  string s = lf->nextLine();
                  cout << time_buf << s << endl;
                  if (of) ofs << time_buf << s << endl;
              }
          }
          else if (nc == 0) {
            high_resolution_clock::time_point this_time = high_resolution_clock::now();
            duration<double> timeSpan = 
              duration_cast<duration<double>>(this_time - last_time);
            if (timeSpan.count() < 0.1) {
              close(serial_port);
              break;             
            } else { 
              last_time = this_time;
            } 
          } else if (nc < 0) {
            close(serial_port);
            break;
          }
      }
  }
}
