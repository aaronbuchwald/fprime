#include <getopt.h>
#include <stdlib.h>
#include <ctype.h>

#include <GpsApp/Top/Components.hpp>

void print_usage() {
    (void) printf("Usage: ./GpsApp [options]\n-p\tport_number\n-a\thostname/IP address\n");
}

#include <signal.h>
#include <stdio.h>

volatile sig_atomic_t terminate = 0;

static void sighandler(int signum) {
    exitTasks();
    terminate = 1;
}

int main(int argc, char* argv[]) {
    U32 port_number;
    I32 option;
    char *hostname;
    port_number = 0;
    option = 0;
    hostname = NULL;

    while ((option = getopt(argc, argv, "hp:a:")) != -1){
        switch(option) {
            case 'h':
                print_usage();
                return 0;
                break;
            case 'p':
                port_number = atoi(optarg);
                break;
            case 'a':
                hostname = optarg;
                break;
            case '?':
                return 1;
            default:
                print_usage();
                return 1;
        }
    }

    (void) printf("Hit Ctrl-C to quit\n");

    constructApp(port_number, hostname);

    // register signal handlers to exit program
    signal(SIGINT,sighandler);
    signal(SIGTERM,sighandler);

    // Give time for threads to exit
    (void) printf("Waiting for threads...\n");
    Os::Task::delay(1000);
    while(1) {
        //GPS-- Given the application's lack of a specific timing element, we
        //      force a call to the rate group driver every second here.
        //      More complex applications may drive this from a system oscillator.
        Svc::TimerVal timer;
        timer.take();
        rateGroupDriverComp.get_CycleIn_InputPort(0)->invoke(timer);
        Os::Task::TaskStatus delayStat = Os::Task::delay(1000);
        FW_ASSERT(Os::Task::TASK_OK == delayStat,delayStat);
    }
    (void) printf("Exiting...\n");

    return 0;
}