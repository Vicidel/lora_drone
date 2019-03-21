#include "serial.h"

int main(int argc, char **argv) {
    int fd = -1;
    setup_usart(fd);
    run(fd);
    
    return 0;
}
