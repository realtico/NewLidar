#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <string.h>
#include <termios.h>
#include <errno.h>

#define FIFO_PATH "/tmp/lidarpipe"
#define DEFAULT_SERIAL_PORT "/dev/ttyUSB0"
#define DEFAULT_MAX_POINTS 1000

long long current_millis() {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return ts.tv_sec * 1000LL + ts.tv_nsec / 1000000;
}

int configure_serial(int fd) {
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr");
        return -1;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        return -1;
    }

    return 0;
}

int read_byte(int fd) {
    unsigned char c;
    int n = read(fd, &c, 1);
    return (n == 1) ? c : -1;
}

int main(int argc, char *argv[]) {
    const char *serial_port = DEFAULT_SERIAL_PORT;
    int max_points = DEFAULT_MAX_POINTS;

    if (argc > 1) serial_port = argv[1];
    if (argc > 2) max_points = atoi(argv[2]);
    if (max_points <= 0 || max_points > 10000) max_points = DEFAULT_MAX_POINTS;

    int serial_fd = open(serial_port, O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd < 0) {
        perror("open serial\n");
        return 1;
    }

    if (configure_serial(serial_fd) != 0) {
        close(serial_fd);
        return 1;
    }

    int fifo_fd = open(FIFO_PATH, O_WRONLY);
    if (fifo_fd < 0) {
        perror("open fifo");
        close(serial_fd);
        return 1;
    }

    float *angles = malloc(sizeof(float) * max_points);
    int *dists = malloc(sizeof(int) * max_points);
    if (!angles || !dists) {
        perror("malloc");
        close(fifo_fd);
        close(serial_fd);
        return 1;
    }

    int count = 0;
    int frame_id = 0;


    while (1) {
        int c = read_byte(serial_fd);
        if (c != 0xAA) continue;
        if (read_byte(serial_fd) != 0x55) continue;

        int CT = read_byte(serial_fd);
        int LSN = read_byte(serial_fd);
        int FSA = read_byte(serial_fd) | (read_byte(serial_fd) << 8);
        int LSA = read_byte(serial_fd) | (read_byte(serial_fd) << 8);
        read_byte(serial_fd);  // CS (ignore)
        read_byte(serial_fd);

        float F = (FSA >> 1) / 64.0;
        float L = (LSA >> 1) / 64.0;

        for (int i = 0; i < LSN; ++i) {
            int Si = read_byte(serial_fd) | (read_byte(serial_fd) << 8);
            int dist = Si >> 2;
            float A_corr = (dist == 0) ? 0.0 : atan(19.16 * (dist - 90.15) / (dist * 90.15));
            float angle = F + ((L - F) / LSN) * i - A_corr;

            if (count < max_points) {
                angles[count] = angle;
                dists[count] = dist;
                count++;
            }
        }

        if (CT == 1) {
            for (int i = 0; i < count; ++i) {
                dprintf(fifo_fd, "%.2f,%d\n", angles[i], dists[i]);
                //printf("%.2f,%d\n", angles[i], dists[i]);
            }
            long long ts = current_millis();
            frame_id++;
            dprintf(fifo_fd, "NEWFRAME %lld\n", ts);
            //printf("[Reader] Frame %d enviado em %lld ms\n", frame_id, ts);
            //printf("NEWFRAME %lld\n", ts);
            fsync(fifo_fd);
            count = 0;
        }
    }

    free(angles);
    free(dists);
    close(fifo_fd);
    close(serial_fd);
    return 0;
}
