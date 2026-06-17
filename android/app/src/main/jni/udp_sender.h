#ifndef UDP_SENDER_H
#define UDP_SENDER_H

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <android/log.h>

class UdpSender
{
public:
    UdpSender() : sock_fd(-1), running(false), has_new_data(false), packet_len(0)
    {
        pthread_mutex_init(&mtx, NULL);
        pthread_cond_init(&cond, NULL);
    }

    ~UdpSender()
    {
        stop();
        pthread_mutex_destroy(&mtx);
        pthread_cond_destroy(&cond);
    }

    bool start(const char* ip, int port)
    {
        if (running) return true;

        sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_fd < 0)
        {
            __android_log_print(ANDROID_LOG_ERROR, "UdpSender", "Failed to create socket");
            return false;
        }

        memset(&dest_addr, 0, sizeof(dest_addr));
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(port);
        inet_pton(AF_INET, ip, &dest_addr.sin_addr);

        running = true;
        has_new_data = false;
        pthread_create(&sender_thread, NULL, thread_func, this);

        __android_log_print(ANDROID_LOG_INFO, "UdpSender", "Started: %s:%d", ip, port);
        return true;
    }

    void stop()
    {
        if (!running) return;

        pthread_mutex_lock(&mtx);
        running = false;
        pthread_cond_signal(&cond);
        pthread_mutex_unlock(&mtx);

        pthread_join(sender_thread, NULL);

        if (sock_fd >= 0)
        {
            close(sock_fd);
            sock_fd = -1;
        }

        __android_log_print(ANDROID_LOG_INFO, "UdpSender", "Stopped");
    }

    // Non-blocking: overwrites previous packet if not yet sent
    void send_packet(const char* data, int len)
    {
        if (!running || len <= 0 || len > (int)sizeof(packet_buf)) return;

        pthread_mutex_lock(&mtx);
        memcpy(packet_buf, data, len);
        packet_len = len;
        has_new_data = true;
        pthread_cond_signal(&cond);
        pthread_mutex_unlock(&mtx);
    }

    bool is_running() const { return running; }

private:
    int sock_fd;
    struct sockaddr_in dest_addr;
    pthread_t sender_thread;
    pthread_mutex_t mtx;
    pthread_cond_t cond;

    char packet_buf[512];
    int packet_len;
    volatile bool has_new_data;
    volatile bool running;

    static void* thread_func(void* arg)
    {
        UdpSender* self = (UdpSender*)arg;
        char send_buf[512];
        int send_len;

        while (true)
        {
            pthread_mutex_lock(&self->mtx);
            while (self->running && !self->has_new_data)
            {
                pthread_cond_wait(&self->cond, &self->mtx);
            }

            if (!self->running)
            {
                pthread_mutex_unlock(&self->mtx);
                break;
            }

            memcpy(send_buf, self->packet_buf, self->packet_len);
            send_len = self->packet_len;
            self->has_new_data = false;
            pthread_mutex_unlock(&self->mtx);

            sendto(self->sock_fd, send_buf, send_len, 0,
                   (struct sockaddr*)&self->dest_addr, sizeof(self->dest_addr));
        }

        return NULL;
    }
};

#endif // UDP_SENDER_H
