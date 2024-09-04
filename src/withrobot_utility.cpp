
#include "withrobot_utility.hpp"

using namespace Withrobot;

/*
 * References: https://www.kernel.org/pub/linux/utils/kernel/hotplug/libudev/ch01.html, http://www.signal11.us/oss/udev/
 */
int Withrobot::get_usb_device_info_list(std::vector<usb_device_info>& info_list)
{
    info_list.clear();

    struct udev* my_device;
    my_device = udev_new();
    if (!my_device) {
        printf("Can't create udev\n");
        return false;
    }

    struct udev_enumerate *enumerate;
    struct udev_list_entry *devices;
    struct udev_list_entry *dev_list_entry;

    struct usb_device_info dev_info;
    int num_dev = 0;

    /* Create a list of the devices in the 'v4l2' subsystem. */
    enumerate = udev_enumerate_new(my_device);
    udev_enumerate_add_match_subsystem(enumerate, "video4linux");
    udev_enumerate_scan_devices(enumerate);
    devices = udev_enumerate_get_list_entry(enumerate);

    udev_list_entry_foreach(dev_list_entry, devices)
    {
        const char *path;

        path = udev_list_entry_get_name(dev_list_entry);
        struct udev_device *dev = udev_device_new_from_syspath(my_device, path);
        const char* dev_node = udev_device_get_devnode(dev);
        dev_info.dev_node = dev_node;

        dev = udev_device_get_parent_with_subsystem_devtype(dev, "usb", "usb_device");
        if (!dev)
        {
            fprintf(stderr, "V4L2_CORE: Unable to find parent usb device [ %s ].\n", dev_node);
            continue;
        }

        num_dev++;

        const char* id_vendor = udev_device_get_sysattr_value(dev, "idVendor");
        if (id_vendor) {
            dev_info.id_vendor = id_vendor;
        }
        const char* id_product = udev_device_get_sysattr_value(dev, "idProduct");
        if (id_product) {
            dev_info.id_product = id_product;
        }
        const char* manufacturer = udev_device_get_sysattr_value(dev, "manufacturer");
        if (manufacturer) {
            dev_info.manufacturer = manufacturer;
        }
        const char* product = udev_device_get_sysattr_value(dev, "product");
        if (product) {
            dev_info.product = product;
        }
        const char* serial = udev_device_get_sysattr_value(dev, "serial");
        if (serial) {
            dev_info.serial = serial;
        }
        const char* busnum = udev_device_get_sysattr_value(dev, "busnum");
        if (busnum) {
            dev_info.busnum = busnum;
        }
        const char* devnum = udev_device_get_sysattr_value(dev, "devnum");
        if (devnum) {
            dev_info.devnum = devnum;
        }
        info_list.push_back(dev_info);
        dev_info.clear();

        udev_device_unref(dev);
    }
    /* Free the enumerator object */
    udev_enumerate_unref(enumerate);

    return num_dev;
}


Thread::Thread() : id(0), thread(0)
{
}

Thread::~Thread()
{
    join();
}

bool Thread::start(void*(*thread_proc)(void*), void* arg)
{
    id = pthread_create(&thread, NULL, thread_proc, (void*) arg);
    if (id != 0) {
        thread = 0;
    }
    return (id == 0);
}

void Thread::join()
{
    if (thread != 0) {
        pthread_join(thread, NULL);
    }
    thread = 0;
}

/*
 * pthread mutex wrapper
 */
Mutex::Mutex()
{
//	mutex = new pthread_mutex_t PTHREAD_MUTEX_INITIALIZER;
    char str_res[16];

    int res = pthread_mutexattr_init(&attr);
    if(res != 0) {
        sprintf(str_res, "%d", res);
        throw (WithRobotException("pthread_mutexattr_init returns " + std::string(str_res)));
    }

    res = pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    if(res != 0) {
        sprintf(str_res, "%d", res);
        throw (WithRobotException("pthread_mutexattr_settype returns " + std::string(str_res)));
    }

    res = pthread_mutex_init (&mutex, &attr);
    if(res != 0) {
        sprintf(str_res, "%d", res);
        throw (WithRobotException("pthread_mutex_init returns " + std::string(str_res)));
    }

    res = pthread_mutexattr_destroy(&attr);
    if(res != 0) {
        sprintf(str_res, "%d", res);
        throw (WithRobotException("pthread_mutexattr_destroy returns " + std::string(str_res)));
    }
}

Mutex::~Mutex()
{
    pthread_mutex_destroy(&mutex);
}


/*
 * Timer
 */

Timer::Timer(std::string name, unsigned int max_cnt) : name(name), cnt(0), max_cnt(max_cnt), elapsed_sum(0), elapsed_avg(0)
{
    if (max_cnt < 1) {
        max_cnt = 1;
    }

    init();
}

Timer::~Timer()
{
    stop();
}

void Timer::start()
{
    init();

    running = true;
    gettimeofday(&start_timeval, NULL);
}

void Timer::stop()
{
    running = false;
    gettimeofday(&end_timeval, NULL);

    start_sec = start_timeval.tv_sec + (start_timeval.tv_usec / 1000000.0);
    end_sec = end_timeval.tv_sec + (end_timeval.tv_usec / 1000000.0);

    elapsed_sum += (end_sec - start_sec);
    cnt++;

    if (cnt == max_cnt) {
        elapsed_avg = elapsed_sum / (double)cnt;
        if (!elapsed_avg) {
            elapsed_avg = (end_sec - start_sec);
        }
        cnt = 0;
        elapsed_sum = 0;
    }
}

double Timer::restart()
{
    double res = get();
    start();
    return res;
}

double Timer::get()
{
    if (running) {
        stop();
    }

    return elapsed_avg;
}

void Timer::print()
{
    double elps = get();
    printf("[ %s ] ElapsedTime: %f sec (%.2f fps)\n", name.c_str(), elps, 1.0/elps); fflush(stdout);
}

void Timer::init()
{
    running = false;
    start_sec = 0;
    end_sec = 0;

    memset(&start_timeval, 0, sizeof(start_timeval));
    memset(&end_timeval, 0, sizeof(end_timeval));
}


