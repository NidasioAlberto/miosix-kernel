#include <miosix.h>

using namespace miosix;

int main() {
    while (true) {
        ledOn();
        sleep(1);
        ledOff();
        sleep(1);
    }
}