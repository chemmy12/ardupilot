//
// Created by chemmy on 11/12/22.
//

#include "AP_MB4_AKS16_Enc.h"

AP_HAL::OwnPtr<AP_HAL::SPIDevice> mb4_spi_dev = nullptr;


MB4_AKS16_Enc::MB4_AKS16_Enc():
    counter(1300), spiDevp(nullptr), notYetInit(true)
{

//    hal.console->printf("\n\n\nHere we are at MB4_AKS16_Enc constructor\n\n\n");
//    SCHED_TASK_CLASS(MB4_AKS16_Enc, init,              100,    130,  3)

}

//bool MB4_AKS16_Enc::_add_backend(MB4_Backend *backend)
//{
//    if (!backend) {
//        return false;
//    }
//
////    driver = backend;
//    return true;
//}


bool MB4_AKS16_Enc::init()
{

//    spiDevp = probe(*this, std::move(hal.spi->get_device("extspi")));
    // Init MB4....
    spiDevp = hal.spi->get_device("extspi");
    spiDevp->set_speed(AP_HAL::Device::SPEED_LOW);

//    spiDevp->register_periodic_callback(100 * AP_USEC_PER_MSEC,
//                                     FUNCTOR_BIND_MEMBER(&update, void));


//    spiDevp = hal.spi->get_device("extspi");
//    hal.console->printf("Here we are at MB4_AKS16_Enc init(). Counter=%ld, device=%lx\n", counter, (long int)&spiDevp);
//
//    if(!spiDevp)
//        return false;

    mb4_spi_dev = spiDevp;


    notYetInit = false;

    return true;
}

void MB4_AKS16_Enc::update()
{
//    const char *spi_name;
    uint8_t buf[16];
    uint8_t *outp;

    if (notYetInit)
        init();
    hal.console->printf("We are at MB4_AKS16_Enc update(). device=%lx\n", (long int)&spiDevp);

//    uint8_t spicount = hal.spi->get_count();
//    for (int i = 0; i < spicount; i++) {
//        spi_name = hal.spi->get_device_name(i);
//        hal.console->printf("MB4::update(): SPI%d name: '%s'\n", i, spi_name);
//    }
    counter = 0xaa55;
    outp = (uint8_t *)&counter;
    spiDevp->transfer(outp, 2, buf, 2);
    hal.console->printf("MB4:: sent %0X, %0X, received %0X, %0X\n", outp[0], outp[1], buf[0], buf[1]);
    counter++;
}


void mb4_spi_transfer(uint8_t *data_tx, uint8_t *data_rx, uint16_t datasize)
{
    mb4_spi_dev->transfer(data_tx, datasize, data_rx, datasize);
//    digitalWrite(NCS_PIN, LOW);
//
//    for (uint8_t i = 0; i < datasize; i++)
//        data_rx[i] = SPI.transfer(data_tx[i]);
//
//    digitalWrite(NCS_PIN, HIGH);
}

//MB4_Backend::MB4_Backend(inst, AP_HAL::OwnPtr<AP_HAL::Device> dev)
//{
//
//}
//
//MB4_AKS16_Enc::probe(this, AP_HAL::OwnPtr<AP_HAL::Device> dev)
//{
//    if (!dev) {
//        return nullptr;
//    }
//    MB4_Backend *sensor = new MB4_Backend(baro, std::move(dev));
//    if (!sensor || !sensor->_init()) {
//        delete sensor;
//        return nullptr;
//    }
//    return sensor;
//}