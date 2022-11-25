//
// Created by chemmy on 11/12/22.
//

#include "AP_MB4_AKS16_Enc.h"

//AP_HAL::OwnPtr<AP_HAL::SPIDevice> mb4_spi_dev = nullptr;


MB4_AKS16_Enc::MB4_AKS16_Enc():
    counter(1300),  notYetInit(true)
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

    if (!aks16.init_AKS16())
        return false;
    notYetInit = false;

//    spiDevp = probe(*this, std::move(hal.spi->get_device("extspi")));
    // Init MB4....
//    spiDevp->get_semaphore()->take_blocking();
//    spiDevp->set_speed(AP_HAL::Device::SPEED_LOW);
//    spiDevp->get_semaphore()->give();
//      if (!aks16.init_AKS16())
//          return false;


//    spiDevp->register_periodic_callback(100 * AP_USEC_PER_MSEC,
//                                     FUNCTOR_BIND_MEMBER(&update, void));


//    spiDevp = hal.spi->get_device("extspi");
//    hal.console->printf("Here we are at MB4_AKS16_Enc init(). Counter=%ld, device=%lx\n", counter, (long int)&spiDevp);
//
//    if(!spiDevp)
//        return false;




    return true;
}

void MB4_AKS16_Enc::update()
{
//    const char *spi_name;
//    uint8_t buf[16];
//    uint8_t *outp;

    if (notYetInit)
        if (!init()) {
            hal.console->printf("MB4_AKS16_Enc::update(): could not init()\n");
            return;
        }


//    hal.console->printf("We are at MB4_AKS16_Enc update(). device=%lx\n", (long int)&spiDevp);

//    uint8_t spicount = hal.spi->get_count();
//    for (int i = 0; i < spicount; i++) {
//        spi_name = hal.spi->get_device_name(i);
//        hal.console->printf("MB4::update(): SPI%d name: '%s'\n", i, spi_name);
//    }
//    counter = 0xaa55;
//    outp = (uint8_t *)&counter;
//    spiDevp->transfer(outp, 2, buf, 2);
//    hal.console->printf("MB4:: sent %0X, %0X, received %0X, %0X\n", outp[0], outp[1], buf[0], buf[1]);
    counter++;
    hal.console->printf("MB4:: counter=%ld\n", counter);
//    counter = 0xaa55bb66;
//    outp = (uint8_t *)&counter;

//    spiDevp->get_semaphore()->take_blocking();
//    spiDevp->transfer(outp, 2, buf, 2);
//    spiDevp->get_semaphore()->give();



//    hal.console->printf("MB4:: sent %02X, %02X, %02X, %02X, received %02X, %02X, %02X, %02X\n",
//                        outp[0], outp[1], outp[2], outp[3], buf[0], buf[1], buf[2], buf[3]);
//    spiDevp->set_chip_select((counter&1)?1:0);

//    aks16.update_encoders();
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