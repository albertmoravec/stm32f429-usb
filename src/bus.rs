use core::mem;
use crate::helpers::*;
use crate::atomic_mutex::AtomicMutex;
use bare_metal::CriticalSection;
use fiprintln::fiprintln;
use cortex_m::interrupt;
use cortex_m::asm::delay;
use crate::endpoint::{Endpoint, InEndpoint, OutEndpoint, EndpointConfiguration, NUM_ENDPOINTS};
use usb_device::bus::UsbBus as UsbBusTrait;
use usb_device::bus::{PollResult, UsbBusAllocator};
use usb_device::endpoint::{EndpointAddress, EndpointType};
use usb_device::{UsbDirection, UsbError, Result};
use stm32f4xx_hal::stm32f429_public::{OTG_HS_DEVICE, OTG_HS_GLOBAL, OTG_HS_PWRCLK, otg_hs_global::{OTG_HS_GRXSTSR_PERIPHERAL, OTG_HS_GRXSTSP_HOST, OTG_HS_GRXSTSR_HOST, OTG_HS_GRXSTSP_PERIPHERAL, OTG_HS_GNPTXFSIZ_HOST, OTG_HS_TX0FSIZ_PERIPHERAL}, otg_fs_global::fs_grxstsr_device::{DPIDR, PKTSTSR}};
use stm32f4xx_hal::stm32f429_public::otg_hs_global::otg_hs_grxstsr_peripheral::R as GrxstsrReg;
use stm32f4xx_hal::stm32f429_public::otg_hs_global::otg_hs_grxstsp_peripheral::R as GrxstspReg;
use stm32f4xx_hal::gpio::{gpioa, gpiob, Alternate, AF12, AF10, Input, Floating};
use stm32f4xx_hal::hal;

pub struct UsbBus {
    regs: AtomicMutex<UsbRegs>,
    gpio_pins: AtomicMutex<GpioPins>,
    in_endpoints: [InEndpoint; NUM_ENDPOINTS],
    out_endpoints: [OutEndpoint; NUM_ENDPOINTS],
    max_rx_packet_size: u16,
    waiting_read: AtomicMutex<WaitingPacketInfoWrapper>,
}

struct WaitingPacketInfoWrapper {
    //    waiting_packet_info: Option<WaitingPacketInfo>
    waiting_packet_info: Option<GrxstspReg>
}

pub struct UsbRegs {
    otg_hs_global: OTG_HS_GLOBAL,
    otg_hs_device: OTG_HS_DEVICE,
    otg_hs_pwrclk: OTG_HS_PWRCLK,
}

pub struct GpioPins {
    dp: gpiob::PB15<Alternate<AF12>>,
    dm: gpiob::PB14<Alternate<AF12>>,
    vbus: gpiob::PB13<Input<Floating>>,
}

//struct WaitingPacketInfo {
//    frame_number: u8,
//    packet_status: PacketStatus,
//    data_pid: DataPID,
//    byte_count: usize,
//    endpoint_number: u8,
//}
//
//impl WaitingPacketInfo {
//    fn from_reg(reg: GrxstsrReg) -> WaitingPacketInfo {
//        WaitingPacketInfo {
//            frame_number: reg.frmnum().bits(),
//            packet_status: PacketStatus::from_pktsts_register(reg.pktsts()),
//            data_pid: DataPID::from_dpid_register(reg.dpid()),
//            byte_count: reg.bcnt().bits() as usize,
//            endpoint_number: reg.epnum().bits(),
//        }
//    }
//}
//
//enum DataPID {
//    DATA0 = 0,
//    DATA1 = 2,
//    DATA2 = 1,
//    MDATA = 3,
//}
//
//impl DataPID {
//    fn from_dpid_register(reg: DPIDR) -> DataPID {
//        match reg.bits() {
//            0 => DataPID::DATA0,
//            2 => DataPID::DATA1,
//            1 => DataPID::DATA2,
//            3 => DataPID::MDATA,
//            _ => unreachable!()
//        }
//    }
//}
//
//enum PacketStatus {
//    GlobalOutNAK = 1,
//    OutDataPacketReceived = 2,
//    OutTransferCompleted = 3,
//    SetupTransactionCompleted = 4,
//    SetupDataPacketReceived = 6,
//}
//
//impl PacketStatus {
//    fn from_pktsts_register(reg: PKTSTSR) -> PacketStatus {
//        match reg.bits() {
//            1 => PacketStatus::GlobalOutNAK,
//            2 => PacketStatus::OutDataPacketReceived,
//            3 => PacketStatus::OutTransferCompleted,
//            4 => PacketStatus::SetupTransactionCompleted,
//            6 => PacketStatus::SetupDataPacketReceived,
//            _ => unreachable!()
//        }
//    }
//}

impl UsbBus {
    pub fn new(otg_hs_global: OTG_HS_GLOBAL,
               otg_hs_device: OTG_HS_DEVICE,
               otg_hs_pwrclk: OTG_HS_PWRCLK,
               dp_pin: gpiob::PB15<Alternate<AF12>>,
               dm_pin: gpiob::PB14<Alternate<AF12>>,
               vbus_pin: gpiob::PB13<Input<Floating>>,
    ) -> UsbBusAllocator<Self> {
        let bus = UsbBus {
            regs: AtomicMutex::new(UsbRegs {
                otg_hs_global,
                otg_hs_device,
                otg_hs_pwrclk,
            }),
            gpio_pins: AtomicMutex::new(GpioPins {
                dm: dm_pin,
                dp: dp_pin,
                vbus: vbus_pin,
            }),
            in_endpoints: unsafe {
                let mut in_ep: [InEndpoint; NUM_ENDPOINTS] = mem::uninitialized();

                for i in 0..NUM_ENDPOINTS {
                    in_ep[i] = InEndpoint::new(i as u8)
                }

                in_ep
            },
            out_endpoints: unsafe {
                let mut out_ep: [OutEndpoint; NUM_ENDPOINTS] = mem::uninitialized();

                for i in 0..NUM_ENDPOINTS {
                    out_ep[i] = OutEndpoint::new(i as u8);
                }

                out_ep
            },
            max_rx_packet_size: 0,
            waiting_read: AtomicMutex::new(WaitingPacketInfoWrapper { waiting_packet_info: None }),
        };

        let usbbus = UsbBusAllocator::new(bus);

        usbbus
    }

    // count of 32-bit blocks in fifo (1280 B)
    const FIFO_SIZE: u16 = 320;

    fn disable_global_interrupt(&self, regs: &UsbRegs) {
        regs.otg_hs_global.otg_hs_gahbcfg.modify(|_, w| w.gint().clear_bit());
    }

    fn enable_global_interrupt(&self, regs: &UsbRegs) {
        regs.otg_hs_global.otg_hs_gahbcfg.modify(|_, w| w.gint().set_bit());
    }

    fn init_rx_buffer(&self, regs: &UsbRegs) {
        let enabled_out_ep_count = self.out_endpoints.iter().filter(|ep| ep.configuration.is_some()).count();

        let buf_size_raw = 10 + 1 + (2 * (self.max_rx_packet_size / 4)) + 1 + (enabled_out_ep_count as u16);
        let buf_size = clamp(16u16, buf_size_raw, 512u16);

        regs.otg_hs_global.otg_hs_grxfsiz.write(|w| unsafe { w.rxfd().bits(buf_size) });

        self.flush_rx_fifo(regs);
    }

    unsafe fn init_tx_buffers(&self, regs: &UsbRegs) {
        self.in_endpoints.iter().filter(|ep| ep.configuration.is_some()).for_each(|ep| {
            let buf_size = clamp(16, (ep.configuration.as_ref().unwrap().max_packet_size + 3) / 4, 256);
        });
        for ep in self.in_endpoints.iter().filter(|ep| ep.configuration.is_some()) {
            // fixme: 256 should be maxuimum only for EP0, leaving that for other EPs as well for now
            let buf_size = clamp(16, (ep.configuration.as_ref().unwrap().max_packet_size + 3) / 4, 256);

            let fifo_head = self.calculate_fifo_head(regs, ep.index());

            unsafe {
                match ep.index() {
                    0 => {
                        mem::transmute::<&OTG_HS_GNPTXFSIZ_HOST, &OTG_HS_TX0FSIZ_PERIPHERAL>(&regs.otg_hs_global.otg_hs_gnptxfsiz_host).write(|w| w.tx0fd().bits(buf_size).tx0fsa().bits(fifo_head));
                        regs.otg_hs_device.otg_hs_diepctl0.modify(|_, w| w.txfnum().bits(ep.index()))
                    }
                    1 => {
                        regs.otg_hs_global.otg_hs_dieptxf1.write(|w| w.ineptxfd().bits(buf_size).ineptxsa().bits(fifo_head));
                        regs.otg_hs_device.otg_hs_diepctl1.modify(|_, w| w.txfnum().bits(ep.index()))
                    }
                    2 => {
                        regs.otg_hs_global.otg_hs_dieptxf2.write(|w| w.ineptxfd().bits(buf_size).ineptxsa().bits(fifo_head));
                        regs.otg_hs_device.otg_hs_diepctl2.modify(|_, w| w.txfnum().bits(ep.index()))
                    }
                    3 => {
                        regs.otg_hs_global.otg_hs_dieptxf3.write(|w| w.ineptxfd().bits(buf_size).ineptxsa().bits(fifo_head));
                        regs.otg_hs_device.otg_hs_diepctl3.modify(|_, w| w.txfnum().bits(ep.index()))
                    }
                    4 => {
                        regs.otg_hs_global.otg_hs_dieptxf4.write(|w| w.ineptxfd().bits(buf_size).ineptxsa().bits(fifo_head));
                        regs.otg_hs_device.otg_hs_diepctl4.modify(|_, w| w.txfnum().bits(ep.index()))
                    }
                    5 => {
                        regs.otg_hs_global.otg_hs_dieptxf5.write(|w| w.ineptxfd().bits(buf_size).ineptxsa().bits(fifo_head));
                        regs.otg_hs_device.otg_hs_diepctl5.modify(|_, w| w.txfnum().bits(ep.index()))
                    }
                    _ => unreachable!()
                }
            }
        }

        self.flush_tx_fifos(regs);
    }

    unsafe fn calculate_fifo_head(&self, regs: &UsbRegs, next_index: u8) -> u16 {
        let mut size = regs.otg_hs_global.otg_hs_grxfsiz.read().rxfd().bits();

        // this expects that fifos are ordered and span continuous block of memory
        for i in 0..next_index {
            size += match i {
                0 => mem::transmute::<&OTG_HS_GNPTXFSIZ_HOST, &OTG_HS_TX0FSIZ_PERIPHERAL>(&regs.otg_hs_global.otg_hs_gnptxfsiz_host).read().tx0fd().bits(),
                1 => regs.otg_hs_global.otg_hs_dieptxf1.read().ineptxfd().bits(),
                2 => regs.otg_hs_global.otg_hs_dieptxf2.read().ineptxfd().bits(),
                3 => regs.otg_hs_global.otg_hs_dieptxf3.read().ineptxfd().bits(),
                4 => regs.otg_hs_global.otg_hs_dieptxf4.read().ineptxfd().bits(),
                5 => regs.otg_hs_global.otg_hs_dieptxf5.read().ineptxfd().bits(),
                _ => unreachable!()
            };
        }

        size
    }

    // todo: check if core is not using the fifo before flushing
    fn flush_tx_fifos(&self, regs: &UsbRegs) {
        regs.otg_hs_global.otg_hs_grstctl.modify(|_, w| unsafe { w.txfnum().bits(0b10000).txfflsh().set_bit() });

        while regs.otg_hs_global.otg_hs_grstctl.read().txfflsh().bit_is_set() {}

        delay(60);
    }

    fn flush_tx_fifo(&self, regs: &UsbRegs, ep_num: u8) {
        regs.otg_hs_global.otg_hs_grstctl.modify(|_, w| unsafe { w.txfnum().bits(ep_num).txfflsh().set_bit() });

        while regs.otg_hs_global.otg_hs_grstctl.read().txfflsh().bit_is_set() {}

        delay(60);
    }

    fn flush_rx_fifo(&self, regs: &UsbRegs) {
        regs.otg_hs_global.otg_hs_grstctl.modify(|_, w| w.rxfflsh().set_bit());

        while regs.otg_hs_global.otg_hs_grstctl.read().rxfflsh().bit_is_set() {}

        delay(60);
    }

    fn reset_core(&self, regs: &UsbRegs) {
        // wait for AHB to become idle
        while regs.otg_hs_global.otg_hs_grstctl.read().ahbidl().bit_is_clear() {}

        regs.otg_hs_global.otg_hs_grstctl.modify(|_, w| w.csrst().set_bit());

        while regs.otg_hs_global.otg_hs_grstctl.read().csrst().bit_is_set() {}
    }

    fn init_phy(&self, regs: &UsbRegs) {
        // this probably has no effect
        regs.otg_hs_global.otg_hs_gusbcfg.write(|w| w.physel().set_bit());

        self.reset_core(regs);

        // activate transceiver
        regs.otg_hs_global.otg_hs_gccfg.modify(|_, w| w.pwrdwn().set_bit());

        delay(60);
    }

    fn init_core(&self, regs: &UsbRegs) {
        self.init_phy(regs);

        // DMA and OTG init goes there once possible
    }

    unsafe fn init_device_core(&self, regs: &UsbRegs) {
        // Restart the Phy Clock
        // fixme: that's how ST does it... but what?!
        regs.otg_hs_pwrclk.otg_hs_pcgcctl.modify(|_, w| unsafe { w.bits(0) });

        // Enable VBUS sensing for B (peripheral) mode
        regs.otg_hs_global.otg_hs_gccfg.modify(|_, w| w.vbusbsen().set_bit());

        // periodic frame interval = 80% & device speed = FS with integrated PHY
        regs.otg_hs_device.otg_hs_dcfg.modify(|_, w| unsafe { w.pfivl().bits(0b0).dspd().bits(0b11) });

        self.init_rx_buffer(regs);
        self.init_tx_buffers(regs);

        self.clear_pending_device_interrupts(regs);

        self.in_endpoints.iter().for_each(|ep| ep.reset(&regs.otg_hs_device));
        self.out_endpoints.iter().for_each(|ep| ep.reset(&regs.otg_hs_device));

        self.in_endpoints.iter()
            .filter(|ep| ep.configuration.is_some())
            .for_each(|ep| ep.enable(&regs.otg_hs_device));

        self.out_endpoints.iter()
            .filter(|ep| ep.configuration.is_some())
            .for_each(|ep| ep.enable(&regs.otg_hs_device));

        self.enable_device_interrupts(regs);
    }

    fn set_device_mode(&self, regs: &UsbRegs) {
        regs.otg_hs_global.otg_hs_gusbcfg.modify(|_, w| w.fdmod().set_bit());

        // 50ms delay
        delay(50 * 12 * 1000);
    }

    fn clear_pending_device_interrupts(&self, regs: &UsbRegs) {
        regs.otg_hs_device.otg_hs_diepmsk.reset();
        regs.otg_hs_device.otg_hs_doepmsk.reset();
        regs.otg_hs_device.otg_hs_diepempmsk.reset();
//        regs.otg_fs_device.fs_daint.write(|w| unsafe { w.bits(0xFFFFFFFF) });
        regs.otg_hs_device.otg_hs_daintmsk.reset();
    }

    fn enable_device_interrupts(&self, regs: &UsbRegs) {
        regs.otg_hs_global.otg_hs_gintmsk.write(|w| unsafe { w.bits(0) });
        regs.otg_hs_global.otg_hs_gintsts.write(|w| unsafe { w.bits(0xBFFFFFFF) });

        regs.otg_hs_global.otg_hs_gintmsk.write(|w| w
            .wuim().set_bit()
            .rxflvlm().set_bit()
            .usbsuspm().set_bit()
            .usbrst().set_bit()
//            .enumdnem().set_bit()
            .iepint().set_bit()
            .oepint().set_bit()
        );

        regs.otg_hs_device.otg_hs_diepmsk.write(|w| w.xfrcm().set_bit());
//        regs.otg_fs_device.fs_doepmsk.write(|w| w.xfrcm().set_bit());
    }

    unsafe fn read_packet_status(&self, regs: &UsbRegs, _cs: &CriticalSection) -> Option<GrxstspReg> {
        // fiprintln!("reading packet status");

        let grxstsp = mem::transmute::<&OTG_HS_GRXSTSP_HOST, &OTG_HS_GRXSTSP_PERIPHERAL>(&regs.otg_hs_global.otg_hs_grxstsp_host);
//        let grxstsr = mem::transmute::<&OTG_HS_GRXSTSR_HOST, &OTG_HS_GRXSTSR_PERIPHERAL>(&self.regs.lock(_cs).otg_hs_global.otg_hs_grxstsr_host);

        // fiprintln!("got packet status register");
        loop {
            let packet_status = grxstsp.read();

            // fifo is empty (can also be checked using RXFLVL interrupt bit)
            if packet_status.bits() == 0 {
                 fiprintln!("packet status empty");
                break None;
            }

            // setup or data packet received
            if packet_status.pktsts().bits() == 2 || packet_status.pktsts().bits() == 6 {
                 fiprintln!("retrieved packet status");
                break Some(packet_status);
            }
        }
    }

    unsafe fn update_packet_status(&self, regs: &UsbRegs, _cs: &CriticalSection) {
        // fiprintln!("updating packet status");
        let mut container = self.waiting_read.lock(_cs);

        if container.waiting_packet_info.is_none() {
            fiprintln!("no previous packet, reading new packet status");
            container.waiting_packet_info = self.read_packet_status(regs, _cs);
        }
    }

    fn enable_stall(&self, regs: &UsbRegs, ep_addr: EndpointAddress) {
        // todo: do it properly, should wait for epdis to take effect and flush tx fifo
        fiprintln!("stall: enable");

        match (ep_addr.direction(), ep_addr.index()) {
            (UsbDirection::In, 0) => regs.otg_hs_device.otg_hs_diepctl0.modify(|_, w| w.stall().set_bit()),
            (UsbDirection::In, 1) => regs.otg_hs_device.otg_hs_diepctl1.modify(|_, w| w.epdis().set_bit().stall().set_bit()),
            (UsbDirection::In, 2) => regs.otg_hs_device.otg_hs_diepctl2.modify(|_, w| w.epdis().set_bit().stall().set_bit()),
            (UsbDirection::In, 3) => regs.otg_hs_device.otg_hs_diepctl3.modify(|_, w| w.epdis().set_bit().stall().set_bit()),
            (UsbDirection::In, 4) => regs.otg_hs_device.otg_hs_diepctl4.modify(|_, w| w.epdis().set_bit().stall().set_bit()),
            (UsbDirection::In, 5) => regs.otg_hs_device.otg_hs_diepctl5.modify(|_, w| w.epdis().set_bit().stall().set_bit()),
            (UsbDirection::Out, 0) => regs.otg_hs_device.otg_hs_doepctl0.modify(|_, w| w.stall().set_bit()),
            (UsbDirection::Out, 1) => regs.otg_hs_device.otg_hs_doepctl1.modify(|_, w| w.epdis().set_bit().stall().set_bit()),
            (UsbDirection::Out, 2) => regs.otg_hs_device.otg_hs_doepctl2.modify(|_, w| w.epdis().set_bit().stall().set_bit()),
            (UsbDirection::Out, 3) => regs.otg_hs_device.otg_hs_doepctl3.modify(|_, w| w.epdis().set_bit().stall().set_bit()),
            (UsbDirection::Out, 4) => regs.otg_hs_device.otg_hs_doepctl4.modify(|_, w| w.epdis().set_bit().stall().set_bit()),
            (UsbDirection::Out, 5) => regs.otg_hs_device.otg_hs_doepctl5.modify(|_, w| w.epdis().set_bit().stall().set_bit()),
            (_, _) => unreachable!()
        };

        if ep_addr.direction() == UsbDirection::In {
            self.flush_tx_fifo(regs, ep_addr.index() as u8);
        }
    }

    fn disable_stall(&self, regs: &UsbRegs, ep_addr: EndpointAddress) {
        fiprintln!("stall: disable");

        match (ep_addr.direction(), ep_addr.index()) {
            (UsbDirection::In, 0) => regs.otg_hs_device.otg_hs_diepctl0.modify(|_, w| w.stall().clear_bit()),
            (UsbDirection::In, 1) => regs.otg_hs_device.otg_hs_diepctl1.modify(|_, w| w.stall().clear_bit().sd0pid_sevnfrm().set_bit()),
            (UsbDirection::In, 2) => regs.otg_hs_device.otg_hs_diepctl2.modify(|_, w| w.stall().clear_bit().sd0pid_sevnfrm().set_bit()),
            (UsbDirection::In, 3) => regs.otg_hs_device.otg_hs_diepctl3.modify(|_, w| w.stall().clear_bit().sd0pid_sevnfrm().set_bit()),
            (UsbDirection::In, 4) => regs.otg_hs_device.otg_hs_diepctl4.modify(|_, w| w.stall().clear_bit().sd0pid_sevnfrm().set_bit()),
            (UsbDirection::In, 5) => regs.otg_hs_device.otg_hs_diepctl5.modify(|_, w| w.stall().clear_bit().sd0pid_sevnfrm().set_bit()),
            (UsbDirection::Out, 0) => regs.otg_hs_device.otg_hs_doepctl0.modify(|_, w| w.stall().clear_bit()),
            (UsbDirection::Out, 1) => regs.otg_hs_device.otg_hs_doepctl1.modify(|_, w| w.stall().clear_bit().sd0pid_sevnfrm().set_bit()),
            (UsbDirection::Out, 2) => regs.otg_hs_device.otg_hs_doepctl2.modify(|_, w| w.stall().clear_bit().sd0pid_sevnfrm().set_bit()),
            (UsbDirection::Out, 3) => regs.otg_hs_device.otg_hs_doepctl3.modify(|_, w| w.stall().clear_bit().sd0pid_sevnfrm().set_bit()),
            (UsbDirection::Out, 4) => regs.otg_hs_device.otg_hs_doepctl4.modify(|_, w| w.stall().clear_bit().sd0pid_sevnfrm().set_bit()),
            (UsbDirection::Out, 5) => regs.otg_hs_device.otg_hs_doepctl5.modify(|_, w| w.stall().clear_bit().sd0pid_sevnfrm().set_bit()),
            (_, _) => unreachable!()
        };
    }

    fn next_free_endpoint(&self, ep_dir: UsbDirection) -> Option<EndpointAddress> {
        match ep_dir {
            UsbDirection::In =>
                self.in_endpoints[1..].iter()
                    .find(|ep| ep.configuration.is_none())
                    .and_then(|ep| Some(EndpointAddress::from_parts(ep.index() as usize, ep_dir))),
            UsbDirection::Out =>
                self.out_endpoints[1..].iter()
                    .find(|ep| ep.configuration.is_none())
                    .and_then(|ep| Some(EndpointAddress::from_parts(ep.index() as usize, ep_dir)))
        }
    }

    fn read_in_ep_interrupts(&self, regs: &UsbRegs, ep_num: u8) -> u32 {
        let in_interrupt_mask = regs.otg_hs_device.otg_hs_diepmsk.read().bits();
        let empty_fifo_interrupt_mask = regs.otg_hs_device.otg_hs_diepempmsk.read().bits();
        let final_mask = in_interrupt_mask | ((empty_fifo_interrupt_mask >> ep_num) & 0b1) << 7; // 7 is TXFE bit

        let unmasked_interrupts = match ep_num {
            0 => regs.otg_hs_device.otg_hs_diepint0.read().bits(),
            1 => regs.otg_hs_device.otg_hs_diepint1.read().bits(),
            2 => regs.otg_hs_device.otg_hs_diepint2.read().bits(),
            3 => regs.otg_hs_device.otg_hs_diepint3.read().bits(),
            4 => regs.otg_hs_device.otg_hs_diepint4.read().bits(),
            5 => regs.otg_hs_device.otg_hs_diepint5.read().bits(),
            _ => unreachable!()
        };

        unmasked_interrupts & final_mask
    }
}

impl UsbBusTrait for UsbBus {
    /// Allocates an endpoint and specified endpoint parameters. This method is called by the device
    /// and class implementations to allocate endpoints, and can only be called before
    /// [`enable`](UsbBus::enable) is called.
    ///
    /// # Arguments
    ///
    /// * `ep_dir` - The endpoint direction.
    /// * `ep_addr` - A static endpoint address to allocate. If Some, the implementation should
    ///   attempt to return an endpoint with the specified address. If None, the implementation
    ///   should return the next available one.
    /// * `max_packet_size` - Maximum packet size in bytes.
    /// * `interval` - Polling interval parameter for interrupt endpoints.
    ///
    /// # Errors
    ///
    /// * [`EndpointOverflow`](crate::UsbError::EndpointOverflow) - Available total number of
    ///   endpoints, endpoints of the specified type, or endpoind packet memory has been exhausted.
    ///   This is generally caused when a user tries to add too many classes to a composite device.
    /// * [`InvalidEndpoint`](crate::UsbError::InvalidEndpoint) - A specific `ep_addr` was specified
    ///   but the endpoint in question has already been allocated.
    fn alloc_ep(
        &mut self,
        ep_dir: UsbDirection,
        ep_addr: Option<EndpointAddress>,
        ep_type: EndpointType,
        max_packet_size: u16,
        interval: u8) -> Result<EndpointAddress> {
        if ep_addr.is_some() && ep_addr.unwrap().index() > NUM_ENDPOINTS {
            fiprintln!("alloc: endpoint overflow");
            return Err(UsbError::EndpointOverflow);
        }

        interrupt::free(|cs| {
            let regs = self.regs.lock(cs);

            let ep_addr_assigned = match ep_addr {
                Some(addr) => addr,
                None => self.next_free_endpoint(ep_dir).ok_or(UsbError::EndpointOverflow)?
            };

            let result = match ep_dir {
                UsbDirection::In => {
                    let ep = &mut self.in_endpoints[ep_addr_assigned.index()];

                    match ep.configuration {
                        Some(_) => Err(UsbError::InvalidEndpoint),
                        None => {
                            ep.configuration.replace(EndpointConfiguration { ep_type, max_packet_size });
                            Ok(EndpointAddress::from_parts(ep.index() as usize, UsbDirection::In))
                        }
                    }
                }
                UsbDirection::Out => {
                    let ep = &mut self.out_endpoints[ep_addr_assigned.index()];

                    match ep.configuration {
                        Some(_) => Err(UsbError::InvalidEndpoint),
                        None => {
                            ep.configuration.replace(EndpointConfiguration { ep_type, max_packet_size });
                            Ok(EndpointAddress::from_parts(ep.index() as usize, UsbDirection::Out))
                        }
                    }
                }
            };

            if result.is_ok() && ep_dir == UsbDirection::Out && max_packet_size > self.max_rx_packet_size {
                self.max_rx_packet_size = max_packet_size;
            }

            match result {
                Ok(res) => fiprintln!("alloc: allocated ep - dir {}, num: {}, type: {}, max pckt size: {}", res.direction() as u8, res.index(), ep_type as u8, max_packet_size),
                Err(_) => fiprintln!("alloc: ep allocation error")
            };

            result
        })
    }

    /// Enables and initializes the USB peripheral. Soon after enabling the device will be reset, so
    /// there is no need to perform a USB reset in this method.
    fn enable(&mut self) {
        interrupt::free(|cs| {
            let regs = self.regs.lock(cs);

            self.disable_global_interrupt(&regs);

            self.init_core(&regs);

            self.set_device_mode(&regs);

            unsafe { self.init_device_core(&regs); }

            self.enable_global_interrupt(&regs);
        });
    }

    /// Performs a USB reset. This method should reset the platform-specific peripheral as well as
    /// ensure that all endpoints previously allocate with alloc_ep are initialized as specified.
    fn reset(&self) {}

    /// Sets the device USB address to `addr`.
    fn set_device_address(&self, addr: u8) {
        interrupt::free(|cs| self.regs.lock(cs).otg_hs_device.otg_hs_dcfg.modify(|_, w| unsafe { w.dad().bits(addr) }));
    }

    /// Writes a single packet of data to the specified endpoint and returns number of bytes
    /// actually written.
    ///
    /// The only reason for a short write is if the caller passes a slice larger than the amount of
    /// memory allocated earlier, and this is generally an error in the class implementation.
    ///
    /// # Errors
    ///
    /// * [`InvalidEndpoint`](crate::UsbError::InvalidEndpoint) - The `ep_addr` does not point to a
    ///   valid endpoint that was previously allocated with [`UsbBus::alloc_ep`].
    /// * [`WouldBlock`](crate::UsbError::WouldBlock) - A previously written packet is still pending
    ///   to be sent.
    /// * [`BufferOverflow`](crate::UsbError::BufferOverflow) - The packet is too long to fit in the
    ///   transmission buffer. This is generally an error in the class implementation, because the
    ///   class shouldn't provide more data than the `max_packet_size` it specified when allocating
    ///   the endpoint.
    ///
    /// Implementations may also return other errors if applicable.
    fn write(&self, ep_addr: EndpointAddress, buf: &[u8]) -> Result<usize> {
        if ep_addr.direction() == UsbDirection::Out {
            fiprintln!("write: invalid direction");
            return Err(UsbError::InvalidEndpoint);
        }

        let ep = &self.in_endpoints[ep_addr.index()];

        if ep.configuration.is_none() {
            fiprintln!("write: not configured");
            return Err(UsbError::InvalidEndpoint);
        }

        if buf.len() > ep.configuration.as_ref().unwrap().max_packet_size as usize {
            fiprintln!("write: buffer overflow");
            return Err(UsbError::BufferOverflow);
        }

        // this needs a critical section(?)
        if buf.len() != 0 {
            let mut buffer_available: usize = 0;

            interrupt::free(|cs| {
                let regs = self.regs.lock(cs);

                buffer_available = match ep.index() {
                    0 => regs.otg_hs_device.otg_hs_dtxfsts0.read().ineptfsav().bits() as usize,
                    1 => regs.otg_hs_device.otg_hs_dtxfsts1.read().ineptfsav().bits() as usize,
                    2 => regs.otg_hs_device.otg_hs_dtxfsts2.read().ineptfsav().bits() as usize,
                    3 => regs.otg_hs_device.otg_hs_dtxfsts3.read().ineptfsav().bits() as usize,
                    4 => regs.otg_hs_device.otg_hs_dtxfsts4.read().ineptfsav().bits() as usize,
                    5 => regs.otg_hs_device.otg_hs_dtxfsts5.read().ineptfsav().bits() as usize,
                    _ => unreachable!()
                };
            });

            fiprintln!("ep index: {}, buffer available: {}", ep.index(), buffer_available);


            if ((buf.len() + 3) >> 2) > buffer_available {
                fiprintln!("write: not enough space in buffer, waiting");
                return Err(UsbError::WouldBlock);
            }
        }

        interrupt::free(|cs| {
            let regs = self.regs.lock(cs);

            unsafe {
                match ep.index() {
                    0 => {
                        regs.otg_hs_device.otg_hs_dieptsiz0.write(|w| w.xfrsiz().bits(buf.len() as u8));
                        regs.otg_hs_device.otg_hs_diepctl0.modify(|r, w| w.cnak().set_bit().epena().set_bit());
                    }
                    1 => {
                        regs.otg_hs_device.otg_hs_dieptsiz1.write(|w| w.pktcnt().bits(1).xfrsiz().bits(buf.len() as u32));
                        regs.otg_hs_device.otg_hs_diepctl1.modify(|_, w| w.cnak().set_bit().epena().set_bit());
                    }
                    2 => {
                        regs.otg_hs_device.otg_hs_dieptsiz2.write(|w| w.pktcnt().bits(1).xfrsiz().bits(buf.len() as u32));
                        regs.otg_hs_device.otg_hs_diepctl2.modify(|_, w| w.cnak().set_bit().epena().set_bit());
                    }
                    3 => {
                        regs.otg_hs_device.otg_hs_dieptsiz3.write(|w| w.pktcnt().bits(1).xfrsiz().bits(buf.len() as u32));
                        regs.otg_hs_device.otg_hs_diepctl3.modify(|_, w| w.cnak().set_bit().epena().set_bit());
                    }
                    4 => {
                        regs.otg_hs_device.otg_hs_dieptsiz4.write(|w| w.pktcnt().bits(1).xfrsiz().bits(buf.len() as u32));
                        regs.otg_hs_device.otg_hs_diepctl4.modify(|_, w| w.cnak().set_bit().epena().set_bit());
                    }
                    5 => {
                        regs.otg_hs_device.otg_hs_dieptsiz5.write(|w| w.pktcnt().bits(1).xfrsiz().bits(buf.len() as u32));
                        regs.otg_hs_device.otg_hs_diepctl5.modify(|_, w| w.cnak().set_bit().epena().set_bit());
                    }
                    _ => unreachable!()
                };
            }
        });

        fiprintln!("write: beginning write");

        ep.write(buf)
    }

    /// Reads a single packet of data from the specified endpoint and returns the actual length of
    /// the packet.
    ///
    /// This should also clear any NAK flags and prepare the endpoint to receive the next packet.
    ///
    /// # Errors
    ///
    /// * [`InvalidEndpoint`](crate::UsbError::InvalidEndpoint) - The `ep_addr` does not point to a
    ///   valid endpoint that was previously allocated with [`UsbBus::alloc_ep`].
    /// * [`WouldBlock`](crate::UsbError::WouldBlock) - There is no packet to be read. Note that
    ///   this is different from a received zero-length packet, which is valid in USB. A zero-length
    ///   packet will return `Ok(0)`.
    /// * [`BufferOverflow`](crate::UsbError::BufferOverflow) - The received packet is too long to
    ///   fit in `buf`. This is generally an error in the class implementation, because the class
    ///   should use a buffer that is large enough for the `max_packet_size` it specified when
    ///   allocating the endpoint.
    ///
    /// Implementations may also return other errors if applicable.
    fn read(&self, ep_addr: EndpointAddress, buf: &mut [u8]) -> Result<usize> {
        if ep_addr.direction() == UsbDirection::In {
            fiprintln!("read: invalid endpoint");
            return Err(UsbError::InvalidEndpoint);
        }

        let rx_empty = interrupt::free(|cs| self.regs.lock(cs).otg_hs_global.otg_hs_gintsts.read().rxflvl().bit_is_clear());

        if rx_empty {
            fiprintln!("read: would block (no packet waiting)");
            return Err(UsbError::WouldBlock);
        }

        let ep = &self.out_endpoints[ep_addr.index()];

        let byte_count = interrupt::free(|cs| {
            let regs = self.regs.lock(cs);
            unsafe { self.update_packet_status(&regs, cs); }

            self.waiting_read.lock(cs).waiting_packet_info.as_ref()
                .ok_or(UsbError::WouldBlock)
                .and_then(|pi| {
                    if pi.epnum().bits() != ep_addr.index() as u8 {
                        fiprintln!("read: would block (invalid endpoint address) - packet epnum: {}, ep addr: {}", pi.epnum().bits(), ep_addr.index() as u8);
                        return Err(UsbError::WouldBlock);
                    }

                    let bytes = pi.bcnt().bits().clone();
                    fiprintln!("read: {} bytes to read", bytes);
                    Ok(bytes)
                })
        });

        if byte_count.is_err() {
            fiprintln!("read: byte count returned error (failed reading waiting packet)");
        }

        let result = byte_count.and_then(|bc| {
            if buf.len() < bc as usize {
                fiprintln!("read: provided buffer too small");
                return Err(UsbError::BufferOverflow)
            }

            Ok(bc)
        }).and_then(|bc| {
            fiprintln!("read: beginning reading");

            let read = ep.read(buf, bc as usize);

            fiprintln!("read: setting waiting packet to none");

            interrupt::free(|cs| self.waiting_read.lock(cs).waiting_packet_info = None);

            fiprintln!("read: done reading");

            read
        });
        // fiprintln!("read: ok");
        result
    }

    /// Sets or clears the STALL condition for an endpoint. If the endpoint is an OUT endpoint, it
    /// should be prepared to receive data again.
    fn set_stalled(&self, ep_addr: EndpointAddress, stalled: bool) {
        // todo: allow stalling control EP

        interrupt::free(|cs| {
            let regs = self.regs.lock(cs);

            match stalled {
                true => self.enable_stall(&regs, ep_addr),
                false => self.disable_stall(&regs, ep_addr)
            }
        });
    }

    /// Gets whether the STALL condition is set for an endpoint.
    fn is_stalled(&self, ep_addr: EndpointAddress) -> bool {
        interrupt::free(|cs| {
            let regs = self.regs.lock(cs);

            match (ep_addr.direction(), ep_addr.index()) {
                (UsbDirection::In, 0) | (UsbDirection::Out, 0) => false,
                (UsbDirection::In, 1) => regs.otg_hs_device.otg_hs_diepctl1.read().stall().bit(),
                (UsbDirection::In, 2) => regs.otg_hs_device.otg_hs_diepctl2.read().stall().bit(),
                (UsbDirection::In, 3) => regs.otg_hs_device.otg_hs_diepctl3.read().stall().bit(),
                (UsbDirection::In, 4) => regs.otg_hs_device.otg_hs_diepctl4.read().stall().bit(),
                (UsbDirection::In, 5) => regs.otg_hs_device.otg_hs_diepctl5.read().stall().bit(),
                (UsbDirection::Out, 1) => regs.otg_hs_device.otg_hs_doepctl1.read().stall().bit(),
                (UsbDirection::Out, 2) => regs.otg_hs_device.otg_hs_doepctl2.read().stall().bit(),
                (UsbDirection::Out, 3) => regs.otg_hs_device.otg_hs_doepctl3.read().stall().bit(),
                (UsbDirection::Out, 4) => regs.otg_hs_device.otg_hs_doepctl4.read().stall().bit(),
                (UsbDirection::Out, 5) => regs.otg_hs_device.otg_hs_doepctl5.read().stall().bit(),
                (_, _) => unreachable!()
            }
        })
    }

    /// Causes the USB peripheral to enter USB suspend mode, lowering power consumption and
    /// preparing to detect a USB wakeup event. This will be called after
    /// [`poll`](crate::device::UsbDevice::poll) returns [`PollResult::Suspend`]. The device will
    /// continue be polled, and it shall return a value other than `Suspend` from `poll` when it no
    /// longer detects the suspend condition.
    fn suspend(&self) {
        fiprintln!("suspend called");
    }

    /// Resumes from suspend mode. This may only be called after the peripheral has been previously
    /// suspended.
    fn resume(&self) {
        fiprintln!("resume called");
    }

    /// Gets information about events and incoming data. Usually called in a loop or from an
    /// interrupt handler. See the [`PollResult`] struct for more information.
    fn poll(&self) -> PollResult {
        interrupt::free(|cs| {
            let regs = self.regs.lock(cs);
            let interrupts = &regs.otg_hs_global.otg_hs_gintsts;
            let current = &interrupts.read();


            if current.usbrst().bit_is_set() {
                interrupts.modify(|_, w| w.usbrst().set_bit());
                fiprintln!("poll: reset");
                return PollResult::Reset;
            }

            if current.usbsusp().bit_is_set() {
                interrupts.modify(|_, w| w.usbsusp().set_bit());
                fiprintln!("poll: suspend");
                return PollResult::Suspend;
            }

            if current.wkuint().bit_is_set() {
                interrupts.modify(|_, w| w.wkuint().set_bit());
                fiprintln!("poll: resume");
                return PollResult::Resume;
            }

            if current.rxflvl().bit_is_set() {
                fiprintln!("rxflvl read");

                unsafe { self.update_packet_status(&regs, cs); }

                fiprintln!("packet status read");

                match &self.waiting_read.lock(cs).waiting_packet_info {
                    Some(info) => {
                        fiprintln!("waiting packet - ep num: {}, byte count: {}, packet status: {}", info.epnum().bits(), info.bcnt().bits(), info.pktsts().bits());

                        if info.pktsts().bits() == 6 {
                            fiprintln!("poll: setup");
                            return PollResult::Data {
                                ep_setup: (1 << info.epnum().bits()) as u16,
                                ep_in_complete: 0,
                                ep_out: 0,
                            };
                        } else if info.pktsts().bits() == 2 {
                            fiprintln!("poll: data");
                            return PollResult::Data {
                                ep_out: (1 << info.epnum().bits()) as u16,
                                ep_in_complete: 0,
                                ep_setup: 0,
                            };
                        }
                    }
                    None => {}
                }
            }

            if current.iepint().bit_is_set() {
                fiprintln!("iepint read");

                // transfer complete interrupt hit
                // fixme: this should check if it actually did

                let ep_bits = regs.otg_hs_device.otg_hs_daint.read().iepint().bits();

                for i in 0..6 {
                    if (ep_bits & (1 << i)) != 0 {
                        match i {
                            0 => regs.otg_hs_device.otg_hs_diepint0.modify(|_, w| w.xfrc().set_bit()),
                            1 => regs.otg_hs_device.otg_hs_diepint1.modify(|_, w| w.xfrc().set_bit()),
                            2 => regs.otg_hs_device.otg_hs_diepint2.modify(|_, w| w.xfrc().set_bit()),
                            3 => regs.otg_hs_device.otg_hs_diepint3.modify(|_, w| w.xfrc().set_bit()),
                            4 => regs.otg_hs_device.otg_hs_diepint4.modify(|_, w| w.xfrc().set_bit()),
                            5 => regs.otg_hs_device.otg_hs_diepint5.modify(|_, w| w.xfrc().set_bit()),
                            _ => unreachable!()
                        }
                    }
                }

                fiprintln!("in complete for {}", ep_bits);

                fiprintln!("poll: in complete");
                return PollResult::Data {
                    ep_in_complete: ep_bits,
                    ep_out: 0,
                    ep_setup: 0,
                };
            }

//            fiprintln!("none read");


            PollResult::None
        })
    }


    /// Simulates a disconnect from the USB bus, causing the host to reset and re-enumerate the
    /// device.
    ///
    /// Mostly used for development. By calling this at the start of your program ensures that the
    /// host re-enumerates your device after a new program has been flashed.
    ///
    /// The default implementation just returns `Unsupported`.
    ///
    /// # Errors
    ///
    /// * [`Unsupported`](crate::UsbError::Unsupported) - This UsbBus implementation doesn't support
    ///   simulating a disconnect or it has not been enabled at creation time.
    fn force_reset(&self) -> Result<()> {
        Err(UsbError::Unsupported)
    }
}