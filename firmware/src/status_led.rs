use embassy_executor::Spawner;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::{Channel, Receiver},
};
use esp_hal_common::{
    clock::Clocks,
    gpio::{GpioPin, Unknown},
    peripherals::RMT,
    rmt::{Channel0, TxChannelConfig, TxChannelCreator},
    Rmt,
};
use espilepsy::Color;
use hal::prelude::*;
use static_cell::StaticCell;

use crate::singleton;

pub struct LedHandle {
    channel: &'static BlinkyChannel,
}

impl LedHandle {
    pub async fn set(&self, cmd: espilepsy::Cmd) {
        let _ = self.channel.send(cmd).await;
    }
}

pub fn init(spawner: &Spawner, rmt: RMT, pin: GpioPin<Unknown, 7>, clocks: &Clocks) -> LedHandle {
    let rmt = Rmt::new(rmt, 80u32.MHz(), clocks).unwrap();
    let rmt_channel = rmt
        .channel0
        .configure(
            pin.into_push_pull_output(),
            TxChannelConfig {
                clk_divider: 1,
                ..TxChannelConfig::default()
            },
        )
        .unwrap();

    let led_channel = singleton!(Channel::new(), BlinkyChannel);
    let _ = spawner.spawn(led_task(rmt_channel, led_channel.receiver()));
    LedHandle {
        channel: led_channel,
    }
}

type BlinkyChannel = Channel<CriticalSectionRawMutex, espilepsy::Cmd, 2>;
type BlinkyReceiver<'a> = Receiver<'a, CriticalSectionRawMutex, espilepsy::Cmd, 2>;

pub const WHITE: Color = Color { r: 5, g: 5, b: 5 };
pub const OFF: Color = Color { r: 0, g: 0, b: 0 };
pub const RED: Color = Color { r: 16, g: 0, b: 0 };
pub const GREEN: Color = Color { r: 0, g: 16, b: 0 };
pub const BLUE: Color = Color { r: 0, g: 0, b: 16 };

#[embassy_executor::task]
async fn led_task(rmt_channel: Channel0<0>, recv: BlinkyReceiver<'static>) {
    espilepsy::task(rmt_channel, recv).await
}
