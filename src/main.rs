#![no_std]
#![no_main]

use defmt::{info, warn};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_nrf::{self as hal, twim::Twim};
use embassy_time::Delay;
use embedded_hal_async::delay::DelayNs;
use hal::twim;
use lsm303agr::Lsm303agr;
use panic_probe as _;

hal::bind_interrupts!(struct Irqs {
    TWISPI0 => twim::InterruptHandler<hal::peripherals::TWISPI0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("initializing micro-compass...");
    // Get a handle to the peripherals
    let dp = hal::init(Default::default());

    // Configure I2C using TWIM
    let config = twim::Config::default();
    let twim0 = Twim::new(dp.TWISPI0, Irqs, dp.P0_16, dp.P0_08, config);

    // Initialize LSM303AGR
    let mut sensor = Lsm303agr::new_with_i2c(twim0);

    // Read magnetometer ID
    match sensor.magnetometer_id().await {
        Ok(_id) => info!("magnetometer id obtained"),
        Err(_e) => info!("error getting magnetometer id"),
    }

    // Initialize sensor
    sensor.init().await.unwrap();

    // Configure accelerometer: High resolution mode, 50 Hz output data rate
    sensor
        .set_accel_mode_and_odr(
            &mut Delay,
            lsm303agr::AccelMode::HighResolution,
            lsm303agr::AccelOutputDataRate::Hz50,
        )
        .await
        .unwrap();

    // Configure magnetometer: High resolution mode, 10 Hz output data rate
    sensor
        .set_mag_mode_and_odr(
            &mut Delay,
            lsm303agr::MagMode::HighResolution,
            lsm303agr::MagOutputDataRate::Hz10,
        )
        .await
        .unwrap();

    // Enable continuous magnetometer mode
    let Ok(mut sensor) = sensor.into_mag_continuous().await else {
        panic!("failed to enter continuous mode");
    };
    sensor.mag_enable_low_pass_filter().await.unwrap();

    loop {
        // Read accelerometer data
        if sensor.accel_status().await.unwrap().xyz_new_data() {
            let accel = sensor.acceleration().await.unwrap();
            info!(
                "Acceleration: ({}, {}, {})",
                accel.x_mg(),
                accel.y_mg(),
                accel.z_mg()
            );
        } else {
            warn!("No new accelerometer data available");
        }

        // Read magnetometer data
        if sensor.mag_status().await.unwrap().xyz_new_data() {
            let data = sensor.magnetic_field().await.unwrap();
            info!(
                "magnetic field: ({}, {}, {})",
                data.x_nt(),
                data.y_nt(),
                data.z_nt()
            );
        } else {
            warn!("No new magnetometer data available");
        }

        // Delay before next read
        Delay.delay_ms(200).await;
    }
}
