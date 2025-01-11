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
use micromath::F32Ext;
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
        let (accel_x, accel_y, accel_z) = if sensor.accel_status().await.unwrap().xyz_new_data() {
            let accel = sensor.acceleration().await.unwrap();
            (
                accel.x_mg() as f32,
                accel.y_mg() as f32,
                accel.z_mg() as f32,
            )
        } else {
            warn!("No new accelerometer data available");
            continue;
        };

        // Read magnetometer data
        let (mag_x, mag_y, mag_z) = if sensor.mag_status().await.unwrap().xyz_new_data() {
            let data = sensor.magnetic_field().await.unwrap();
            (data.x_nt() as f32, data.y_nt() as f32, data.z_nt() as f32)
        } else {
            warn!("No new magnetometer data available");
            continue;
        };

        // Compute tilt compensation
        let heading = compute_heading(accel_x, accel_y, accel_z, mag_x, mag_y, mag_z);
        info!(
            "Heading: {}.{:02}°",
            heading as i32,
            (heading.fract() * 100.0) as i32
        );

        // Delay before next read
        Delay.delay_ms(200).await;
    }
}

fn compute_heading(
    accel_x: f32,
    accel_y: f32,
    accel_z: f32,
    mag_x: f32,
    mag_y: f32,
    mag_z: f32,
) -> f32 {
    // Normalize accelerometer values
    let accel_norm = (accel_x * accel_x + accel_y * accel_y + accel_z * accel_z).sqrt();
    let ax = accel_x / accel_norm;
    let ay = accel_y / accel_norm;
    let az = accel_z / accel_norm;

    // Compute pitch and roll angles
    let pitch = (-ax).asin();
    let roll = ay.atan2(az);

    // Tilt compensation
    let mag_xh = mag_x * roll.cos() + mag_y * roll.sin() * pitch.sin() - mag_z * pitch.cos();
    let mag_yh = mag_y * pitch.cos() + mag_z * pitch.sin();

    // Comput heading using atan2
    let mut heading = mag_yh.atan2(mag_xh).to_degrees();

    // Convert range from -180 to 180 into 0 to 360
    if heading < 0.0 {
        heading += 360.0;
    }

    heading
}
