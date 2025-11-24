# STM32F100 I2C & USART Communication

Learning bare metal programming on the STM32F100RB Discovery board. This project reads accelerometer and gyroscope data from an MPU6050 sensor via I2C and sends it to a PC through USART for real-time plotting./

## What It Does

- Communicates with MPU6050 using I2C protocol
- Reads 7 measurements: 3-axis accelerometer, 3-axis gyroscope, and temperature
- Sends data to PC via USART (USB virtual COM port)
- Displays live sensor data on Serial Plotter

## Hardware Setup

### Components

- STM32F100RB Discovery Board
- MPU6050 Module (GY-521)
- Jumper wires

### Wiring

![Wiring Schematic](images/schematic.png)

**Note:** Make sure your MPU6050 module has built-in pull-up resistors on SCL and SDA lines.

## Software Setup

1. Clone this repository
2. Open in PlatformIO
3. Build and upload to board
4. Open Serial Monitor or Serial Plotter at **250000 baud**
5. You should see live sensor data!

## Serial Plotter Output

![MPU6050 Data Plot](images/plotted_data.png)

The plotter shows all 7 data streams in real-time.

## What I Learned

- Setting up I2C communication from scratch (no HAL!)
- Configuring USART for serial communication
- Working with registers directly
- Reading sensor data and formatting it for plotting
- The importance of clearing hardware flags properly (especially ADDR flag!)

## Code Structure

- **RCC_Configuration()** - Sets up system clocks (24MHz)
- **GPIO_Configuration()** - Configures pins for I2C and USART
- **I2C_Configuration()** - Initializes I2C1 peripheral
- **USART_Configuration()** - Sets up USART1 for serial communication
- **main()** - Wakes up MPU6050, reads sensor data, sends via USART

## Resources

- [STM32F100 Reference Manual](https://www.st.com/resource/en/reference_manual/cd00246267-stm32f101xx-stm32f102xx-stm32f103xx-stm32f105xx-and-stm32f107xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [MPU6050 Register Map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)
