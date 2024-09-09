# USB Project using Nuvoton NUC121

This project is based on the Nuvoton NUC121 microcontroller and implements USB functionality. The NUC121 is a cost-effective ARM Cortex-M0 based MCU with built-in USB support, making it ideal for various embedded applications requiring USB communication.

## Project Overview

This project focuses on utilizing the USB capabilities of the Nuvoton NUC121 to develop a USB device. The project can be tailored for various USB applications such as USB HID (Human Interface Device), USB CDC (Communication Device Class), or USB Mass Storage, depending on the specific needs.

### Features
- **USB Communication:** Implements USB device communication using the NUC121’s USB hardware.
- **USB Classes:** Supports multiple USB classes such as HID, CDC, and Mass Storage.
- **Low Power Consumption:** Efficiently manages power consumption, leveraging the low-power features of the NUC121.
- **Interrupt-driven USB Handling:** Uses interrupt-based handling for USB communication to ensure real-time responsiveness.
- **Configurable Endpoints:** Provides flexibility in configuring multiple USB endpoints for different use cases.

## Requirements

- **Hardware:**
  - Nuvoton NUC121 development board or custom board.
  - USB cable for communication with the host device (e.g., PC).
  
- **Software:**
  - Keil MDK or IAR Embedded Workbench for ARM for development.
  - Nuvoton BSP (Board Support Package) and USB libraries.

## Installation and Setup

1. **Clone the Repository:**

    ```bash
    git clone https://github.com/JinNamil/NUC_125.git
    ```

2. **Install Nuvoton BSP and USB Libraries:**
   Download and install the [Nuvoton NUC121 Board Support Package](https://www.nuvoton.com/resource-download.jsp?tp_GUID=DC0120200630123100) and the USB library from Nuvoton’s website or via the package manager in your IDE (e.g., Keil).

3. **Open the Project in Your IDE:**
   Import the project into Keil MDK or IAR Embedded Workbench.

4. **Configure USB Settings:**
   In the project, configure the USB device settings (e.g., device class, endpoints, and descriptors) according to your application requirements.

5. **Build and Flash the Project:**
   Select your build configuration (e.g., Debug or Release) and build the project. Flash the compiled firmware onto the Nuvoton NUC121 board using the programmer/debugger.

6. **Connect the USB Device:**
   Connect the NUC121 to your PC or another host device via a USB cable. The device should be recognized according to the USB class implemented (e.g., HID, CDC, or Mass Storage).

## Usage

Once the project is built and flashed onto the NUC121, the USB device will operate based on the selected USB class. Here’s how the different classes can be used:

- **USB HID:** The device can act as a Human Interface Device (e.g., a keyboard, mouse, or custom input device). No additional driver installation is typically required for HID devices.
  
- **USB CDC:** The device can act as a virtual COM port (USB to Serial communication). Install the appropriate drivers for the CDC class if necessary.

- **USB Mass Storage:** The device can act as a removable storage device. The device will be mounted automatically by most operating systems.

## Example Applications

- **HID Device:** Implement a simple USB HID device such as a custom game controller or a keyboard.
- **Virtual COM Port:** Use the USB CDC class to implement a serial communication interface with a PC for data logging or debugging purposes.
- **Mass Storage Device:** Create a mass storage device using an external flash or SD card connected to the NUC121.

## Customization

You can customize the project by modifying the USB device descriptors, endpoints, and class-specific functionality to suit your application.

- **Modify USB Descriptors:** Update the USB device, configuration, and interface descriptors in the code to define your device’s identity and capabilities.
- **Configure Endpoints:** Adjust the number and type of endpoints for different data transfer requirements.

## Contribution

Contributions to the project are welcome. Here’s how you can contribute:

1. **Fork the repository.**
2. **Create a branch for your feature:** (`git checkout -b feature-name`)
3. **Commit your changes:** (`git commit -m 'Add feature'`)
4. **Push to the branch:** (`git push origin feature-name`)
5. **Submit a pull request.**

## Troubleshooting

- **Device Not Recognized:** Ensure the USB cable is connected properly, and the correct device class is selected. Check if the host operating system has the appropriate drivers installed.
- **Communication Issues:** Verify the USB configuration, including endpoint settings and descriptor values. Ensure that interrupts are correctly handled.

## Future Enhancements

- **Multiple USB Classes:** Add support for composite USB devices that can handle more than one class simultaneously (e.g., HID + CDC).
- **Improved Power Management:** Optimize power usage during USB communication, especially for battery-powered applications.
- **Advanced USB Features:** Implement additional USB features such as OTG (On-The-Go) support or USB audio class.

## License

This project is licensed under the MIT License. For more details, see the [LICENSE](LICENSE) file.
