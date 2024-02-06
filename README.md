# PSoC&trade; 6 MCU: GPIO pins

This example demonstrates the GPIO pin operation on the PSoC™ 6 MCU, using Eclipse IDE for ModusToolbox™. This includes reading, writing, interrupts, and full port configurations.

[Provide feedback on this code example.](https://cypress.co1.qualtrics.com/jfe/form/SV_1NTns53sK2yiljn?Q_EED=eyJVbmlxdWUgRG9jIElkIjoiQ0UyMjAyNjMiLCJTcGVjIE51bWJlciI6IjAwMi0yMDI2MyIsIkRvYyBUaXRsZSI6IlBTb0MmdHJhZGU7IDYgTUNVOiBHUElPIHBpbnMiLCJyaWQiOiJxbmNsIiwiRG9jIHZlcnNpb24iOiIyLjAuMCIsIkRvYyBMYW5ndWFnZSI6IkVuZ2xpc2giLCJEb2MgRGl2aXNpb24iOiJNQ0QiLCJEb2MgQlUiOiJJQ1ciLCJEb2MgRmFtaWx5IjoiUFNPQyJ9)

## Requirements

- [ModusToolbox&trade; software](https://www.infineon.com/modustoolbox) v3.0 or later (tested with v3.0)
- Board support package (BSP) minimum required version: 4.0.0
- Programming language: C
- Associated parts: All [PSoC&trade; 6 MCU](https://www.infineon.com/PSoC6) parts

## Supported toolchains (make variable 'TOOLCHAIN')

- GNU Arm&reg; embedded compiler v10.3.1 (`GCC_ARM`) - Default value of `TOOLCHAIN`
- Arm&reg; compiler v6.16 (`ARM`)
- IAR C/C++ compiler v9.30.1 (`IAR`)


## Supported kits (make variable 'TARGET')

- [PSoC&trade; 6 Wi-Fi Bluetooth&reg; prototyping kit](https://www.infineon.com/CY8CPROTO-062-4343W) (`CY8CPROTO-062-4343W`) – Default value of `TARGET`
- [PSoC&trade; 6 Wi-Fi Bluetooth&reg; pioneer kit](https://www.infineon.com/CY8CKIT-062-WIFI-BT) (`CY8CKIT-062-WIFI-BT`)
- [PSoC&trade; 6 Bluetooth&reg; LE pioneer kit](https://www.infineon.com/CY8CKIT-062-BLE) (`CY8CKIT-062-BLE`)
- [PSoC&trade; 6 Bluetooth&reg; LE prototyping kit](https://www.infineon.com/CY8CPROTO-063-BLE) (`CY8CPROTO-063-BLE`)
- [PSoC&trade; 62S2 Wi-Fi Bluetooth&reg; pioneer kit](https://www.infineon.com/CY8CKIT-062S2-43012) (`CY8CKIT-062S2-43012`)
- [PSoC&trade; 62S1 Wi-Fi Bluetooth&reg; pioneer kit](https://www.infineon.com/CYW9P62S1-43438EVB-01) (`CYW9P62S1-43438EVB-01`)
- [PSoC&trade; 62S1 Wi-Fi Bluetooth&reg; pioneer kit](https://www.infineon.com/CYW9P62S1-43012EVB-01) (`CYW9P62S1-43012EVB-01`)
- [PSoC&trade; 62S3 Wi-Fi Bluetooth&reg; prototyping kit](https://www.infineon.com/CY8CPROTO-062S3-4343W) (`CY8CPROTO-062S3-4343W`)
- [PSoC&trade; 64 "Secure Boot" Wi-Fi Bluetooth&reg; pioneer kit](https://www.infineon.com/CY8CKIT-064B0S2-4343W) (`CY8CKIT-064B0S2-4343W`)
- [PSoC&trade; 62S4 pioneer kit](https://www.infineon.com/CY8CKIT-062S4) (`CY8CKIT-062S4`)
- [PSoC&trade; 62S2 evaluation kit](https://www.infineon.com/CY8CEVAL-062S2) (`CY8CEVAL-062S2`, `CY8CEVAL-062S2-LAI-4373M2`)

## Hardware setup

This example uses the board's default configuration. See the kit user guide to ensure that the board is configured correctly.

**Note:** The PSoC&trade; 6 Bluetooth&reg; LE pioneer kit (CY8CKIT-062-BLE) and the PSoC&trade; 6 Wi-Fi Bluetooth&reg; pioneer kit (CY8CKIT-062-WIFI-BT) ship with KitProg2 installed. The ModusToolbox&trade; software requires KitProg3. Before using this code example, make sure that the board is upgraded to KitProg3. The tool and instructions are available in the [Firmware loader](https://github.com/Infineon/Firmware-loader) GitHub repository. If you do not upgrade, you will see an error like "unable to find CMSIS-DAP device" or "KitProg firmware is out of date".


## Software setup

This example requires no additional software or tools.

## Using the code example

Create the project and open it using one of the following:

<details><summary><b>In Eclipse IDE for ModusToolbox&trade; software</b></summary>

1. Click the **New Application** link in the **Quick Panel** (or, use **File** > **New** > **ModusToolbox&trade; Application**). This launches the [Project Creator](https://www.infineon.com/ModusToolboxProjectCreator) tool.

2. Pick a kit supported by the code example from the list shown in the **Project Creator - Choose Board Support Package (BSP)** dialog.

   When you select a supported kit, the example is reconfigured automatically to work with the kit. To work with a different supported kit later, use the [Library Manager](https://www.infineon.com/ModusToolboxLibraryManager) to choose the BSP for the supported kit. You can use the Library Manager to select or update the BSP and firmware libraries used in this application. To access the Library Manager, click the link from the **Quick Panel**.

   You can also just start the application creation process again and select a different kit.

   If you want to use the application for a kit not listed here, you may need to update the source files. If the kit does not have the required resources, the application may not work.

3. In the **Project Creator - Select Application** dialog, choose the example by enabling the checkbox.

4. (Optional) Change the suggested **New Application Name**.

5. The **Application(s) Root Path** defaults to the Eclipse workspace which is usually the desired location for the application. If you want to store the application in a different location, you can change the *Application(s) Root Path* value. Applications that share libraries should be in the same root path.

6. Click **Create** to complete the application creation process.

For more details, see the [Eclipse IDE for ModusToolbox&trade; software user guide](https://www.infineon.com/MTBEclipseIDEUserGuide) (locally available at *{ModusToolbox&trade; software install directory}/docs_{version}/mt_ide_user_guide.pdf*).

</details>

<details><summary><b>In command-line interface (CLI)</b></summary>

ModusToolbox&trade; software provides the Project Creator as both a GUI tool and the command line tool, "project-creator-cli". The CLI tool can be used to create applications from a CLI terminal or from within batch files or shell scripts. This tool is available in the *{ModusToolbox&trade; software install directory}/tools_{version}/project-creator/* directory.

Use a CLI terminal to invoke the "project-creator-cli" tool. On Windows, use the command line "modus-shell" program provided in the ModusToolbox&trade; software installation instead of a standard Windows command-line application. This shell provides access to all ModusToolbox&trade; software tools. You can access it by typing `modus-shell` in the search box in the Windows menu. In Linux and macOS, you can use any terminal application.

The "project-creator-cli" tool has the following arguments:

Argument | Description | Required/optional
---------|-------------|-----------
`--board-id` | Defined in the `<id>` field of the [BSP](https://github.com/Infineon?q=bsp-manifest&type=&language=&sort=) manifest | Required
`--app-id`   | Defined in the `<id>` field of the [CE](https://github.com/Infineon?q=ce-manifest&type=&language=&sort=) manifest | Required
`--target-dir`| Specify the directory in which the application is to be created if you prefer not to use the default current working directory | Optional
`--user-app-name`| Specify the name of the application if you prefer to have a name other than the example's default name | Optional

<br />

The following example clones the "[mtb-example-psoc6-gpio-pins](https://github.com/Infineon/mtb-example-psoc6-gpio-pins)" application with the desired name "GpioPins" configured for the *CY8CKIT-062-WIFI-BT* BSP into the specified working directory, *C:/mtb_projects*:

   ```
   project-creator-cli --board-id CY8CKIT-062-WIFI-BT --app-id mtb-example-psoc6-gpio-pins --user-app-name GpioPins --target-dir "C:/mtb_projects"
   ```

**Note:** The project-creator-cli tool uses the `git clone` and `make getlibs` commands to fetch the repository and import the required libraries. For details, see the "Project creator tools" section of the [ModusToolbox&trade; software user guide](https://www.cypress.com/ModusToolboxUserGuide) (locally available at *{ModusToolbox&trade; software install directory}/docs_{version}/mtb_user_guide.pdf*).

To work with a different supported kit later, use the [Library Manager](https://www.infineon.com/ModusToolboxLibraryManager) to choose the BSP for the supported kit. You can invoke the Library Manager GUI tool from the terminal using `make modlibs` command or use the Library Manager CLI tool "library-manager-cli" to change the BSP.

The "library-manager-cli" tool has the following arguments:

Argument | Description | Required/optional
---------|-------------|-----------
`--add-bsp-name` | Name of the BSP that should be added to the application | Required
`--set-active-bsp` | Name of the BSP that should be as active BSP for the application | Required
`--add-bsp-version`| Specify the version of the BSP that should be added to the application if you do not wish to use the latest from manifest | Optional
`--add-bsp-location`| Specify the location of the BSP (local/shared) if you prefer to add the BSP in a shared path | Optional

<br />

Following example adds the CY8CPROTO-062-4343W BSP to the already created application and makes it the active BSP for the app:

   ```
   library-manager-cli --project "C:/mtb_projects/MyHelloWorld" --add-bsp-name CY8CPROTO-062-4343W --add-bsp-version "latest-v4.X" --add-bsp-location "local"

   library-manager-cli --project "C:/mtb_projects/MyHelloWorld" --set-active-bsp APP_CY8CPROTO-062-4343W
   ```

</details>

<details><summary><b>In third-party IDEs</b></summary>

Use one of the following options:

- **Use the standalone [Project Creator](https://www.infineon.com/ModusToolboxProjectCreator) tool:**

   1. Launch Project Creator from the Windows Start menu or from *{ModusToolbox&trade; software install directory}/tools_{version}/project-creator/project-creator.exe*.

   2. In the initial **Choose Board Support Package** screen, select the BSP, and click **Next**.

   3. In the **Select Application** screen, select the appropriate IDE from the **Target IDE** drop-down menu.

   4. Click **Create** and follow the instructions printed in the bottom pane to import or open the exported project in the respective IDE.

<br />

- **Use command-line interface (CLI):**

   1. Follow the instructions from the **In command-line interface (CLI)** section to create the application.

   2. Export the application to a supported IDE using the `make <ide>` command.

   3. Follow the instructions displayed in the terminal to create or import the application as an IDE project.

For a list of supported IDEs and more details, see the "Exporting to IDEs" section of the [ModusToolbox&trade; software user guide](https://www.infineon.com/ModusToolboxUserGuide) (locally available at *{ModusToolbox&trade; software install directory}/docs_{version}/mtb_user_guide.pdf*).

</details>


## Operation

If using a PSoC&trade; 64 "Secure" MCU kit (like CY8CKIT-064B0S2-4343W), the PSoC&trade; 64 device must be provisioned with keys and policies before being programmed. Follow the instructions in the ["Secure Boot" SDK user guide](https://www.cypress.com/documentation/software-and-drivers/psoc-64-secure-mcu-secure-boot-sdk-user-guide) to provision the device. If the kit is already provisioned, copy-paste the keys and policy folder to the application folder.

1. Connect the board to your PC using the provided USB cable through the KitProg3 USB connector.

2. Open a terminal program and select the KitProg3 COM port. Set the serial port parameters to 8N1 and 115200 baud.

3. Program the board using one of the following:

   <details><summary><b>Using Eclipse IDE for ModusToolbox&trade; software</b></summary>

      1. Select the application project in the Project Explorer.

      2. In the **Quick Panel**, scroll down, and click **\<Application Name> Program (KitProg3_MiniProg4)**.
   </details>

   <details><summary><b>Using CLI</b></summary>

     From the terminal, execute the `make program` command to build and program the application using the default toolchain to the default target. The default toolchain is specified in the application's Makefile but you can override this value manually:
      ```
      make program TOOLCHAIN=<toolchain>
      ```

      Example:
      ```
      make program TOOLCHAIN=GCC_ARM
      ```
   </details>

4. After programming, the application starts automatically. Confirm that "\<CE Title>" is displayed on the UART terminal.

4. Press the User button (**KIT_BTN 1**) and observe the User LED (KIT_LED1) turn ON while pressed, demonstrating the GPIO read and write function.

5. Release the User button and observe that the User LED turns OFF and then the User LED blinks twice demonstrating the pin interrupt functionality.

6. Open the *main.c* file in your editor and do the following:

   1. At the top of the file, locate the `EXAMPLE_SELECTION` macro.

   2. Change the value from the default of `HAL_EXAMPLE` to `PDL_EXAMPLE`.

      This selects the example that is written using the [PSoC&trade; 6 peripheral drive library](https://infineon.github.io/mtb-pdl-cat1/pdl_api_reference_manual/html/index.html).

7. Repeat steps 2 through 5 to run the PDL_EXAMPLE.

## Debugging

You can debug the example to step through the code. In the IDE, use the **\<Application name> debug (KitProg3_MiniProg4)** configuration in the **Quick panel**. For details, see the "Program and debug" section in the [Eclipse IDE for ModusToolbox&trade; software user guide](https://www.infineon.com/MTBEclipseIDEUserGuide).

**Note:** **(Only while debugging)** On the CM4 CPU, some code in `main()` may execute before the debugger halts at the beginning of `main()`. This means that some code executes twice – once before the debugger stops execution, and again after the debugger resets the program counter to the beginning of `main()`. See [KBA231071](https://community.infineon.com/docs/DOC-21143) to learn about this and for the workaround.


## Design and implementation

### Overview

This example demonstrates the GPIO pin configuration, reading, writing, full port access, and interrupts using multiple GPIO PDL and HAL driver methods. The PDL and HAL examples are run by changing `#Define EXAMPLE_SELECTION` at the top of the *main.c* file. This example shows the different implementations of using the GPIO pins to meet the needs of the project.

To demonstrate individual GPIO pin access, this example reads the value from the reference pin (User button) and writes it to the User LED. The User LED blinks twice to demonstrate various GPIO functions. The User button is configured to generate an interrupt on a falling edge, which occurs on a button release. The interrupt routine sets a flag to run the blinking sequence that is in the example loop. This design and implementation section is separated into PDL and HAL sections.

### PDL

See the [PSoC&trade; 6 peripheral driver library API documentation](https://infineon.github.io/mtb-pdl-cat1/pdl_api_reference_manual/html/index.html).

#### Pin configuration

Device configuration tools such as ModusToolbox&trade; Device Configurator automatically generates the GPIO configuration code and execute it as part of the device boot process. See the [ModusToolbox&trade; device configurator guide](https://www.infineon.com/file/512731/download) for details.

GPIO PDL initialization methods are typically only used with manual PDL GPIO configuration when not using a configuration tool. They may also be used at run time to dynamically reconfigure GPIO pins independent of how the initial configuration was performed.

Most GPIO pins require only their basic parameters to be set and can use default values for all other settings. This allows the use of a simplified initialization function.

[Cy_GPIO_Pin_FastInit ()](https://infineon.github.io/mtb-pdl-cat1/pdl_api_reference_manual/html/group__group__gpio__functions__init.html#gaf57c501727276013d3e8974a9fb7d0a7) supports only parameterized configuration of drive mode, output logic level, and High Speed Input/Output Multiplexer (HSIOM) setting. The HSIOM setting determines a pin’s high-level software, peripheral, analog control, and connectivity. All other configuration settings are unchanged from their reset or previously set state. This function is very useful at run time to dynamically change a pin's configuration.

For example, to configure a pin to strong drive mode to write data, and then reconfigure the pin as High-Z to read data, use the following snippet:

```
/* Initialize USER_LED */
Cy_GPIO_Pin_FastInit(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN, CY_GPIO_DM_STRONG, 1UL, HSIOM_SEL_GPIO);
```

A method to configure all attributes of a single pin is to use the [Cy_GPIO_Pin_Init ()](https://infineon.github.io/mtb-pdl-cat1/pdl_api_reference_manual/html/group__group__gpio__functions__init.html#gad61553f65d4e6bd827eb6464a7913461https://infineon.github.io/mtb-pdl-cat1/pdl_api_reference_manual/html/group__group__gpio__functions__init.html#gad61553f65d4e6bd827eb6464a7913461) function and a pin configuration structure. While easy to use, it generates a larger code than other configuration methods.

```
Cy_GPIO_Pin_Init(P0_4_PORT, P0_4_PIN, &P0_4_Pin_Init);
```

The most code-efficient method to configure all attributes for a full port of pins is to use the `Cy_GPIO_Port_Init()` function and a port configuration structure. It packs all the configuration data into direct register writes for the whole port. Its limitation is that it must configure all pins in a port and the user must calculate the combined register values for all pins.

```
Cy_GPIO_Port_Init(GPIO_PRT7, &port7_Init);
```

#### Pin read

The following methods all perform the same read from a GPIO pin by different available read methods. Choose the most appropriate method for your specific use case. The [Cy_GPIO_Read ()](https://infineon.github.io/mtb-pdl-cat1/pdl_api_reference_manual/html/group__group__gpio__functions__gpio.html#ga3d44507453ea90abde3013c1ac867df8) function is thread- and multi-core-safe.

Most GPIO driver functions require a minimum of two arguments to define the port and pin in that port. The port argument expects the base address of the port’s registers. The pin argument expects the pin number within the port.

```c++
Cy_GPIO_Read(CYBSP_USER_BTN_PORT, CYBSP_USER_BTN_PIN);
```


Pin reads using port and pin numbers are also supported. This method is useful for algorithmically generated port and pin numbers. `Cy_GPIO_PortToAddr()` is a helper function that converts the port number into the required port register base address required by other GPIO driver functions.

```c++
portNumber = 0;
pinReadValue = Cy_GPIO_Read(Cy_GPIO_PortToAddr(portNumber), 4);
```

Like any MCU, direct port register access is always available and useful for accessing multiple pins in a port simultaneously or developing application-optimized port accesses. The following example shows a port IN register read with mask and shift of the desired pin data:

```c++
pinReadValue = (GPIO_PRT0->IN >> P0_4_NUM) & CY_GPIO_IN_MASK;
```

#### Pin write

The PDL API documentation provides multiple ways of writing to GPIO pins. The main method is to use the [Cy_GPIO_Write()](https://infineon.github.io/mtb-pdl-cat1/pdl_api_reference_manual/html/group__group__gpio__functions__gpio.html#ga849c813d6771bf8d3c59b89b28a07bca) function.

```c++
Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN, CYBSP_LED_STATE_OFF);
```

The pin Invert function inverts the current state of the pin:

```c++
Cy_GPIO_Inv(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);
```

The pin Clear function sets the pin output to logic state LOW:

```c++
Cy_GPIO_Clr(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);
```

The pin Set function sets the pin output to logic state HIGH:

```c++
Cy_GPIO_Set(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);
```

#### Port access

Direct register access is used to interface with multiple pins in one port at the same time. These accesses may not be thread- or multi-core-safe due to possible read-modify-write operations. All pins in a port under direct register control should be accessed only by a single CPU core unless access protections are provided at the system level.

```c++
portReadValue = GPIO_PRT7->IN;

portReadValue++;

GPIO_PRT7->OUT = portReadValue;
```

#### Pin interrupts

To generate a pin interrupt, configure it to trigger on a rising, falling, or both edges, and mask it so that the pin signal is sent to the interrupt controller vector for that port.

```c++
Cy_GPIO_SetInterruptEdge(CYBSP_USER_BTN_PORT, CYBSP_USER_BTN_PIN, CY_GPIO_INTR_RISING);
Cy_GPIO_SetInterruptMask(CYBSP_USER_BTN_PORT, CYBSP_USER_BTN_PIN, CY_GPIO_INTR_EN_MASK);
```

The port interrupt vector must then be configured, cleared, and enabled to be triggered from the port interrupt signal and mapped to the desired interrupt service routine (ISR). See the [PDL Cy_SysInt()](https://infineon.github.io/mtb-pdl-cat1/pdl_api_reference_manual/html/group__group__sysint__functions.html#gab2ff6820a898e9af3f780000054eea5d) documentation for more information on interrupt configuration and use.

```c++
/* Configure CM4+ CPU GPIO interrupt vector for Port 0 */
Cy_SysInt_Init(&intrCfg, gpio_interrupt_handler_PDL);
NVIC_ClearPendingIRQ(intrCfg.intrSrc);
NVIC_EnableIRQ((IRQn_Type)intrCfg.intrSrc);
```

After an interrupt occurs, the pin interrupt must be cleared before exiting the ISR so that the edge detection logic is reset to allow the detection of the next edge:

```c++
static void gpio_interrupt_handler_PDL()
{
   gpio_intr_flag = true;

   /* Clear pin interrupt logic. Required to detect next interrupt */
   Cy_GPIO_ClearInterrupt(CYHAL_GET_PORTADDR(CYBSP_USER_BTN), CYHAL_GET_PIN(CYBSP_USER_BTN));

}
```

### HAL

For more details on using HAL, see the [PSoC&trade; 6 hardware abstraction layer (HAL) library API documentation](https://infineon.github.io/mtb-hal-cat1/html/index.html).

The HAL provides a high-level interface to configure and use hardware blocks on PSoC&trade; MCUs. It is a generic interface that can be used across multiple product families. The focus on ease-of-use and portability means the HAL does not expose all of the low-level peripheral functionality.

#### Pin configuration

When using HAL functions, the configuration of the pins must be done using a HAL initialization function. To initialize a GPIO pin, use the [cyhal_gpio_init()](https://infineon.github.io/mtb-hal-cat1/html/group__group__hal__gpio.html#gab93322030909d3af6a9fc1a3b2eccbaa) function:

```c++
/* Initialize the user LED  */
result = cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
```

#### Pin read

To read the state of a pin, use the [cyhal_gpio_read()](https://infineon.github.io/mtb-hal-cat1/html/group__group__hal__gpio.html#ga3d38f1dd86c2e74b8ad10f1f6d1945c2) function:

```c++
/* Read current button state from the user button */
read_val = cyhal_gpio_read(CYBSP_USER_BTN);
```

#### Pin write

To write to a pin, use the [cyhal_gpio_write()](https://infineon.github.io/mtb-hal-cat1/html/group__group__hal__gpio.html#gaf66e7c3ed7b3b0711635d7687ae92291) function:

```c++
/* LED OFF */
cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_OFF);
```

#### Pin interrupts

To configure an interrupt in HAL, two functions must be called:

First, call [cyhal_gpio_register_callback()](https://infineon.github.io/mtb-hal-cat1/html/group__group__hal__gpio.html#gaefb03ec6e090ec57fae9e79b4d3bfb4e) to register/clear a callback handler for pin events. This function will be called when one of the events enabled by [cyhal_gpio_enable_event()](https://infineon.github.io/mtb-hal-cat1/html/group__group__hal__gpio.html#ga0e0346810451d9940d31bb6111153593) occurs.

```
cyhal_gpio_register_callback(CYBSP_USER_BTN, gpio_interrupt_handler_HAL, NULL);
```

The second function is [cyhal_gpio_enable_event()](https://infineon.github.io/mtb-hal-cat1/html/group__group__hal__gpio.html#ga0e0346810451d9940d31bb6111153593), which enables or disables the specified GPIO event. When an enabled event occurs, the function specified by [cyhal_gpio_register_callback()](https://infineon.github.io/mtb-hal-cat1/html/group__group__hal__gpio.html#gaefb03ec6e090ec57fae9e79b4d3bfb4e) will be called.

```
cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_RISE, GPIO_INTERRUPT_PRIORITY, true);
```

### Resources and settings

**Table 1. Application resources**

| Resource  |  Alias/object     |    Purpose     |
| :------- | :------------    | :------------ |
| GPIO (BSP) | CYBSP_USER_LED    | User LED to show the output           |
| GPIO (BSP) | CYBSP_USER_BTN | User button to generate the interrupt |
| GPIO(BSP)  | GPIO_PRT5         | Simultaneous port pin access          |

<br>

## Related resources

Resources  | Links
-----------|----------------------------------
Application notes  | [AN228571](https://www.infineon.com/AN228571) – Getting started with PSoC&trade; 6 MCU on ModusToolbox&trade; software <br>  [AN215656](https://www.infineon.com/AN215656) – PSoC&trade; 6 MCU: Dual-CPU system design <br>
Code examples  | [Using ModusToolbox&trade; software](https://github.com/Infineon/Code-Examples-for-ModusToolbox-Software) on GitHub <br> 
Device documentation | [PSoC&trade; 6 MCU datasheets](https://www.infineon.com/cms/en/product/microcontroller/32-bit-psoc-arm-cortex-microcontroller/psoc-6-32-bit-arm-cortex-m4-mcu/#!documents) <br> [PSoC&trade; 6 technical reference manuals](https://www.infineon.com/cms/en/product/microcontroller/32-bit-psoc-arm-cortex-microcontroller/psoc-6-32-bit-arm-cortex-m4-mcu/#!documents)<br> 
Development kits | Select your kits from the [evaluation board finder](https://www.infineon.com/cms/en/design-support/finder-selection-tools/product-finder/evaluation-board) page
Libraries on GitHub  | [mtb-pdl-cat1](https://github.com/Infineon/mtb-pdl-cat1) – PSoC&trade; 6 peripheral driver library (PDL)  <br> [mtb-hal-cat1](https://github.com/Infineon/mtb-hal-cat1) – Hardware abstraction layer (HAL) library <br> [retarget-io](https://github.com/Infineon/retarget-io) – Utility library to retarget STDIO messages to a UART port <br> [mtb-hal-cat2](https://github.com/Infineon/mtb-hal-cat2) – Hardware abstraction layer (HAL) library
Middleware on GitHub  | [capsense](https://github.com/Infineon/capsense) – CAPSENSE&trade; library and documents <br> [psoc6-middleware](https://github.com/Infineon/modustoolbox-software#psoc-6-middleware-libraries) – Links to all PSoC&trade; 6 MCU middleware
Tools  | [Eclipse IDE for ModusToolbox&trade; software](https://www.infineon.com/modustoolbox) – ModusToolbox&trade; software is a collection of easy-to-use software and tools enabling rapid development with Infineon MCUs, covering applications from embedded sense and control to wireless and cloud-connected systems using AIROC&trade; Wi-Fi and Bluetooth® connectivity devices. <br> [PSoC&trade; Creator](https://www.infineon.com/products/psoc-creator-integrated-design-environment-ide) – IDE for PSoC&trade; and FM0+ MCU development

<br>

## Other resources

Infineon provides a wealth of data at www.infineon.com to help you select the right device, and quickly and effectively integrate it into your design.

For PSoC&trade; 6 MCU devices, see [How to design with PSoC&trade; 6 MCU - KBA223067](https://community.infineon.com/docs/DOC-14644) in the Infineon Developer community.

## Document history

Document title: *CE220263* - *PSoC&trade; 6 MCU GPIO pins*

| Version | Description of change |
| ------- | --------------------- |
| 1.0.0   | New code example      |
| 2.0.0   | Major update to support ModusToolbox™ v3.0 and BSPs v4.X.<br> This version is not backward compatible with previous versions of ModusToolbox  |

<br>


---------------------------------------------------------

© Cypress Semiconductor Corporation, 2020-2022. This document is the property of Cypress Semiconductor Corporation, an Infineon Technologies company, and its affiliates ("Cypress").  This document, including any software or firmware included or referenced in this document ("Software"), is owned by Cypress under the intellectual property laws and treaties of the United States and other countries worldwide.  Cypress reserves all rights under such laws and treaties and does not, except as specifically stated in this paragraph, grant any license under its patents, copyrights, trademarks, or other intellectual property rights.  If the Software is not accompanied by a license agreement and you do not otherwise have a written agreement with Cypress governing the use of the Software, then Cypress hereby grants you a personal, non-exclusive, nontransferable license (without the right to sublicense) (1) under its copyright rights in the Software (a) for Software provided in source code form, to modify and reproduce the Software solely for use with Cypress hardware products, only internally within your organization, and (b) to distribute the Software in binary code form externally to end users (either directly or indirectly through resellers and distributors), solely for use on Cypress hardware product units, and (2) under those claims of Cypress’s patents that are infringed by the Software (as provided by Cypress, unmodified) to make, use, distribute, and import the Software solely for use with Cypress hardware products.  Any other use, reproduction, modification, translation, or compilation of the Software is prohibited.
<br />
TO THE EXTENT PERMITTED BY APPLICABLE LAW, CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH REGARD TO THIS DOCUMENT OR ANY SOFTWARE OR ACCOMPANYING HARDWARE, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  No computing device can be absolutely secure.  Therefore, despite security measures implemented in Cypress hardware or software products, Cypress shall have no liability arising out of any security breach, such as unauthorized access to or use of a Cypress product. CYPRESS DOES NOT REPRESENT, WARRANT, OR GUARANTEE THAT CYPRESS PRODUCTS, OR SYSTEMS CREATED USING CYPRESS PRODUCTS, WILL BE FREE FROM CORRUPTION, ATTACK, VIRUSES, INTERFERENCE, HACKING, DATA LOSS OR THEFT, OR OTHER SECURITY INTRUSION (collectively, "Security Breach").  Cypress disclaims any liability relating to any Security Breach, and you shall and hereby do release Cypress from any claim, damage, or other liability arising from any Security Breach.  In addition, the products described in these materials may contain design defects or errors known as errata which may cause the product to deviate from published specifications. To the extent permitted by applicable law, Cypress reserves the right to make changes to this document without further notice. Cypress does not assume any liability arising out of the application or use of any product or circuit described in this document. Any information provided in this document, including any sample design information or programming code, is provided only for reference purposes.  It is the responsibility of the user of this document to properly design, program, and test the functionality and safety of any application made of this information and any resulting product.  "High-Risk Device" means any device or system whose failure could cause personal injury, death, or property damage.  Examples of High-Risk Devices are weapons, nuclear installations, surgical implants, and other medical devices.  "Critical Component" means any component of a High-Risk Device whose failure to perform can be reasonably expected to cause, directly or indirectly, the failure of the High-Risk Device, or to affect its safety or effectiveness.  Cypress is not liable, in whole or in part, and you shall and hereby do release Cypress from any claim, damage, or other liability arising from any use of a Cypress product as a Critical Component in a High-Risk Device. You shall indemnify and hold Cypress, including its affiliates, and its directors, officers, employees, agents, distributors, and assigns harmless from and against all claims, costs, damages, and expenses, arising out of any claim, including claims for product liability, personal injury or death, or property damage arising from any use of a Cypress product as a Critical Component in a High-Risk Device. Cypress products are not intended or authorized for use as a Critical Component in any High-Risk Device except to the limited extent that (i) Cypress’s published data sheet for the product explicitly states Cypress has qualified the product for use in a specific High-Risk Device, or (ii) Cypress has given you advance written authorization to use the product as a Critical Component in the specific High-Risk Device and you have signed a separate indemnification agreement.
<br />
Cypress, the Cypress logo, and combinations thereof, WICED, ModusToolbox, PSoC, CapSense, EZ-USB, F-RAM, and Traveo are trademarks or registered trademarks of Cypress or a subsidiary of Cypress in the United States or in other countries. For a more complete list of Cypress trademarks, visit cypress.com. Other names and brands may be claimed as property of their respective owners.
