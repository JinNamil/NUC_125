SET PATH=C:\Keil_v5\ARM\ARMCC\Bin;C:\Program Files (x86)\Intel\iCLS Client\;C:\Program Files\Intel\iCLS Client\;C:\Windows\system32;C:\Windows;C:\Windows\System32\Wbem;C:\Windows\System32\WindowsPowerShell\v1.0\;C:\Program Files (x86)\Intel\Intel(R) Management Engine Components\DAL;C:\Program Files\Intel\Intel(R) Management Engine Components\DAL;C:\Program Files (x86)\Intel\Intel(R) Management Engine Components\IPT;C:\Program Files\Intel\Intel(R) Management Engine Components\IPT;C:\Program Files (x86)\GNU Tools Arm Embedded\7 2018-q2-update\bin;C:\Program Files\Git\cmd;C:\cygwin\bin;C:\Users\namil\AppData\Local\Microsoft\WindowsApps;C:\Program Files\Bandizip\
SET CPU_TYPE=NUC121SC2AE
SET CPU_VENDOR=Nuvoton
SET UV2_TARGET=HID_FIDO_and_MassStorage
SET CPU_CLOCK=0x02FAF080
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "lst\main._ip"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\main.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "lst\HID_FI~1._IP"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\hid_fido_and_massstorage.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "lst\DESCRI~1._IP"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\descriptors.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "lst\spiflash._ip"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\spiflash.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "lst\uspicomm._ip"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\uspicomm.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "lst\SYSTEM~1._IP"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\system_nuc121.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmAsm" --Via ".\obj\startup_nuc121._ia"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "lst\retarget._ip"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\retarget.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "lst\usbd._ip"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\usbd.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "lst\sys._ip"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\sys.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "lst\clk._ip"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\clk.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "lst\fmc._ip"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\fmc.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "lst\spi_i2s._ip"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\spi_i2s.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "lst\usci_spi._ip"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\usci_spi.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "lst\gpio._ip"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\obj\gpio.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmLink" --Via ".\obj\eWBM_HID_FIDO_and_MassStorage.lnp"
"C:\Keil_v5\ARM\ARMCC\Bin\fromelf.exe" ".\obj\eWBM_HID_FIDO_and_MassStorage.axf" --i32combined --output ".\obj\eWBM_HID_FIDO_and_MassStorage.hex"
fromelf --bin ".\obj\eWBM_HID_FIDO_and_MassStorage.axf" --output ".\obj\eWBM_HID_FIDO_and_MassStorage.bin"
fromelf --text -c ".\obj\eWBM_HID_FIDO_and_MassStorage.axf" --output ".\obj\eWBM_HID_FIDO_and_MassStorage.txt"
