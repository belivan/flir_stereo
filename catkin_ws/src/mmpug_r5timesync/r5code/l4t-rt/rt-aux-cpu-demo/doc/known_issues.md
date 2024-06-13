Known Issues {#rt-knownissues}
============
@tableofcontents

This section describes outstanding issues with the samples.

# Flashing Failure

When the SPE binary size is greater than 128-kilobytes, flashing fails.
The binary size includes the binary and NVIDIA-specific headers. To workaround
this issue follow below steps.

# For Jetson TX2

```
Open file flash_l4t_t186.xml, this file can be found under Linux_for_Tegra directory
Look for SPENAME and SPENAME_b partitions and change their size subsections from 131072 to 262144
```

# For Jetson AGX

```
Open file flash_t194_sdmmc.xml, this file can be found under Linux_for_Tegra directory
Look for spe-fw and spe-fw_b partitions and change their size subsections from 131072 to 262144
```

# SC7 Incompatibility

The SPE binary generated with this SDK is not compatible with
NVIDIA<sup>&reg;</sup> SC7.