LTE&5G diag tool.

## Version History


1. V20210421_001
    Mandatory use qmdl v2.
2. V20210421_002
    Fix the issue of log coverage.
3. V20210521_001
    Add loop mode when diaggrab running.
4. V20210521_003
    Remove some compile time warnings.


## Usage: diaggrab [options]

Examples:
Start diaggrab
	diaggrab
Start diaggrab with maximum file num 312
	diaggrab -n 312

Options:
 Usage for ./diaggrab:

-h  --help:	 usage help

-p  --port:	 TTY device to use. Example /dev/ttyUSB0

-s  --size:	 maximum file size in MB, default is 100

-n  --lognum:	 maximum file num[0-512], default is 0. 0 means no limit.

-c  --filemdm:	 mask file name for MDM

-u, --qmdl2_v2:	 Guid-diagid mapping in qmdl2 header

