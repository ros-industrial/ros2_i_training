# Networking basics

ROS 2 systems often communicate across a network, for example between a laptop
and a robot computer. This page introduces the Linux networking commands used
in the workshop.

## IP addresses

Each network interface normally has an IP address. An address such as
`192.168.3.42` identifies a computer on a local network.

Show the interfaces and their addresses:

```bash
ip addr
```

Show only the addresses assigned to the computer:

```bash
hostname -I
```

Interface names vary between computers. Typical names include:

- `lo`: the local loopback interface
- `enp...` or `eth...`: an Ethernet interface
- `wlp...` or `wlan...`: a Wi-Fi interface

The older `ifconfig` command may not be installed by default. Prefer
`ip addr` on current Ubuntu systems.

## Check connectivity with ping

`ping` tests whether another machine is reachable:

```bash
ping -c 4 192.168.3.42
```

The `-c 4` option stops after four requests. Without it, press `Ctrl+C` to
stop the command.

A successful response looks similar to:

```text
64 bytes from 192.168.3.42: icmp_seq=1 ttl=64 time=1.23 ms
```

A failed ping can mean that the target is offline, connected to a different
network, using a different address, or blocking ping with a firewall.

## Connect with SSH

SSH opens a secure terminal on another computer:

```bash
ssh student@192.168.3.42
```

The first connection may ask you to confirm the remote host fingerprint. Verify
the address before accepting it. Leave the remote session with:

```bash
exit
```

## Copy files with scp

Copy a local file to a remote computer:

```bash
scp my_file.txt student@192.168.3.42:/home/student/
```

Copy a remote file into the current local directory:

```bash
scp student@192.168.3.42:/home/student/my_file.txt .
```

Copy a directory recursively:

```bash
scp -r my_directory student@192.168.3.42:/home/student/
```

## ROS 2 networking checklist

For two ROS 2 computers to discover each other:

1. Connect them to the same network.
2. Confirm that they can reach each other.
3. Use compatible ROS 2 distributions and middleware configurations.
4. Set the same `ROS_DOMAIN_ID` on machines that should communicate.
5. Check firewall and multicast settings if discovery still fails.

Display the configured domain:

```bash
printenv ROS_DOMAIN_ID
```

## Exercise

1. Connect to the network specified by the instructor.
2. Find your computer's IP address with `ip addr` or `hostname -I`.
3. Exchange IP addresses with another participant.
4. Test both directions with `ping`.
5. If SSH is enabled, connect to the other computer using the account provided
   by the instructor.
