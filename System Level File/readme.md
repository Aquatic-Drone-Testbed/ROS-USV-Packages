# DHCP Server Setup Guide

This guide provides detailed steps for setting up a DHCP server, including configuring network settings, managing the DHCP service, and specifying DHCP server options.

prerequire installation:
  ```bash
  apt install isc-dhcp-server`
  ```

## 1. Network Configuration Preparation

### Configure Netplan

- **Set Permissions for Netplan Configuration File `/etc/netplan/01-use-network-manager.yaml`:** 

  permissions can be temporarily adjusted to:

    ```bash
    chmod 666 /etc/netplan/01-use-network-manager.yaml
    ```

  Ensure  has the correct permissions for security and accessibility after you modifity it:

    ```bash
    sudo chmod 644 /etc/netplan/01-use-network-manager.yaml
    ```

## 2. Static IP Address Configuration on Raspberry Pi (Server)
- **Update `/etc/netplan/01-use-network-manager.yaml`:** Adjust network settings for your DHCP server environment.

    ```yaml
    network:
      version: 2
      renderer: NetworkManager
      ethernets:
        eth0:
          addresses:
            - 193.168.1.2/24
          routes:
            - to: default
              via: 193.168.1.1
          nameservers:
              addresses: [8.8.8.8, 8.8.4.4]
    ```

    Apply the changes:

    ```bash
    sudo netplan apply
    sudo systemctl restart NetworkManager
    ```

## 3. DHCP Server Configuration and Management

- **Configure `/etc/default/isc-dhcp-server`:** Designate which network interfaces the DHCP server should use.

    ```plaintext
    INTERFACESv4="eth0"
    INTERFACESv6=""
    ```
- **check if the port is up**:
    ```bash
    nmcli device status
    ```
- **debug mode apply**:
    ```bash
    sudo netplan --debug apply
    ```
- **show netplan-eth0 details**:
    ```bash
    nmcli connection show netplan-eth0
    ```
- **Configure `dhcp.conf`:** Set up and manage the DHCP server service to start automatically and check its status.
    
    Enable isc-dhcp-server at boot:

    ```bash
    sudo systemctl enable isc-dhcp-server
    ```

    **Optional** if the DHCP server isn't already running:

    ```bash
    sudo systemctl start isc-dhcp-server
    ```

    Check the DHCP server service status, it should be `alive`:

    ```bash
    sudo systemctl status isc-dhcp-server
    ```

    **Optional:** Use `nmap` to scan the network:

    ```bash
    sudo nmap -sn 193.168.1.0/24
    ```

### DHCP Server Configuration File

- **Edit `/etc/dhcp/dhcp.conf`:** Provide DHCP server with a range of IP addresses to distribute and specify network settings.

    ```plaintext
    subnet 193.168.1.0 netmask 255.255.255.0 {
      range 193.168.1.10 193.168.1.49;
      range 193.168.1.51 193.168.1.100;
      option domain-name-servers 8.8.8.8;
      option routers 193.168.1.1;
      option broadcast-address 193.168.1.255;
      default-lease-time 6000;
      max-lease-time 7200;
    }
    host radar {
      hardware ethernet C4:93:00:2C:A9:81;
      fixed-address 193.168.1.50;
    }
    ```

### Specify Interfaces for the DHCP Server


## Netplan Configuration for Network Management
