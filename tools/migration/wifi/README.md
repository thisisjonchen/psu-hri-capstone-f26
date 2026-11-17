# Duckiedrone Wi-Fi startup

`startup.patch` is for DD24 Pi hosts using `dhcpcd`, `wpa_supplicant` and the
Duckietown `wifi-access-point` Compose service. It gives `wlan0` one DHCP client,
loads the saved Wi-Fi settings before association, and keeps the client
connection up when the container starts. The launcher supports client mode and
jumper-selected access-point mode.

## Apply

Run on the Pi host as root, with the drone disarmed and the patch available
locally. Use a console or Ethernet for recovery if Wi-Fi drops.

```sh
set -e
patch_file=/home/duckie/catkin_ws/psu-hri-capstone-f26/tools/migration/wifi/startup.patch
backup=/var/backups/pals-wifi-startup
test ! -e "$backup"
test ! -e /etc/pals-wifi-client-launcher.sh
test ! -e /etc/systemd/system/wpa_supplicant.service.d/pals-client-config.conf
test -s /config/wpa_supplicant.conf
systemctl is-enabled --quiet wpa_supplicant.service
systemctl is-enabled --quiet dhcpcd.service

install -d -m 0700 "$backup/original"
cp -a --parents /etc/dhcpcd.conf /etc/network/interfaces.d/wlan0 \
    /data/autoboot/duckiedrone.yaml "$backup/original"
docker cp wifi-access-point:/launch/dt-wifi-access-point/default.sh \
    "$backup/original/etc/pals-wifi-client-launcher.sh"
cp -a "$backup/original" "$backup/staged"
mkdir -p "$backup/staged/etc/systemd/system/wpa_supplicant.service.d"
patch -d "$backup/staged" -p1 --batch --forward --fuzz=0 --dry-run < "$patch_file"
patch -d "$backup/staged" -p1 --batch --forward --fuzz=0 < "$patch_file"
bash -n "$backup/staged/etc/pals-wifi-client-launcher.sh"
docker-compose -p duckietown -f "$backup/staged/data/autoboot/duckiedrone.yaml" config --quiet

cp -a "$backup/staged/etc/dhcpcd.conf" /etc/dhcpcd.conf
cp -a "$backup/staged/etc/network/interfaces.d/wlan0" /etc/network/interfaces.d/wlan0
install -m 0755 "$backup/staged/etc/pals-wifi-client-launcher.sh" /etc/pals-wifi-client-launcher.sh
dropin=etc/systemd/system/wpa_supplicant.service.d/pals-client-config.conf
install -D -m 0644 "$backup/staged/$dropin" "/$dropin"
cp -a "$backup/staged/data/autoboot/duckiedrone.yaml" /data/autoboot/duckiedrone.yaml
docker-compose -p duckietown -f /data/autoboot/duckiedrone.yaml up -d --no-deps --no-build wifi-access-point
systemctl reboot
```

After reboot, check hostname-based SSH through the full boot, one IPv4 lease
on `wlan0` with `ip -4 addr show dev wlan0`, and normal dashboard **Connect**.

## Roll back

From the Pi host as root, restore the saved files and recreate the stock
container before rebooting.

```sh
set -e
backup=/var/backups/pals-wifi-startup/original
cp -a "$backup/etc/dhcpcd.conf" /etc/dhcpcd.conf
cp -a "$backup/etc/network/interfaces.d/wlan0" /etc/network/interfaces.d/wlan0
cp -a "$backup/data/autoboot/duckiedrone.yaml" /data/autoboot/duckiedrone.yaml
rm -f /etc/systemd/system/wpa_supplicant.service.d/pals-client-config.conf
docker-compose -p duckietown -f /data/autoboot/duckiedrone.yaml up -d --no-deps --no-build wifi-access-point
rm -f /etc/pals-wifi-client-launcher.sh
systemctl reboot
```
